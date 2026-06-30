// ============= EVENT TYPES & STRUCTURES =============
#define EVT_IGN_LOW  0
#define EVT_IGN_HIGH 1
#define EVT_CYP      2

#define HISTORY_SIZE 64
struct Event {
    uint8_t type;
    unsigned long timestamp;
    bool pair_a;
    uint8_t seq;
};

volatile Event history[HISTORY_SIZE];
volatile uint8_t history_head = 0;
volatile bool history_full = false;
volatile uint8_t ign_sequence_count = 0;

// ============= DUAL PROFILING: A21 + CYP =============
#include <EEPROM.h>
#define EEPROM_ADDR_CYP_PROFILE 32
#define EEPROM_ADDR_PHASE_FILTER 64

struct CYPProfile {
    uint16_t session_count;
    uint16_t total_valid_pulses;
    int32_t mean_offset_seq0_deg;
    int32_t mean_offset_seq1_deg;
    uint16_t offset_variance_seq0;
    uint16_t offset_variance_seq1;
    uint8_t pulse_confidence;
    uint8_t confidence;
};

// ===== Kalman filter for PHASE_ACCUM window length (fixed point, x16) =====
struct PhaseWindowFilter {
    int32_t  estimate_x16;        // current best guess of needed fire-window length, x16 fixed point
    uint32_t estimate_variance;   // P: filter's confidence in the estimate
};

volatile CYPProfile cyp_profile = {0, 0, 90, 90, 15, 15, 0, 0};

// Default: 8 fires (128 in x16), high uncertainty
volatile PhaseWindowFilter phase_filter = {128, 1000};

#define PHASE_FILTER_PROCESS_NOISE      5     // Q: small session-to-session drift allowance
#define PHASE_FILTER_R_GOOD             400   // measurement noise on success -> gentler correction
#define PHASE_FILTER_R_BAD              150   // measurement noise on failure -> stronger correction
#define PHASE_FILTER_PENALTY_FIRES_X16  32    // bad commit: nudge estimate UP by ~2 fires
#define PHASE_FILTER_REWARD_FIRES_X16   (-8)  // good commit: nudge estimate DOWN by ~0.5 fires

volatile unsigned long a21_last_low_time = 0;
volatile uint8_t shift = 2;

volatile bool active_pair_A = true;
volatile unsigned long last_cyp_time = 0;
volatile unsigned long rpm = 0;
volatile bool ignition_enabled = false;

volatile unsigned long last_ign_time = 0;

volatile unsigned long ign_isr_count = 0;
volatile unsigned long ign_debounce_pass_count = 0;
volatile unsigned long ign_state_low_count = 0;
volatile unsigned long ign_state_high_count = 0;
volatile bool last_ign_state = false;

volatile unsigned long cyp_isr_count = 0;
volatile unsigned long cyp_debounce_pass_count = 0;
volatile unsigned long cyp_valid_count = 0;
volatile unsigned long cyp_invalid_count = 0;
volatile uint8_t cyp_mismatch_count = 0;

volatile unsigned long last_fire_time = 0;
volatile uint32_t fire_to_fire_interval = 0;

volatile bool flag_sync_state_changed = false;
volatile bool flag_phase_no_cyp_warning = false;
volatile bool flag_phase_flip_triggered = false;

const int IGN_IN = 2;
const int CYP_IN = 3;
const int COIL_A = 4;
const int COIL_B = 5;

unsigned long runtime_min;
static unsigned long last_min_update = 0;

// ============= SYNC STATE MACHINE =============
enum SyncState { STARTUP_GUARD, PHASE_ACCUM, LOCKED };
volatile SyncState sync_state = STARTUP_GUARD;

#define STARTUP_GUARD_MS 1500
#define CYP_MISMATCHES_FOR_LOSS 3
#define CYP_SLOW_INT 300000
#define CYP_FAST_INT 15000
#define SEQ_MAX 2

// ===== PHASE_ACCUM windowed-tally state =====
#define PHASE_WINDOW_MAX  24   // hard ceiling, no-data forced coin flip
#define TIE_EXTEND_FIRES  4    // 1 rotation grace period on tie

struct PhaseAccum {
    uint8_t  fire_count;
    uint16_t hits[2];
    int32_t  offset_sum[2];
    uint32_t weight_sum[2];
    bool     tie_extended;
    uint8_t  tie_recheck_at;   // fire_count value to recheck tie, 0 = none pending
};
volatile PhaseAccum phase_accum = {0, {0,0}, {0,0}, {0,0}, false, 0};

volatile uint8_t base_window = 8;       // derived from phase_filter at PHASE_ACCUM entry
volatile bool cyp_armed = false;
volatile unsigned long boot_guard_start = 0;
volatile uint8_t last_commit_fire_count = 0;

// ===== Stall watchdog (A21-only, single-use phase flip) =====
#define STALL_WATCHDOG_FIRES 6

volatile uint32_t stall_watch_start_interval = 0;
volatile uint8_t  stall_watch_fire_count = 0;
volatile bool     stall_watch_armed = false;
volatile bool     phase_flip_used = false;   // the flip-flop: set once, cleared only on reset/reboot
volatile bool was_running = false;

void load_profiles() {
    EEPROM.get(EEPROM_ADDR_CYP_PROFILE, cyp_profile);
    EEPROM.get(EEPROM_ADDR_PHASE_FILTER, phase_filter);

    if (cyp_profile.session_count > 10000) {
        memset(&cyp_profile, 0, sizeof(cyp_profile));
        cyp_profile.mean_offset_seq0_deg = 90;
        cyp_profile.mean_offset_seq1_deg = 90;
        cyp_profile.offset_variance_seq0 = 15;
        cyp_profile.offset_variance_seq1 = 15;
    }
    // Corrupt or first-boot guard for the phase filter
    if (phase_filter.estimate_x16 < 64 || phase_filter.estimate_x16 > 384 ||
        phase_filter.estimate_variance > 1000000UL) {
        phase_filter.estimate_x16 = 128;     // 8 fires default
        phase_filter.estimate_variance = 1000;
    }

    cyp_profile.session_count++;

    cyp_profile.confidence = (cyp_profile.confidence * 85) / 100;

    if (cyp_profile.confidence  < 30) {
        shift = 2;
    } else if (cyp_profile.confidence  < 70) {
        shift = 3;
    } else {
        shift = 4;
    }
}

void save_profiles() {

    cyp_profile.pulse_confidence = (int)(((uint64_t)cyp_profile.total_valid_pulses * 100) / (cyp_profile.total_valid_pulses + 20));

    cyp_profile.confidence = cyp_profile.pulse_confidence;

    EEPROM.put(EEPROM_ADDR_CYP_PROFILE, cyp_profile);
    EEPROM.put(EEPROM_ADDR_PHASE_FILTER, phase_filter);
}

void log_event(uint8_t type, uint8_t seq_snapshot) {
    bool pair_for_seq = (seq_snapshot == 0);
    history[history_head] = {type, micros(), pair_for_seq, seq_snapshot};
    history_head = (history_head + 1) % HISTORY_SIZE;
    if (history_head == 0) history_full = true;
}


// ===== Kalman update for PHASE_ACCUM window length (fixed point) =====
inline void kalman_update_window(int32_t measurement_x16, uint32_t R) {
    // Predict step: no motion model, just inflate variance slightly for drift
    phase_filter.estimate_variance += PHASE_FILTER_PROCESS_NOISE;

    uint32_t P = phase_filter.estimate_variance;
    uint32_t denom = P + R;
    uint32_t K_x256 = (denom > 0) ? ((uint32_t)((uint64_t)P * 256) / denom) : 0;

    int32_t innovation = measurement_x16 - phase_filter.estimate_x16;
    int32_t correction = (int32_t)(((int64_t)K_x256 * innovation) >> 8);

    phase_filter.estimate_x16 += correction;
    phase_filter.estimate_variance = (uint32_t)(((uint64_t)(256 - K_x256) * P) >> 8);

    // Clamp: 4 to 24 fires (x16 = 64 to 384)
    if (phase_filter.estimate_x16 < 64)  phase_filter.estimate_x16 = 64;
    if (phase_filter.estimate_x16 > 384) phase_filter.estimate_x16 = 384;
}

// ===== PHASE_ACCUM helpers =====
inline void reset_phase_accum() {
    phase_accum.fire_count = 0;
    phase_accum.hits[0] = phase_accum.hits[1] = 0;
    phase_accum.offset_sum[0] = phase_accum.offset_sum[1] = 0;
    phase_accum.weight_sum[0] = phase_accum.weight_sum[1] = 0;
    phase_accum.tie_extended = false;
    phase_accum.tie_recheck_at = 0;
    cyp_armed = false;

    // Derive this attempt's base window from the learned filter estimate
    uint8_t derived = (uint8_t)(phase_filter.estimate_x16 >> 4);
    if (derived < 4) derived = 4;
    if (derived > PHASE_WINDOW_MAX) derived = PHASE_WINDOW_MAX;
    base_window = derived;
}

inline void commit_phase(uint8_t winner) {
    ign_sequence_count = winner;
    active_pair_A = (winner == 0);

    if (phase_accum.hits[winner] > 0) {
        int32_t window_mean = phase_accum.offset_sum[winner] / phase_accum.hits[winner];
        if (winner == 0) cyp_profile.mean_offset_seq0_deg = window_mean;
        else              cyp_profile.mean_offset_seq1_deg = window_mean;
    }

    last_commit_fire_count = phase_accum.fire_count;

    cyp_mismatch_count = 0;
    sync_state = LOCKED;
    ignition_enabled = true;
    flag_sync_state_changed = true;
    log_event(EVT_CYP, winner);

    // arm stall watchdog fresh on every lock entry
    stall_watch_armed = true;
    stall_watch_fire_count = 0;
    stall_watch_start_interval = 0;
}

void setup() {
    PORTD |= (1 << COIL_A);
    DDRD |= (1 << COIL_A);
    PORTD |= (1 << COIL_B);
    DDRD |= (1 << COIL_B);
    DDRD &= ~(1 << IGN_IN);
    DDRD &= ~(1 << CYP_IN);

    EICRA &= ~((1 << ISC11) | (1 << ISC10) | (1 << ISC01) | (1 << ISC00));
    EICRA |= (1 << ISC00);
    EICRA |= (1 << ISC11) | (1 << ISC10);
    EIMSK |= (1 << INT0) | (1 << INT1);

    Serial.begin(9600);
    Serial.println(F("\n=== H22A Wasted-Spark Ignition (Dual Profiling) ==="));
    Serial.println(F("INT0/A21 = Master Clock | INT1/CYP = Learning Validator"));
    Serial.println(F("Startup guard on INT0 | Windowed phase-accum lock | A21-only stall watchdog\n"));

    load_profiles();
    Serial.print(F("[BOOT] CYP confidence: ")); Serial.print(cyp_profile.confidence); Serial.println(F("%"));
    Serial.print(F("[BOOT] Phase window estimate: ")); Serial.print(phase_filter.estimate_x16 >> 4); Serial.println(F(" fires\n"));

    sei();
}

// ============= INT0: A21 MASTER CLOCK =============
// Startup guard lives here now (INT0 is the noise source at power-on).
// Coil drive is gated on ignition_enabled; timing/sequence tracking is NOT,
// so CYP always has a real last_fire_time / ign_sequence_count to read.
ISR(INT0_vect) {
    if (sync_state == STARTUP_GUARD) {
        ign_isr_count++;
        unsigned long now = micros();
        if (boot_guard_start == 0) boot_guard_start = now;
        if (now - boot_guard_start < (unsigned long)STARTUP_GUARD_MS * 1000UL) {
            return;  // still settling, discard everything
        }
        sync_state = PHASE_ACCUM;
        flag_sync_state_changed = true;
        reset_phase_accum();
        // fall through: this edge is processed normally below,
        // but cyp_armed only flips true on the NEXT fire event
    }

    ign_isr_count++;

    unsigned long now = micros();

    uint32_t min_ign_interval = 3750;
    if (rpm > 0) {
        uint32_t local_rpm = rpm;
        min_ign_interval = 30000000UL / local_rpm;
        if (min_ign_interval < 3750)   min_ign_interval = 3750;
        if (min_ign_interval > 150000) min_ign_interval = 150000;
    }

    if (now - last_ign_time < min_ign_interval) return;
    last_ign_time = now;
    ign_debounce_pass_count++;

    bool state = (PIND & (1 << IGN_IN)) ? HIGH : LOW;
    if (state == last_ign_state) return;
    last_ign_state = state;

    uint8_t seq_snapshot = ign_sequence_count;

    if (state == LOW) {
        log_event(EVT_IGN_LOW, seq_snapshot);
        ign_state_low_count++;
        a21_last_low_time = now;

        if (ignition_enabled) {
            if (active_pair_A) PORTD &= ~(1 << COIL_A);
            else                 PORTD &= ~(1 << COIL_B);
        }
    } else {
        log_event(EVT_IGN_HIGH, seq_snapshot);
        ign_state_high_count++;

        unsigned long dwell_dur = now - a21_last_low_time;

        if (ignition_enabled) {
            if (active_pair_A) PORTD |= (1 << COIL_A);
            else                 PORTD |= (1 << COIL_B);
        }

        if (last_fire_time > 0) {
            fire_to_fire_interval = now - last_fire_time;
        }
        last_fire_time = now;

        // RPM EMA with rounding to avoid integer dead-zone at small deltas
        if (fire_to_fire_interval > 3750 && fire_to_fire_interval < 150000) {
            uint32_t instant_rpm = 30000000UL / fire_to_fire_interval;
            uint32_t delta = (instant_rpm >= rpm) ? (instant_rpm - rpm) : (rpm - instant_rpm);
            int32_t step = (delta < 4) ? (int32_t)delta : (int32_t)((delta + 2) >> 2);
            if (instant_rpm >= rpm) rpm += step;
            else                     rpm -= step;
        }

        ign_sequence_count++;
        if (ign_sequence_count >= SEQ_MAX) ign_sequence_count = 0;
        active_pair_A = (ign_sequence_count == 0);

        // ===== PHASE_ACCUM window bookkeeping (fires only) =====
        if (sync_state == PHASE_ACCUM) {
            if (!cyp_armed) {
                cyp_armed = true;   // CYP starts being processed from the NEXT fire event onward
            }

            phase_accum.fire_count++;

            // tie recheck boundary (mid-window, +TIE_EXTEND_FIRES after the tie was seen)
            if (phase_accum.tie_recheck_at != 0 &&
                phase_accum.fire_count >= phase_accum.tie_recheck_at) {
                bool has_data = (phase_accum.hits[0] + phase_accum.hits[1]) > 0;
                if (has_data) {
                    if (phase_accum.weight_sum[0] != phase_accum.weight_sum[1]) {
                        uint8_t winner = (phase_accum.weight_sum[0] > phase_accum.weight_sum[1]) ? 0 : 1;
                        commit_phase(winner);
                        return;
                    } else {
                        // still tied after grace rotation: stay organic
                        commit_phase(ign_sequence_count);
                        return;
                    }
                }
                phase_accum.tie_recheck_at = 0;  // fall through to standard boundary logic
            }

            // standard window boundaries, spaced by the filter-derived base_window
            if (phase_accum.fire_count % base_window == 0) {
                bool has_data = (phase_accum.hits[0] + phase_accum.hits[1]) > 0;

                if (!has_data) {
                    flag_phase_no_cyp_warning = true;
                    if (phase_accum.fire_count >= PHASE_WINDOW_MAX) {
                        commit_phase((uint8_t)(now & 1));  // forced coin flip
                        return;
                    }
                    // else: extend silently, keep accumulating
                } else if (phase_accum.weight_sum[0] == phase_accum.weight_sum[1]) {
                    if (!phase_accum.tie_extended) {
                        phase_accum.tie_extended = true;
                        phase_accum.tie_recheck_at = phase_accum.fire_count + TIE_EXTEND_FIRES;
                    } else {
                        // already extended once, still tied: stay organic
                        commit_phase(ign_sequence_count);
                        return;
                    }
                } else {
                    uint8_t winner = (phase_accum.weight_sum[0] > phase_accum.weight_sum[1]) ? 0 : 1;
                    commit_phase(winner);
                    return;
                }
            }
        }

        // ===== STALL WATCHDOG (A21-only, LOCKED state, single-use flip) =====
        if (sync_state == LOCKED && stall_watch_armed && !phase_flip_used) {
            if (fire_to_fire_interval > 0) {
                if (stall_watch_fire_count == 0) {
                    stall_watch_start_interval = fire_to_fire_interval;
                    stall_watch_fire_count = 1;
                } else {
                    stall_watch_fire_count++;
                    if (stall_watch_fire_count >= STALL_WATCHDOG_FIRES) {
                        // Real catch (200->1100 crank rpm in <1s) implies roughly a
                        // halved interval by fire #6. Anything less is "not catching".
                        uint32_t required_max = stall_watch_start_interval / 2;

                        if (fire_to_fire_interval >= required_max) {
                            // NO SPEEDUP — flip phase, once, unflinchingly
                            ign_sequence_count = (ign_sequence_count == 0) ? 1 : 0;
                            active_pair_A = (ign_sequence_count == 0);
                            phase_flip_used = true;
                            stall_watch_armed = false;
                            flag_phase_flip_triggered = true;

                            // Kalman: bad commit, push window estimate up (mild)
                            int32_t measurement_x16 = ((int32_t)last_commit_fire_count << 4) + PHASE_FILTER_PENALTY_FIRES_X16;
                            kalman_update_window(measurement_x16, PHASE_FILTER_R_BAD);
                        } else {
                            // genuinely catching — good commit, inch window down (gentle)
                            int32_t measurement_x16 = ((int32_t)last_commit_fire_count << 4) + PHASE_FILTER_REWARD_FIRES_X16;
                            kalman_update_window(measurement_x16, PHASE_FILTER_R_GOOD);
                            stall_watch_armed = false;
                            was_running = true;
                        }
                    }
                }
            }
        }
    }
}

// ============= INT1: CYP LEARNING VALIDATOR =============
ISR(INT1_vect) {
    if (!cyp_armed) return;             // gated by INT0 arming, replaces old startup-guard-in-INT1 logic
    if (sync_state == STARTUP_GUARD) return;  // safety net

    cyp_isr_count++;

    unsigned long now = micros();
    unsigned long elapsed = now - last_cyp_time;
    last_cyp_time = now;

    uint32_t min_cyp_interval = 15000;
    uint32_t local_rpm = rpm;   // single snapshot to avoid torn multi-byte volatile reads

    if (local_rpm > 0) {
        min_cyp_interval = 120000000UL / local_rpm;
        if (min_cyp_interval < CYP_FAST_INT)  min_cyp_interval = CYP_FAST_INT;
        if (min_cyp_interval > CYP_SLOW_INT) min_cyp_interval = CYP_SLOW_INT;
    }

    if (elapsed < min_cyp_interval) return;
    cyp_debounce_pass_count++;

    uint8_t seq_snapshot = ign_sequence_count;

    int32_t offset_us = now - last_fire_time;
    int32_t offset_deg = (local_rpm > 0) ? ((int64_t)offset_us * local_rpm) / 166666UL : 0;

    if (sync_state == PHASE_ACCUM) {
        // Weighted tally only — no validation/learning during accumulation
        int32_t err = labs(offset_deg - 90);
        uint32_t w = (err < 45) ? (uint32_t)(45 - err) : 1;

        phase_accum.hits[seq_snapshot]++;
        phase_accum.offset_sum[seq_snapshot] += offset_deg;
        phase_accum.weight_sum[seq_snapshot] += w;
        return;
    }

    if (sync_state == LOCKED) {
        bool correct_seq = (ign_sequence_count == 0);

        int32_t active_mean_offset_deg =
            correct_seq ? cyp_profile.mean_offset_seq0_deg
                        : cyp_profile.mean_offset_seq1_deg;

        uint16_t active_variance_deg =
            correct_seq ? cyp_profile.offset_variance_seq0
                        : cyp_profile.offset_variance_seq1;

        uint32_t active_err_deg = labs(offset_deg - active_mean_offset_deg);

        bool timing_ok = false;

        if (cyp_profile.confidence < 10) {
            timing_ok = true;
        } else {
            uint32_t variance_tol_deg = active_variance_deg * 4 + 1;

            const uint32_t max_deg_window = 15;
            const uint32_t min_deg_window = 2;
            uint32_t conf = ((uint64_t)cyp_profile.confidence * 1671168) >> 16;
            if (conf > 255) conf = 255;
            uint32_t conf_squared = conf * conf;
            uint32_t degree_tol = max_deg_window - (((max_deg_window - min_deg_window) * conf_squared) >> 16);
            if (degree_tol < min_deg_window) degree_tol = min_deg_window;

            uint32_t dynamic_tolerance_deg = max(variance_tol_deg, degree_tol);

            if (dynamic_tolerance_deg < min_deg_window) dynamic_tolerance_deg = min_deg_window;
            if (dynamic_tolerance_deg > max_deg_window) dynamic_tolerance_deg = max_deg_window;

            timing_ok = (active_err_deg < dynamic_tolerance_deg);

        }

        // NOTE: per design, CYP no longer forces a desync back to PHASE_ACCUM.
        // It keeps scoring/learning only. Phase is irreversible (flip-flop)
        // except via the A21-only stall watchdog.
        if (!correct_seq)
        {
            cyp_invalid_count++;
            cyp_mismatch_count++;
        }
        else if (!timing_ok)
        {
            cyp_profile.mean_offset_seq0_deg += ((offset_deg - cyp_profile.mean_offset_seq0_deg) >> shift);
            cyp_profile.offset_variance_seq0  += ((active_err_deg - cyp_profile.offset_variance_seq0) >> shift);
            cyp_mismatch_count++;

        }
        else
        {
            cyp_valid_count++;
            cyp_profile.total_valid_pulses++;

            if (correct_seq) {
                cyp_profile.mean_offset_seq0_deg += ((offset_deg - cyp_profile.mean_offset_seq0_deg) >> shift);
                cyp_profile.offset_variance_seq0  += ((active_err_deg - cyp_profile.offset_variance_seq0) >> shift);
            } else {
                cyp_profile.mean_offset_seq1_deg += ((offset_deg - cyp_profile.mean_offset_seq1_deg) >> shift);
                cyp_profile.offset_variance_seq1  += ((active_err_deg - cyp_profile.offset_variance_seq1) >> shift);
            }

            log_event(EVT_CYP, seq_snapshot);
        }
    }
}

void loop() {
    static SyncState last_sync = STARTUP_GUARD;

    if (sync_state != last_sync || flag_sync_state_changed) {
        last_sync = sync_state;
        flag_sync_state_changed = false;

        if (sync_state == LOCKED) {
            Serial.println(F("\n[SYNC] LOCKED - A21 master, CYP learning validator"));
        } else if (sync_state == PHASE_ACCUM) {
            Serial.print(F("\n[SYNC] PHASE_ACCUM - tallying phase, window="));
            Serial.print(base_window);
            Serial.println(F(" fires"));
        } else {
            Serial.println(F("\n[SYNC] STARTUP_GUARD - settling"));
        }
    }

    if (flag_phase_no_cyp_warning) {
        flag_phase_no_cyp_warning = false;
        Serial.println(F("[WARN] PHASE_ACCUM window closed with no CYP data, extending"));
    }

    if (flag_phase_flip_triggered) {
        flag_phase_flip_triggered = false;
        Serial.println(F("\n[WATCHDOG] No speedup detected - phase flipped (one-shot)\n"));
    }

    if(rpm < 500){
        last_min_update = millis();
    }else{
        if (millis() - last_min_update > 60000) {
            runtime_min++;
            last_min_update = millis();
        }
    }


    if (was_running && rpm == 0) {
        save_profiles();
        was_running = false;
        Serial.print(F("[SAVE] A21 edges: ")); Serial.print(ign_state_low_count + ign_state_high_count);
        Serial.print(F(" | CYP valid: ")); Serial.print(cyp_valid_count);
        Serial.print(F("% | CYP conf: ")); Serial.print(cyp_profile.confidence);
        Serial.print(F("% | window est: ")); Serial.print(phase_filter.estimate_x16 >> 4);
        Serial.println(F(" fires"));
    }

    static unsigned long last_print = 0;
    if (millis() - last_print > 500) {
        Serial.print(F("[STATUS] "));
        if (sync_state == LOCKED) Serial.print(F("LOCKED"));
        else if (sync_state == PHASE_ACCUM) Serial.print(F("PHASE_ACCUM"));
        else Serial.print(F("STARTUP"));

        Serial.print(F(" | IGN: ")); Serial.print(ign_state_high_count);
        Serial.print(F(" | RPM: ")); Serial.print(rpm);
        Serial.print(F(" | CYP: ")); Serial.print(cyp_valid_count);
        Serial.print(F("/")); Serial.print(cyp_invalid_count);
        Serial.print(F(" | PAIR: ")); Serial.println(active_pair_A ? F("A") : F("B"));

        last_print = millis();
    }

    if (Serial.available()) {
        char cmd = Serial.read();

        if (cmd == 'h') {
            Serial.println(F("\n--- EVENT HISTORY (last 64) ---"));
            uint8_t start = history_full ? history_head : 0;
            uint8_t count = history_full ? HISTORY_SIZE : history_head;
            unsigned long first_time = (count > 0) ? history[start].timestamp : 0;

            for (uint8_t i = 0; i < count; i++) {
                uint8_t idx = (start + i) % HISTORY_SIZE;
                volatile Event& e = history[idx];
                unsigned long rel_time = e.timestamp - first_time;

                Serial.print(rel_time); Serial.print(F("us "));
                switch (e.type) {
                    case EVT_IGN_LOW:  Serial.print(F("DWELL  ")); break;
                    case EVT_IGN_HIGH: Serial.print(F("FIRE   ")); break;
                    case EVT_CYP:      Serial.print(F("CYP    ")); break;
                    default:           Serial.print(F("???    ")); break;
                }
                Serial.print(F("SEQ:")); Serial.print(e.seq);
                Serial.print(F(" ")); Serial.println(e.pair_a ? F("PAIR_A") : F("PAIR_B"));
            }
            Serial.println(F("--- END ---\n"));
        }

        else if (cmd == 's') {
            Serial.println(F("\n========== SYSTEM SNAPSHOT =========="));
            Serial.println(F("[LEGEND]"));
            Serial.println(F("  A21 = P28 ignition master clock (should be reliable)"));
            Serial.println(F("  CYP = crankshaft sensor validator (learning its behavior)"));
            Serial.println(F("  Conf = confidence 0-100% (higher = more data)"));

            Serial.println(F("[MASTER - A21/INT0]"));
            Serial.print(F("  DWELL events: ")); Serial.print(ign_state_low_count);
            Serial.print(F("  FIRE events: ")); Serial.print(ign_state_high_count);
            Serial.print(F("  Current RPM: ")); Serial.println(rpm);
            Serial.println();

            Serial.println(F("[VALIDATOR - CYP/INT1 Learning]"));
            Serial.print(F("  Valid CYP pulses: ")); Serial.println(cyp_valid_count);
            Serial.print(F("  Invalid CYP pulses: ")); Serial.println(cyp_invalid_count);
            Serial.print(F("  Mean offset_deg at SEQ:0: ")); Serial.print(cyp_profile.mean_offset_seq0_deg);
            Serial.println(F("deg"));
            Serial.print(F("  Mean offset_deg at SEQ:1: ")); Serial.print(cyp_profile.mean_offset_seq1_deg);
            Serial.println(F("deg"));
            Serial.print(F("  CYP Confidence: ")); Serial.print(cyp_profile.confidence);
            Serial.println(F("%"));
            Serial.println();

            Serial.println(F("[PHASE WINDOW FILTER]"));
            Serial.print(F("  Estimate: ")); Serial.print(phase_filter.estimate_x16 >> 4); Serial.println(F(" fires"));
            Serial.print(F("  Variance: ")); Serial.println(phase_filter.estimate_variance);
            Serial.print(F("  Phase flip used this session: ")); Serial.println(phase_flip_used ? F("YES") : F("no"));
            Serial.println();

            Serial.println(F("[STATE]"));
            Serial.print(F("  Sync state: "));
            if (sync_state == LOCKED) Serial.println(F("LOCKED"));
            else if (sync_state == PHASE_ACCUM) Serial.println(F("PHASE_ACCUM"));
            else Serial.println(F("STARTUP_GUARD"));
            Serial.print(F("  Sequence: ")); Serial.println(ign_sequence_count);
            Serial.print(F("  Active pair: ")); Serial.println(active_pair_A ? F("A (cyls 1&4)") : F("B (cyls 2&3)"));
            Serial.println(F("========== END SNAPSHOT ===========\n"));
        }

        else if (cmd == 'r') {
            //memset(&a21_profile, 0, sizeof(a21_profile));
            memset(&cyp_profile, 0, sizeof(cyp_profile));
            cyp_profile.mean_offset_seq0_deg = 90;
            cyp_profile.mean_offset_seq1_deg = 90;
            cyp_profile.offset_variance_seq0 = 15;
            cyp_profile.offset_variance_seq1 = 15;
            cyp_profile.total_valid_pulses = 0;
            cyp_profile.pulse_confidence = 0;
            cyp_profile.confidence = 0;
            ign_state_high_count = ign_state_low_count = 0;
            cyp_debounce_pass_count = 0;
            ign_isr_count = cyp_isr_count = 0;
            ign_debounce_pass_count = cyp_debounce_pass_count = 0;
            cyp_valid_count = cyp_invalid_count = 0;
            cyp_mismatch_count = 0;
            last_fire_time = 0;
            fire_to_fire_interval = 0;
            boot_guard_start = 0;
            runtime_min = 0;
            was_running = false;

            // Phase filter reset to default (NOT preserved on manual reset)
            phase_filter.estimate_x16 = 128;
            phase_filter.estimate_variance = 1000;

            // Phase accum / watchdog / flip-flop reset
            reset_phase_accum();
            phase_flip_used = false;
            stall_watch_armed = false;
            stall_watch_fire_count = 0;
            stall_watch_start_interval = 0;
            flag_phase_flip_triggered = false;
            flag_phase_no_cyp_warning = false;
            ignition_enabled = false;
            sync_state = STARTUP_GUARD;

            save_profiles();
            Serial.println(F("[RESET] All profiles cleared\n"));
        }
    }
}
