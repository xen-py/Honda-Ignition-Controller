// ============= EVENT TYPES & STRUCTURES =============
#define EVT_IGN_LOW  0
#define EVT_IGN_HIGH 1
#define EVT_CYP      2
#define EVT_SNAPSHOT 3

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
#define EEPROM_ADDR_A21_PROFILE 0
#define EEPROM_ADDR_CYP_PROFILE 32

// A21 (MASTER CLOCK) PROFILE
// Tracks dwell/fire timing consistency from P28 ECU
// Used to verify A21 signal health and detect anomalies
struct A21Profile {
    uint16_t session_count;      // Number of power cycles recorded
    uint16_t total_edges;        // Total DWELL+FIRE edges seen (capped at 65535)
    int32_t mean_dwell_us;       // EMA of coil charge time in µs
                                 // ~3000µs at 12V, longer at low voltage/low RPM
    uint16_t dwell_variance;     // Proxy for dwell consistency (not true variance)
                                 // 100 = few samples, 50 = stable (>20 samples)
    uint16_t fire_variance;      // Same proxy for fire phase duration
    uint8_t confidence;          // 0-100: saturates with edge count
                                 // ~50% at 5 edges, ~90% at 90 edges
};

// CYP (VALIDATOR) PROFILE
// Tracks when CYP pulse arrives relative to the last FIRE event
// Stored in DEGREES (not µs) so profile is valid across entire RPM range
// Learning system: more successful validations = tighter tolerance = higher confidence
struct CYPProfile {
    uint16_t session_count;          // Number of power cycles recorded
    uint16_t total_valid_pulses;     // Cumulative valid CYP pulses (feeds pulse_confidence)
    int32_t mean_offset_seq0_deg;    // EMA of CYP arrival angle after SEQ:0 fire
    int32_t mean_offset_seq1_deg;    // EMA of CYP arrival angle after SEQ:1 fire
                                     // Two separate profiles: CYP jitter differs per sequence
    uint16_t offset_variance_seq0;   // EMA of absolute degree error at SEQ:0
    uint16_t offset_variance_seq1;   // EMA of absolute degree error at SEQ:1
                                     // Drives dynamic tolerance window (2-15 degrees)
    uint8_t pulse_confidence;        // 0-100: based on total_valid_pulses count alone
    uint8_t confidence;              // 0-100: blended (75% pulse_confidence, 25% learning_score)
    uint32_t learning_score;         // RPM and timing-quality weighted reward
                                     // Controls EMA shift rate and tolerance tightness
                                     // Range: 0 to REWARD_MAX (1,000,000)
};

// A21 defaults:
// mean_dwell_us = 3000: reasonable 12V estimate before any learning
// dwell/fire_variance = 100: maximum uncertainty (no samples yet)
// confidence = 0: no data yet
volatile A21Profile a21_profile = {0, 0, 3000, 100, 100, 0};

// CYP defaults:
// mean_offset_seq0/1_deg = 90: CYP expected ~90 degrees after fire
//   (quarter revolution is a reasonable prior for a distributor-mounted sensor)
// offset_variance_seq0/1 = 15: start at max allowed window (15 degrees)
//   wide open until learning narrows it
// pulse_confidence = 0: no pulses seen yet
// confidence = 0: no data yet
// learning_score = 0: start cautious, earn trust
volatile CYPProfile cyp_profile = {0, 0, 90, 90, 15, 15, 0, 0, 0};

//========A21 running accumulators=========//
//initialize a21_profile.mean_dwell_us with value?
//used to calculate dwell_dur
volatile unsigned long a21_last_low_time = 0;
volatile uint8_t shift = 2;

// State machine
volatile bool active_pair_A = true;
volatile bool dwell_active = false;
volatile unsigned long last_cyp_time = 0;
volatile unsigned long rpm = 0;
volatile bool ignition_enabled = false;
//volatile bool is_running = false;

// INT0 (A21) debounce
volatile unsigned long last_ign_time = 0;

// Monitoring counters
volatile unsigned long ign_isr_count = 0;
volatile unsigned long ign_debounce_pass_count = 0;
volatile unsigned long ign_state_low_count = 0;
volatile unsigned long ign_state_high_count = 0;
volatile bool last_ign_state = false;

volatile unsigned long cyp_isr_count = 0;
volatile unsigned long cyp_debounce_pass_count = 0;
volatile unsigned long cyp_valid_count = 0;
volatile unsigned long cyp_invalid_count = 0;
volatile uint8_t cyp_pulse_count = 0;  // Track which CYP pulse we're on
volatile uint8_t cyp_match_count = 0;
volatile uint8_t cyp_mismatch_count = 0;

// RPM calculation from INT0
volatile unsigned long last_fire_time = 0;
volatile uint32_t fire_to_fire_interval = 0;

// ============= SERIAL OUTPUT FLAGS (ISRs just set these) =============
volatile bool flag_sync_state_changed = false;
//volatile bool flag_sync_locked = false;
//volatile bool flag_sync_lost = false;
//volatile bool flag_stats_saved = false;
//volatile uint8_t flag_snapshot_requested = 0;

// Pin assignments
const int IGN_IN = 2;    // A21 from P28
const int CYP_IN = 3;    // CYP sensor
const int COIL_A = 4;
const int COIL_B = 5;

unsigned long runtime_min;
static unsigned long last_min_update = 0;

// ============= SYNC STATE MACHINE =============
enum SyncState { STARTUP_GUARD, WAIT_FOR_SYNC, LOCKED};
volatile SyncState sync_state = STARTUP_GUARD;

#define STARTUP_GUARD_MS 1500        // Ignore data for 1.5s after power-on
#define MIN_CYP_FOR_LOCK 2           // Lock on 2nd valid pulse (throw out first)
#define CYP_MISMATCHES_FOR_LOSS 3
#define CYP_SLOW_INT 300000 //0.3s 300ms 
#define CYP_FAST_INT 15000 //0.015s 15ms 
#define SEQ_MAX 2
#define REWARD_MAX 1000000

void load_profiles() {
    EEPROM.get(EEPROM_ADDR_A21_PROFILE, a21_profile);
    EEPROM.get(EEPROM_ADDR_CYP_PROFILE, cyp_profile);
    
    // Corrupt or first boot: restore safe defaults
    if (a21_profile.session_count > 10000) {
        memset(&a21_profile, 0, sizeof(a21_profile));
        a21_profile.mean_dwell_us  = 3000;
        a21_profile.dwell_variance = 100;
        a21_profile.fire_variance  = 100;
    }
    if (cyp_profile.session_count > 10000) {
        memset(&cyp_profile, 0, sizeof(cyp_profile));
        cyp_profile.mean_offset_seq0_deg = 90;
        cyp_profile.mean_offset_seq1_deg = 90;
        cyp_profile.offset_variance_seq0 = 15;
        cyp_profile.offset_variance_seq1 = 15;
    }

    a21_profile.session_count++;
    cyp_profile.session_count++;

    // Decay confidence on boot: last session's data is less trustworthy
    // than live data. 85% retention keeps history relevant but not dominant.
    cyp_profile.confidence = (cyp_profile.confidence * 85) / 100;

    // Set EMA shift rate based on accumulated learning
    if (cyp_profile.learning_score < 200) {
        shift = 2;   // 1/4 - fast updates, low confidence
    } else if (cyp_profile.learning_score < 1000) {
        shift = 3;   // 1/8 - moderate
    } else {
        shift = 4;   // 1/16 - slow updates, high confidence
    }
}

void save_profiles() {
    // ==========================================
    // Finalize A21 Profile
    // ==========================================
    if (ign_state_low_count > 0) {
        a21_profile.dwell_variance = (ign_state_low_count < 20) ? 100 : 50;
    }
    if (ign_state_high_count > 0) {
        a21_profile.fire_variance = (ign_state_high_count < 20) ? 100 : 50;
    }
    
    // Use uint32_t or uint64_t for engine sensor accumulators
    uint32_t a21_total = (uint32_t)ign_state_low_count + ign_state_high_count;
    a21_profile.total_edges = a21_total;
    
    // Saturation curve with explicit 64-bit cast to prevent overflow on engine data
    // Tuned with a horizontal stretch factor of 10
    a21_profile.confidence = (int)(((uint64_t)a21_total * 100) / (a21_total + 10)); 

    // ==========================================
    // Finalize CYP Profile
    // ==========================================

    // Saturation curve for CYP (using 20 to require slightly more pulses for high confidence)
    cyp_profile.pulse_confidence = (int)(((uint64_t)cyp_profile.total_valid_pulses * 100) / (cyp_profile.total_valid_pulses + 20));

    uint8_t learn_conf = min(100UL, cyp_profile.learning_score / (REWARD_MAX / 100));

    cyp_profile.confidence = (cyp_profile.pulse_confidence*3 + learn_conf)/4;

    
    

    // ==========================================
    // Save to Persistent Storage
    // ==========================================
    EEPROM.put(EEPROM_ADDR_A21_PROFILE, a21_profile);
    EEPROM.put(EEPROM_ADDR_CYP_PROFILE, cyp_profile);
}

void log_event(uint8_t type, uint8_t seq_snapshot) {
    bool pair_for_seq = (seq_snapshot == 0);
    history[history_head] = {type, micros(), pair_for_seq, seq_snapshot};
    history_head = (history_head + 1) % HISTORY_SIZE;
    if (history_head == 0) history_full = true;
}


//prevents reward underflow
inline void punish(uint32_t amount)
{
    if(cyp_profile.learning_score > amount)
        cyp_profile.learning_score -= amount;
    else
        cyp_profile.learning_score = 0;
}

//prevents reward overflow
inline void reward(uint32_t amount)
{
    
    if (REWARD_MAX - cyp_profile.learning_score > amount)
        cyp_profile.learning_score += amount;
    else
        cyp_profile.learning_score = REWARD_MAX;
}

/**
* Max deg = 15, Min = 2;
* Depending on rpm and confidence the acceptable physical degree window
* will grow or shrink. Used for dynamic tolerance measurement
*/
inline uint32_t allowed_degrees_to_us(unsigned long current_rpm){
    const uint32_t max_deg_window = 15; 
    const uint32_t min_deg_window = 2;       
    
    // Scale confidence from 0-255 instead of 0-100
    uint32_t conf = ((uint64_t)cyp_profile.confidence * 1671168) >> 16;
    if (conf > 255) conf = 255;

    // // Quadratic scaling factor using integer math: (conf^2) 
    // Max value is 255 * 255 = 65,025 (Fits in uint16_t/uint32_t)
    uint32_t conf_squared = conf * conf;
    uint32_t total_span = max_deg_window - min_deg_window;

    // Bitwise shift replaces division completely (255 * 255 = 65025, close to 65536)
    // >> 16 is equivalent to dividing by 65536. Quadratic curve drop
    uint32_t allowed_degrees = max_deg_window - ((total_span * conf_squared) >> 16);
    // Guard against edge case
    if (allowed_degrees < min_deg_window) allowed_degrees = min_deg_window;
    // Calculate and return microseconds
    if (current_rpm == 0) return 0; // Prevent division by zero if engine is stopped
    
    return (166666UL * allowed_degrees) / current_rpm;
}

//rpm and err based adjustments depending 
inline void learning_adjustments(int level, int8_t quality){
    
    switch(level){//======================perfect
        case 1:

            if (rpm < 500){
                //reward consecutive successful validations
                reward(cyp_profile.learning_score + (cyp_match_count*cyp_match_count)/2) ;
            }else if (rpm > 500 && rpm < 700){
                reward(30); 
            }else if (rpm < 1100){
                reward(10); 
            }else{
                //logarithmic scaling
                reward(5 + (cyp_profile.learning_score >> 5));
            }

            if(quality>0){
                reward(quality);
            }else{
                punish(abs(quality));
            }

            break;
        case 2: //============================timing off
            reward(2);          // Keep confidence alive

            if (quality > 0)
                reward(quality/4);
            else
                punish(abs(quality)/2);

            break;
            case 3: //====================sequence off
                punish(5 + cyp_mismatch_count);
            break;
        default:
            break;
    }

    if (cyp_profile.learning_score < 200){
        shift = 2;      // 1/4
    }else if (cyp_profile.learning_score < 1000){
        shift = 3;      // 1/8
    }else{
        shift = 4;      // 1/16
    }

}

void setup() {
    PORTD |= (1 << COIL_A);
    DDRD |= (1 << COIL_A);
    PORTD |= (1 << COIL_B);
    DDRD |= (1 << COIL_B);
    DDRD &= ~(1 << IGN_IN);
    DDRD &= ~(1 << CYP_IN);

    EICRA &= ~((1 << ISC11) | (1 << ISC10) | (1 << ISC01) | (1 << ISC00));
    EICRA |= (1 << ISC00);  // ISC01=0, ISC00=0 → 01 = any change
    EICRA |= (1 << ISC11) | (1 << ISC10);  // INT1: rising edge (11)
    EIMSK |= (1 << INT0) | (1 << INT1);

    Serial.begin(9600);
    Serial.println(F("\n=== H22A Wasted-Spark Ignition (Dual Profiling) ==="));
    Serial.println(F("INT0/A21 = Master Clock | INT1/CYP = Learning Validator"));
    Serial.println(F("Startup guard: 1.5s | Throws out first CYP pulse\n"));
    
    load_profiles();
    Serial.print(F("[BOOT] A21 confidence: ")); Serial.print(a21_profile.confidence); Serial.println(F("%"));
    Serial.print(F("[BOOT] CYP confidence: ")); Serial.print(cyp_profile.confidence); Serial.println(F("%\n"));
    

    sei();
}

// RPM CALCULATION ISSUES and STATE MACHINE LOCKOUT ISSUES

// ============= INT0: A21 MASTER CLOCK (PURE) =============
// NO serial.prints here - flag-based output only
ISR(INT0_vect) {
    
    ign_isr_count++;

    unsigned long now = micros();
    
    // Dynamic debounce
    // IGN fires twice per crank revolution (wasted spark)
    // interval = 30,000,000 / rpm
    uint32_t min_ign_interval = 3750; // default ~8000 RPM floor
    if (rpm > 100) {
        min_ign_interval = 30000000UL / rpm;
        if (min_ign_interval < 3750)   min_ign_interval = 3750;   // redline floor
        if (min_ign_interval > 150000) min_ign_interval = 150000; // slow crank ceiling
    }
    
    if (now - last_ign_time < min_ign_interval) return;
    last_ign_time = now;
    ign_debounce_pass_count++;

    bool state = (PIND & (1 << IGN_IN)) ? HIGH : LOW;
    if (state == last_ign_state) return;
    last_ign_state = state;

    uint8_t seq_snapshot = ign_sequence_count;

    if (state == LOW) {
        // DWELL PHASE
        log_event(EVT_IGN_LOW, seq_snapshot);
        ign_state_low_count++;
        dwell_active = true;
        a21_last_low_time = now;
        if(ignition_enabled){
            if (active_pair_A) {
                PORTD &= ~(1 << COIL_A);
            } else {
                PORTD &= ~(1 << COIL_B);
            }
        }
    } else {
        // FIRE PHASE
        log_event(EVT_IGN_HIGH, seq_snapshot);
        ign_state_high_count++;

        // Track dwell duration for A21 profiling
        unsigned long dwell_dur = now - a21_last_low_time;
        if (!a21_profile.mean_dwell_us) {
            a21_profile.mean_dwell_us = dwell_dur;
        } else {
            a21_profile.mean_dwell_us += (dwell_dur - a21_profile.mean_dwell_us) >> shift;
        }
        dwell_active = false;
        
        if(ignition_enabled){
           if (active_pair_A) {
            PORTD |= (1 << COIL_A);
            } else {
                PORTD |= (1 << COIL_B);
            } 
        }
        
        

        // Calculate RPM from fire-to-fire interval
        if (last_fire_time > 0) {
            fire_to_fire_interval = now - last_fire_time;
        }
        last_fire_time = now;
        
        // Fixed RPM formula - fire happens twice per crank rev
            // 3,750us interval = 4000 RPM | 150,000us interval = 100 RPM
            if (fire_to_fire_interval > 3750 && fire_to_fire_interval < 150000) {
                uint32_t instant_rpm = 30000000UL / fire_to_fire_interval;
                
                if (instant_rpm >= rpm) {
                    // Engine is accelerating
                    rpm += (instant_rpm - rpm) >> 2;
                } else {
                    // Engine is decelerating
                    rpm -= (rpm - instant_rpm) >> 2;
                }
            }
        
        // ADVANCE SEQUENCE
        ign_sequence_count++;
        if (ign_sequence_count >= SEQ_MAX) ign_sequence_count = 0;
        active_pair_A = (ign_sequence_count == 0);
    }
}
static unsigned long startup_time = 0;//possible bug start3 -> start2 -> start3 will alow noise
// ============= INT1: CYP LEARNING VALIDATOR =============
// Profiles CYP timing, no serial prints
ISR(INT1_vect) {
    cyp_isr_count++;
    
    unsigned long now = micros();
    unsigned long elapsed = now - last_cyp_time;
    last_cyp_time = now;

    // Dynamic debounce based on RPM
    // CYP fires once per cam revolution = once per 2 crank revs
    uint32_t min_cyp_interval = 15000; // default for cranking ~redline safe floor

    if (rpm > 0) {
    // rpm here is crank RPM, cam = crank/2
    // CYP interval = 60,000,000 / (rpm/2) = 120,000,000 / rpm
    min_cyp_interval = 120000000UL / rpm;
    if (min_cyp_interval < CYP_FAST_INT)  min_cyp_interval = CYP_FAST_INT;  // redline floor ~8000 crank RPM
    if (min_cyp_interval > CYP_SLOW_INT) min_cyp_interval = CYP_SLOW_INT; // slow crank ceiling
    }

    if (elapsed < min_cyp_interval) return;
    cyp_debounce_pass_count++;
    
    // ===== STARTUP GUARD: Ignore first 1.5s and first pulse =====
    
    if (sync_state == STARTUP_GUARD) {
        if (startup_time == 0) startup_time = millis();
        if (millis() - startup_time < STARTUP_GUARD_MS) {
            return;  // Still in startup window, ignore
        }
        // Exited startup window, move to WAIT_FOR_SYNC
        sync_state = WAIT_FOR_SYNC;
        flag_sync_state_changed = true;
        return;  // Discard this pulse (first one)
    }
    
    uint8_t seq_snapshot = ign_sequence_count;

    // Calculate offset: when did CYP arrive relative to last fire?
    int32_t offset_us = now - last_fire_time;

    // Convert to degrees (RPM-agnostic, consistent across the rev range)
    // degrees = (offset_us * rpm) / 166,666
    int32_t offset_deg = (rpm > 0) ? ((int64_t)offset_us * rpm) / 166666UL : 0;

    switch(sync_state) {
        case WAIT_FOR_SYNC:
            if (elapsed < CYP_SLOW_INT) {
                cyp_pulse_count++;

                if (cyp_pulse_count >= MIN_CYP_FOR_LOCK) {
                    if (offset_us < CYP_SLOW_INT) {
                        ign_sequence_count = 0;
                        active_pair_A = true;
                        cyp_match_count = 1;
                        cyp_pulse_count = 0;
                        cyp_mismatch_count = 0;
                        sync_state = LOCKED;
                        ignition_enabled = true;
                        flag_sync_state_changed = true;
                        log_event(EVT_CYP, 0);
                    }
                }
            }
            break;
            
        case LOCKED: {
            bool correct_seq = (ign_sequence_count == 0);

            // Everything in DEGREES from here
            int32_t active_mean_offset_deg =
                correct_seq ? cyp_profile.mean_offset_seq0_deg
                            : cyp_profile.mean_offset_seq1_deg;

            uint16_t active_variance_deg =
                correct_seq ? cyp_profile.offset_variance_seq0
                            : cyp_profile.offset_variance_seq1;

            // Error in degrees (apples-to-apples)
            uint32_t active_err_deg = labs(offset_deg - active_mean_offset_deg);

            bool timing_ok = false;
            uint32_t amount = 0;

            if (cyp_profile.confidence < 10) {
                timing_ok = true;
            } else {
                // Learned tolerance in degrees
                uint32_t variance_tol_deg = active_variance_deg * 4 + 1;  // +1 deg floor

                // Physical tolerance already in degrees (max_deg - scaled)
                // allowed_degrees_to_us returns µs, we need degrees here
                // Reuse the degree window directly before converting
                const uint32_t max_deg_window = 15;
                const uint32_t min_deg_window = 2;
                uint32_t conf = ((uint64_t)cyp_profile.confidence * 1671168) >> 16;
                if (conf > 255) conf = 255;
                uint32_t conf_squared = conf * conf;
                uint32_t degree_tol = max_deg_window - (((max_deg_window - min_deg_window) * conf_squared) >> 16);
                if (degree_tol < min_deg_window) degree_tol = min_deg_window;

                // Use whichever is larger
                uint32_t dynamic_tolerance_deg = max(variance_tol_deg, degree_tol);

                // Clamp: 2-15 degrees
                if (dynamic_tolerance_deg < min_deg_window) dynamic_tolerance_deg = min_deg_window;
                if (dynamic_tolerance_deg > max_deg_window) dynamic_tolerance_deg = max_deg_window;

                timing_ok = (active_err_deg < dynamic_tolerance_deg);

                if      (active_err_deg < (dynamic_tolerance_deg >> 2)) amount = 3;
                else if (active_err_deg < (dynamic_tolerance_deg >> 1)) amount = 2;
                else if (active_err_deg <  dynamic_tolerance_deg)       amount = 1;
                else                                                     amount = 2;
            }

            if (!correct_seq)//======Phase error (serious)======//
            {
                // INVALID CYP PULSE
                cyp_invalid_count++;
                cyp_mismatch_count++;
                cyp_match_count = 0;
                
                if (cyp_mismatch_count >= (CYP_MISMATCHES_FOR_LOSS + shift)) {
                    uint8_t penalty = CYP_MISMATCHES_FOR_LOSS + shift;
                    cyp_profile.total_valid_pulses = (cyp_profile.total_valid_pulses > penalty)
                        ? cyp_profile.total_valid_pulses - penalty : 0;
                    
                    punish(15);
                    sync_state = WAIT_FOR_SYNC;
                    cyp_pulse_count = 0;
                    cyp_match_count = 0;
                    flag_sync_state_changed = true;
                }
                learning_adjustments(3, amount);
            }
            else if (!timing_ok)//=====timing outlier (forgivable)==//
            {
                punish(1);
                cyp_match_count++;
                // update EMA's: when did CYP arrive relative to last fire?
                // Profile by sequence using exponential average mean
                //(CYP might have different jitter at SEQ:0 vs SEQ:1)
                cyp_profile.mean_offset_seq0_deg += ((offset_deg - cyp_profile.mean_offset_seq0_deg) >> shift);
                cyp_profile.offset_variance_seq0  += ((active_err_deg - cyp_profile.offset_variance_seq0) >> shift);
             

                learning_adjustments(2, amount);
            }
            else//==============perfect==============//
            {
                // VALID CYP PULSE - LEARN ITS TIMING
                cyp_valid_count++;
                cyp_match_count++;
                cyp_mismatch_count = 0;
                cyp_profile.total_valid_pulses++;

                
                if (correct_seq) {
                    cyp_profile.mean_offset_seq0_deg += ((offset_deg - cyp_profile.mean_offset_seq0_deg) >> shift);
                    cyp_profile.offset_variance_seq0  += ((active_err_deg - cyp_profile.offset_variance_seq0) >> shift);
                } else {
                    cyp_profile.mean_offset_seq1_deg += ((offset_deg - cyp_profile.mean_offset_seq1_deg) >> shift);
                    cyp_profile.offset_variance_seq1  += ((active_err_deg - cyp_profile.offset_variance_seq1) >> shift);
                }

                log_event(EVT_CYP, seq_snapshot);
                learning_adjustments(1, amount);
            }
            
            break;
        } // End of case LOCKED scope block
        
        default:
            break;
    }
}

void loop() {
    static SyncState last_sync = STARTUP_GUARD;
    static unsigned long last_stats_save = 0;
    
    // ===== SYNC STATE CHANGES (logged via flags) =====
    if (sync_state != last_sync || flag_sync_state_changed) {
        last_sync = sync_state;
        flag_sync_state_changed = false;
        
        if (sync_state == LOCKED) {
            Serial.println(F("\n[SYNC] LOCKED - A21 master, CYP learning validator"));
        } else if (sync_state == WAIT_FOR_SYNC) {
            Serial.println(F("\n[SYNC] WAIT_FOR_SYNC - awaiting valid CYP"));
        } else {
            Serial.println(F("\n[SYNC] STARTUP_GUARD - 1.5s warm-up"));
        }
    }

    if(rpm < 500){
        last_min_update = millis();
    }else{
        if (millis() - last_min_update > 60000) {
            runtime_min++;
            last_min_update = millis();
        }
    }
    uint16_t save_interval = 5000;
    uint8_t avg_conf = (a21_profile.confidence + cyp_profile.confidence) / 2;
    if      (avg_conf > 90 && runtime_min >= 5) save_interval = 300000;  // 2 min: high conf, 5+ min runtime
    else if (avg_conf > 90)                       save_interval = 30000;   // 30s: high conf but early
    else if (avg_conf > 80)                       save_interval = 15000;   // 15s
    else if (avg_conf < 20)                       save_interval = 2000;    // 2s: learning fast
    
    if (sync_state == LOCKED && millis() - last_stats_save > save_interval) {
        save_profiles();
        last_stats_save = millis();
        //flag_stats_saved = true;
        
        Serial.print(F("[SAVE] A21 edges: ")); Serial.print(ign_state_low_count + ign_state_high_count);
        Serial.print(F(" | CYP valid: ")); Serial.print(cyp_valid_count);
        Serial.print(F(" | A21 conf: ")); Serial.print(a21_profile.confidence);
        Serial.print(F("% | CYP conf: ")); Serial.print(cyp_profile.confidence);
        Serial.println(F("%"));
    }

    // ===== STATUS REPORT (500ms) =====
    static unsigned long last_print = 0;
    if (millis() - last_print > 500) {
        Serial.print(F("[STATUS] "));
        if (sync_state == LOCKED) Serial.print(F("LOCKED"));
        else if (sync_state == WAIT_FOR_SYNC) Serial.print(F("WAIT"));
        else Serial.print(F("STARTUP"));
        
        Serial.print(F(" | IGN: ")); Serial.print(ign_state_high_count);
        Serial.print(F(" | RPM: ")); Serial.print(rpm);
        Serial.print(F(" | CYP: ")); Serial.print(cyp_valid_count);
        Serial.print(F("/")); Serial.print(cyp_invalid_count);
        Serial.print(F(" | PAIR: ")); Serial.println(active_pair_A ? F("A") : F("B"));
        
        last_print = millis();
    }
    static uint32_t last_decay = 0;

    if(millis()-last_decay>1000)
    {
        last_decay=millis();

        if(cyp_profile.learning_score>REWARD_MAX) 
        cyp_profile.learning_score -= cyp_profile.learning_score>>8;
    }


    // ===== SERIAL COMMANDS (safe, outside ISRs) =====
    if (Serial.available()) {
        char cmd = Serial.read();
        
        if (cmd == 'h') {
            // Event history
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
            // Full system snapshot
            Serial.println(F("\n========== SYSTEM SNAPSHOT =========="));
            Serial.println(F("[LEGEND]"));
            Serial.println(F("  A21 = P28 ignition master clock (should be reliable)"));
            Serial.println(F("  CYP = crankshaft sensor validator (learning its behavior)"));
            Serial.println(F("  Conf = confidence 0-100% (higher = more data)"));
            Serial.println(F("  RPM reward = accumulated RPM when CYP was valid\n"));
            
            Serial.println(F("[MASTER - A21/INT0]"));
            Serial.print(F("  DWELL events: ")); Serial.print(ign_state_low_count);
            Serial.print(F(" | mean duration: ")); Serial.print(a21_profile.mean_dwell_us);
            Serial.println(F("µs"));
            Serial.print(F("  FIRE events: ")); Serial.print(ign_state_high_count);
            Serial.print(F("  A21 Confidence: ")); Serial.print(a21_profile.confidence);
            Serial.println(F("%"));
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
            Serial.print(F("  RPM-weighted reward: ")); Serial.println(cyp_profile.learning_score);
            Serial.println();
            
            Serial.println(F("[STATE]"));
            Serial.print(F("  Sync state: "));
            if (sync_state == LOCKED) Serial.println(F("LOCKED"));
            else if (sync_state == WAIT_FOR_SYNC) Serial.println(F("WAIT_FOR_SYNC"));
            else Serial.println(F("STARTUP_GUARD"));
            Serial.print(F("  Sequence: ")); Serial.println(ign_sequence_count);
            Serial.print(F("  Active pair: ")); Serial.println(active_pair_A ? F("A (cyls 1&4)") : F("B (cyls 2&3)"));
            Serial.println(F("========== END SNAPSHOT ===========\n"));
        }
        
        else if (cmd == 'r') {
            // Reset profiles
            memset(&a21_profile, 0, sizeof(a21_profile));
            memset(&cyp_profile, 0, sizeof(cyp_profile));
            a21_profile.mean_dwell_us = 3000;
            a21_profile.dwell_variance = 100;
            a21_profile.fire_variance = 100;
            cyp_profile.mean_offset_seq0_deg = 90;
            cyp_profile.mean_offset_seq1_deg = 90;
            cyp_profile.offset_variance_seq0 = 15;
            cyp_profile.offset_variance_seq1 = 15;
            cyp_profile.total_valid_pulses = 0;
            cyp_profile.pulse_confidence = 0;        
            cyp_profile.confidence = 0;               
            cyp_profile.learning_score = 0; 
            ign_state_high_count = ign_state_low_count = 0;
            cyp_debounce_pass_count = 0;
            ign_isr_count = cyp_isr_count = 0;
            ign_debounce_pass_count = cyp_debounce_pass_count = 0;
            cyp_valid_count = cyp_invalid_count = 0;
            cyp_match_count = cyp_mismatch_count = 0;
            last_fire_time = 0;
            fire_to_fire_interval = 0;
            startup_time = 0;
            runtime_min = 0;
            save_profiles();
            Serial.println(F("[RESET] All profiles cleared\n"));
        }
    }
}
