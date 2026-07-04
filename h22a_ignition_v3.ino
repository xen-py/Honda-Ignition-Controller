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

// ============= DUAL PROFILING: A21 + CYP =============
#include <EEPROM.h>
#define EEPROM_A21_PROFILE 0
#define EEPROM_ADDR_CYP_PROFILE 32
#define EEPROM_ADDR_PHASE_FILTER 64


volatile unsigned long a21_last_low_time = 0;
volatile uint8_t shift = 2;

volatile unsigned long last_cyp_time = 0;
volatile unsigned long rpm = 0;
volatile bool ignition_enabled = false;

volatile unsigned long last_ign_time = 0;

volatile unsigned long ign_isr_count = 0;
volatile unsigned long ign_debounce_pass_count = 0;
volatile unsigned long ign_state_low_count = 0;
volatile unsigned long ign_state_high_count = 0;
volatile bool last_ign_state = false;
volatile bool last_cyp_state = false;

volatile unsigned long cyp_isr_count = 0;
volatile unsigned long cyp_debounce_pass_count = 0;
volatile unsigned long cyp_invalid_count = 0;
volatile uint8_t quad_index = 0;
volatile uint8_t mean_quad_index = 0;

volatile unsigned long last_fire_time = 0;
volatile uint32_t fire_to_fire_interval = 0;

volatile bool flag_sync_state_changed = false;
volatile bool flag_phase_no_cyp_warning = false;
volatile bool flag_phase_flip_triggered = false;

volatile uint8_t pulse_phase = 0;// ABABAB/010101... //unchanging untampered raw relative phase (SINUS NODE)
volatile uint8_t phase_offset = 0; //inverts (1) or leaves alone(0) the raw phase to match found reality? 
volatile uint8_t current_phase = 0; //the lovechild of pulse_phase(the VFIB heartbeat) ^ (inverted by the) phase_offset (defibrillator signal) 

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
#define CYP_SLOW_INT 300000
#define CYP_FAST_INT 15000
#define SEQ_MAX 2

// ===== PHASE_ACCUM windowed-tally state =====
#define PHASE_WINDOW_MAX  14   // hard ceiling, no-data forced coin flip
struct PhaseAccum
{
    uint8_t fire_count;

    uint8_t quadrant_hits[4];

    int16_t mean_quadrant_x8;
};

volatile bool quadrant_seen[4];
volatile PhaseAccum phase_accum =
{
    0,
    {0,0,0,0},
    -1
};

// ===== EMA: phase stability (NOT quadrant tracking) =====
// Tracks consistency of CYP alignment vs current hypothesis
volatile uint8_t base_window = 7;       //no need to check last interval, chances are good
volatile unsigned long boot_guard_start = 0;
volatile uint8_t last_commit_fire_count = 0;

// ===== Stall watchdog (A21-only, single-use phase flip) =====
#define STALL_WATCHDOG_FIRES 6

volatile uint32_t stall_watch_start_interval = 0;
volatile uint8_t  stall_watch_fire_count = 0;
volatile bool     stall_watch_armed = false;
volatile bool     phase_flip_used = false;   // the flip-flop: set once, cleared only on reset/reboot
volatile bool was_running = false;
// ===== STALL DETECTION =====
// Based on raw time since last fire event, not rpm EMA.
// 50 crank RPM = 600,000us between fires. Anything beyond that
// is physically implausible even on a dead battery crank attempt.
// stall_detected is set in loop() — never in an ISR.
// #define STALL_COIL_SAFETY to actually pull coils high on stall.
// Leave undefined to run detection framework only (current state).
#define STALL_THRESHOLD_US  600000UL   // > 600ms since last fire = stalled
#define STALL_COIL_SAFETY               // uncomment to enable coil protection
volatile bool stall_detected = false;

void log_event(uint8_t type)
{
    history[history_head] = {
        type,
        micros(),
        current_phase == 0,
        current_phase
    };

    history_head = (history_head + 1) % HISTORY_SIZE;
    if (history_head == 0)
        history_full = true;
}

volatile bool cyp_armed = false;
volatile unsigned long cyp_fall_time = 0;
volatile unsigned long last_valid_cyp_time = 0;


// ===== PHASE_ACCUM helpers =====

inline void reset_phase_accum()
{
    phase_accum.fire_count = 0;

    for(uint8_t i=0;i<4;i++)
        phase_accum.quadrant_hits[i]=0;

    phase_accum.mean_quadrant_x8 = 0;
}

inline void defibrillator(uint8_t best) {
    //-------------------------------------------------
    // Convert quadrant -> phase
    //-------------------------------------------------
    quad_index=best;
    phase_offset=(best&1);
    current_phase = pulse_phase ^ phase_offset;// 

    last_commit_fire_count = phase_accum.fire_count;
    
    sync_state = LOCKED;
    ignition_enabled = true;
    flag_sync_state_changed = true;
    log_event(EVT_CYP);

    // arm stall watchdog fresh on every lock entry
    stall_watch_armed = true;
    stall_watch_fire_count = 0;
    stall_watch_start_interval = 0;
}

//**


inline bool processPhaseAccum()
{
    phase_accum.fire_count++;

    //-------------------------------------------------
    // Wait for observation window
    //-------------------------------------------------

    if(phase_accum.fire_count<base_window)
        return false;

    //-------------------------------------------------
    // Find dominant quadrant
    //-------------------------------------------------
    
    uint8_t best=0;

    for (uint8_t i = 1; i < 4; i++)
    if (phase_accum.quadrant_hits[i] > phase_accum.quadrant_hits[best])
        best = i;
    uint8_t best_hits = phase_accum.quadrant_hits[best];
    uint8_t total_hits = 0;
    for (uint8_t i = 0; i < 4; i++) total_hits += phase_accum.quadrant_hits[i];

    //-------------------------------------------------
    // EMA nearest integer
    //-------------------------------------------------

    uint8_t ema_quad=
        (phase_accum.mean_quadrant_x8+4)>>3;

    ema_quad&=3;

    //-------------------------------------------------
    // Case 1
    // Two perfect observations
    //-------------------------------------------------

    if(best_hits >= 2)
    {
        best=ema_quad;

        goto LOCK;
    }

    //-------------------------------------------------
    // Case 2
    // One observation only
    //-------------------------------------------------

    if(total_hits == 1)
    {
        best=ema_quad;

        goto LOCK;
    }

    //-------------------------------------------------
    // Case 3
    // Two conflicting observations
    //-------------------------------------------------

    if(total_hits >= 2 && best_hits < 2)
    {
        return false;
    }

    //-------------------------------------------------
    // Timeout
    //-------------------------------------------------

    if(phase_accum.fire_count>=PHASE_WINDOW_MAX)
    {
        best=ema_quad;

        goto LOCK;
    }

    return false;

LOCK:


    defibrillator(best);

    return true;
}

//one time ICD (Implantable Cardioverter-Defibrillator)
//Might aswell keep the analogy accurate
inline void watchdog(){
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
                        // deus ex machina defib for "Pulseless Electrical Activity"
                        phase_offset ^= 1;
                        phase_flip_used = true;
                        stall_watch_armed = false;
                        flag_phase_flip_triggered = true;

                    } else {
                        // genuinely catching
                        stall_watch_armed = false;
                        was_running = true;
                    }
                }
            }
        }
    }
}

void setup() {
    //Pin 4 (1 & 4) used with OPTOCOUPLERS to switch the base of 2N2222 transistors in 
    //Emmitter Follower/common collector Orientation, 10kpulldown on emitter output to prevent floating 
    //all with its on 5V power supply tap(cig light adapter)
    //to meet current flow and voltage requirements for the K20/K24 smart coil IGBTs(on/off switches)
    //since optocouplers invert the signal, this code actually drives the signal HIGH to keep the Optocoupler output LOW
    //and LOW means the 2N2222 transistors are OFF and the COIL is OFF
    PORTD |= (1 << COIL_A);
    DDRD |= (1 << COIL_A);
    //Pin 5 (2 & 3)
    PORTD |= (1 << COIL_B);
    DDRD |= (1 << COIL_B);
    //Pin 2 (5V 10K pull-up on ECU IGN-OUT PIN A21(OBD1)) 
    //This was designed for a P28, Stock P5M has a different OBD2A pinout 
    //if P5M and no OBD1 conversion harness to mess around with, get it from the ICM/Dist yellow-green wire
    DDRD &= ~(1 << IGN_IN);
    //Pin 3, 5V 470ohm pull-up on GM's 4Pin HEI Module C pin, 12V to B, 
    //CYP sensor Positive TAP to W and CYP sensor Neg Tap to G
    //And ground properly, for the love of god, ground properly or go crazy with unreliable signals
    //Tap is neccesary or engine will throw a code
    DDRD &= ~(1 << CYP_IN);

    //Tach just needs a 12V RPM Pulse to work
    //so either a second HEI Module/Variable Reluctance conditioner or another OPTO/Trans
    //depending on whether its a based on the A21 sig or the CYP pulse
    //also add a hardware redundancy to keep coils unpowered when they should be or they might explode
    //if for example code is being updated and the key is turned to the off position A21 will send multiple HIGH pulses
    //and end the chain on a LOW(activate coil) probably meant to clear cylinders of fuel when turned off. 
    //WHEN turned on it sends multiple HIGHS and ends on HIGH
    //ask me how I verifed my wasted spark wire harness was working perfectly before the Phase Resolver worked(I hope)

    EICRA &= ~((1 << ISC11) | (1 << ISC10) | (1 << ISC01) | (1 << ISC00));
    EICRA |= (1 << ISC00);//any
    EICRA |= (1 << ISC10);//any
    EIMSK |= (1 << INT0) | (1 << INT1);

    Serial.begin(9600);
    Serial.println(F("\n=== H22A Wasted-Spark Ignition + Phase Resolver (HOUSE MD Edition) ==="));
    Serial.println(F("INT0/A21 = Master Clock(SINUS RYTHM) | INT1/CYP (Phase Indicator)"));
    Serial.println(F("Startup guard on INT0 | Windowed phase-accum lock | A21-only stall watchdog\n"));

    sei();
}

// ============= INT0: A21 MASTER CLOCK =============
// Startup guard lives here now (INT0 is the noise source at power-on).
// Coil drive is gated on ignition_enabled; timing/sequence tracking is NOT,
// so CYP always has a real last_fire_time 
ISR(INT0_vect) {
    if (sync_state == STARTUP_GUARD) {
        //ign_isr_count++;
        unsigned long now = micros();
        if (boot_guard_start == 0) boot_guard_start = now;
        if (now - boot_guard_start < (unsigned long)STARTUP_GUARD_MS * 1000UL) {
            return;  // still settling, discard everything
        }
        sync_state = PHASE_ACCUM;
        flag_sync_state_changed = true;
        reset_phase_accum();
        // fall through: this edge is processed normally below,
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

    if (state == LOW) {
        log_event(EVT_IGN_LOW);
        ign_state_low_count++;
        a21_last_low_time = now;

        if (ignition_enabled) {
            if (current_phase == 0) PORTD &= ~(1 << COIL_A);
            else                 PORTD &= ~(1 << COIL_B);
        }
    } else {
        log_event(EVT_IGN_HIGH);
        ign_state_high_count++;
        unsigned long dwell_dur = now - a21_last_low_time;

        //avoids burnt coils
        if (current_phase == 0) PORTD |= (1 << COIL_A);
        else                 PORTD |= (1 << COIL_B);


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
        

        // ===== PHASE_ACCUM =====
        if (sync_state == PHASE_ACCUM)
        {
            if(processPhaseAccum())
            return;
            quad_index = (quad_index + 1) & 3;
            if (quad_index ==0){
                memset(quadrant_seen, 0, sizeof(quadrant_seen));
            }

            
        }

        watchdog();
        
        pulse_phase ^= 1; //no touchy
        current_phase = pulse_phase ^ phase_offset;//only place it can be changed
    }
}

ISR(INT1_vect)
{
    if (sync_state == STARTUP_GUARD) return;

    unsigned long now = micros();

    bool state = (PIND & (1 << CYP_IN));

    // edge filter (prevents duplicate ISR processing)
    if (state == last_cyp_state) return;
    last_cyp_state = state;

    cyp_isr_count++;

    // =========================
    // FALLING EDGE: start CYP feature window
    // =========================
    if (state == LOW) {
        cyp_fall_time = now;
        cyp_armed = true;
        return;
    }

    // =========================
    // RISING EDGE: validate CYP feature
    // =========================
    if (state == HIGH && cyp_armed)
    {
        unsigned long width = now - cyp_fall_time;
        unsigned long spacing = now - last_valid_cyp_time;

        cyp_armed = false;

        // ---- noise rejection ----
        if (width < 80) return;         // chatter / EMI
        if (spacing < 30000) return;    // double trigger rejection

        last_valid_cyp_time = now;
        cyp_debounce_pass_count++;

        // =========================
        // PHASE ACCUMULATION MODE
        // =========================
        if (sync_state == PHASE_ACCUM)
        {
            

            if (!quadrant_seen[quad_index]) {
                quadrant_seen[quad_index] = true;
                phase_accum.quadrant_hits[quad_index]++;
             }

            //-------------------------------------------------
            // EMA
            //-------------------------------------------------

           if( phase_accum.mean_quadrant_x8 < 0)
            {
                phase_accum.mean_quadrant_x8 = quad_index<<3;
            }
            else
            {
                phase_accum.mean_quadrant_x8 +=
                    ((quad_index<<3)-phase_accum.mean_quadrant_x8)>>2;
            }

            
            return;
        }

        // =========================
        // LOCKED MODE (learning only)
        // =========================
        if (sync_state == LOCKED)
        {
            const bool expected = (current_phase == 0);

            if (!expected) cyp_invalid_count++;
            else log_event(EVT_CYP);
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

    noInterrupts();
    unsigned long fire_snapshot = last_fire_time;
    interrupts();

    bool engine_stalled = (micros() - fire_snapshot) > 400000UL;


    if (was_running && engine_stalled) {
        was_running = false;
        rpm = 0;
        Serial.print(F("[SAVE] A21 edges: ")); Serial.print(ign_state_low_count + ign_state_high_count);
        Serial.print(F(" | CYP invalid: ")); Serial.print(cyp_invalid_count);
        Serial.println(F(" fires"));
    }

    // ===== STALL DETECTION =====
    // Uses the already-snapshotted fire_snapshot (atomic read done above).
    // Completely independent of rpm EMA — raw elapsed time only.
    bool prev_stall = stall_detected;
    stall_detected = (micros() - fire_snapshot) > STALL_THRESHOLD_US;

    // Transition: running → stalled
    if (stall_detected && !prev_stall) {
        rpm = 0;  // safe to zero here since we've confirmed no fires
        Serial.println(F("[STALL] Engine stopped"));

        #ifdef STALL_COIL_SAFETY
        // Pull both coils HIGH (off) via direct port write.
        // Bypasses ignition_enabled logic entirely — hardware safety.
        // Correct regardless of optocoupler inversion because HIGH = coil off
        // in your circuit (verified by setup() init state).
        PORTD |= (1 << COIL_A) | (1 << COIL_B);
        ignition_enabled = false;
        #endif
    }

    // Transition: stalled → running
    if (!stall_detected && prev_stall) {
        Serial.println(F("[STALL] Engine cranking detected"));
        // ignition_enabled restored by PHASE_ACCUM → LOCKED path naturally,
        // no manual re-enable needed here.
    }

    static unsigned long last_print = 0;
    if (millis() - last_print > 500) {
        Serial.print(F("[STATUS] "));
        if (sync_state == LOCKED) Serial.print(F("LOCKED"));
        else if (sync_state == PHASE_ACCUM) Serial.print(F("PHASE_ACCUM"));
        else Serial.print(F("STARTUP"));

        Serial.print(F(" | IGN: ")); Serial.print(ign_state_high_count);
        Serial.print(F(" | RPM: ")); Serial.print(rpm);
        Serial.print(F(" | CYP Inv: ")); Serial.print(cyp_invalid_count);
        Serial.print(F(" | CYP isr/deb: ")); Serial.print(cyp_isr_count);
        Serial.print(F("/")); Serial.print(cyp_debounce_pass_count);
        Serial.print(F(" | PAIR: ")); Serial.print((current_phase == 0) ? F("A") : F("B"));
        if(sync_state != LOCKED){
        //Serial.print(F(" | Hits 0: ")); Serial.print(phase_accum.hits[0]);
        //Serial.print(F(" | Hits 1: ")); Serial.print(phase_accum.hits[1]);
        }
        Serial.println();

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

            Serial.println(F("[MASTER - A21/INT0]"));
            Serial.print(F("  DWELL events: ")); Serial.print(ign_state_low_count);
            Serial.print(F("  FIRE events: ")); Serial.print(ign_state_high_count);
            Serial.print(F("  Current RPM: ")); Serial.println(rpm);
            Serial.println();

            Serial.println(F("[VALIDATOR - CYP/INT1 Learning]"));
            Serial.print(F("  Invalid CYP pulses: ")); Serial.println(cyp_invalid_count);

            Serial.println(F("[PHASE WINDOW FILTER]"));
            Serial.print(F("  Phase flip used this session: ")); Serial.println(phase_flip_used ? F("YES") : F("no"));
            Serial.println();

            Serial.println(F("[STATE]"));
            Serial.print(F("  Engine stall: ")); Serial.println(stall_detected ? F("YES") : F("no"));
            Serial.print(F("  Sync state: "));
            if (sync_state == LOCKED) Serial.println(F("LOCKED"));
            else if (sync_state == PHASE_ACCUM) Serial.println(F("PHASE_ACCUM"));
            else Serial.println(F("STARTUP_GUARD"));
            Serial.print(F("  Sequence: ")); Serial.println(current_phase);
            Serial.print(F("  Active pair: ")); Serial.println((current_phase == 0) ? F("A (cyls 1&4)") : F("B (cyls 2&3)"));
            Serial.println(F("========== END SNAPSHOT ===========\n"));
        }

        else if (cmd == 'r') {
            //memset(&a21_profile, 0, sizeof(a21_profile));
            //memset(&cyp_profile, 0, sizeof(cyp_profile));
            ign_state_high_count = ign_state_low_count = 0;
            ign_debounce_pass_count = cyp_debounce_pass_count = 0;
            ign_isr_count = cyp_isr_count = 0;
            cyp_invalid_count = 0;
            last_fire_time = 0;
            fire_to_fire_interval = 0;
            boot_guard_start = 0;
            runtime_min = 0;
            was_running = false;
            stall_detected = false;


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

            Serial.println(F("[RESET] All profiles cleared\n"));
        }
    }
}
