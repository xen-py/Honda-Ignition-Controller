// Event types
#define EVT_IGN_LOW  0
#define EVT_IGN_HIGH 1
#define EVT_CYP      2
#define EVT_SYNC     3

// Circular buffer for event history
#define HISTORY_SIZE 32
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

// Call this from ISRs only
// Derive pair from sequence: 0 → A; 1 → B
void log_event(uint8_t type, uint8_t seq_snapshot) {
    bool pair_for_seq = (seq_snapshot == 0);  // 0=A, 1=B
    history[history_head] = {type, micros(), pair_for_seq, seq_snapshot};
    history_head = (history_head + 1) % HISTORY_SIZE;
    if (history_head == 0) history_full = true;
}

// Pin assignments
const int IGN_IN = 2;    // A21 from P28 ECU
const int CYP_IN = 3;    // B11/B12 MVR sensor
const int COIL_A = 4;    // cylinders 1 & 4 (wasted-spark pair)
const int COIL_B = 5;    // cylinders 2 & 3 (wasted-spark pair)

const bool DEBUG_MODE = false;

// Startup sync confirmation
#define MIN_CYP_FOR_SYNC 3
#define SEQ_MAX 2  // Wasted-spark: only 2 fire events per crankshaft rotation (A, B)
volatile uint8_t cyp_sync_count = 0;

// Hysteresis: lock out CYP resets for 500ms after last one
#define CYP_RESET_LOCKOUT 500000  // microseconds
volatile unsigned long last_cyp_reset_time = 0;

// State machine
volatile bool sync_established = false;
volatile bool active_pair_A = true;
volatile bool pending_sync = false;
volatile bool dwell_active = false;
volatile unsigned long last_cyp_time = 0;
volatile unsigned long rpm = 0;
volatile bool is_running = false;

// Debounce timer - MUST be global and volatile
volatile unsigned long last_ign_time = 0;

// Monitoring counters
volatile unsigned long ign_isr_count = 0;
volatile unsigned long ign_debounce_pass_count = 0;
volatile unsigned long ign_state_low_count = 0;
volatile unsigned long ign_state_high_count = 0;
volatile bool last_ign_state = false;

volatile unsigned long cyp_isr_count = 0;
volatile unsigned long cyp_debounce_pass_count = 0;
volatile unsigned long cyp_reset_lockout_count = 0;

void setup() {
    // Configure pins
    DDRD |= (1 << COIL_A);
    PORTD |= (1 << COIL_A);
    
    DDRD |= (1 << COIL_B);
    PORTD |= (1 << COIL_B);
    
    DDRD &= ~(1 << IGN_IN);
    DDRD &= ~(1 << CYP_IN);
    PORTD |= (1 << CYP_IN);

    // Configure interrupts
    EICRA &= ~((1 << ISC11) | (1 << ISC10) | (1 << ISC01) | (1 << ISC00));
    EICRA |= (1 << ISC00);   // INT0: any change
    EICRA |= (1 << ISC11);   // INT1: falling edge
    
    EIMSK |= (1 << INT0) | (1 << INT1);

    Serial.begin(9600);
    Serial.println("\n=== H22A Wasted-Spark Ignition Controller ===");
    Serial.println("Config: 2 coil pairs (Pair A: cyls 1&4, Pair B: cyls 2&3)");
    Serial.println("Mode: Wasted-spark (2 FIRE events per CYP rotation)");
    Serial.println("Waiting for CYP sync confirmation...");
    
    sei();
}

// CYP (crankshaft position) sync - MVR sensor
ISR(INT1_vect) {
    cyp_isr_count++;
    
    unsigned long now = micros();
    unsigned long elapsed = now - last_cyp_time;
    

    // Debounce: 15ms for MVR noise immunity
    if (elapsed < 15000) return;
    cyp_debounce_pass_count++;
    last_cyp_time = now;
    // Hysteresis gate: only allow resets every 500ms
    unsigned long time_since_last_reset = now - last_cyp_reset_time;
    if (time_since_last_reset < CYP_RESET_LOCKOUT) {
        cyp_reset_lockout_count++;
        if (cyp_sync_count < MIN_CYP_FOR_SYNC) {
            cyp_sync_count++;
            if (cyp_sync_count == MIN_CYP_FOR_SYNC) {
                sync_established = true;
                Serial.println("SYNC: Confirmed");
            }
        }
        return;
    }

    // Capture sequence BEFORE modifications
    uint8_t seq_snapshot = ign_sequence_count;
    log_event(EVT_CYP, seq_snapshot);

    // Startup sync confirmation
    if (cyp_sync_count < MIN_CYP_FOR_SYNC) {
        cyp_sync_count++;
        if (cyp_sync_count == MIN_CYP_FOR_SYNC) {
            sync_established = true;
            Serial.print("SYNC: Confirmed after ");
            Serial.print(MIN_CYP_FOR_SYNC);
            Serial.println(" valid CYP pulses");
        }
    } else {
        sync_established = true;
    }

    // CRITICAL SECTION: update sync state atomically
    cli();
    
    if (!dwell_active) {
        // Safe window - reset sequence to 0 (Pair A fires next)
        ign_sequence_count = 0;
        pending_sync = false;
        last_cyp_reset_time = now;
    } else {
        // Mid-dwell - defer sync
        pending_sync = true;
    }
    
    sei();

    // Calculate RPM
    if (elapsed > 500000) {
        rpm = 0;
        is_running = false;
    } else if (elapsed < 1000) {
        return;
    } else {
        is_running = true;
        rpm = 120000000UL / elapsed;
        if (rpm > 10000) rpm = 10000;
    }
}
volatile bool coil_firing = false;
volatile unsigned long fire_time = 0;

// IGN signal - P28 ECU A21 output
ISR(INT0_vect) {
    // Gate: require startup sync confirmation
    if (!sync_established || cyp_sync_count < MIN_CYP_FOR_SYNC) return;
    // In INT0 at top, add gate:
    if (coil_firing) return;  // Ignore IGN edges during spark transient
    
    ign_isr_count++;

    unsigned long now = micros();
    
    // Debounce: 2ms
    if (now - last_ign_time < 2000) return;
    last_ign_time = now;
    ign_debounce_pass_count++;

    // Read pin state
    bool state = (PIND & (1 << IGN_IN)) ? HIGH : LOW;
    last_ign_state = state;

    // Capture sequence count BEFORE modifications
    uint8_t seq_snapshot = ign_sequence_count;

    if (state == LOW) {
        // ===== DWELL PHASE =====
        log_event(EVT_IGN_LOW, seq_snapshot);
        ign_state_low_count++;
        dwell_active = true;
        
        // Enable coil
        if (active_pair_A) {
            PORTD &= ~(1 << COIL_A);
        } else {
            PORTD &= ~(1 << COIL_B);
        }
    } else {
        // ===== FIRE PHASE =====
        log_event(EVT_IGN_HIGH, seq_snapshot);
        ign_state_high_count++;
        
        // Disable coil
        if (active_pair_A) {
            PORTD |= (1 << COIL_A);
        } else {
            PORTD |= (1 << COIL_B);
        }
        dwell_active = false;
        // START COIL FIRING GUARD
        coil_firing = true;
        fire_time = micros();  // ← Capture the time NOW

        // ===== ADVANCE STATE MACHINE =====
        // For wasted-spark H22A: only 2 fire events per CYP rotation (A, B)
        
        cli();
        
        if (pending_sync && !DEBUG_MODE) {
            ign_sequence_count = 0;  // Reset to Pair A
            pending_sync = false;
        } else {
            ign_sequence_count++;
            if (ign_sequence_count >= SEQ_MAX) ign_sequence_count = 0;  // Wrap at 2
        }
        
        // Derive pair: 0 → Pair A (cyls 1&4), 1 → Pair B (cyls 2&3)
        active_pair_A = (ign_sequence_count == 0);
        
        sei();
    }
}

void loop() {
    static bool last_sync = false;
    
    if (sync_established != last_sync) {
        last_sync = sync_established;
        Serial.println(sync_established ? "SYNC: ACTIVE" : "SYNC: LOST");
    }

    if (DEBUG_MODE && !sync_established) {
        static bool debug_printed = false;
        if (!debug_printed) {
            Serial.println("DEBUG_MODE: Blind pair alternation");
            debug_printed = true;
        }
    }

    // Print status every 500ms
    static unsigned long last_print = 0;
    if (millis() - last_print > 500) {
        Serial.print("IGN: "); Serial.print(ign_isr_count);
        Serial.print(" | DB: "); Serial.print(ign_debounce_pass_count);
        Serial.print(" | DWELL: "); Serial.print(ign_state_low_count);
        Serial.print(" | FIRE: "); Serial.print(ign_state_high_count);
        Serial.print(" | CYP: "); Serial.print(cyp_isr_count);
        Serial.print(" | CYP_DB: "); Serial.print(cyp_debounce_pass_count);
        Serial.print(" | LOCKOUT: "); Serial.print(cyp_reset_lockout_count);
        Serial.print(" | SYNC: "); Serial.print(cyp_sync_count);
        Serial.print("/"); Serial.print(MIN_CYP_FOR_SYNC);
        Serial.print(" | RPM: "); Serial.print(rpm);
        Serial.print(" | PAIR: "); Serial.println(active_pair_A ? "A" : "B");
        
        last_print = millis();
    }

    if (coil_firing) {
        unsigned long elapsed = micros() - fire_time;  // Handles wrap automatically
        if (elapsed > 5000) {
            coil_firing = false;
        }
    }

    // Dump event history on 'h' command
    if (Serial.available()) {
        char cmd = Serial.read();
        if (cmd == 'h') {
            Serial.println("\n--- EVENT HISTORY (last 32 events) ---");
            uint8_t start = history_full ? history_head : 0;
            uint8_t count = history_full ? HISTORY_SIZE : history_head;
            
            unsigned long first_time = (count > 0) ? history[start].timestamp : 0;
            
            for (uint8_t i = 0; i < count; i++) {
                uint8_t idx = (start + i) % HISTORY_SIZE;
                volatile Event& e = history[idx];
                
                unsigned long rel_time = e.timestamp - first_time;
                
                Serial.print(rel_time);
                Serial.print("us ");
                
                switch (e.type) {
                    case EVT_IGN_LOW:  Serial.print("DWELL  "); break;
                    case EVT_IGN_HIGH: Serial.print("FIRE   "); break;
                    case EVT_CYP:      Serial.print("CYP    "); break;
                    case EVT_SYNC:     Serial.print("SYNC   "); break;
                    default:           Serial.print("???    "); break;
                }
                
                Serial.print("SEQ:"); Serial.print(e.seq);
                Serial.print(" ");
                Serial.println(e.pair_a ? "PAIR_A" : "PAIR_B");
            }
            Serial.println("--- END ---\n");
        }
    }
}
