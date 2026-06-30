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
// Derive pair from sequence: 0,2 → A; 1,3 → B
void log_event(uint8_t type, uint8_t seq_snapshot) {
    bool pair_for_seq = (seq_snapshot == 0 || seq_snapshot == 2);
    history[history_head] = {type, micros(), pair_for_seq, seq_snapshot};
    history_head = (history_head + 1) % HISTORY_SIZE;
    if (history_head == 0) history_full = true;
}

// Pin assignments
const int IGN_IN = 2;    // 10k pull-up
const int CYP_IN = 3;    // crankshaft sync
const int COIL_A = 4;    // cylinders 1 & 4
const int COIL_B = 5;    // cylinders 2 & 3

const bool DEBUG_MODE = false;

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

void setup() {
    // Configure pins
    DDRD |= (1 << COIL_A);                 // Output
    PORTD |= (1 << COIL_A);                // Idle LOW (PC817 on, coil off)
    
    DDRD |= (1 << COIL_B);                 // Output
    PORTD |= (1 << COIL_B);                // Idle LOW
    
    DDRD &= ~(1 << IGN_IN);                // Input
    DDRD &= ~(1 << CYP_IN);                // Input
    PORTD |= (1 << CYP_IN);                // Pull-up

    // Configure interrupts
    EICRA &= ~((1 << ISC11) | (1 << ISC10) | (1 << ISC01) | (1 << ISC00));
    EICRA |= (1 << ISC00);                 // INT0: any change
    EICRA |= (1 << ISC11);                 // INT1: falling edge
    
    EIMSK |= (1 << INT0) | (1 << INT1);

    Serial.begin(9600);
    Serial.println("Ignition controller starting...");
    
    sei();  // Enable interrupts globally
}

// CYP (crankshaft position) sync - fires once per engine rotation
ISR(INT1_vect) {
    cyp_isr_count++;
    
    unsigned long now = micros();
    unsigned long elapsed = now - last_cyp_time;
    last_cyp_time = now;

    // Debounce: ignore pulses < 7.5ms apart
    if (elapsed < 7500) return;
    cyp_debounce_pass_count++;
    
    // Capture sequence BEFORE modifications
    uint8_t seq_snapshot = ign_sequence_count;
    log_event(EVT_CYP, seq_snapshot);
    
    sync_established = true;

    // CRITICAL SECTION: update sync state atomically
    cli();
    
    if (!dwell_active) {
        // Safe window - no coil currently energizing
        // Reset sequence to 0 immediately (next fire will be pair A)
        ign_sequence_count = 0;
        pending_sync = false;
    } else {
        // Mid-dwell - defer sync until after fire event
        pending_sync = true;
    }
    
    sei();

    // Calculate RPM (safe from overflow for reasonable engine speeds)
    if (elapsed > 500000) {
        // No pulse for > 500ms = stalled
        rpm = 0;
        is_running = false;
    } else if (elapsed < 1000) {
        // Noise or stuck sensor - ignore this pulse
        return;
    } else {
        is_running = true;
        // rpm = (60 * 1000000) / elapsed, but we only get one pulse per rotation
        // so multiply by 2 for 4-cyl (two pairs fire per rotation)
        rpm = 120000000UL / elapsed;
        if (rpm > 10000) rpm = 10000;  // Cap at reasonable max for performance vehicles
    }
}

// IGN signal - toggles between dwell (LOW) and fire (HIGH) for timing
ISR(INT0_vect) {
    // Gate: require sync before firing
    if (!sync_established) return;
    
    ign_isr_count++;

    unsigned long now = micros();
    
    // Debounce: ignore edges < 2.0ms apart
    // At 8000 RPM, minimum edge spacing is 3.75ms, leaving 1.75ms safety margin
    if (now - last_ign_time < 2000) return;
    last_ign_time = now;
    ign_debounce_pass_count++;

    // Read pin state
    bool state = (PIND & (1 << IGN_IN)) ? HIGH : LOW;
    last_ign_state = state;

    // Capture sequence count BEFORE any state changes for accurate logging
    uint8_t seq_snapshot = ign_sequence_count;

    if (state == LOW) {
        // ===== DWELL PHASE =====
        // Start charging coil
        log_event(EVT_IGN_LOW, seq_snapshot);
        ign_state_low_count++;
        dwell_active = true;
        
        // Enable coil (via PC817 optoisolator: LOW on Arduino pin = HIGH on coil)
        if (active_pair_A) {
            PORTD &= ~(1 << COIL_A);  // LOW → coil HIGH
        } else {
            PORTD &= ~(1 << COIL_B);
        }
    } else {
        // ===== FIRE PHASE =====
        // Discharge coil to spark plugs
        log_event(EVT_IGN_HIGH, seq_snapshot);
        ign_state_high_count++;
        
        // Disable coil (HIGH on Arduino pin = LOW on coil)
        if (active_pair_A) {
            PORTD |= (1 << COIL_A);   // HIGH → coil LOW
        } else {
            PORTD |= (1 << COIL_B);
        }
        dwell_active = false;

        // ===== ADVANCE STATE MACHINE =====
        // Update sequence counter and pair selection atomically.
        // If sync arrived during dwell (pending_sync=true), force to 0.
        // Otherwise, increment normally.
        
        cli();  // Disable interrupts for critical section
        
        if (pending_sync && !DEBUG_MODE) {
            // Sync pulse arrived during dwell - force sequence to 0
            // This is a hard reset, not an increment
            ign_sequence_count = 0;
            pending_sync = false;
        } else {
            // Normal progression - increment and wrap
            ign_sequence_count++;
            if (ign_sequence_count >= 4) ign_sequence_count = 0;
        }
        
        // Derive pair from sequence count (single source of truth)
        // Sequence 0, 2 → Pair A (cylinders 1&4)
        // Sequence 1, 3 → Pair B (cylinders 2&3)
        active_pair_A = (ign_sequence_count == 0 || ign_sequence_count == 2);
        
        sei();  // Re-enable interrupts
        
        // Sequence mapping (for reference):
        // 0 = Cyl 1 fires (Pair A)
        // 1 = Cyl 3 fires (Pair B)
        // 2 = Cyl 4 fires (Pair A)
        // 3 = Cyl 2 fires (Pair B)
    }
}

void loop() {
    static bool last_sync = false;
    
    // Report sync status changes
    if (sync_established != last_sync) {
        last_sync = sync_established;
        Serial.println(sync_established ? "SYNC: ON" : "SYNC: LOST");
    }

    // Report startup mode
    if (DEBUG_MODE && !sync_established) {
        static bool debug_printed = false;
        if (!debug_printed) {
            Serial.println("DEBUG_MODE: Running without CYP sync (blind pair alternation)");
            debug_printed = true;
        }
    }

    // Print status every 500ms
    static unsigned long last_print = 0;
    if (millis() - last_print > 500) {
        Serial.print("IGN_ISR: "); Serial.print(ign_isr_count);
        Serial.print(" | DB: "); Serial.print(ign_debounce_pass_count);
        Serial.print(" | LOW: "); Serial.print(ign_state_low_count);
        Serial.print(" | HIGH: "); Serial.print(ign_state_high_count);
        Serial.print(" | CYP_ISR: "); Serial.print(cyp_isr_count);
        Serial.print(" | CYP_DB: "); Serial.print(cyp_debounce_pass_count);
        Serial.print(" | RPM: "); Serial.print(rpm);
        Serial.print(" | PAIR: "); Serial.println(active_pair_A ? "A" : "B");
        
        last_print = millis();
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
