// Event types
#define EVT_IGN_LOW  0
#define EVT_IGN_HIGH 1
#define EVT_CYP     2
#define EVT_SYNC     3

// Circular buffer
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
volatile uint8_t ign_sequence_count = 0; // Tracks 0, 1, 2, 3 across the 4 pulses

// Call this from ISRs only - fast
void log_event(uint8_t type, bool pair) {
    history[history_head] = {type, micros(), pair, ign_sequence_count};
    history_head = (history_head + 1) % HISTORY_SIZE;
    if (history_head == 0) history_full = true;
}

//first rail
const int IGN_IN = 2; //yellow 10k pull-up
const int CYP_IN = 3; //orange 
const int COIL_A = 4; // cyl 2 and 3 green - red
const int COIL_B = 5; // cyl 1 and 4 white - black
// ground

const bool DEBUG_MODE = false; // bypass cyp effect sync, alternate pairs blindly

volatile bool sync_established = false;
volatile bool active_pair_A = true; // change to true by default
volatile bool pending_sync = false;
volatile bool dwell_active = false;
volatile unsigned long last_cyp_time = 0;
volatile unsigned long rpm = 0;
volatile bool is_running = false;
volatile int cyp_pulse_count = 0;

//FLAGS for Monitoring
volatile unsigned long ign_isr_count = 0;
volatile unsigned long ign_debounce_pass_count = 0;
volatile unsigned long ign_state_low_count = 0;
volatile unsigned long ign_state_high_count = 0;
volatile bool last_ign_state = false;

volatile unsigned long cyp_isr_count = 0;
volatile unsigned long cyp_debounce_pass_count = 0;

void setup() {

    //Port DDigital Pins 0 to 7
    //Port BDigital Pins 8 to 13
    //Port CAnalog Pins A0 to A5

    //DDRD |=(1<<portnum); //output
    //DDRD &= ~(1<<portnum); //input on Port D
    // Sets Digital Pin 13 (Port B, Bit 5) High
    //PORTB |= (1 << 5);
    // Sets Digital Pin 13 (Port B, Bit 5) LOW
    //PORTB &= ~(1 << 5);

    //********** idle state LOW for k20 coils *************/
    DDRD |=(1<<COIL_A); //pinMode(COIL_A, OUTPUT);
    PORTD |= (1 << COIL_A); // Arduino HIGH → PC817 on → coil LOW (idle)
    //digitalWrite(COIL_A, LOW);

    DDRD |=(1<<COIL_B); //pinMode(COIL_B, OUTPUT);
    PORTD |= (1 << COIL_B);//digitalWrite(COIL_B, LOW);
    /************cyp Effect Sensor and IGN ECU A21 signal*/
    DDRD &= ~(1<<IGN_IN); //pinMode(IGN_IN, INPUT);

    DDRD &= ~(1<<CYP_IN);//pinMode(CYP_IN, INPUT);
    PORTD |= (1<<CYP_IN); //pinMode(CYP_IN, INPUT_PULLUP);
    //****************attach interupts ***********************///
    //**old code still works but**///
    //attachInterrupt(digitalPinToInterrupt(IGN_IN), ign_ISR, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(CYP_IN), cyp_ISR, FALLING);


    // 1. Clear all control bits for INT0 and INT1 safely
    EICRA &= ~((1 << ISC11) | (1 << ISC10) | (1 << ISC01) | (1 << ISC00));

    // 2. IGN_IN (Pin 2) -> Configure INT0 for ANY LOGIC CHANGE (CHANGE)
    // Mode table for CHANGE: ISC01 = 0, ISC00 = 1
    // (ISC01 is already cleared from step 1, so we only need to set ISC00)
    EICRA |= (1 << ISC00);

    // 3. CYP_IN (Pin 3) -> Configure INT1 for Falling EDGE (Falling)
    // Mode table for Falling: ISC11 = 1, ISC10 = 0
    EICRA |= (1 << ISC11);
    EICRA &= ~(1 << ISC10);

    // Enable External Interrupt 0
    // EIMSK (External Interrupt Mask Register)
    EIMSK |= (1 << INT0);

    // Enable External Interrupt 1
    // EIMSK (External Interrupt Mask Register)
    EIMSK |= (1 << INT1);

    //Enable External Interrupts 0 and 1
    //EIMSK |= (1 << INT0) | (1 << INT1);


    Serial.begin(9600);
    Serial.println("Start Up complete");
    sei();
}

// This macro creates the hardware-level ISR vector for INT1 (Pin 3)
ISR(INT1_vect) { //void cyp_ISR() {
    cyp_isr_count++;
    unsigned long now = micros();
    unsigned long elapsed = now - last_cyp_time;
    last_cyp_time = now;

    if (elapsed < 7500) return;
    cyp_debounce_pass_count++;
    
    log_event(EVT_CYP, active_pair_A);
    sync_established = true;

    cli();// PROTECT with cli/sei
    if (!dwell_active) {
        // safe window - reset sequence, single source of truth
        ign_sequence_count = 3; // so next increment → 0 → pair A
        // do NOT set active_pair_A here, let INT0 derive it from counter
        pending_sync = false;
    } else {
        pending_sync = true; // defer to after current fire event
    }
    sei();

    if (elapsed > 500000) {
    rpm = 0;
    is_running = false;
    } else if (elapsed < 1000) {
        // Noise or stuck sensor - ignore
        return;
    } else {
        is_running = true;
        rpm = 120000000UL / elapsed;
        if (rpm > 9000) rpm = 9000;  // Cap at reasonable max
    }
}

volatile unsigned long last_ign_time = 0;
    // This macro creates the hardware-level ISR vector for INT0 (Pin 2)
ISR(INT0_vect) { //void ign_ISR() {
    //Serial.println("Ignition ISR Triggered");
    if (!sync_established) return; // absolute guard, no exceptions
    ign_isr_count++;

    unsigned long now = micros();
    
    if (now - last_ign_time < 2000) return; // 3.3ms, shortest possible is 3.75
    last_ign_time = now;
    ign_debounce_pass_count++;
    //Serial.print("Debounce success: "); Serial.println(digitalRead(IGN_IN));

    //bool state = digitalRead(IGN_IN);

    // Checks if Digital Pin 2 (Port D, Bit 2) is HIGH
    bool state = (PIND & (1 << IGN_IN)) ? HIGH : LOW;
    last_ign_state = state;
    
    //start charging/dwell
    if (state == LOW) {
        // dwell - coil needs HIGH - Arduino outputs LOW through PC817
        log_event(EVT_IGN_LOW, active_pair_A);
        ign_state_low_count++;
        dwell_active = true;
        if (active_pair_A) {
            PORTD &= ~(1 << COIL_A); // LOW → PC817 off → coil HIGH
        } else {
            PORTD &= ~(1 << COIL_B);
        }
    } else {
        // ==========================================
        // 2. FIRE EVENT (Discharge Coils)
        // ==========================================
        log_event(EVT_IGN_HIGH, active_pair_A);
        ign_state_high_count++;
        
        if (active_pair_A) {
            PORTD |= (1 << COIL_A); // Fire Pair A
        } else {
            PORTD |= (1 << COIL_B); // Fire Pair B
        }


        // ==========================================
        // 3. STATE MACHINE ENGINE (For the NEXT event)
        // ==========================================
        
        // If a sync pulse arrived during this engine cycle, force our 
        // pointer to index 0 so that the engine tracks perfectly.
        // apply pending sync reset if deferred
        cli(); // Disable interrupts
        dwell_active = false;
        if (pending_sync && !DEBUG_MODE) {
            ign_sequence_count = 0;
            pending_sync = false;
        } else {
            ign_sequence_count++;
            if (ign_sequence_count >= 4) ign_sequence_count = 0;
        }
        
        // single source of truth for pair selection
        active_pair_A = (ign_sequence_count == 0 || ign_sequence_count == 2);
        sei();  // Re-enable
        // Map the sequence count directly to the physical pairs
        // Sequence Mapping:
        // 0 = Cyl 1 (Pair A)
        // 1 = Cyl 3 (Pair B)
        // 2 = Cyl 4 (Pair A)
        // 3 = Cyl 2 (Pair B)

    }
}

void loop() {
    static bool last_sync = false;

    if (sync_established != last_sync) {
        last_sync = sync_established;
        Serial.println(sync_established ? "SYNC ESTABLISHED" : "SYNC LOST");
    }

    if (DEBUG_MODE && !sync_established) {
        static bool debug_printed = false;
    
        if (!debug_printed) {
            Serial.println("DEBUG MODE - running without cyp sync");
            debug_printed = true;
        }
    }

    static unsigned long last_print = 0;
    if (millis() - last_print > 500) {
        Serial.print("IGN_ISR:"); Serial.print(ign_isr_count);
        Serial.print(" IGN_DB:"); Serial.print(ign_debounce_pass_count);
        Serial.print(" LOW:"); Serial.print(ign_state_low_count);
        Serial.print(" HIGH:"); Serial.print(ign_state_high_count);
        Serial.print(" CYP_ISR:"); Serial.print(cyp_isr_count);
        Serial.print(" CYP_DB:"); Serial.print(cyp_debounce_pass_count);
        Serial.print(" SYNC:"); Serial.print(sync_established);
        Serial.print(" RPM:"); Serial.print(rpm);
        Serial.print(" PAIR:"); Serial.println(active_pair_A ? "A" : "B");
        /**
        Serial.print("IGN: ");
        Serial.print(digitalRead(2));
        Serial.print(" cyp: "); //
        / Read the pin state (It will read HIGH when open, LOW when pressed)
        if (!(PIND & (1 << 2))) {
        // Button is pressed (connected to ground)
        }
        Serial.print(digitalRead(3  )? "Idle" : "TDC");
        Serial.print(" RPM: ");
        Serial.print(rpm);
        Serial.print(" SYNC: ");
        Serial.print(sync_established);
        Serial.print(" PAIR: ");
        Serial.println(active_pair_A ? "1&4" : "2&3");
        */
        last_print = millis();
    }
    // type 'h' in serial monitor to dump history
    if (Serial.available() && Serial.read() == 'h') {
        Serial.println("--- EVENT HISTORY ---");
        uint8_t start = history_full ? history_head : 0;
        uint8_t count = history_full ? HISTORY_SIZE : history_head;
        
        unsigned long first_time = history[start].timestamp;
        
        for (uint8_t i = 0; i < count; i++) {
            uint8_t idx = (start + i) % HISTORY_SIZE;
            volatile Event& e = history[idx];
            
            // time relative to first event in microseconds
            unsigned long rel_time = e.timestamp - first_time;
            
            Serial.print(rel_time);
            Serial.print("us ");
            
            switch(e.type) {
                case EVT_IGN_LOW:  Serial.print("DWELL  "); break;
                case EVT_IGN_HIGH: Serial.print("FIRE   "); break;
                case EVT_CYP:     Serial.print("cyp   "); break;
                case EVT_SYNC:     Serial.print("SYNC   "); break;
            }
            Serial.println(e.pair_a ? "PAIR_A" : "PAIR_B");
        }
        Serial.println("--- END ---");
    }
}