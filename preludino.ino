// Event types
#define EVT_IGN_LOW  0
#define EVT_IGN_HIGH 1
#define EVT_HALL     2
#define EVT_SYNC     3

// Circular buffer
#define HISTORY_SIZE 32
struct Event {
    uint8_t type;
    unsigned long timestamp;
    bool pair_a;
};

volatile Event history[HISTORY_SIZE];
volatile uint8_t history_head = 0;
volatile bool history_full = false;

// Call this from ISRs only - fast
void log_event(uint8_t type, bool pair) {
    history[history_head] = {type, micros(), pair};
    history_head = (history_head + 1) % HISTORY_SIZE;
    if (history_head == 0) history_full = true;
}

//first rail
const int IGN_IN = 2; //yellow 10k pull-up
const int HALL_IN = 3; //orange 
const int COIL_A = 5; // cyl 1 and 4 white - black
const int COIL_B = 4; // cyl 2 and 3 green - red
// ground

const bool DEBUG_MODE = false; // bypass hall effect sync, alternate pairs blindly

volatile bool sync_established = false;
volatile bool active_pair_A = true; // change to true by default
volatile bool pending_sync = false;
volatile bool dwell_active = false;
volatile unsigned long last_hall_time = 0;
volatile unsigned long rpm = 0;
volatile bool is_running = false;
volatile int hall_pulse_count = 0;

//FLAGS for Monitoring
volatile unsigned long ign_isr_count = 0;
volatile unsigned long ign_debounce_pass_count = 0;
volatile unsigned long ign_state_low_count = 0;
volatile unsigned long ign_state_high_count = 0;
volatile bool last_ign_state = false;

volatile unsigned long hall_isr_count = 0;
volatile unsigned long hall_debounce_pass_count = 0;

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
    PORTD &= ~(1 << COIL_A);//digitalWrite(COIL_A, LOW);

    DDRD |=(1<<COIL_B); //pinMode(COIL_B, OUTPUT);
    PORTD &= ~(1 << COIL_B);//digitalWrite(COIL_B, LOW);
    /************Hall Effect Sensor and IGN ECU A21 signal*/
    DDRD &= ~(1<<IGN_IN); //pinMode(IGN_IN, INPUT);

    DDRD &= ~(1<<HALL_IN);
    PORTD |= (1<<HALL_IN); //pinMode(HALL_IN, INPUT_PULLUP);
    //****************attach interupts ***********************///
    //**old code still works but**///
    //attachInterrupt(digitalPinToInterrupt(IGN_IN), ign_ISR, CHANGE);
    //attachInterrupt(digitalPinToInterrupt(HALL_IN), hall_ISR, FALLING);

    // IGN_IN(2)
    // Configure INT0 to trigger on a CHANGING edge
    // EICRA (External Interrupt Control Register A)
    // ISC01 = 1, ISC00 = 0 sets changing edge for INT0
    EICRA &= ~(1 << ISC01);
    EICRA |= (1 << ISC00);

    //HALL_IN (3)
    // Configure INT1 to trigger on a FALLING edge
    // ISC11 = 1, ISC10 = 0 sets falling edge for INT1
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
ISR(INT1_vect) { //void hall_ISR() {
    hall_isr_count++;

    unsigned long now = micros();
    unsigned long elapsed = now - last_hall_time;
    last_hall_time = now;

    if (elapsed < 7500) return; // ignore noise
    hall_debounce_pass_count++;
    log_event(EVT_HALL, active_pair_A);

    sync_established = true;
    if (dwell_active) pending_sync = true;

    // RPM is just informational now, no stall logic tied to sync
    if (elapsed > 500000) {
        rpm = 0;
        is_running = false;
    } else {
        is_running = true;
        rpm = 120000000UL / elapsed;
    }
}

    // This macro creates the hardware-level ISR vector for INT0 (Pin 2)
ISR(INT0_vect) { //void ign_ISR() {
    //Serial.println("Ignition ISR Triggered");
    if (!sync_established) return; // absolute guard, no exceptions
    ign_isr_count++;

    unsigned long now = micros();
    static unsigned long last_ign_time = 0;
    if (now - last_ign_time < 500) return; // 500us instead of 10ms
    last_ign_time = now;
    ign_debounce_pass_count++;
    //Serial.print("Debounce success: "); Serial.println(digitalRead(IGN_IN));

    //bool state = digitalRead(IGN_IN);

    // Checks if Digital Pin 2 (Port D, Bit 2) is HIGH
    bool state = (PIND & (1 << IGN_IN)) ? HIGH : LOW;
    last_ign_state = state;
    //start charging/dwell
    if (state == LOW) {
        log_event(EVT_IGN_LOW, active_pair_A);
        ign_state_low_count++;
        dwell_active = true;
        if (active_pair_A) {
            PORTD |= (1 << COIL_A);//digitalWrite(COIL_A, HIGH);
            //Serial.println("pair a dwell:");
        } else {
            PORTD |= (1 << COIL_B);//digitalWrite(COIL_B, HIGH);
            //Serial.println("pair b dwell: ");
        }
    } else {//*********fire/return to idle*********//
        log_event(EVT_IGN_HIGH, active_pair_A);
        ign_state_high_count++;
        if (active_pair_A) {
            PORTD &= ~(1<<COIL_A);//digitalWrite(COIL_A, LOW);
            //Serial.println("pair a fire: ");
        } else {
            PORTD &= ~(1<<COIL_B); //digitalWrite(COIL_B, LOW);
            //Serial.println("pair b fire: ");
        }
        dwell_active = false;
        if (!DEBUG_MODE && pending_sync) {
            active_pair_A = true;
            pending_sync = false;
        } else {
            active_pair_A = !active_pair_A;
        }
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
            Serial.println("DEBUG MODE - running without hall sync");
            debug_printed = true;
        }
    }

    static unsigned long last_print = 0;
    if (millis() - last_print > 500) {
        Serial.print("IGN_ISR:"); Serial.print(ign_isr_count);
        Serial.print(" IGN_DB:"); Serial.print(ign_debounce_pass_count);
        Serial.print(" LOW:"); Serial.print(ign_state_low_count);
        Serial.print(" HIGH:"); Serial.print(ign_state_high_count);
        Serial.print(" HALL_ISR:"); Serial.print(hall_isr_count);
        Serial.print(" HALL_DB:"); Serial.print(hall_debounce_pass_count);
        Serial.print(" SYNC:"); Serial.print(sync_established);
        Serial.print(" RPM:"); Serial.print(rpm);
        Serial.print(" PAIR:"); Serial.println(active_pair_A ? "A" : "B");
        /**
        Serial.print("IGN: ");
        Serial.print(digitalRead(2));
        Serial.print(" HALL: "); //
        / Read the pin state (It will read HIGH when open, LOW when pressed)
        if (!(PIND & (1 << 2))) {
        // Button is pressed (connected to ground)
        }
        Serial.print(digitalRead(3)? "Idle" : "TDC");
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
                case EVT_HALL:     Serial.print("HALL   "); break;
                case EVT_SYNC:     Serial.print("SYNC   "); break;
            }
            Serial.println(e.pair_a ? "PAIR_A" : "PAIR_B");
        }
        Serial.println("--- END ---");
    }
}