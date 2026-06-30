//first rail
const int IGN_IN = 2; //yellow
const int HALL_IN = 3; //orange
const int COIL_A1 = 4;  // cyl 1 blue
const int COIL_B1 = 5;  // cyl 2 green
const int HALL_OUT = 6;
const int LED = 8;
//** DON'T FORGET TO CONVERT THIS TO DIRECT PORT MANIPULATION later!!

//opposite rail
// sensor power 3v power purple
// pull up 5v reference yellow
//pull down blue
//pull down green
//logic ground from immobilizer wires
//power from cig lighter, regulated with 78mo5, or tapped USB

const bool DEBUG_MODE = false; // bypass hall effect sync, alternate pairs blindly

volatile bool sync_established = false;
volatile bool active_pair_A = true; // change to true by default
volatile bool pending_sync = false;
volatile bool dwell_active = false;
volatile unsigned long last_hall_time = 0;
volatile unsigned long rpm = 0;
volatile bool is_running = false;
volatile int hall_pulse_count = 0;

void setup() {
    pinMode(IGN_IN, INPUT);
    pinMode(HALL_IN, INPUT_PULLUP);
    pinMode(HALL_OUT, OUTPUT); digitalWrite(HALL_OUT, HIGH);
    pinMode(7, OUTPUT);digitalWrite(7, HIGH);
    pinMode(LED, OUTPUT);digitalWrite(LED, HIGH);
    // idle state LOW for k20 coils
    pinMode(COIL_A1, OUTPUT); digitalWrite(COIL_A1, LOW);
    pinMode(COIL_B1, OUTPUT); digitalWrite(COIL_B1, LOW);
    attachInterrupt(digitalPinToInterrupt(IGN_IN), ign_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(HALL_IN), hall_ISR, FALLING);
    Serial.begin(9600);
    Serial.println("Start Up complete");
}

void hall_ISR() {
    unsigned long now = micros();

    //wait for two halls to minimiza start up noise
    hall_pulse_count++;
    if (hall_pulse_count < 2) {
        last_hall_time = now;
        return;
    }

    unsigned long elapsed = now - last_hall_time;
    last_hall_time = now;

    // noise filter - reset if pulses impossibly close
    if (elapsed < 7500) {
        hall_pulse_count = 0;
        return;
    }
    Serial.println("Hall Fired");
    //resync but not during dwell(during dwell right now)
    sync_established = true;
    if (!dwell_active) {
        // remove active_pair_A = true entirely - just let it free-run
    } else {
        pending_sync = true;
        Serial.print(" SYNC: ");
        Serial.println("pending");
    }

    // upper limit check - stall detection
    if (elapsed > 500000) {
        rpm = 0;
        is_running = false;
        sync_established = false;
        hall_pulse_count = 0;
        return;
    }

    is_running = true;
    rpm = 120000000UL / elapsed;

}

void ign_ISR() {
    Serial.println("Ignition ISR Triggered");

    unsigned long now = micros();
    static unsigned long last_ign_time = 0;
    
    if (now - last_ign_time < 10000) return; // ignore pulses closer than 10ms
    last_ign_time = now;
    Serial.print("Debounce success: "); Serial.println(digitalRead(IGN_IN)); 

    // in debug mode skip sync requirement, 50/50
    if (!DEBUG_MODE && !sync_established) return;

    bool state = digitalRead(IGN_IN);
    //start charging/dwell
    if (state == LOW) {
        dwell_active = true;
        if (active_pair_A) {
            digitalWrite(COIL_A1, HIGH);
             Serial.println("pair a dwell:");
        } else {
            digitalWrite(COIL_B1, HIGH);
             Serial.println("pair b dwell: ");
        }
    } else {//fire/return to idle
        if (active_pair_A) {
            digitalWrite(COIL_A1, LOW);
            Serial.println("pair a fire: ");
        } else {
            digitalWrite(COIL_B1, LOW);
            Serial.println("pair b fire: ");
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

    if (micros() - last_hall_time > 200000) {
        rpm = 0;
        is_running = false;
        sync_established = false;
        hall_pulse_count = 0;
    }

    static unsigned long last_print = 0;
    if (millis() - last_print > 500) {
        /*
        Serial.print("IGN: ");
        Serial.print(digitalRead(2));
        Serial.print(" HALL: ");
        Serial.print(digitalRead(3));
        Serial.print(" RPM: ");
        Serial.print(rpm);
        Serial.print(" SYNC: ");
        Serial.print(sync_established);
        Serial.print(" PAIR: ");
        Serial.println(active_pair_A ? "1&4" : "2&3");
        */
        last_print = millis();
    }
}