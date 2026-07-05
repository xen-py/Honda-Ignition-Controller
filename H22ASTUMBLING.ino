#include <Arduino.h>

// =========================================================
// PIN MAP (Arduino UNO / ATmega328P)
// =========================================================
#define COIL_A 4
#define COIL_B 5
#define ICP1_PIN 8   // PB0 / ICP1

// =========================================================
// TIMING RULES
// =========================================================
#define MIN_DWELL_US 2000UL
#define MAX_DWELL_US 4000UL

// =========================================================
// STATE MACHINE
// =========================================================
enum State {
    IDLE_WAIT_LOW,
    DWELLING,
    WAIT_HIGH_CONFIRM
};

volatile State state = IDLE_WAIT_LOW;

// =========================================================
// PHASE (50/50 DISTRIBUTOR)
// =========================================================
volatile uint8_t phase = 0; // 0 = Coil A, 1 = Coil B

// =========================================================
// TIMING
// =========================================================
volatile unsigned long t_dwell_start = 0;
volatile unsigned long last_event_time = 0;
volatile unsigned long last_fire_interval = 0;
volatile uint16_t dwell_bounce_rejects = 0; 
// =========================================================
// PHASE-PRESERVATION WATCHDOG (one-shot)
// =========================================================
// The pullup-death LOW edge already forces one implicit phase flip
// (via state->DWELLING->MAX_DWELL_US->end_dwell). This watchdog fires
// exactly once per idle period to cancel that flip back out, so phase
// is correct again on the next real start — no resync needed.
#define IDLE_PHASE_ADVANCE_MS   20000UL

volatile unsigned long watchdog_last_seen = 0;   // updated in ISR on every edge
volatile bool idle_advance_done = false;         // latch: only ever fire once per idle period
volatile bool last_edge_was_low = false;   // true only if the last real edge was a falling/LOW edge
volatile bool flag_idle_phase_advance = false;
volatile unsigned long idle_phase_advance_count = 0;

// =========================================================
// HISTORY LOG
// =========================================================
#define HISTORY_SIZE 32   // must stay a power of 2 (mask trick below)

struct Event {
    unsigned long t;
    uint8_t type;   // 0 LOW, 1 HIGH
    uint8_t phase;
};

volatile Event history[HISTORY_SIZE];
volatile uint8_t hpos = 0;
volatile bool history_full = false;

inline void log_event(uint8_t type, uint8_t ph)
{
    history[hpos] = { micros(), type, ph };
    hpos = (hpos + 1) & (HISTORY_SIZE - 1);
    if (hpos == 0) history_full = true;
}

volatile unsigned long last_edge_time = 0;
// =========================================================
// COIL CONTROL (single source of truth)
// =========================================================
inline void coil_off()
{
    PORTD |= (1 << COIL_A);
    PORTD |= (1 << COIL_B);
}

inline void coil_on()
{
    if (phase == 0) PORTD &= ~(1 << COIL_A);
    else            PORTD &= ~(1 << COIL_B);
}

// =========================================================
// END DWELL (ONLY PLACE PHASE FLIPS)
// =========================================================
inline void end_dwell(unsigned long now, uint8_t fired_phase)
{
    coil_off();

    // timing
    last_fire_interval = now - last_event_time;
    last_event_time = now;

    // flip distributor
    phase ^= 1;

    state = IDLE_WAIT_LOW;
    log_event(1, fired_phase);   // log with the phase that actually fired
}

// =====================================================
// DEAD-SIGNAL WATCHDOG (loop responsibility, like MAX_DWELL)
// =====================================================
inline void mortician()
{
    unsigned long last_seen_snapshot;
    bool last_low_snapshot;

    noInterrupts();
    last_seen_snapshot = watchdog_last_seen;
    last_low_snapshot  = last_edge_was_low;
    interrupts();

    if (last_seen_snapshot != 0 && !idle_advance_done && last_low_snapshot &&
        (micros() - last_seen_snapshot) >= (IDLE_PHASE_ADVANCE_MS * 1000UL))
    {
        noInterrupts();
        phase ^= 1;
        idle_advance_done = true;
        interrupts();

        idle_phase_advance_count++;
        flag_idle_phase_advance = true;
    }
}
// =========================================================
// ICP1 ISR
// =========================================================
ISR(TIMER1_CAPT_vect)
{
    unsigned long now = micros();
    watchdog_last_seen = now;
    idle_advance_done = false;   // fresh edges mean we're not idle anymore; rearm the latch 
    unsigned long gap = now - last_edge_time;
    last_edge_time = now;

    bool rising = (TCCR1B & (1 << ICES1));

    // flip hardware edge detection
    if (rising) TCCR1B &= ~(1 << ICES1);
    else         TCCR1B |= (1 << ICES1);

    bool level = rising; // HIGH = rising edge event, LOW = falling event
 
    // =====================================================
    // LOW EDGE -> START DWELL
    // =====================================================
    if (!level)
    {
        if (state == IDLE_WAIT_LOW)
        {
            state = DWELLING;
            t_dwell_start = now;
            coil_on();
            log_event(0, phase);
            last_edge_was_low = true;   // real, accepted dwell-start
        }
        return;
    }
    else
    {
        // =====================================================
        // HIGH EDGE -> END DWELL (only if valid timing)
        // =====================================================
        if (state == DWELLING)
        {
            unsigned long dt = now - t_dwell_start;

            if (dt >= MIN_DWELL_US)
            {
                end_dwell(now, phase);
                last_edge_was_low = false;   // real, accepted dwell-end
            }
            else
            {
                dwell_bounce_rejects++;
                // bounce — not a real transition, leave last_edge_was_low untouche
            }
        }
        // state == IDLE_WAIT_LOW here → stray HIGH between cycles, correctly ignored
    }

    
    // if state == IDLE_WAIT_LOW here, this HIGH edge is just noise
    // between cycles — correctly ignored, no dt computed, no counters touched
}

// =========================================================
// SETUP
// =========================================================
void setup()
{
    DDRD |= (1 << COIL_A);
    DDRD |= (1 << COIL_B);

    coil_off();

    // ICP1 input
    DDRB &= ~(1 << PB0);

    // Timer1 ICP setup
    TCCR1A = 0;
    TCCR1B = 0;

    TCCR1B |= (1 << ICNC1); // noise canceller (critical)
    TCCR1B |= (1 << ICES1); // start rising edge
    TCCR1B |= (1 << CS11);  // prescaler 8

    TIMSK1 |= (1 << ICIE1);

    Serial.begin(9600);
}

// =========================================================
// LOOP (timeout + debug only)
// =========================================================
void loop()
{
    unsigned long now = micros();

    // =====================================================
    // MAX DWELL SAFETY (loop responsibility)
    // =====================================================
    if (state == DWELLING)
    {
        if (now - t_dwell_start >= MAX_DWELL_US)
        {
            end_dwell(now, phase);
        }
    }

    // =====================================================
    // MINIMAL STATUS
    // =====================================================
    static unsigned long last_print = 0;

    if (millis() - last_print > 500)
    {
        last_print = millis();

        Serial.print("[STATE] phase=");
        Serial.print(phase);
        Serial.print(" dt=");
        Serial.print(last_fire_interval);
        Serial.println();
    }

    // =====================================================
    // HISTORY DUMP
    // =====================================================
    if (Serial.available())
    {
        if (Serial.read() == 'h')
        {
            Serial.println("\n--- HISTORY (chronological) ---");

            uint8_t start = history_full ? hpos : 0;
            uint8_t count = history_full ? HISTORY_SIZE : hpos;

            unsigned long t0 = count ? history[start].t : 0;

            for (uint8_t i = 0; i < count; i++)
            {
                uint8_t idx = (start + i) & (HISTORY_SIZE - 1);
                volatile Event &e = history[idx];

                Serial.print(e.t - t0);
                Serial.print("us  ");
                Serial.print(e.type ? F("FIRE (HIGH)") : F("DWELL(LOW) "));
                Serial.print("  pair=");
                Serial.println(e.phase ? "B (cyl 2&3)" : "A (cyl 1&4)");
            }

            Serial.println("--- END ---\n");
        }
    }

    mortician();
    if (flag_idle_phase_advance )
    {
        flag_idle_phase_advance  = false;
        Serial.print(F("[WATCHDOG] Dead signal >20s, phase force-advanced. Count="));
        Serial.println(idle_phase_advance_count);
    }
}