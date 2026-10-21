#include <Arduino.h>

// =========================================================
// PIN MAP (Arduino UNO / ATmega328P)
// =========================================================
#define CYP_IN 2
#define COIL_A 4
#define COIL_B 5
#define ICP1_PIN 8   // PB0 / ICP1

// =========================================================
// PHILOSOPHY (per direct oscilloscope verification):
// Both A21 (via clean factory ICM/daughter-board tap) and CYP
// (via direct header into empty daughter-board slot) are clean,
// unfiltered, no debounce needed. Complexity from the previous
// version (confirm-windows, consensus voting, WINDOW_BLOCKED)
// is deliberately removed. This version trusts both signals at
// face value.
//
// STATE MODEL:
// - Cold start: coils fully OFF. A21 edges are discarded
//   entirely (not even logged) until the first CYP pulse sets
//   phase = A. Only then does A21 begin driving real dwells,
//   starting with Coil_A on the very next A21 edge.
// - Once running: continues firing on last-known phase
//   indefinitely, WITHOUT requiring repeated CYP confirmation
//   every cycle -- even through a stall/recrank, as long as CYP
//   never disagrees.
// - CYP is always watching in the background. If a CYP pulse
//   ever arrives while phase != A (i.e. CYP says "A should be
//   next" but phase disagrees), that's an immediate correction:
//   flip phase for the NEXT dwell cycle (current dwell, if any,
//   finishes normally -- never cut short).
// - "Unsynced" only truly means "never yet been correct even
//   once since boot." Once synced, it never goes back to fully
//   gated/unsynced on its own -- only a fresh boot does that.
// =========================================================

// =========================================================
// TIMING
// =========================================================
#define STARTUP_GUARD_MS   2000UL   // ignore all A21 edges for this long after
                                    // boot, purely to skip key-on transient chatter

volatile bool synced = false;        // true once CYP has confirmed phase at least once
volatile uint8_t phase = 1;          // 0 = Coil A, 1 = Coil B. Starts at B so the
                                     // first real CYP hit (which sets phase=A)
                                     // is a real, visible transition, not a no-op.
volatile bool flag_first_sync = false;
volatile bool flag_cyp_disagree = false;

volatile unsigned long boot_time = 0;

// =========================================================
// STALL GUARD (1000ms, time-based)
// =========================================================
#define STALL_GUARD_MS 1000UL

volatile unsigned long last_activity_time = 0;
volatile bool stalled = false;

// =========================================================
// DWELL STATE (single source of truth for coil control)
// =========================================================
enum State {
    IDLE_WAIT_LOW,
    DWELLING
};

volatile State state = IDLE_WAIT_LOW;
volatile unsigned long t_dwell_start = 0;
volatile uint8_t dwell_fired_phase = 0; // phase captured at dwell-start, so a
                                        // mid-dwell correction can't retroactively
                                        // change which coil this dwell belongs to

// =========================================================
// RPM (internal, derived from CYP pulses -- 1 pulse per distributor
// revolution = 1 pulse per 4 A21 fires, distributor spins at half
// crank speed)
// =========================================================
volatile unsigned long last_cyp_time = 0;
volatile unsigned long cyp_period_us = 0;
volatile unsigned long internal_rpm = 0; // crank RPM, derived from CYP period

// =========================================================
// CYP -- confirmed via oscilloscope: 80% duty cycle, clean
// signal. Rising-edge-only detection, real hardware interrupt
// on INT0 (pin 2). One hit per rising edge, no duration check,
// no debounce -- signal is trusted at face value.
// =========================================================
volatile bool cyp_hit_pending = false; // set by ISR, consumed once per fire cycle
                                        // by the A21 dwell-end logic

volatile unsigned long last_cyp_fall = 0;
#define CYP_MIN_PERIOD_US 500

ISR(INT0_vect)
{
    unsigned long now = micros();

    // Startup guard applies here too -- ignore CYP edges during
    // the same settling window as A21.
    if (millis() - boot_time < STARTUP_GUARD_MS)
    {
        return;
    }

    // Reject impossible duplicate falling edges
    if (last_cyp_fall != 0)
    {
        unsigned long dt = now - last_cyp_fall;
        if (dt < CYP_MIN_PERIOD_US)
            return;
    }

    last_cyp_fall = now;
    last_activity_time = now;

    if (!cyp_hit_pending)
        cyp_hit_pending = true;

    log_event(now, 2, phase);

    // RPM calculation: CYP period -> distributor RPM -> x2 for crank RPM
    if (last_cyp_time != 0)
    {
        cyp_period_us = now - last_cyp_time;
        if (cyp_period_us > 0)
        {
            unsigned long dist_rpm = 60000000UL / cyp_period_us;
            internal_rpm = dist_rpm * 2;
        }
    }

    // ---------------------------------------------
    // FIRST-EVER sync: this is the moment phase=A gets
    // established and A21 starts being trusted.
    // ---------------------------------------------
    if (!synced)
    {
        phase = 0; // A is next
        synced = true;
        state = IDLE_WAIT_LOW; // clean slate for the dwell machine
        // Can't Serial.print from inside an ISR reliably at speed --
        // flagged via flag_first_sync instead, printed from loop().
        flag_first_sync = true;
    }
    else
    {
        if (phase == 0)//phase should be zero when cyp is arriving
        {
            flag_cyp_disagree = true; // printed from loop(), same reasoning
        }
    }
}

// =========================================================
// HISTORY LOG
// =========================================================
#define HISTORY_SIZE 32

struct Event {
    unsigned long t;      // for LOW/CYP: capture timestamp. For HIGH: dwell length (us)
    uint8_t type;          // 0 = LOW (dwell start), 1 = HIGH (dwell end, t=dwell length), 2 = CYP
    uint8_t phase;
};

volatile Event history[HISTORY_SIZE];
volatile uint8_t hpos = 0;
volatile bool history_full = false;

inline void log_event(unsigned long t, uint8_t type, uint8_t ph)
{
    history[hpos] = { t, type, ph };
    hpos = (hpos + 1) & (HISTORY_SIZE - 1);
    if (hpos == 0) history_full = true;
}

// =========================================================
// COIL CONTROL -- direct fire, non-inverted, no opto stage.
// Per K20a.org: coil rests at 0V/LOW, charges (dwells) at
// 5V/HIGH, and fires on the falling edge back to LOW.
inline void coil_off()
{
    PORTD &= ~(1 << COIL_A);
    PORTD &= ~(1 << COIL_B);
}

inline void coil_on(uint8_t ph)
{
    if (ph == 0) PORTD |= (1 << COIL_A);
    else         PORTD |= (1 << COIL_B);
}

// =========================================================
// SETUP
// =========================================================
void setup()
{
    coil_off();
    DDRD |= (1 << COIL_A);
    DDRD |= (1 << COIL_B);

    DDRD &= ~(1 << CYP_IN);  // no pull_up needed if tapping FC1 daughterboard slot

    // ICP1 input
    DDRB &= ~(1 << PB0); //external pullup needed or enable internal

    // Timer1 ICP setup
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1B |= (1 << ICNC1); // noise canceller (harmless to leave on even though
                            // scope shows clean signal -- costs nothing)
    TCCR1B |= (1 << ICES1); // start rising edge
    TCCR1B |= (1 << CS11);  // prescaler 8
    TIMSK1 |= (1 << ICIE1);

    // CYP on INT0 (pin 2), rising edge only, per oscilloscope
    // confirmation of 80% duty cycle / clean signal
    EICRA &= ~((1 << ISC01) | (1 << ISC00));
    EICRA |= (1 << ISC01) ; // falling edge triggers INT0
    EIMSK |= (1 << INT0);

    Serial.begin(115200);
    Serial.println(F("=== H22A Digital Distributor (v3: CYP-gated, no debounce) ==="));
    Serial.print(F("Startup guard: ")); Serial.print(STARTUP_GUARD_MS); Serial.println(F("ms"));

    boot_time = millis();
}

// =========================================================
// ICP1 ISR -- peek-then-flip pattern, unchanged from prior
// versions (hardware ground truth, self-correcting, provably
// can't drift). No confirm-window: signal is trusted at face
// value per oscilloscope verification.
// =========================================================
ISR(TIMER1_CAPT_vect)
{
    unsigned long now = micros();
    last_activity_time = now;

    bool rising = (TCCR1B & (1 << ICES1));
    if (rising) TCCR1B &= ~(1 << ICES1);
    else         TCCR1B |= (1 << ICES1);
    bool level = rising;

    // -----------------------------------------------------
    // Startup guard: discard everything, don't even log,
    // until STARTUP_GUARD_MS has passed since boot.
    // -----------------------------------------------------
    if (millis() - boot_time < STARTUP_GUARD_MS)
    {
        return;
    }

    // -----------------------------------------------------
    // Unsynced gate: until CYP has confirmed phase at least
    // once since boot, A21 edges are completely discarded --
    // no dwell logic runs, nothing gets logged.
    // -----------------------------------------------------
    if (!synced)
    {
        return;
    }

    // -----------------------------------------------------
    // Normal dwell state machine -- trusted at face value,
    // no confirm delay.
    // -----------------------------------------------------
    if (!level)
    {
        // LOW edge -> dwell start
        if (state == IDLE_WAIT_LOW)
        {
            state = DWELLING;
            t_dwell_start = now;
            dwell_fired_phase = phase; // snapshot phase now -- a CYP correction
                                       // arriving mid-dwell must not change
                                       // which coil THIS dwell belongs to
            coil_on(dwell_fired_phase);
            log_event(now, 0, dwell_fired_phase);
        }
    }
    else
    {
        // HIGH edge -> dwell end
        if (state == DWELLING)
        {
            coil_off();
            unsigned long dwell_len = now - t_dwell_start;
            log_event(dwell_len, 1, dwell_fired_phase); // t field holds LENGTH here

            // Phase only advances for the NEXT cycle if a CYP correction
            // is pending; otherwise keep alternating normally.
            if (cyp_hit_pending)
            {
                cyp_hit_pending = false;
                phase = 0; // CYP hit means "A should be next" -- authoritative
            }
            else
            {
                phase ^= 1; // normal alternation
            }

            state = IDLE_WAIT_LOW;
        }
    }
}

// =========================================================
// STALL GUARD (1000ms, time-based)
// =========================================================
// If neither A21 nor CYP has produced any activity for
// STALL_GUARD_MS, treat as stalled: reset back to the same
// cold-start posture as boot (synced=false, coils off, dwell
// state reset). Everything else about the design -- CYP always
// watching, instant correction on disagreement, no separate
// resync logic beyond this -- is unchanged.
// =========================================================
inline void check_stall_guard()
{
    unsigned long last_seen;
    noInterrupts();
    last_seen = last_activity_time;
    interrupts();

    bool now_stalled = (last_seen != 0) && (millis() - (last_seen / 1000UL) >= STALL_GUARD_MS);

    if (now_stalled && !stalled)
    {
        stalled = true;
        noInterrupts();
        synced = false;
        state = IDLE_WAIT_LOW;
        coil_off();
        interrupts();
        Serial.println(F("[STALL] No activity for 1s -- reset to unsynced."));
    }
    else if (!now_stalled && stalled)
    {
        stalled = false;
        Serial.println(F("[STALL] Activity resumed."));
    }
}

// =========================================================
// HISTORY DUMP
// =========================================================
inline void print_history()
{
    if (Serial.available())
    {
        if (Serial.read() == 'h')
        {
            Serial.println(F("\n--- HISTORY (chronological) ---"));

            uint8_t start = history_full ? hpos : 0;
            uint8_t count = history_full ? HISTORY_SIZE : hpos;

            for (uint8_t i = 0; i < count; i++)
            {
                uint8_t idx = (start + i) & (HISTORY_SIZE - 1);
                volatile Event &e = history[idx];

                if (e.type == 1)
                {
                    // HIGH entries show DWELL LENGTH, not capture time
                    Serial.print(F("dwell_len="));
                    Serial.print(e.t);
                    Serial.print(F("us  FIRE (HIGH)  pair="));
                }
                else if (e.type == 2)
                {
                    Serial.print(F("t="));
                    Serial.print(e.t);
                    Serial.print(F("us  CYP           pair="));
                }
                else
                {
                    Serial.print(F("t="));
                    Serial.print(e.t);
                    Serial.print(F("us  DWELL(LOW)    pair="));
                }

                Serial.println(e.phase ? "B (cyl 2&3)" : "A (cyl 1&4)");
            }

            Serial.println(F("--- END ---\n"));
        }
    }
}

// =========================================================
// LOOP
// =========================================================
void loop()
{
    check_stall_guard();

    if (flag_first_sync)
    {
        flag_first_sync = false;
        Serial.println(F("[SYNC] First CYP hit -- phase=A established. A21 now live."));
    }

    if (flag_cyp_disagree)
    {
        flag_cyp_disagree = false;
        Serial.println(F("[SYNC] CYP disagrees with running phase -- correcting next cycle."));
    }

    static unsigned long last_status = 0;
    if (millis() - last_status > 1000)
    {
        last_status = millis();
        Serial.print(F("[STATUS] synced="));
        Serial.print(synced ? F("YES") : F("NO"));
        Serial.print(F("  phase="));
        Serial.print(phase == 0 ? F("A") : F("B"));
        Serial.print(F("  rpm="));
        Serial.println(internal_rpm);
    }

    print_history();
}
