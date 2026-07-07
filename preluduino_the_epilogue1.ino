#include <Arduino.h>

// =========================================================
// PIN MAP (Arduino UNO / ATmega328P)
// =========================================================
#define CYP_IN 2
#define COIL_A 4
#define COIL_B 5
#define ICP1_PIN 8   // PB0 / ICP1

// =========================================================
// TIMING RULES
// =========================================================
#define MIN_DWELL_US 2000UL
#define MAX_DWELL_US 6000UL    // absolute safety ceiling only — normal dwell-end is
                              // now confirmed via WAIT_HIGH_CONFIRM below
#define EDGE_CONFIRM_US 30UL   // symmetric debounce-by-persistence floor for BOTH
                              // edges
// =========================================================
// CYP / INT0 — LOW-TRIGGER-BIASED HIT STREAM
// =========================================================
// Same philosophy as the ICP1/A21 dwell logic: a hit only counts
// once the pin has been LOW for a minimum sustained duration
// (debounce-by-persistence, not just "any edge happened"). If it
// stays low past a ceiling, we treat that as "stuck/expired" and
// auto-return to armed state rather than waiting forever.
//
// MIN_LOW_US   - how long the pin must stay low, continuously,
//                before we count it as a real hit
// MAX_LOW_US   - if it's still low after this long, force-reset
//                back to armed (mirrors MAX_DWELL_US pattern)
//
// Wiring: CYP sensor signal -> D2 (polled digitalRead, no
// external interrupt needed for this version)
// =========================================================




#define MIN_LOW_US   80UL     // HEI open-collector dwell floor; real noise sits well below this
#define MAX_LOW_US   1000UL   // stuck-sensor safety ceiling only — real 1mm-notch LOW pulses
                              // should never approach this except at near-stall speed

enum CYPState {
    ARMED,          // waiting for LOW to begin
    CONFIRMING,     // seen LOW, waiting to see if it holds MIN_LOW_US
    HIT_LATCHED,    // confirmed hit, waiting for HIGH or MAX_LOW_US timeout
    WINDOW_BLOCKED  // timed out mid-pulse; ignore everything until pin
                    // actually releases high, so the same physical pulse
                    // can't be counted twice
};

CYPState hitstate = ARMED;
unsigned long low_start_time = 0;
unsigned long hit_count = 0;
volatile bool CYP_TRIGGERED = false;
// =========================================================
// STATE MACHINE
// =========================================================
enum State {
    IDLE_WAIT_LOW,
    WAIT_LOW_CONFIRM,     // seen a LOW edge, confirming it's real before dwelling
    DWELLING,
    WAIT_HIGH_CONFIRM,    // seen a HIGH edge, confirming it's real before ending
    LOCKED_OUT            // post-stall: ignore everything until first confirmed
                          // real LOW, which also becomes the first valid dwell
};

volatile State state = LOCKED_OUT;   // boot into lockout — first real LOW after
                                      // power-on is what starts everything, same
                                      // philosophy as post-stall recovery

// =========================================================
// PHASE (5DISTRIBUTOR)
// =========================================================
volatile uint8_t phase = 0; // 0 = Coil A, 1 = Coil B

// =========================================================
// TIMING
// =========================================================
volatile unsigned long t_dwell_start = 0;
volatile unsigned long last_event_time = 0;
volatile unsigned long last_fire_interval = 0;
volatile uint16_t dwell_bounce_rejects = 0; 
volatile unsigned long low_confirm_start = 0;
volatile unsigned long high_confirm_start = 0;
// =========================================================
// PHASE-PRESERVATION WATCHDOG (one-shot)
// =========================================================
// The pullup-death LOW edge already forces one implicit phase flip
// (via state->DWELLING->MAX_DWELL_US->end_dwell). This watchdog fires
// exactly once per idle period to cancel that flip back out, so phase
// is correct again on the next real start — no resync needed.
#define IDLE_PHASE_ADVANCE_MS   20000UL

volatile bool last_edge_was_low = false;   // true only if the last real edge was a falling/LOW edge
volatile bool flag_phase_sync = false;
// =========================================================
// sync
// =========================================================
volatile bool synced = false;
// ===== PHASE_ACCUM windowed-tally state =====

struct PhaseAccum
{
    uint8_t fire_count;

    uint8_t hits[2];

};
volatile PhaseAccum phase_accum =
{
    0,
    {0,0}
};

// =========================================================
// HISTORY LOG
// =========================================================
#define HISTORY_SIZE 32   // must stay a power of 2 (mask trick below)

struct Event {
    unsigned long t;
    uint8_t type;   // 0 LOW, 1 HIGH, 2 CYP
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
volatile unsigned long last_edge_time = 0;
// =========================================================
// SETUP
// =========================================================
void setup()
{
    coil_off();
    DDRD |= (1 << COIL_A);
    DDRD |= (1 << COIL_B);

    DDRD&= ~(1<<CYP_IN);// external pull-up assumed
    //PORTD |= (1 << CYP_IN);

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
    Serial.println(F("=== H22A Digital Distributor ==="));
    Serial.print(F("Max CYP Dwell =")); Serial.println(MAX_LOW_US);
    Serial.print(F("Min Coil Dwell =")); Serial.println(MIN_DWELL_US);
    Serial.print(F("Max Coil Dwell =")); Serial.println(MAX_DWELL_US);
}
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
  

  bool hit_to_consume = CYP_TRIGGERED;
    CYP_TRIGGERED = false;   // always clear here — this fire either consumes
                             // it now or it's gone; no hit ever survives
                             // past the fire immediately after it

    if(!synced){
        phase_accum.fire_count++;
        if (hit_to_consume){
            log_event(2, phase);
            phase_accum.hits[phase]++;
        }
        sync();
    }
  
  if (phase == 0) PORTD &= ~(1 << COIL_A);
  else            PORTD &= ~(1 << COIL_B);

}
//engine sync
inline void sync(){
  if(phase_accum.fire_count<8) return;
  bool diff = abs(phase_accum.hits[phase] - phase_accum.hits[!phase])>=2;
  if (diff){
    //since phase is updated on fire not this edge
    //hits[tracks] the phase before cyp was called
    //so if the most hit phase is equal to the current one 
    // that means the cyp is not indicating this one
    phase = (phase_accum.hits[phase] < phase_accum.hits[!phase]);
    //current phase = to whicheverone has less hit 
    synced = true;
    reset_phase_accum();
  }

}

inline void reset_phase_accum()
{
  phase_accum.fire_count = 0;
  phase_accum.hits[0]=0;
  phase_accum.hits[1]=0;

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
// HISTORY DUMP
// =====================================================
inline void print_history(){
  if (Serial.available())
  {
      if (Serial.read() == 'h')
      {
          Serial.println(F("\n--- HISTORY (chronological) ---"));

          uint8_t start = history_full ? hpos : 0;
          uint8_t count = history_full ? HISTORY_SIZE : hpos;

          unsigned long t0 = count ? history[start].t : 0;

          for (uint8_t i = 0; i < count; i++)
          {
              uint8_t idx = (start + i) & (HISTORY_SIZE - 1);
              volatile Event &e = history[idx];

              Serial.print(e.t - t0);
              Serial.print("us  ");
              Serial.print(e.type == 2? F("CYP") :e.type ? F("FIRE (HIGH)") : F("DWELL(LOW) "));
              Serial.print("  pair=");
              Serial.println(e.phase ? "B (cyl 2&3)" : "A (cyl 1&4)");
          }

          Serial.println("--- END ---\n");
      }
  }
}

inline void stall_detector(unsigned long now){
  // ===== STALL DETECTION =====
    // Uses the already-snapshotted fire_snapshot (atomic read done above).
    // Completely independent of rpm EMA — raw elapsed time only.
    noInterrupts();
    unsigned long fire_snapshot = last_edge_time;//previously last_fire_time
    interrupts();
    bool prev_stall = stall_detected;
    stall_detected = (micros() - fire_snapshot) > STALL_THRESHOLD_US;

    // Transition: running → stalled
    if (stall_detected && !prev_stall) {
        Serial.println(F("[STALL] Engine stopped"));

        #ifdef STALL_COIL_SAFETY
        coil_off();
        reset_phase_accum();
        synced = false;
        state = LOCKED_OUT;
        #endif
    }

    // Transition: stalled → running
    if (!stall_detected && prev_stall) {
        Serial.println(F("[STALL] Engine cranking detected"));
    }
}
// =========================================================
// ICP1 ISR
// =========================================================
ISR(TIMER1_CAPT_vect)
{
    unsigned long now = micros();
    last_edge_time = now;

    bool rising = (TCCR1B & (1 << ICES1));
    if (rising) TCCR1B &= ~(1 << ICES1);
    else         TCCR1B |= (1 << ICES1);
    bool level = rising;

    if (!level)
    {
        if (state == IDLE_WAIT_LOW || state == LOCKED_OUT)
        {
            state = WAIT_LOW_CONFIRM;
            low_confirm_start = now;
        }
        else if (state == WAIT_HIGH_CONFIRM)
        {
            // dropped back low before HIGH confirmed — ringing on the
            // real edge, resume dwelling as if nothing happened
            state = DWELLING;
        }
        return;
    }
    else
    {
        if (state == WAIT_LOW_CONFIRM)
        {
            // went high again before LOW confirmed — noise, not real.
            // IMPORTANT: return to whichever "waiting" state we came
            // from
            state = IDLE_WAIT_LOW;
        }
        else if (state == DWELLING)
        {
            unsigned long dt = now - t_dwell_start;
            if (dt >= MIN_DWELL_US)
            {
                state = WAIT_HIGH_CONFIRM;
                high_confirm_start = now;
            }
            else
            {
                dwell_bounce_rejects++;
            }
        }
    }
}



// =========================================================
// LOOP (timeout + CYP_Detection)
// =========================================================
void loop()
{
    unsigned long now = micros();
    bool pin_low = !(PIND & (1 << CYP_IN));

    // =====================================================
    // MAX DWELL SAFETY (loop responsibility)
    // =====================================================
    bool a21_low = !(PINB & (1 << PB0));   // ICP1_PIN = 8 = PB0

    if (state == WAIT_LOW_CONFIRM)
    {
        if (!a21_low)
        {
            state = IDLE_WAIT_LOW;   // consistent fallback if loop() catches the bounce first
        }
        else if (now - low_confirm_start >= EDGE_CONFIRM_US)
        {
            // confirmed real LOW — dwell begins, lockout implicitly cleared
            state = DWELLING;
            t_dwell_start = now;
            coil_on();
            log_event(0, phase);
            last_edge_was_low = true;
        }
    }
    else if (state == DWELLING)
    {
        if (now - t_dwell_start >= MAX_DWELL_US)
        {
            end_dwell(now, phase);   // safety ceiling only
        }
    }
    else if (state == WAIT_HIGH_CONFIRM)
    {
        if (a21_low)
        {
            state = DWELLING;   // consistent fallback if loop() catches it first
        }
        else if (now - high_confirm_start >= EDGE_CONFIRM_US)
        {
            end_dwell(now, phase);
            last_edge_was_low = false;
        }
        else if (now - t_dwell_start >= MAX_DWELL_US)
        {
            end_dwell(now, phase);   // absolute ceiling even if HIGH never confirms
        }
    }


  if(1/*!synced*/){
    
    switch (hitstate)
    {
      case ARMED:
          if (pin_low)
          {
              hitstate = CONFIRMING;
              low_start_time = now;
          }
          break;

      case CONFIRMING:
          if (!pin_low)
          {
              // went high again before MIN_LOW_US elapsed — noise, not a hit
              hitstate = ARMED;

          }
          else if (now - low_start_time >= MIN_LOW_US)
          {
              // sustained long enough — this is a real hit
              hit_count++;

              Serial.print(F("HIT #"));
              Serial.println(hit_count);
              

              hitstate = HIT_LATCHED;
              
              CYP_TRIGGERED = !synced;

          }
          break;

      case HIT_LATCHED:
        if (!pin_low)
        {
            // real release before the ceiling — fine, rearm normally
            hitstate = ARMED;
        }
        else if (now - low_start_time >= MAX_LOW_US)
        {
            // still low past the ceiling — already counted once.
            // Don't rearm yet, or the still-low pulse gets counted again.
            hitstate = WINDOW_BLOCKED;
        }
        break;

    case WINDOW_BLOCKED:
        if (!pin_low)
        {
            // pin actually released — safe to arm for the next real hit
            hitstate = ARMED;
        }
        // still low: do nothing, same physical pulse
        break;
      }
  }

    stall_detector(now);

    if (flag_phase_sync)
    {
        flag_phase_sync = false;
        Serial.print(F("phase sync happened recently"));
    }
    print_history();
}