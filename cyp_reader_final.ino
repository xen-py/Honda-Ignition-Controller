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

#include <Arduino.h>

const int CYP_IN = 2;

#define MIN_LOW_US   80UL     // HEI open-collector dwell floor; real noise sits well below this
#define MAX_LOW_US   1000UL   // stuck-sensor safety ceiling only — real 1mm-notch LOW pulses
                              // should never approach this except at near-stall speed

enum HitState {
    ARMED,        // waiting for LOW to begin
    CONFIRMING,   // seen LOW, waiting to see if it holds MIN_LOW_US
    HIT_LATCHED   // confirmed hit, waiting for HIGH or MAX_LOW_US timeout
};

HitState state = ARMED;
unsigned long low_start_time = 0;
unsigned long hit_count = 0;
volatile bool CYP_TRIGGERED = false;

void setup()
{
    pinMode(CYP_IN, INPUT); // external pull-up assumed
    pinMode(2, INPUT_PULLUP); // but for the bench
    Serial.begin(9600);
    Serial.println(F("=== CYP low-trigger-biased hit stream ==="));
    Serial.print(F("MIN_LOW_US=")); Serial.print(MIN_LOW_US);
    Serial.print(F("  MAX_LOW_US=")); Serial.println(MAX_LOW_US);
    Serial.println(F("Spin it. Confirmed hits print below."));
}

void loop()
{
    unsigned long now = micros();
    bool pin_low = (digitalRead(CYP_IN) == LOW);

    switch (state)
    {
        case ARMED:
            if (pin_low)
            {
                state = CONFIRMING;
                low_start_time = now;
            }
            break;

        case CONFIRMING:
            if (!pin_low)
            {
                // went high again before MIN_LOW_US elapsed — noise, not a hit
                state = ARMED;

            }
            else if (now - low_start_time >= MIN_LOW_US)
            {
                // sustained long enough — this is a real hit
                hit_count++;

                Serial.print(F("HIT #"));
                Serial.print(hit_count);

                

                state = HIT_LATCHED;
                CYP_TRIGGERED = true;

            }
            break;

        case HIT_LATCHED:

            if (now - low_start_time >= MAX_LOW_US)
            {
                // stuck low past ceiling — force-return to armed anyway
                Serial.println(F("  [auto-return] LOW exceeded MAX_LOW_US, rearming"));
                state = ARMED;
            }
            break;
    }
}