#include <Arduino.h>

// =========================================================
// ICP1 SIGNAL PROFILER
// Arduino UNO / ATmega328P
//
// Pin 8 (PB0 / ICP1)
//
// Captures EVERY Timer1 input capture edge.
// No ignition logic.
// No filtering.
// No debounce.
// =========================================================

#define HISTORY_SIZE 64

struct EdgeEvent
{
    unsigned long time;
    bool rising;
};

volatile EdgeEvent history[HISTORY_SIZE];
volatile uint8_t index_pos = 0;
volatile bool buffer_full = false;

volatile unsigned long last_capture = 0;
volatile unsigned long edge_count = 0;


// =========================================================
// SETUP
// =========================================================
#define CYP_IN 2
#define COIL_A 4
#define COIL_B 5
#define ICP1_PIN 8   // PB0 / ICP1
void setup()
{

    PORTD |= (1 << COIL_A);
    PORTD |= (1 << COIL_B);
  DDRD |= (1 << COIL_A);
    DDRD |= (1 << COIL_B);
    
    Serial.begin(115200);

    Serial.println();
    Serial.println(F("=== ICP1 EDGE PROFILER ==="));

    // ICP1 input
    DDRB &= ~(1 << PB0);


    // Timer1 setup
    TCCR1A = 0;
    TCCR1B = 0;

    // no noise canceler for raw profiling
    // TCCR1B |= (1 << ICNC1);

    // start looking for rising edge
    TCCR1B |= (1 << ICES1);

    // prescaler 8
    // 16MHz / 8 = 2MHz
    // 0.5us resolution
    TCCR1B |= (1 << CS11);
    TCCR1B |= (1 << ICNC1);


    // Enable input capture interrupt
    TIMSK1 |= (1 << ICIE1);


    sei();
}



// =========================================================
// TIMER1 INPUT CAPTURE ISR
// =========================================================

ISR(TIMER1_CAPT_vect)
{
    unsigned long now = micros();


    // IMPORTANT:
    // ICES1 still represents the edge we just captured
    // at the beginning of ISR on AVR.

    bool rising = (TCCR1B & (1 << ICES1));


    history[index_pos].time = now;
    history[index_pos].rising = rising;


    index_pos++;

    if(index_pos >= HISTORY_SIZE)
    {
        index_pos = 0;
        buffer_full = true;
    }


    edge_count++;


    // toggle capture edge
    if(rising)
    {
        TCCR1B &= ~(1 << ICES1);   // next capture falling
    }
    else
    {
        TCCR1B |= (1 << ICES1);    // next capture rising
    }
}



// =========================================================
// PRINT HISTORY WITH 'h'
// =========================================================

void print_history()
{
    if(!Serial.available())
        return;

    if(Serial.read() != 'h')
        return;

    noInterrupts();

    uint8_t count = buffer_full ? HISTORY_SIZE : index_pos;
    uint8_t start = buffer_full ? index_pos : 0;

    Serial.println();
    Serial.println(F("=== EDGE HISTORY ==="));

    unsigned long previous = history[start].time;

    for(uint8_t i=0;i<count;i++)
    {
        uint8_t idx = (start+i) % HISTORY_SIZE;

        unsigned long delta = history[idx].time - previous;

        Serial.print(history[idx].time);
        Serial.print(F(" us  "));

        Serial.print(delta);
        Serial.print(F(" us since last  "));

        if(history[idx].rising)
            Serial.println(F("RISING (LOW->HIGH)"));
        else
            Serial.println(F("FALLING (HIGH->LOW)"));

        previous = history[idx].time;
    }

    interrupts();

    Serial.println(F("===================="));
}

// =========================================================
// LOOP
// =========================================================

void loop()
{
    print_history();


    static unsigned long last = 0;

    if(millis()-last > 1000)
    {
        last = millis();

        noInterrupts();
        unsigned long count = edge_count;
        interrupts();


        Serial.print(F("Edges/sec: "));
        Serial.println(count);
    }
}