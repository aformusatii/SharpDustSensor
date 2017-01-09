/********************************************************************************
    Includes
********************************************************************************/
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include <stdlib.h>
#include <inttypes.h>

#include "../rf-common-lib/atmega328/mtimer.h"
#include "../rf-common-lib/common/util.h"

extern "C" {
    #include "../rf-common-lib/atmega328/usart.h"
}

/**
http://www.healthyfacilitiesinstitute.com/a_128-Indoor_Air_Quality_and_Particle_Measurement
Bronze  40 - 49%    ≤ 100 µg/m3 of dust particles
Silver  50 - 54%    ≤ 100 µg/m3 of dust particles
Gold    ≥ 55%       ≤ 35 µg/m3 of dust particles
 */

/********************************************************************************
 Macros and Defines
 ********************************************************************************/
#define REF_VOLTAGE 4.925

#define GP2Y1010AU0F_SLOPE      0.172
#define GP2Y1010AU0F_INTERCEPT  0.0999
//#define GP2Y1010AU0F_INTERCEPT  0

/********************************************************************************
 Function Prototypes
 ********************************************************************************/
void initTimer();
void initGPIO();
void initADC();
uint16_t adc_read();

/********************************************************************************
 Global Variables
 ********************************************************************************/
volatile uint16_t lastADCValue = 0;
volatile uint16_t lastADCAVGValue = 0;
volatile uint16_t minADCValue = 60000;
volatile uint16_t maxADCValue = 0;
volatile uint64_t sumADCValue = 0;
volatile uint16_t adcReads = 0;
volatile uint16_t adcEx = 0;
uint16_t sensor_values[255];
volatile bool resetADCAVG = false;


/********************************************************************************
 Interrupt Service
 ********************************************************************************/
ISR(USART_RX_vect) {
    handle_usart_interrupt();
}

ISR (TIMER1_COMPA_vect)
{
    _on(PC5, PORTC);
    //_delay_us(228);
    //_delay_us(176);
    //_delay_us(220);
    _delay_us(270);
    lastADCValue = adc_read();
    //_delay_us(40);
    _off(PC5, PORTC);

    if (resetADCAVG) {
        resetADCAVG = false;
        sumADCValue = 0;
        adcReads = 0;
        minADCValue = 60000;
        maxADCValue = 0;
        adcEx = 0;
    }

    if (lastADCValue < minADCValue) {
        minADCValue = lastADCValue;
    }

    if (lastADCValue > maxADCValue) {
        maxADCValue = lastADCValue;
    }

    sumADCValue += lastADCValue;

    sensor_values[adcReads] = lastADCValue;

    adcReads++;

    lastADCAVGValue = sumADCValue / adcReads;
}

/********************************************************************************
    Main
********************************************************************************/
int main(void) {
    // initialize code
    usart_init();

    // init GPIO
    initGPIO();

    // initialize Timer 1
    initTimer();

    // init ADC
    initADC();

    // enable interrupts
    sei();

    // Output initialization log
    printf("Start...");
    printf(CONSOLE_PREFIX);

    _delay_ms(1000);

    printf("\nADC,AVG,MIN,MAX,ADCReads,Voltage,Dust");

    // main loop
    while (1) {
        // main usart loop
        usart_check_loop();

        double voltage = (double) ( (lastADCAVGValue * REF_VOLTAGE)/ (double) 1024 );
        double ret = (GP2Y1010AU0F_SLOPE * voltage) - GP2Y1010AU0F_INTERCEPT;

        if (ret < 0) {
            ret = 0;
        } else if (ret > 0.5) {
            ret = 0.5;
        }

        ret = ret * 1000; //convert milligrams to micrograms

        adcEx = 0;
        uint16_t avg = lastADCAVGValue + 20;
        for (uint8_t i = 0; i < adcReads; i++) {
            if (sensor_values[i] > avg) {
                adcEx++;
            }
        }


        printf("\n ADC=[%d] AVG=[%d] MIN=[%d] MAX=[%d] ex=[%d] adcReads=[%d] voltage=[%.05f], dust=[%.05f]", lastADCValue, lastADCAVGValue, minADCValue, maxADCValue, adcEx, adcReads, voltage, ret);

        //printf("\n%d,%d,%d,%d,%d,%.05f,%.05f", lastADCValue, lastADCAVGValue, minADCValue, maxADCValue, adcReads, voltage, ret);

        resetADCAVG = true;

        _delay_ms(1000);
    }
}

/********************************************************************************
 Functions
 ********************************************************************************/
void handle_usart_cmd(char *cmd, char* args[], uint8_t arg_count) {
    if (strcmp(cmd, "test") == 0) {
        for (uint8_t i = 0; i < arg_count; i++) {
            printf("\n ARG[%d]=[%s]", i, args[i]);
        }
    }

    if (strcmp(cmd, "set") == 0) {
        OCR2A = atoi(args[0]);
        OCR2B = atoi(args[0]);
    }

}

void initTimer() {
    // set up timer with CTC mode and pre-scaler:
    /*
     * |-------------------------------------------------------------------------------|
       | ICNC1 ICES1 – WGM13 WGM12 CS12 CS11 CS10 [TCCR1B]                             |
       |-------------------------------------------------------------------------------|
       | CS12 | CS11 | CS10 | Description                                              |
       |-------------------------------------------------------------------------------|
       |    0 |    0 |    0 | No clock source (Timer/Counter stopped).                 |
       |    0 |    0 |    1 | clkI/O/1 (No prescaling)                                 |
       |    0 |    1 |    0 | clkI/O/8 (From prescaler)                                |
       |    0 |    1 |    1 | clkI/O/64 (From prescaler)                               |
       |    1 |    0 |    0 | clkI/O/256 (From prescaler)                              |
       |    1 |    0 |    1 | clkI/O/1024 (From prescaler)                             |
       |    1 |    1 |    0 | External clock source on T1 pin. Clock on falling edge.  |
       |    1 |    1 |    1 | External clock source on T1 pin. Clock on rising edge.   |
       |-------------------------------------------------------------------------------|
    */
    TCCR1B |= (1 << WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10);

    // initialize counter
    TCNT1 = 0;

    // initialize compare value
    OCR1A = 20000;

    // enable compare interrupt
    TIMSK1 |= (1 << OCIE1A);

    // =================================================================================

    // Setup Timer/Counter2
    // Mode 3 - Fast PWM
    // Clear OC2B on Compare Match, set OC2B at BOTTOM (non-inverting mode)
    TCCR2A = (1<<COM2B1) | (0<<COM2B0) | (1<<WGM21) | (1<<WGM20);

    // clkT2S/64 (From prescaler)
    TCCR2B = (0<<WGM22) | (1<<CS22) | (1<<CS21) | (0<<CS20);

    OCR2A = 255;
    OCR2B = 255;

    _delay_ms(1000);

    OCR2A = 80;
    OCR2B = 80;
}

void initGPIO() {
    _out(DDC5, DDRC);
    _off(PC5, PORTC);

    _in(DDC4, DDRC); // Analog input 0
    _off(PC4, PORTC);

    _out(DDD3, DDRD); // OC2B
}

void initADC() {
    // Enable the ADC
    // Set ADC clock to FCPU/128
    ADCSRA |= _BV(ADEN) | (1 << ADPS2)| (1 << ADPS1) | (0 << ADPS0);

    // Voltage Reference - AVCC with external capacitor at AREF pin
    // Analog Channel - ADC4
    ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | 4;
}

uint16_t adc_read() {
    /* This starts the conversion. */
    ADCSRA |= _BV(ADSC);

    /* This is an idle loop that just wait around until the conversion
     * is finished.  It constantly checks ADCSRA's ADSC bit, which we just
     * set above, to see if it is still set.  This bit is automatically
     * reset (zeroed) when the conversion is ready so if we do this in
     * a loop the loop will just go until the conversion is ready. */
    //while ((ADCSRA & _BV(ADSC)))
    //    ;
    _delay_us(60);

    /* Finally, we return the converted value to the calling function. */
    return ADC;
}
