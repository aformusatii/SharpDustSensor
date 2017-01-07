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

/********************************************************************************
 Macros and Defines
 ********************************************************************************/
#define LED_OFF                    0
#define LED_ON_BEFORE_MEASUREMENT  1
#define LED_ON_AFTER_MEASUREMENT   2

/********************************************************************************
 Function Prototypes
 ********************************************************************************/
void initGPIO();
void checkJobs();
void sensorCycle();
uint16_t adc_read(uint8_t);

/********************************************************************************
 Global Variables
 ********************************************************************************/
volatile uint16_t _delay = 1000;

volatile uint64_t onLEDTimerCicles = 0;
volatile uint64_t offLEDTimerCicles = 0;
volatile uint64_t mesureTimerCicles = 0;
volatile uint16_t currentState = LED_OFF;

/********************************************************************************
 Interrupt Service
 ********************************************************************************/
ISR(USART_RX_vect) {
    handle_usart_interrupt();
}

ISR(TIMER1_OVF_vect) {
    timer1_ovf_count++;
}

/********************************************************************************
    Main
********************************************************************************/
int main(void) {
    // initialize code
    usart_init();

    // initialize Timer 1
    //initTimer1();

    // init GPIO
    initGPIO();

    // enable interrupts
    sei();

    // Enable the ADC
    ADCSRA |= _BV(ADEN);

    // Output initialization log
    printf("Start...");
    printf(CONSOLE_PREFIX);

    uint64_t startTime = 0;
    uint64_t endTime = 0;

    // main loop
    while (1) {
        // main usart loop
        usart_check_loop();

        startTime = getCurrentTimeCicles();
        _delay_us(280);
        endTime = getCurrentTimeCicles();

        uint64_t e = endTime - startTime;

        printf("\n %lu", (unsigned long) e);
        _delay_ms(500);

        //checkJobs();

        //sensorCycle()'
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
        if (arg_count > 0) {
            _delay = (uint16_t) atoi(args[0]);
            printf("\n _delay=[%d]", _delay);
        }
    }
}

void initGPIO() {
    _out(DDC5, DDRC);
    _off(PC5, PORTC);

    _in(DDC4, DDRC); // Analog input 0
    _off(PC4, PORTC);
}

uint16_t adc_read(uint8_t adcx) {
    /* adcx is the analog pin we want to use.  ADMUX's first few bits are
     * the binary representations of the numbers of the pins so we can
     * just 'OR' the pin's number with ADMUX to select that pin.
     * We first zero the four bits by setting ADMUX equal to its higher
     * four bits. */
    ADMUX = adcx;
    ADMUX |= (1 << REFS1) | (1 << REFS0) | (0 << ADLAR);

    _delay_us(300);

    /* This starts the conversion. */
    ADCSRA |= _BV(ADSC);

    /* This is an idle loop that just wait around until the conversion
     * is finished.  It constantly checks ADCSRA's ADSC bit, which we just
     * set above, to see if it is still set.  This bit is automatically
     * reset (zeroed) when the conversion is ready so if we do this in
     * a loop the loop will just go until the conversion is ready. */
    while ((ADCSRA & _BV(ADSC)))
        ;

    /* Finally, we return the converted value to the calling function. */
    return ADC;
}

void checkJobs() {

    switch (currentState) {
        case LED_OFF:
            uint64_t now = getCurrentTimeCicles();
            if ( now > onLEDTimerCicles ) {
                _on(PC5, PORTC);
                onLEDTimerCicles = now + 100;
                mesureTimerCicles = now + 100;
                offLEDTimerCicles = now + 100;
                currentState = LED_ON_BEFORE_MEASUREMENT;
            }

            break;

        case LED_ON_BEFORE_MEASUREMENT:
            uint64_t now = getCurrentTimeCicles();
            if ( now > mesureTimerCicles ) {
                _off(PC5, PORTC);
                currentState = LED_ON_AFTER_MEASUREMENT;
            }

            break;

        case LED_ON_AFTER_MEASUREMENT:
            uint64_t now = getCurrentTimeCicles();
            if ( now > offLEDTimerCicles ) {
                _off(PC5, PORTC);
                currentState = LED_OFF;
            }

            break;
    }

}

void sensorCycle() {
    _on(PC5, PORTC);
    _delay_us(280);
    _delay_us(40);
    _off(PC5, PORTC);
    _delay_us(9680);
}
