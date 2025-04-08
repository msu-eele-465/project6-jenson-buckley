#include "intrinsics.h"
#include "msp430fr2355.h"
#include <driverlib.h>
#include <math.h>
#include <string.h>

//------------------------------------------------SETUP------------------------------------------------
//-- KEYPAD

//-- ADC SAMPLING

//-- ADC CODE TO CELCIUS / FAHRENHEIT

//-- SAMPLING TIMER

//-- WINDOWED AVERAGING (x2)

//-- I2C MASTER

//-- RTC (I2C INFO + vars?)

//-- LM92 (I2C INFO + vars?)

//-- PELTIER GPIO CONTROL

//-- LCD

//-- STATE MACHINE

//----------------------------------------------END SETUP----------------------------------------------

int main(void) {

    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    // Set P1.0 LED
    P1DIR |= BIT0;
    P1OUT &= ~BIT0;

    // Set P6.6 LED
    P6DIR |= BIT6;
    P6OUT &= ~BIT6;

    //------------------------------------------------INITIALIZATIONS------------------------------------------------
    //-- KEYPAD

    //-- ADC SAMPLING

    //-- ADC CODE TO CELCIUS / FAHRENHEIT

    //-- SAMPLING TIMER

    //-- WINDOWED AVERAGING (x2)

    //-- I2C MASTER

    //-- RTC (I2C INFO + vars?)

    //-- LM92 (I2C INFO + vars?)

    //-- PELTIER GPIO CONTROL

    //-- LCD

    //-- STATE MACHINE

    //----------------------------------------------END INITIALIZATIONS----------------------------------------------

    // enable interrupts
    __enable_interrupt();

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();

    //------------------------------------------------STATE MACHINE------------------------------------------------
    while(1)
    {
        // Toggle LEDs
        P1OUT ^= BIT0;
        P6OUT ^= BIT6;

        // Delay
        for(i=10000; i>0; i--);
    }
    //----------------------------------------------END STATE MACHINE----------------------------------------------
}

//------------------------------------------------FUNCTIONS AND ISRS------------------------------------------------
//-- KEYPAD

//-- ADC SAMPLING

//-- ADC CODE TO CELCIUS / FAHRENHEIT

//-- SAMPLING TIMER

//-- WINDOWED AVERAGING (x2)

//-- I2C MASTER

//-- RTC (I2C INFO + vars?)

//-- LM92 (I2C INFO + vars?)

//-- PELTIER GPIO CONTROL

//-- LCD

//-- STATE MACHINE

//--------------------------------------------END FUNCTIONS AND ISRS--------------------------------------------