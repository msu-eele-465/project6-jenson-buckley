#include "gpio.h"
#include "intrinsics.h"
#include "msp430fr2355.h"
#include <driverlib.h>
#include <math.h>
#include <string.h>

//------------------------------------------------SETUP------------------------------------------------
//-- KEYPAD
void setupKeypad();                         // init
char readKeypad();                          // checks for pressed keys on keypad
int checkCols();                            // ussed internally by readKeypad()
char lastKey = 'X';                         // used internally for debouncing

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
    setupKeypad();

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
void setupKeypad() {
    
    // columns as outputs on P4.4, P6.6, P6.5, P6.4 initialized to 0
    P4DIR |= BIT4;
    P6DIR |= BIT6 | BIT5 | BIT4;
    P1OUT &= ~BIT4;
    P1OUT &= ~BIT6 | ~BIT5 | ~BIT4   

    // rows as inputs pulled down internally on P6.3, P6.2, P6.1, P6.0
    P6DIR &= ~BIT3 | ~BIT2 | ~BIT1 | ~BIT0;     // inputs
    P6REN |= BIT3 | BIT2 | BIT1 | BIT0;         // internal resistors
	P2OUT &=~ BIT3 | BIT2 | BIT1 | BIT0;        // pull-downs

}

char readKeypad() {
    // columns on P4.4, P6.6, P6.5, P6.4
    // rows on P6.3, P6.2, P6.1, P6.0

    char keys[4][4] = {
        {'1', '2', '3', 'A'},
        {'4', '5', '6', 'B'},
        {'7', '8', '9', 'C'},
        {'*', '0', '#', 'D'}
    };

    char pressed = 'X';

    // check row 1
    P4OUT |= BIT4;
    int col = checkCols();
    if (col!=-1) {
        pressed = keys[0][col];
    } 
    P4OUT &= ~BIT4;

    // check row 2
    P6OUT |= BIT6;
    col = checkCols();
    if (col!=-1) {
        pressed =  keys[1][col];
    } 
    P6OUT &= ~BIT6;

    // check row 3
    P6OUT |= BIT5;
    col = checkCols();
    if (col!=-1) {
        pressed =  keys[2][col];
    }
    P6OUT &= ~BIT5;

    // check row 4
    P6OUT |= BIT4;
    col = checkCols();
    if (col!=-1) {
        pressed =  keys[3][col];
    }
    P6OUT &= ~BIT4;

    if (pressed != lastKey) {
        lastKey = pressed;
        return pressed;
    } else {
        return 'X';
    }
}

int checkCols() {
    // check inputs for rows on P6.3, P6.2, P6.1, P6.0 (rows 1-4)
    if (P6IN & BIT3) {
        return 0;
    } else if (P6IN & BIT2) {
        return 1;
    } else if (P6IN & BIT1) {
        return 2;
    } else if (P6IN & BIT0) {
        return 3;
    } else {
        return -1;
    }
}

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