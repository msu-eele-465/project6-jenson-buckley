#include "gpio.h"
#include "intrinsics.h"
#include "msp430fr2310.h"
#include <driverlib.h>
#include <math.h>
#include <string.h>

//------------------------------------------------SETUP------------------------------------------------
//-- LED BAR

//-- 2-PIN LOGIC INPUT

//----------------------------------------------END SETUP----------------------------------------------

int main(void) {

    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    // Set P2.0 LED
    P2DIR |= BIT0;
    P2OUT &= ~BIT0;

    //------------------------------------------------INITIALIZATIONS------------------------------------------------
    //-- LED BAR

    //-- 2-PIN LOGIC INPUT

    //----------------------------------------------END INITIALIZATIONS----------------------------------------------

    // enable interrupts
    __enable_interrupt();

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();

    //------------------------------------------------STATE MACHINE------------------------------------------------
    while(1)
    {
        // Toggle LED
        P2OUT ^= BIT0;

        // Delay
        for(i=10000; i>0; i--);
    }
    //----------------------------------------------END STATE MACHINE----------------------------------------------
}

//------------------------------------------------FUNCTIONS AND ISRS------------------------------------------------
//-- LED BAR

//-- 2-PIN LOGIC INPUT

//--------------------------------------------END FUNCTIONS AND ISRS--------------------------------------------