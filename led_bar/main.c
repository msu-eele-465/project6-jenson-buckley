#include "intrinsics.h"
#include <msp430fr2310.h>
#include <stdbool.h>

//------------------------------------------------SETUP------------------------------------------------
//-- LED BAR
int stepIndex = 0;                  // Current step index
int stepStart = 0;                  // Start of the selected pattern
int seqLength = 1;                  // Length of selected sequence
int basePeriod = 128;               // Default base period
unsigned char stepSequence[] = {
                // Pattern 0 - OFF
                0b00000000,
                0b00000000,
                // Pattern 1 - HEAT
                0b0,
                0b1,
                0b11,
                0b111,
                0b1111,
                0b11111,
                0b111111,
                0b1111111,
                0b11111111,
                // Pattern 2 - COOL
                0b11111111,
                0b1111111,
                0b111111,
                0b11111,
                0b1111,
                0b111,
                0b11,
                0b1,
                0b0,
                // Pattern 3 - WARNING
                0b0,
                0b11111111,
};
void setupLeds();

//----------------------------------------------END SETUP----------------------------------------------

int main(void) {

    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    //------------------------------------------------INITIALIZATIONS------------------------------------------------

    //-- 2-PIN LOGIC INPUT: P1.2 is COOL & P1.3 is HEAT
    P1DIR &= !(BIT2 | BIT3);    // input on P1.2 an 1.3
    P2REN |= (BIT1 | BIT3);     // enable internal resistors
    P2OUT &= !(BIT2 | BIT3);    // pull-down resistors

    //-- LED BAR
    setupLeds();

    //-- HEATBEAT LED
    // Set P2.0 LED
    P2DIR |= BIT0;
    P2OUT &= ~BIT0;
    //----------------------------------------------END INITIALIZATIONS----------------------------------------------

    // enable interrupts
    __enable_interrupt();

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    while(1) {}
}

//------------------------------------------------FUNCTIONS AND ISRS------------------------------------------------
//-- LED BAR
void setupLeds() {
    // Configure Leds (P1.1, P1.0, P2.7, P2.6, P1.4, P1.5, P1.6, P1.7)
    P1DIR |= BIT1 | BIT0 | BIT4 | BIT5 |BIT6 | BIT7;
    P2DIR |= BIT7 | BIT6;
    P1OUT &= ~(BIT1 | BIT0 | BIT4 | BIT5 |BIT6 | BIT7);
    P2OUT &= ~(BIT7 | BIT6);

    // Setup Timer 0 B3
    TB1CTL = TBSSEL__ACLK | MC__UP | TBCLR | ID__8; // SMCLK (1Mhz), Stop mode, clear timer, divide by 8
    TB1EX0 = TBIDEX__8 ;   // Extra division by 4
    TB1CCR0 = basePeriod;  // Set speed
    TB1CCTL0 |= CCIE;      // Enable compare interrupt
}

void setPattern(unsigned int a) {
    switch (a) {
        case 0:     // off
            stepStart = 0;
            seqLength = 2;
            break;

        case 1:     // heat
            stepStart = 2;
            seqLength = 9;
            break;

        case 2:     // cool
            stepStart = 11;
            seqLength = 9;
        break;
        
        case 3:     // warning (heat and cool active)
            stepStart = 20;
            seqLength = 2;
        break;
    }    
}

#pragma vector = TIMER1_B0_VECTOR
__interrupt void ISR_TB3_CCR0(void)
{
    // Toggle heartbeat LED
    P2OUT ^= BIT0;

    // Read inputs:
    //  P1.2        -> 2
    //  P1.3        -> 1
    //  P1.2 & P1.3 -> 3
    unsigned int value = ((P1IN & BIT2) >> 1) | ((P1IN & BIT3) >> 3);

    // Update pattern
    setPattern(value);

    // Update LEDs (P1.1, P1.0, P2.7, P2.6, P1.4, P1.5, P1.6, P1.7)
    P1OUT = ((stepSequence[stepIndex + stepStart] <<1 ) & 0b10) |
            ((stepSequence[stepIndex + stepStart] >>1 ) & 0b1) |
            ((stepSequence[stepIndex + stepStart]) & 0b10000) |
            ((stepSequence[stepIndex + stepStart]) & 0b100000) |
            ((stepSequence[stepIndex + stepStart]) & 0b1000000) |
            ((stepSequence[stepIndex + stepStart]) & 0b10000000); // LSB (0) to P1.1 | 1 to P1.0 | 4 to P1.4 | 5 to P1.5 | 6 to P1.6 | 7 to P1.7
    P2OUT = ((stepSequence[stepIndex + stepStart] <<5 ) & 0b10000000) |
            ((stepSequence[stepIndex + stepStart] <<3 ) & 0b1000000);  // 2 to P2.7 | 3 to P2.6

    // Update pattern index
    stepIndex = (stepIndex + 1) % seqLength;

    // Clear flag
    TB1CCTL0 &= ~CCIFG;  // clear CCR0 IFG
}

//--------------------------------------------END FUNCTIONS AND ISRS--------------------------------------------