//------------------------------------Header-----------------------------------//
// main.c
// P.Buckley, EELE-371, Lab 14.1 Tx
// Nov 4 2024
//----------------------------------End-Header---------------------------------//


//--------------------------------Initialization---------------------------------//
#include <msp430.h>

// Delay var for ISR
int i;

// Message and position for transmission interrupts
unsigned int position;
char message[] = "Peter Buckley";

//--------------------------------End-Initilization------------------------------//


//-------------------------------------Main--------------------------------------//
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer

    //-- Setup UART
    UCA1CTLW0 |= UCSWRST;       // Reset
    UCA1CTLW0 |= UCSSEL__SMCLK;  // Use 1M clock
    UCA1BRW = 8;                // Set to low freq baud rate mode (divides by 8)
    UCA1MCTLW |= 0xD600;        // Gets really close to 115200

    //-- Configure TX ports
    P4SEL1 &= ~BIT3;            // Config P4.3 to use UCATDX
    P4SEL0 |= BIT3;

    //-- Configure LEDs
    P1DIR |= BIT0;      // P1.0 as output
    P1OUT &=~ BIT0;
    P6DIR |= BIT6;      // P6.6 as output
    P6OUT &=~ BIT6;

    //-- Configure SWs
    P4DIR &=~ BIT1;     // SW1 as an input with pull up resistor, falling edge sensitive
    P4REN |= BIT1;
    P4OUT |= BIT1;
    P4IES |= BIT1;
    P4IFG &=~ BIT1;     // Clear interrupt flag
    P4IE |= BIT1;       // Enable interrupt

    P2DIR &=~ BIT3;     // SW2 as an input with pull up resistor, falling edge sensitive
    P2REN |= BIT3;
    P2OUT |= BIT3;
    P2IES |= BIT3;
    P2IFG &=~ BIT3;     // Clear interrupt flag
    P2IE |= BIT3;       // Enable interrupt

    __enable_interrupt();   // Global interrupt enable
    PM5CTL0 &= ~LOCKLPM5;   // Enable GPIO
    UCA1CTLW0 &= ~UCSWRST;  // Take eUSCI_A1 out of software reset

    while(1) {}
    return 0;
}

//------------------------------------End-Main-----------------------------------//


//-----------------------------Interupt-Service-Routines-----------------------------//
#pragma vector = PORT4_VECTOR
__interrupt void ISR_S1_FIRST(void)
{
    P4IFG &=~ BIT1;     // Clear SW flag
    P1OUT ^= BIT0;     // Toggle LED1

    position = 0;
    UCA1IE |= UCTXCPTIE;        // Enable interrupt
    UCA1IFG &=~ UCTXCPTIFG;     // Clear flag
    UCA1TXBUF = message[0];     // Start at first name
}

#pragma vector = PORT2_VECTOR
__interrupt void ISR_S2_LAST(void)
{
    P2IFG &=~ BIT3;     // Clear flag
    P6OUT ^= BIT6;     // Toggle LED2

    position = 6;
    UCA1IE |= UCTXCPTIE;        // Enable interrupt
    UCA1IFG &=~ UCTXCPTIFG;     // Clear flag
    UCA1TXBUF = message[6];     // Start message at last name
}

#pragma vector=EUSCI_A1_VECTOR
__interrupt void ISR_EUSCI_A1(void)
{
    // If message sent or space is reached, disable TX interrupts
    if(message[position+1] == ' ' || position == sizeof(message) - 1) {
        UCA1IE &=~ UCTXCPTIE;
    }
    else {
        position++;                     // Inc position and transmit
        UCA1TXBUF = message[position];
    }
    UCA1IFG &=~ UCTXCPTIFG;
}

//---------------------------End-Interupt-Service-Routines---------------------------//
