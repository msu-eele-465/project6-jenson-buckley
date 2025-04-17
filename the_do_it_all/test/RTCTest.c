#include "intrinsics.h"
#include "msp430fr2355.h"
#include <stdint.h>

//------------------------------------------------SETUP------------------------------------------------

//-- I2C MASTER
volatile uint8_t rx_byte_count = 0;
volatile uint8_t rx_data[19];
void setupI2C();

//-- DS3231 (I2C INFO + vars?)
#define DS3231_ADDR  0x68

//-- UART
unsigned int position;
char message[] = "Time: 00:00:00\n";
void setupUART(); 
void printTime();


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

    //-- I2C MASTER
    setupI2C();

    //-- UART
    setupUART();   


    //----------------------------------------------END INITIALIZATIONS----------------------------------------------

    // enable interrupts
    __enable_interrupt();

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();

    // Take eUSCI_A1 out of software reset
    UCA1CTLW0 &= ~UCSWRST;  

    //------------------------------------------------STATE MACHINE------------------------------------------------
    while(1)
    {
        // Toggle LEDs
        P1OUT ^= BIT0;
        P6OUT ^= BIT6;

        // Step 1: Set pointer register to 0x00 (Seconds Register)
        UCB1CTLW0 |= UCTR | UCTXSTT;         // TX mode, generate START
        while (!(UCB1IFG & UCTXIFG0));       // Wait for TX buffer ready
        UCB1TXBUF = 0x00;                    // Send pointer byte
        while (!(UCB1IFG & UCTXIFG0));       // Wait for it to finish
        UCB1CTLW0 |= UCTXSTP;                // Generate STOP
        while (UCB1CTLW0 & UCTXSTP);      // Wait for STOP to finish

        // Step 2: Read 19 bytes (HH:MM:SS)
        rx_byte_count = 0;
        UCB1TBCNT = 19;                      // Set byte count before starting transaction
        UCB1CTLW0 &= ~UCTR;                 // RX mode
        UCB1CTLW0 |= UCTXSTT;               // Generate repeated START
        while (UCB1CTLW0 & UCTXSTT);        // Wait for START to finish
        while (!(UCB1IFG & UCSTPIFG));      // Wait for STOP to be generated
        UCB1IFG &= ~UCSTPIFG;               // Clear STOP flag

        // Write raw BCD bytes into message (2 hex characters each)
        message[6]  = "0123456789ABCDEF"[rx_data[2] >> 4];
        message[7]  = "0123456789ABCDEF"[rx_data[2] & 0x0F];
        message[9]  = "0123456789ABCDEF"[rx_data[1] >> 4];
        message[10] = "0123456789ABCDEF"[rx_data[1] & 0x0F];
        message[12] = "0123456789ABCDEF"[rx_data[0] >> 4];
        message[13] = "0123456789ABCDEF"[rx_data[0] & 0x0F];

        printTime();

        // Delay
        __delay_cycles(1000000);
    }
    //----------------------------------------------END STATE MACHINE----------------------------------------------
}

//------------------------------------------------FUNCTIONS AND ISRS------------------------------------------------

//-- I2C MASTER
void setupI2C(){
    UCB1CTLW0 |= UCSWRST;
    UCB1CTLW0 |= UCSSEL__SMCLK;
    UCB1BRW = 10;

    UCB1CTLW0 |= UCMODE_3 | UCMST;
    UCB1CTLW1 |= UCASTP_2;          // Autostop on byte count

    P4SEL1 &= ~(BIT6 | BIT7);
    P4SEL0 |=  (BIT6 | BIT7);       // SDA=P4.6, SCL=P4.7

    UCB1TBCNT = 3;                  // Expect 2 bytes
    UCB1I2CSA = DS3231_ADDR;               // LM92 address

    UCB1CTLW0 &= ~UCTR;             // RX mode
    UCB1CTLW0 &= ~UCSWRST;

    UCB1IE |= UCRXIE0;              // Enable RX interrupt
}

#pragma vector = EUSCI_B1_VECTOR
__interrupt void EUSCI_B1_ISR(void) {
    switch (__even_in_range(UCB1IV, USCI_I2C_UCBIT9IFG)) {
        case USCI_I2C_UCRXIFG0:
            if (rx_byte_count < 19) {
                rx_data[rx_byte_count++] = UCB1RXBUF;
            }
            break;
        default:
            break;
    }
}

//-- UART
void setupUART(){
    UCA1CTLW0 |= UCSWRST;       // Reset
    UCA1CTLW0 |= UCSSEL__SMCLK;  // Use 1M clock
    UCA1BRW = 8;                // Set to low freq baud rate mode (divides by 8)
    UCA1MCTLW |= 0xD600;        // Gets really close to 115200

    P4SEL1 &= ~BIT3;            // Config P4.3 to use UCATDX
    P4SEL0 |= BIT3; 
}

void printTime(){
    position = 0;
    UCA1IE |= UCTXCPTIE;        // Enable interrupt
    UCA1IFG &=~ UCTXCPTIFG;     // Clear flag
    UCA1TXBUF = message[0];     // Start at first name
}

#pragma vector=EUSCI_A1_VECTOR
__interrupt void ISR_EUSCI_A1(void)
{
    // If message sent or space is reached, disable TX interrupts
    if(message[position+1] == '\0' || position >= sizeof(message) - 1) {
        UCA1IE &=~ UCTXCPTIE;
    }
    else {
        position++;                     // Inc position and transmit
        UCA1TXBUF = message[position];
    }
    UCA1IFG &=~ UCTXCPTIFG;
}


//--------------------------------------------END FUNCTIONS AND ISRS--------------------------------------------