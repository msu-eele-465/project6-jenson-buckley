//------------------------------------Header-----------------------------------//
// main.c
// P.Buckley, LM92 Temp Print w/ UART TX Interrupt
// April 2025
//----------------------------------End-Header---------------------------------//

#include <msp430.h>
#include <stdint.h>
#include <stdio.h>

//---------------------------- Global Variables ----------------------------//
unsigned int position = 0;
char uartBuf[32] = "LM92 Test\r\n";

//---------------------------- Function Prototypes ----------------------------//
void setupUART();
void setupI2C();
void startUARTMessage(const char* msg);
void formatTemperature(float tempC);

//---------------------------- Main Program ----------------------------//
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;
    PM5CTL0 &= ~LOCKLPM5;

    setupUART();
    setupI2C();
    __enable_interrupt();

    // Start UART message using interrupt
    startUARTMessage(uartBuf);

    while (1)
    {
        // Set LM92 pointer to 0x00
        UCB0CTLW0 |= UCTR | UCTXSTT;
        while (!(UCB0IFG & UCTXIFG0));
        UCB0TXBUF = 0x00;
        while (!(UCB0IFG & UCTXIFG0));
        UCB0CTLW0 |= UCTXSTP;
        while (UCB0CTLW0 & UCTXSTP);

        // Read 2 bytes
        UCB0CTLW0 &= ~UCTR;      // RX mode
        UCB0CTLW0 |= UCTXSTT;
        while (UCB0CTLW0 & UCTXSTT);

        while (!(UCB0IFG & UCRXIFG0));
        uint8_t msb = UCB0RXBUF;
        while (!(UCB0IFG & UCRXIFG0));
        uint8_t lsb = UCB0RXBUF;

        UCB0CTLW0 |= UCTXSTP;
        while (UCB0CTLW0 & UCTXSTP);

        int16_t raw = ((int16_t)msb << 8 | lsb) >> 3;
        float tempC = raw * 0.0625;

        formatTemperature(tempC);         // Format to uartBuf[]
        startUARTMessage(uartBuf);        // Kick off interrupt UART TX

        __delay_cycles(100000);          // Delay ~1s @ 1MHz
    }
}

//---------------------------- Setup UART (Interrupt-based) ----------------------------//
void setupUART() {
    UCA1CTLW0 |= UCSWRST;
    UCA1CTLW0 |= UCSSEL__SMCLK;
    UCA1BRW = 8;
    UCA1MCTLW = 0xD600;
    P4SEL1 &= ~BIT3;
    P4SEL0 |= BIT3;
    UCA1CTLW0 &= ~UCSWRST;
}

//---------------------------- Setup I2C ----------------------------//
void setupI2C() {
    UCB0CTLW0 |= UCSWRST;
    UCB0CTLW0 = UCSSEL__SMCLK | UCMODE_3 | UCMST | UCTR | UCSWRST;
    UCB0BRW = 10;                   // 100kHz
    UCB0I2CSA = 0x48;               // LM92 address
    UCB0CTLW1 |= UCASTP_2;
    P1SEL1 &= ~(BIT2 | BIT3);
    P1SEL0 |=  (BIT2 | BIT3);
    UCB0CTLW0 &= ~UCSWRST;
}

//---------------------------- Start UART TX via Interrupt ----------------------------//
void startUARTMessage(const char* msg) {
    int i;
    for (i = 0; i < sizeof(uartBuf); i++) uartBuf[i] = 0;  // Clear buffer
    int j = 0;
    while (msg[j] != '\0' && j < sizeof(uartBuf) - 1) {
        uartBuf[j] = msg[j];
        j++;
    }
    uartBuf[j] = '\0';

    position = 0;
    UCA1TXBUF = uartBuf[position];
    UCA1IE |= UCTXCPTIE;
    UCA1IFG &= ~UCTXCPTIFG;
}

//---------------------------- Format Temperature to uartBuf[] ----------------------------//
void formatTemperature(float tempC) {
    int whole = (int)tempC;
    int frac = (int)((tempC - whole) * 100);
    if (frac < 0) frac = -frac;
    if (whole < 0) {
        whole = -whole;
        snprintf(uartBuf, sizeof(uartBuf), "Temp: %d.%02d C\r\n", whole, frac);
    } else {
        snprintf(uartBuf, sizeof(uartBuf), "Temp: -%d.%02d C\r\n", whole, frac);
    }
}

//---------------------------- UART ISR ----------------------------//
#pragma vector=EUSCI_A1_VECTOR
__interrupt void EUSCI_A1_ISR(void)
{
    if (uartBuf[position + 1] == '\0') {
        UCA1IE &= ~UCTXCPTIE;  // Disable interrupt when done
    } else {
        position++;
        UCA1TXBUF = uartBuf[position];
    }
    UCA1IFG &= ~UCTXCPTIFG;
}
