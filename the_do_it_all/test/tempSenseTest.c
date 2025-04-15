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
#define LCD_RS BIT0     // Register Select
#define LCD_RW BIT1     // Read/Write
#define LCD_E  BIT2     // Enable
#define LCD_DATA P2OUT  // Data bus on Port 1
char message[] = "LOCKED                          ";    // 33 characters long, 16 first row, 16 top row, \0
void delay(unsigned int count);
void lcd_enable_pulse();
void lcd_write_command(unsigned char cmd);
void lcd_write_data(unsigned char data);
void lcd_init();
void lcd_set_cursor(unsigned char address);
void lcd_display_string(char *str);
void lcd_display_message(char *str);
void lcd_clear();

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
    lcd_init();            

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
        int i;
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
    P6OUT &= ~BIT4;
    P6OUT &= ~(BIT6 | BIT5 | BIT4);

    // rows as inputs pulled down internally on P6.3, P6.2, P6.1, P6.0
    P6DIR &= ~(BIT3 | BIT2 | BIT1 | BIT0);     // inputs
    P6REN |= BIT3 | BIT2 | BIT1 | BIT0;         // internal resistors
	P6OUT &=~ BIT3 | BIT2 | BIT1 | BIT0;        // pull-downs

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
void delay(unsigned int count) {
    while(count--) __delay_cycles(1000);
}

void lcd_enable_pulse() {
    P3OUT |= LCD_E;
    delay(1);
    P3OUT &= ~LCD_E;
}

void lcd_write_command(unsigned char cmd) {
    P3OUT &= ~LCD_RS;  // RS = 0 for command
    P3OUT &= ~LCD_RW;  // RW = 0 for write
    LCD_DATA = cmd;    // Write command to data bus
    lcd_enable_pulse();
    delay(1);         // Command execution delay
}

void lcd_write_data(unsigned char data) {
    P3OUT |= LCD_RS;   // RS = 1 for data
    P3OUT &= ~LCD_RW;  // RW = 0 for write
    LCD_DATA = data;   // Write data to data bus
    lcd_enable_pulse();
    delay(1);         // Data write delay
}

void lcd_init() {
    P3DIR |= LCD_RS | LCD_RW | LCD_E;
    P2DIR |= 0xFF;   // Set Port 1 as output for data bus

    delay(50);    // Power-on delay

    lcd_write_command(0x38); // Function set: 8-bit, 2 lines, 5x8 dots
    lcd_write_command(0x0C); // Display ON, Cursor OFF
    lcd_clear();
    lcd_write_command(0x06); // Entry mode set: Increment cursor
}

void lcd_set_cursor(unsigned char address) {
    lcd_write_command(0x80 | address);
    delay(1);
}

void lcd_display_string(char *str) {
    while(*str) {
        lcd_write_data(*str++);
    }
}

void lcd_clear(){
    lcd_write_command(0x01); // Clear display
    delay(1);             // Delay for clear command
}

void lcd_display_message(char *str ){ //Untested
    int i;
    lcd_set_cursor(0x00);
    for (i=0; i<16; i++){
        lcd_write_data(*str++);
    }

    lcd_set_cursor(0x40);
    for (i=0; i<16; i++){
        lcd_write_data(*str++);
    }
}

//-- STATE MACHINE

//--------------------------------------------END FUNCTIONS AND ISRS--------------------------------------------