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
void setupADC();                            // init ADC on P5.2
void readAmbient();                         // read value of LM19 using ADC into ambient_val
int ambient_val;                            // ADC code updated in ADC ISR

//-- CONVERSIONS
#define ADC_SCALER (3.3 / 4095.0)           // 3.3V / 2^12 - 1
float adc2c(int);                           // convert ADC code to celcius temerature value for LM19
float twoscomp2c(int);                      // convert 2s compliment to celcius temerature value for LM92

//-- SAMPLING TIMER
void setupSampleClock();                    // setup clock on TB3 to sample ADC every 0.5s

//-- WINDOWED AVERAGING (x2)
unsigned int ambient_sensor_array[100];     // array to store ambient LM19 values (ADC code) used to calculate average
unsigned int plant_sensor_array[100];       // array to store plant LM92 values (converted 2s compliment) used to calculate average
unsigned int ambient_sensor_avg = 0;        // LM19 ambinet sensor average in ADC code
unsigned int plant_sensor_avg = 0;          // LM92 plant sensor average in ADC code
unsigned int temp_buffer_cur = 0;           // number of values used in average (when adc_filled == adc_buffer_length, average is accurate)
unsigned int temp_buffer_length = 3;        // number of values to  be used in average: can be [1,100]
unsigned int soon_temp_buffer_length = 0;   // holds number of values to be used in average while user is entering the number
unsigned int window_tens = 0;               // used to input multiple-digit number for window size

//-- I2C MASTER

//-- RTC (I2C INFO + vars?)
char read_rtc_bool = 0;                     // value toggeled so RTC is read every other sampling interrupt (every 1s)
unsigned int readSeconds();                 // read in seconds value from RTC

//-- LM92 (I2C INFO + vars?)
void readPlant();                           // read value of LM92 using I2C into plant_val
int plant_val;                              // readPlant() reads into this value

//-- PELTIER GPIO CONTROL

//-- LCD
#define LCD_RS BIT0     // Register Select
#define LCD_RW BIT1     // Read/Write
#define LCD_E  BIT2     // Enable
#define LCD_DATA P2OUT  // Data bus on Port 1
char message[] = "off     A:xx.x CN xxxs  P:xx.x C ";   // 33 characters long, 16 first row, 16 top row, \0
void lcd_init();                        // Initialize the LCD display
void lcd_display_message(char *str);    // Display a 32 character message
void delay(unsigned int count);                         // INTERNAL
void lcd_enable_pulse();                                // INTERNAL
void lcd_write_command(unsigned char cmd);              // INTERNAL
void lcd_write_data(unsigned char data);                // INTERNAL
void lcd_set_cursor(unsigned char address);             // INTERNAL
void lcd_display_string(char *str);                     // INTERNAL
void lcd_clear();                                       // INTERNAL

//-- STATE MACHINE
// STATE:
        // 0    Off (D)
        // 1    Heat (A)
        // 2    Cool (B)
        // 3    Match Ambient (C)
        // 4    Set Window Size (1)
        // 5    Match Input Temp (2)
int state = 0;
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
    //setupADC();

    //-- ADC CODE TO CELCIUS / FAHRENHEIT

    //-- SAMPLING TIMER
    //setupSampleClock();

    //-- WINDOWED AVERAGING (x2)

    //-- I2C MASTER

    //-- RTC (I2C INFO + vars?)

    //-- LM92 (I2C INFO + vars?)

    //-- PELTIER GPIO CONTROL

    //-- LCD
    //lcd_init();  
    //message[14] = 0xB0;                                     // set degree symbol character in first row
    //message[30] = 0xB0;                                     // set degree symbol character in second row          
    //lcd_display_message(message);

    //-- STATE MACHINE
    char key_val = 'X';                                     // value read from keypad - 'X' if no new input read

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

        key_val = readKeypad();
        if (key_val != 'X') {

            // OFF
            if (key_val == 'D') {
                // TODO:
                // drive heat and cool pins low
                // update LCD to display "off"
                memcpy(&message[0], "off     ", 8);
                lcd_display_message(message);
            }

            // HEAT
            else if (key_val == 'A') {
                // TODO:
                // drive heat high and cool low
                // update LCD to display "heat"
                memcpy(&message[0], "heat    ", 8);
                lcd_display_message(message);
            }

            // COOL
            else if (key_val == 'B') {
                // TODO:
                // drive heat low and cool high
                // update LCD to display "cool"
                memcpy(&message[0], "cool    ", 8);
                lcd_display_message(message);
            }

            // MATCH
            else if (key_val == 'C') {
                // TODO:
                // enable controller with setpoint = ambient reading
                // update LCD to display "match"
                memcpy(&message[0], "match   ", 8);
                lcd_display_message(message);
            }

            // SET WINDOW SIZE
            else if (key_val == '1') {
                // TODO:
                // drive heat low and cool low
                // update LCD to display "window"
                memcpy(&message[0], "window  ", 8);
                lcd_display_message(message);
                // logic for inputting window size
                // save with '*'

                // update window size
                if ((key_val >= '0') & (key_val <= '9')) {
                    soon_temp_buffer_length = soon_temp_buffer_length*10+(key_val-'0');
                    window_tens++;
                } else if (key_val=='*') {
                    if ((soon_temp_buffer_length > 0) & (soon_temp_buffer_length < 101)) {    // update length of rolling average
                        temp_buffer_length = soon_temp_buffer_length;
                    } else {
                        temp_buffer_length = 3;
                    }
                    memset(ambient_sensor_array, 0, sizeof(ambient_sensor_array));          // clear collected values used for ambient average
                    memset(plant_sensor_array, 0, sizeof(plant_sensor_array));              // clear collected values used for plant average
                    ambient_sensor_avg = 0;                                                 // clear ambient average
                    plant_sensor_avg = 0;                                                   // clear plant average
                    temp_buffer_cur = 0;                                                    // reset counter for values used in average
                    updateWidowSize(temp_buffer_length);                                    // update window size display
                    
                    // reset state machine
                    state = 0;              // set state to OFF
                    // TODO:
                    // drive heat and cool pins low
                    // update LCD to display "off"
                }
            }

            // MATCH SET TEMP
            else if (key_val == '2') {
                // drive heat low and cool low
                // update LCD to display "set-init"
                memcpy(&message[0], "set-init", 8);
                lcd_display_message(message);
                // logic for inputting set temperature
                // save with '*'
                // update LCD to display "set"
                memcpy(&message[0], "set     ", 8);
                lcd_display_message(message);
            }
        }
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
    P6DIR &= ~(BIT3 | BIT2 | BIT1 | BIT0);      // inputs
    P6REN |= BIT3 | BIT2 | BIT1 | BIT0;         // internal resistors
	P6OUT &= ~(BIT3 | BIT2 | BIT1 | BIT0);        // pull-downs

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
void setupADC() {
    // Configure ADC A10 pin on P5.2
    P5SEL0 |= BIT2;
    P5SEL1 |= BIT2;

    // Configure ADC12
    ADCCTL0 |= ADCSHT_2 | ADCON;                             // ADCON, S&H=16 ADC clks
    ADCCTL1 |= ADCSHP;                                       // ADCCLK = MODOSC; sampling timer
    ADCCTL2 &= ~ADCRES;                                      // clear ADCRES in ADCCTL
    ADCCTL2 |= ADCRES_2;                                     // 12-bit conversion results
    ADCMCTL0 |= ADCINCH_10;                                   // A10 ADC input select; Vref=AVCC
    ADCIE |= ADCIE0;                                         // Enable ADC conv complete interrupt
}


void readAmbient() {
    // start sampling and conversion
    ADCCTL0 |= ADCENC | ADCSC;
}

#pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void)
{
    switch(ADCIV)
    {
        case ADCIV_ADCIFG:
            ambient_val = ADCMEM0;
            break;
        default:
            break;
    }
}

//-- CONVERSIONS
float adc2c(int code) {
    // 12-bit adc conversion with 3.3V reference
    float voltage = ((float) code) * ADC_SCALER;
    // Vo to C conversion from LM19 datasheet
    float celcius = -1481.96 + sqrt(2.1962e6 + (1.8639 - voltage) / (3.88e-6));
    return celcius;
}

float twoscomp2c(int twos) {
    // check for sign, extending as needed
    if(twos & (1 <<12)) {
        twos |= 0xFFFFE000;
    }
    // LSB is 0.0625 degC
    return ((float) twos) * 0.0625;
}

//-- SAMPLING TIMER
void setupSampleClock() {
    TB3CTL |= TBCLR;                            // reset settings
    TB3CTL |= TBSSEL__ACLK | MC__UP | ID__8;    // 32.768 kHz / 8 = 4096 / 2 - 1 = 2047
    TB3CCR0 = 2047;                             // period of .5s
    TB3CCTL0 |= CCIE;                           // Enable capture compare
    TB3CCTL0 &= ~CCIFG;                         // Clear IFG
}

#pragma vector = TIMER3_B0_VECTOR
__interrupt void ISR_TB0_CCR0(void)
{
    // read ambient temperature (analog LM19)
    readAmbient();

    // read plant temperature (I2C LM92)
    readPlant();

    // read RTC seconds (every other interrupt)
    read_rtc_bool ^= 1;
    if (read_rtc_bool) {
        // TODO: read RTC
    }

    // update ambient temperature array       
    int ambient_popped = ambient_sensor_array[temp_buffer_length-1];
    int i;
    for (i=temp_buffer_length-1; i>0; i--) {
        ambient_sensor_array[i] = ambient_sensor_array[i-1];
    }
    ambient_sensor_array[0] = ambient_val;

    // update plant temperature array       
    int plant_popped = plant_sensor_array[temp_buffer_length-1];
    for (i=temp_buffer_length-1; i>0; i--) {
        plant_sensor_array[i] = plant_sensor_array[i-1];
    }
    plant_sensor_array[0] = plant_val;

    // update averages with cool move
    ambient_sensor_avg += (ambient_val-ambient_popped)/temp_buffer_length;
    plant_sensor_avg += (plant_val-plant_popped)/temp_buffer_length;

    // keep track of how many values have been read for average
    if (temp_buffer_cur < temp_buffer_length) {
        temp_buffer_cur++;
    } else {
        // convert ambient to celcius and update message
        float ambient_c = adc2c(ambient_sensor_avg);
        updateAmbientTemp(ambient_c);
        // convert plant to celcius
        float plant_c = adc2c(plant_sensor_avg);
        updatePlantTemp(plant_c);
    }

    // update LCD
    lcd_display_message(message);

    // clear CCR0 IFG
    TB3CCTL0 &= ~CCIFG;
}

//-- WINDOWED AVERAGING (x2)

//-- I2C MASTER

//-- RTC (I2C INFO + vars?)
// TODO:
unsigned int readSeconds() {
    // read in seconds value from RTC
    return 0;
}

//-- LM92 (I2C INFO + vars?)
// TODO:
void readPlant() {
    // read value of LM92 using I2C into plant_val
    return;
}

//-- PELTIER GPIO CONTROL

//-- LCD
void updateAmbientTemp(float ave) {
    unsigned int n = (int) (ave*100);
    unsigned int tens = n / 1000;
    unsigned int ones = (n - 1000*tens) / 100;
    unsigned int tenths = (n-1000*tens-100*ones) / 10;
    unsigned int hudredths = n-1000*tens-100*ones-10*tenths;
    message[10] = tens+48;
    message[11] = ones+48;
    message[13] = tenths+48;
}

void updatePlantTemp(float ave) {
    unsigned int n = (int) (ave*100);
    unsigned int tens = n / 1000;
    unsigned int ones = (n - 1000*tens) / 100;
    unsigned int tenths = (n-1000*tens-100*ones) / 10;
    unsigned int hudredths = n-1000*tens-100*ones-10*tenths;
    message[26] = tens+48;
    message[27] = ones+48;
    message[29] = tenths+48;
}

void updateWidowSize(unsigned int n) {
    unsigned int hundreds = n / 100;
    unsigned int tens = (n - 100*hundreds) / 10;
    unsigned int ones = (n-100*hundreds-10*tens);
    message[28] = hundreds+48;
    message[29] = tens+48;
    message[30] = ones+48;
}


void delay(unsigned int count) {
    while(count--) __delay_cycles(1000);
}

void lcd_enable_pulse() {
    P3OUT |= LCD_E;
    delay(1);
    P3OUT &= ~LCD_E;
}

void lcd_write_command(unsigned char cmd) {
    P3OUT &= ~LCD_RS;   // RS = 0 for command
    P3OUT &= ~LCD_RW;   // RW = 0 for write
    LCD_DATA = cmd;     // Write command to data bus
    lcd_enable_pulse();
    delay(1);           // Command execution delay
}

void lcd_write_data(unsigned char data) {
    P3OUT |= LCD_RS;    // RS = 1 for data
    P3OUT &= ~LCD_RW;   // RW = 0 for write
    LCD_DATA = data;    // Write data to data bus
    lcd_enable_pulse();
    delay(1);           // Data write delay
}

void lcd_init() {
    P3DIR |= LCD_RS | LCD_RW | LCD_E;
    P2DIR |= 0xFF;      // Set Port 1 as output for data bus

    delay(50);          // Power-on delay

    lcd_write_command(0x38);    // Function set: 8-bit, 2 lines, 5x8 dots
    lcd_write_command(0x0C);    // Display ON, Cursor OFF
    lcd_clear();                // Clear
    lcd_write_command(0x06);    // Entry mode set: Increment cursor
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
    lcd_write_command(0x01);    // Clear display
    delay(1);                   // Delay for clear command
}

void lcd_display_message(char *str ){   // Write all 32 characters to display - UNTESTED
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