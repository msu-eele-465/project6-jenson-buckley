#include "intrinsics.h"
#include "msp430fr2355.h"
#include <driverlib.h>
#include <math.h>
#include <string.h>

//------------------------------------------------SETUP------------------------------------------------
//-- KEYPAD
void setupKeypad();                         // Init
char readKeypad();                          // Checks for pressed keys on keypad
int checkCols();                            // Used internally by readKeypad()
char lastKey = 'X';                         // Used internally for debouncing

//-- ADC SAMPLING
void setupADC();                            // Init ADC on P5.2
void readAmbient();                         // Read value of LM19 using ADC into ambient_val
int ambient_val;                            // ADC code updated in ADC ISR

//-- CONVERSIONS
#define ADC_SCALER (3.3 / 4095.0)           // 3.3V / 2^12 - 1
float adc2c(int);                           // Convert ADC code to celcius temerature value for LM19
float twoscomp2c(int);                      // Convert 2s compliment to celcius temerature value for LM92

//-- SAMPLING TIMER
void setupSampleClock();                    // Setup clock on TB3 to sample ADC every 0.5s

//-- WINDOWED AVERAGING (x2)
int ambient_sensor_array[100];              // Array to store ambient LM19 values (ADC code) used to calculate average
int plant_sensor_array[100];                // Array to store plant LM92 values (converted 2s compliment) used to calculate average
int ambient_sensor_avg = 0;                 // LM19 ambinet sensor average in ADC code
int plant_sensor_avg = 0;                   // LM92 plant sensor average in ADC code
float ambient_c = 0.0;                      // LM19 ambient sensor average in celcius
float plant_c = 0.0;                        // LM92 ambient sensor average in celcius
unsigned int temp_buffer_cur = 0;           // Number of values used in average (when adc_filled == adc_buffer_length, average is accurate)
unsigned int temp_buffer_length = 3;        // Number of values to  be used in average: can be [1,100]
unsigned int soon_temp_buffer_length = 0;   // Holds number of values to be used in average while user is entering the number

//-- I2C MASTER
volatile uint8_t rx_byte_count = 0;         // Number of bytes received
volatile uint8_t rx_data[2];                // Array to hold received bytes
void setupI2C();                            // Setup I2C on P4.6 (SDA) and P4.7 (SCL)

//-- RTC (I2C INFO + vars?)
char read_rtc_bool = 0;                     // Value toggeled so RTC is read every other sampling interrupt (every 1s)
unsigned int readSeconds();                 // Read in seconds value from RTC
unsigned int seconds = 0;                   // Number of seconds elapsed

//-- LM92 (I2C INFO + vars?)
#define LM92_ADDR  0x48 
void readPlant();                           // Read value of LM92 using I2C into plant_val
volatile int plant_val;                     // readPlant() reads into this value

//-- PELTIER GPIO CONTROL
void setupPeltier();                        // Initializes P4.0 (26 - heat) and P4.1 (25 - cool) as outputs driven low
void updatePeltier(char *str);              // Update the state of the plant ("off", "heat", or "cool")

//-- LCD
#define LCD_RS BIT0     // Register Select
#define LCD_RW BIT1     // Read/Write
#define LCD_E  BIT2     // Enable
#define LCD_DATA P2OUT  // Data bus on Port 1
char message[] = "off     A:xx.x C03 xxxs P:xx.x C";   // 33 characters long, 16 first row, 16 top row, \0
void lcd_init();                            // Initialize the LCD display
void lcd_display_message(char *str);        // Display a 32 character message
void updateAmbientTemp(float);              // Update the ambient temperature displayed
void updatePlantTemp(float);                // Update the plant temperature displayed
void updateWidowSize(unsigned int);         // Updated the window size displayed
void updateSeconds(unsigned int);           // Update the seconds displayed
void updateSetPoint(unsigned int);          // Update the set point displayed
void delay(unsigned int count);                         // INTERNAL
void lcd_enable_pulse();                                // INTERNAL
void lcd_write_command(unsigned char cmd);              // INTERNAL
void lcd_write_data(unsigned char data);                // INTERNAL
void lcd_set_cursor(unsigned char address);             // INTERNAL
void lcd_display_string(char *str);                     // INTERNAL
void lcd_clear();                                       // INTERNAL

//-- TEMPERATURE CONTROL
int set_temp;                               // temperature setpoint to control to
int soon_set_temp;                          // temporary variable to store setpoint as it is entered
int bang_bang_bool = 0;                     // Control whether bang-bang control is active (1) or off (0)
int ambient_bool = 0;                       // Control whether set point is ambient (1) or user-set (0)

//-- STATE MACHINE
// STATE:
        // 0    Free (can press any button)
        // 1    Set Window Size (1)
        // 2    Match Input Temp (2)
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
    setupADC();

    //-- ADC CODE TO CELCIUS / FAHRENHEIT

    //-- SAMPLING TIMER
    setupSampleClock();

    //-- WINDOWED AVERAGING (x2)

    //-- I2C MASTER
    setupI2C(); 

    //-- RTC (I2C INFO + vars?)

    //-- LM92 (I2C INFO + vars?)

    //-- PELTIER GPIO CONTROL
    setupPeltier();
    updatePeltier("off");

    //-- LCD
    lcd_init();  
    message[14] = 223;                                     // set degree symbol character in first row
    message[30] = 223;                                     // set degree symbol character in second row          
    lcd_display_message(message);

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
            if (state == 0) {
                if (key_val == 'D') {
                    updatePeltier("off");                   // drive heat and cool pins low
                    bang_bang_bool = 0;                     // disable bang-bang control
                    memcpy(&message[0], "off     ", 8);     // update LCD to display "off"
                    lcd_display_message(message);           // display message
                }

                // HEAT
                else if (key_val == 'A') {
                    updatePeltier("heat");                  // drive heat high and cool low
                    seconds = 0;                            // reset 5min timer that resets the state
                    memcpy(&message[0], "heat    ", 8);     // update LCD to display "heat"
                    lcd_display_message(message);           // display message
                }

                // COOL
                else if (key_val == 'B') {
                    updatePeltier("cool");                  // drive heat low and cool high
                    seconds = 0;                            // reset 5min timer that resets the state
                    memcpy(&message[0], "cool    ", 8);     // update LCD to display "cool"
                    lcd_display_message(message);           // display message
                }

                // MATCH
                else if (key_val == 'C') {
                    bang_bang_bool = 1;                     // enable bang-bang control
                    ambient_bool = 1;                       // setpoint is ambient temperature
                    seconds = 0;                            // reset 5min timer that resets the state
                    memcpy(&message[0], "match   ", 8);     // update LCD to display "match"
                    lcd_display_message(message);           // display message
                }

                // SET WINDOW SIZE
                else if (key_val == '1') {
                    state = 1;                              // update state to input window size
                    updatePeltier("off");                   // drive heat low and cool low
                    bang_bang_bool = 0;                     // disable bang-bang control
                    memcpy(&message[0], "window  ", 8);     // update LCD to display "window"
                    lcd_display_message(message);           // display message
                    soon_temp_buffer_length = 0;            // reset temorary storage for buffer length
                }

                // MATCH SET TEMP
                else if (key_val == '2') {
                    state = 2;                              // update state to input set point
                    updatePeltier("off");                   // drive heat low and cool low
                    bang_bang_bool = 0;                     // disable bang-bang control
                    memcpy(&message[0], "set-init", 8);     // update LCD to display "set-init"
                    lcd_display_message(message);           // display message
                    soon_set_temp = 0;                      // reset temorary storage for set temperature
                }

            } else if (state == 1) {
                // logic for inputing window size
                if ((key_val >= '0') & (key_val <= '9')) {
                    soon_temp_buffer_length = soon_temp_buffer_length*10+(key_val-'0');
                } else if (key_val=='*') {
                    if ((soon_temp_buffer_length > 0) & (soon_temp_buffer_length < 100)) {    // update length of rolling average
                        temp_buffer_length = soon_temp_buffer_length;
                    } else {
                        temp_buffer_length = 3;
                    }
                    memset(ambient_sensor_array, 0, sizeof(ambient_sensor_array));          // clear collected values used for ambient average
                    memset(plant_sensor_array, 0, sizeof(plant_sensor_array));              // clear collected values used for plant average
                    ambient_sensor_avg = 0;                                                 // clear ambient average
                    plant_sensor_avg = 0;                                                   // clear plant average
                    temp_buffer_cur = 0;                                                    // reset counter for values used in average
                    memcpy(&message[0], "off     ", 8);                                     // update status display
                    updateWidowSize(temp_buffer_length);                                    // update window size display
                    updateAmbientTemp(-1.0);                                                // clear ambient temperature display
                    updatePlantTemp(-1.0);                                                  // clear ambient temperature display
                    lcd_display_message(message);                                           // update LCD

                    state = 0;              // set state to free
                    updatePeltier("off");   // drive heat and cool pins low
                    bang_bang_bool = 0;                     // disable bang-bang control
                }
            
            } else if (state == 2) {
                // logic for inputting set temperature
                if ((key_val >= '0') & (key_val <= '9')) {
                    soon_set_temp = soon_set_temp*10+(key_val-'0');
                } else if (key_val=='*') {
                    if ((soon_set_temp >= 0) & (soon_set_temp <= 100)) {    // update length of rolling average
                        set_temp = soon_set_temp;
                        memcpy(&message[0], "set:    ", 8);
                        updateSetPoint(set_temp);
                        lcd_display_message(message);
                        bang_bang_bool = 1;                                 // enable bang-bang control
                        ambient_bool = 0;                                   // setpoint is user-set temperature
                        seconds = 0;                                        // reset 5min timer that resets the state
                    } else {
                        memcpy(&message[0], "off     ", 8);
                        lcd_display_message(message);
                        updatePeltier("off");                               // disable peltier
                        bang_bang_bool = 0;                     // disable bang-bang control
                    }
                    state = 0;              // set state to free
                }
            
            } else {
                state = 0;
                // TODO:
                // drive heat low and cool low
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
    if(twos & (1 << 12)) {
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
        seconds++;
        if (seconds > 300) {
            seconds = 0;                            // reset seconds
            updatePeltier("off");                   // drive heat and cool pins low
            bang_bang_bool = 0;                     // disable bang-bang control
            memcpy(&message[0], "off     ", 8);     // update LCD to display "off"
            lcd_display_message(message);           // display message
        }
        updateSeconds(seconds);
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
    ambient_sensor_avg += (ambient_val-ambient_popped) / (int) temp_buffer_length;
    plant_sensor_avg += (plant_val-plant_popped)/ (int) temp_buffer_length;

    // keep track of how many values have been read for average
    if (temp_buffer_cur < temp_buffer_length) {
        temp_buffer_cur++;
    } else {
        // convert ambient to celcius and update message
        ambient_c = adc2c(ambient_sensor_avg);
        updateAmbientTemp(ambient_c);
        // convert plant to celcius
        plant_c = twoscomp2c(plant_sensor_avg);
        updatePlantTemp(plant_c);
    }

    // update LCD
    lcd_display_message(message);

    // bang-bang control
    if (bang_bang_bool == 1) {
        // calculate error
        float diff;
        if (ambient_bool == 1) {
            diff = ambient_c - plant_c;
        } else {
            diff = ((float) set_temp) - plant_c;
        }

        // update peltier
        if (diff > 0) {
            updatePeltier("heat");
        } else if (diff < 0) {
            updatePeltier("cool");
        } else {
            updatePeltier("off");
        }
    }

    // clear CCR0 IFG
    TB3CCTL0 &= ~CCIFG;
}

//-- WINDOWED AVERAGING (x2)

//-- I2C MASTER
void setupI2C(){
    UCB1CTLW0 |= UCSWRST;           // Put into software reset
    UCB1CTLW0 |= UCSSEL__SMCLK;     // Use SMCLK 
    UCB1BRW = 10;                   // Divide by 10 (100kHz)

    UCB1CTLW0 |= UCMODE_3 | UCMST;  // Master
    UCB1CTLW1 |= UCASTP_2;          // Autostop on byte count

    P4SEL1 &= ~(BIT6 | BIT7);       // SDA=P4.6, SCL=P4.7
    P4SEL0 |=  (BIT6 | BIT7);       // SDA=P4.6, SCL=P4.7

    UCB1TBCNT = 2;                  // Expect 2 bytes
    UCB1I2CSA = LM92_ADDR;          // LM92 address

    UCB1CTLW0 &= ~UCTR;             // RX mode
    UCB1CTLW0 &= ~UCSWRST;          // Take out of software reset

    UCB1IE |= UCRXIE0;              // Enable RX interrupt
}

#pragma vector = EUSCI_B1_VECTOR
__interrupt void EUSCI_B1_ISR(void) {
    switch (__even_in_range(UCB1IV, USCI_I2C_UCBIT9IFG)) {
        case USCI_I2C_UCRXIFG0:
            if (rx_byte_count < 2) {
                rx_data[rx_byte_count++] = UCB1RXBUF;
            }
            break;
        default:
            break;
    }
}

//-- RTC (I2C INFO + vars?)
unsigned int readSeconds() {
    // read in seconds value from RTC
    return 0;
}

//-- LM92 (I2C INFO + vars?)
// TODO:
void readPlant() {
    // read value of LM92 using I2C into plant_val
    
    // Step 1: Set pointer register to 0x00 (temp register)
    UCB1CTLW0 |= UCTR | UCTXSTT;        // TX mode, generate START
    while (!(UCB1IFG & UCTXIFG0));      // Wait for TX buffer ready
    UCB1TXBUF = 0x00;                   // Send pointer byte
    while (!(UCB1IFG & UCTXIFG0));      // Wait for it to finish
    UCB1CTLW0 |= UCTXSTP;               // Generate STOP
    while (UCB1CTLW0 & UCTXSTP);        // Wait for STOP to finish

    // Step 2: Read temp
    UCB1TBCNT = 2;                      // Set number of bytes to receive
    rx_byte_count = 0;                  // Reset number of bytes received
    UCB1CTLW0 &= ~UCTR;                 // RX mode
    UCB1CTLW0 |= UCTXSTT;               // Generate repeated START
    while (UCB1CTLW0 & UCTXSTT);        // Wait for it to finish
    while (UCB1CTLW0 & UCTXSTP);        // Wait for read to complete

    plant_val = (rx_data[0] << 5) | (rx_data[1] >> 3);

    return;
}

//-- PELTIER GPIO CONTROL
void setupPeltier() {
    // Setup outputs on P4.0 (26 - heat) and P4.1 (25 - cool)
    P4DIR |= BIT0 | BIT1;
    P4OUT &= ~(BIT0 | BIT1);
}

void updatePeltier(char *str) {
    if (strcmp(str, "off")==0) {
        P4OUT &= ~(BIT0 | BIT1);
    } else if (strcmp(str, "heat")==0) {
        P4OUT |= BIT0;
        P4OUT &= ~BIT1;
    } else if (strcmp(str, "cool")==0) {
        P4OUT &= ~BIT0;
        P4OUT |= BIT1;
    } else {
        P4OUT &= ~(BIT0 | BIT1);
    }
}

//-- LCD
void updateAmbientTemp(float ave) {
    if ((ave>=100.0) | (ave<0.0)) {
        message[10] = 'x';
        message[11] = 'x';
        message[13] = 'x';
    } else {
        unsigned int n = (int) (ave*100);
        unsigned int tens = n / 1000;
        unsigned int ones = (n - 1000*tens) / 100;
        unsigned int tenths = (n-1000*tens-100*ones) / 10;
        unsigned int hudredths = n-1000*tens-100*ones-10*tenths;
        message[10] = tens+48;
        message[11] = ones+48;
        message[13] = tenths+48;
    }
}

void updatePlantTemp(float ave) {
    if ((ave>=100.0) | (ave<0.0)) {
        message[26] = 'x';
        message[27] = 'x';
        message[29] = 'x';
    } else {
        unsigned int n = (int) (ave*100);
        unsigned int tens = n / 1000;
        unsigned int ones = (n - 1000*tens) / 100;
        unsigned int tenths = (n-1000*tens-100*ones) / 10;
        unsigned int hudredths = n-1000*tens-100*ones-10*tenths;
        message[26] = tens+48;
        message[27] = ones+48;
        message[29] = tenths+48;
    }
}

void updateWidowSize(unsigned int n) {
    unsigned int hundreds = n / 100;
    unsigned int tens = (n - 100*hundreds) / 10;
    unsigned int ones = (n-100*hundreds-10*tens);
    message[16] = tens+48;
    message[17] = ones+48;
}

void updateSeconds(unsigned int n) {
    unsigned int hundreds = n / 100;
    unsigned int tens = (n - 100*hundreds) / 10;
    unsigned int ones = (n-100*hundreds-10*tens);
    message[19] = hundreds+48;
    message[20] = tens+48;
    message[21] = ones+48;
}

void updateSetPoint(unsigned int n) {
    unsigned int hundreds = n / 100;
    unsigned int tens = (n - 100*hundreds) / 10;
    unsigned int ones = (n-100*hundreds-10*tens);
    message[4] = hundreds+48;
    message[5] = tens+48;
    message[6] = ones+48;
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