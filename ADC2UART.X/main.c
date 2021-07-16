// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)


#include <xc.h>
#include <stdint.h>

#define ADC_CH1 0x03 // AN3 @RA4 PIN3
#define ADC_CH2 0x02 // AN2 @RA2 PIN5
#define ADC_TEMP 0x1D // Temperature Indicator

#define ADC_CH1_VREF 0x01; // VREF @ 1.024V
#define ADC_CH2_VREF 0x02; // VREF @ 2.048V

void delay(uint16_t ms);

void UART_send_char(uint8_t data);
void UART_send_nah(uint8_t nibble); // sends nibble as ASCII HEX
void UART_send_2nah(uint8_t byte); // sends as 2 nibble ASCII HEX
void UART_send_uint16_4dec(uint16_t val);

uint8_t adc_ch1_val_msb = 0;
uint8_t adc_ch1_val_lsb = 0;
uint8_t adc_ch2_val_msb = 0;
uint8_t adc_ch2_val_lsb = 0;

uint8_t adc_temp_val_msb = 0; // temperature value
uint8_t adc_temp_val_lsb = 0; // temperature value

void main() {
    OSCCON = 0x73; // PLL off, 8MHz int osc
    while (OSCSTATbits.HFIOFS == 0) {}
    
    PORTA = 0x00;
    LATA = 0x00;
    TRISA = 0xFF;
    TRISAbits.TRISA0 = 0; // UART TX pin
    
    LATAbits.LATA5 = 1; // MAX487 transmit mode
    TRISAbits.TRISA5 = 0; // MAX487 mode pin as output
    
    TXSTA = 0x84;
    RCSTA = 0x80;
    BAUDCON = 0x00;
    
    TXSTAbits.BRGH = 1;
    BAUDCONbits.BRG16 = 1;
    
    SPBRG = 0x67; // 103dec
    
    TXSTAbits.TXEN = 1;
    
    // setup 1mS delay timer
    OPTION_REGbits.TMR0CS = 0; //internal instruction cycle clock
    OPTION_REGbits.PSA = 0; //prescaler is to timer 0 module
    OPTION_REGbits.PS = 0x2; // prescaler is 8 which gives @ 250 timer clocks => 1mS
    
    // setup ADC
    ADCON0 = 0x00; // reset and disable ADC
    ADCON1 = (0x80 | 0x03); // size most significant bits of ADRESH are set to '0', VREF -> internal FVR
    ADCON1bits.ADCS = 0x05; // Fosc/16
    ADCON0bits.ADON = 1;
    
    FVRCON = 0xF0; // FVR enabled, temperature indicator enabled low range, both VREF outputs off
    delay(10); // 10mS
    
    while (1) {
        
        FVRCONbits.ADFVR = ADC_CH1_VREF;
        ADCON0bits.CHS = ADC_CH1;
        delay(1); // 1mS
        ADCON0bits.ADGO = 1; // starts conversion
        while (ADCON0bits.ADGO == 1) {} // wait for conversion to complete
        adc_ch1_val_msb = ADRESH;
        adc_ch1_val_lsb = ADRESL;
        
        FVRCONbits.ADFVR = ADC_CH2_VREF;
        ADCON0bits.CHS = ADC_CH2;
        delay(1); // 1mS
        ADCON0bits.ADGO = 1; // starts conversion
        while (ADCON0bits.ADGO == 1) {} // wait for conversion to complete
        adc_ch2_val_msb = ADRESH;
        adc_ch2_val_lsb = ADRESL;
        
        /* temp sensor don't work
        ADCON0bits.CHS = 0x1F; // 1F is FVR buffer output
        delay(1); // 1mS
        ADCON0bits.ADGO = 1; // starts conversion
        while (ADCON0bits.ADGO == 1) {} // wait for conversion to complete
        adc_temp_val_msb = ADRESH;
        adc_temp_val_lsb = ADRESL;
        */
        
        UART_send_char('@'); // start of message
        UART_send_char('S'); // slave response
        UART_send_char('P'); // unique id "Pressure"
        UART_send_char('S'); // unique id "Sensor"
        /*UART_send_nah(adc_ch1_val_msb);
        UART_send_2nah(adc_ch1_val_lsb);
        UART_send_char(' '); // space for easier debug read
        UART_send_nah(adc_ch2_val_msb);
        UART_send_2nah(adc_ch2_val_lsb);
        */
        UART_send_char(' '); // space for easier debug read
        UART_send_uint16_4dec(adc_ch1_val_msb * 256 + adc_ch1_val_lsb); // is in 1mV step
        UART_send_char(' '); // space for easier debug read
        UART_send_uint16_4dec((adc_ch2_val_msb * 256 + adc_ch2_val_lsb)*2); // *2 as each value represents 2mV
        /* temp sensor don't work
        UART_send_char(' '); // space for easier debug read
        UART_send_nah(adc_temp_val_msb);
        UART_send_2nah(adc_temp_val_lsb);
         */
        
        UART_send_char(0x0D); // end of message
        UART_send_char(0x0A); // end of message
        
        delay(1000);
        
    }
}

void delay(uint16_t ms) {
    
    while (ms != 0) {
        INTCONbits.TMR0IF = 0;
        TMR0 = 0x05; // gives 1mS at 8 prescaler @ fosc 8MHz
        asm("NOP");
        while (INTCONbits.TMR0IF == 0) {}
        ms--;
    }
}

void UART_send_char(uint8_t data) {
    TXREG = data;
    asm("NOP");
    while (TXSTAbits.TRMT == 0) { };
}

void UART_send_nah(uint8_t nibble) {
    nibble &= 0x0F;
    if (nibble < 10) UART_send_char(nibble + 0x30);
    else UART_send_char(nibble + 0x37);
}

void UART_send_2nah(uint8_t byte) { // sends as 2 nibble ASCII HEX
    
    // send_nah:s have nibble &= 0x0F; internally
    UART_send_nah(byte >> 4);
    UART_send_nah(byte);
}

void UART_send_uint16_4dec(uint16_t val) {
    uint8_t toSend = val / 1000;
    val = val % 1000;
    UART_send_char(toSend + 0x30);
    toSend = val / 100;
    val = val % 100;
    UART_send_char(toSend + 0x30);
    toSend = val / 10;
    val = val % 10;
    UART_send_char(toSend + 0x30);
    toSend = val;
    UART_send_char(toSend + 0x30);
}