// PIC16F18857 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (FSCM timer disabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will not cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31 // WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF         // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7  // WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC        // WDT input clock selector (Software Control)

// CONFIG4
#pragma config WRT = OFF          // UserNVM self-write protection bits (Write protection off)
#pragma config SCANE = not_available // Scanner Enable bit (Scanner module is not available for use)
#pragma config LVP = ON           // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF           // UserNVM Program memory code protection bit (Program Memory code protection disabled)
#pragma config CPD = OFF          // DataNVM code protection bit (Data EEPROM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 32000000  // Internal oscillator Hz
#define BaudRate 115200      // bps
#define ST7032i_ADDR 0x7C    // ST7032i slave address is 0x7C

#define BUFFER_SIZE 16
volatile char rx_buffer[BUFFER_SIZE];
volatile uint8_t rx_index = 0;
volatile uint8_t data_ready = 0;

void UART_Init(void) {
    TRISCbits.TRISC6 = 1; // TX output
    TRISCbits.TRISC7 = 1; // RX input
    ANSELCbits.ANSC6 = 0; // TX digital
    ANSELCbits.ANSC7 = 0; // RX digital

    TX1STAbits.BRGH = 1; // High speed
    BAUD1CONbits.BRG16 = 1; // 16-bit baud rate

    uint16_t SP1BRG = (_XTAL_FREQ / (4 * BaudRate)) - 1;
    SP1BRGL = (uint8_t) (SP1BRG & 0xFF);
    SP1BRGH = (uint8_t) ((SP1BRG >> 8) & 0xFF);

    // PPS setting
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0; // Unlock PPS

    RC6PPS = 0x10; // TX
    RXPPS = 0x17; // RX

    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1; // Lock PPS

    TX1STAbits.SYNC = 0; // Asynchronous mode
    RC1STAbits.SPEN = 1; // Enable serial port
    TX1STAbits.TXEN = 1; // Enable transmission
    RC1STAbits.CREN = 1; // Enable continuous receive
    TX1STAbits.TX9 = 0;
    RC1STAbits.RX9 = 0; // 8-bit receive (standard)

    PIE3bits.RCIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
}

void UART_SendChar(char c) {
    while (!TRMT); // Wait for transmit buffer to be empty
    TX1REG = c;
}

void UART_SendString(const char *str) {
    while (*str) {
        UART_SendChar(*str++);
    }
}

void BLE_SendCmd(const char *cmd) {
    UART_SendString(cmd);
    UART_SendString("\r\n");
}

char uart_getc(void) {
    if ((RC1STAbits.OERR) || (RC1STAbits.FERR)) {
        RC1STA = 0;
        RC1STA = 0x90;
        return '\0';
    }
    return RC1REG;
}

void BLE_Init(void) {
    __delay_ms(100);
    BLE_SendCmd("SF,1");
    __delay_ms(100);
    BLE_SendCmd("SS,80000000"); // Enable MLDP and Device Info
    BLE_SendCmd("SR,32000800"); // Support MLDP + Auto-enter MLDP
    BLE_SendCmd("SN,Mr.BLE");
    BLE_SendCmd("R,1"); // Reset module
}

// I2C initialization

void I2C1_Init(void) {
    // PPS: RC3 = SCL, RC4 = SDA
    RC3PPS = 0x14; // SCL output
    RC4PPS = 0x15; // SDA output
    SSP1CLKPPS = 0x13; // SCL input
    SSP1DATPPS = 0x14; // SDA input

    // Configure I2C pins
    TRISCbits.TRISC3 = 1; // Input
    TRISCbits.TRISC4 = 1; // Input
    ANSC3 = 0; // Digital
    ANSC4 = 0; // Digital

    // Configure MSSP module as I2C Master, 400kHz
    SSP1CON1 = 0x28; // I2C master mode
    SSP1CON2 = 0x00;
    SSP1CON3 = 0x00;
    SSP1ADD = (_XTAL_FREQ / (4 * 100000)) - 1;
}

// Send I2C start condition

void I2C1_Start(void) {
    SSP1IF = 0;
    SSP1CON2bits.SEN = 1;
    while (SSP1IF == 0) {
    }
    SSP1IF = 0;
    return;
}

// Send I2C stop condition

void I2C1_Stop(void) {
    SSP1IF = 0;
    SSP1CON2bits.PEN = 1;
    while (SSP1IF == 0) {
    }
    SSP1IF = 0;
    return;
}

// Send one byte over I2C

void I2C1_Write(uint8_t data) {
    SSP1IF = 0;
    SSP1BUF = data;
    while (SSP1IF == 0) {
    }
    SSP1IF = 0;
    return;
}

// Send command to ST7032i

void LCD_Command(uint8_t cmd) {
    I2C1_Start();
    I2C1_Write(ST7032i_ADDR); // 8-bit address
    I2C1_Write(0x00); // Control byte: Co=0, RS=0 (command)
    I2C1_Write(cmd);
    I2C1_Stop();
    __delay_ms(2);
}

// Send data (character) to ST7032i

void LCD_Data(uint8_t data) {
    I2C1_Start();
    I2C1_Write(ST7032i_ADDR);
    I2C1_Write(0x40); // Control byte: Co=0, RS=1 (data)
    I2C1_Write(data);
    I2C1_Stop();
    __delay_us(50);
}

// Initialize ST7032i LCD

void ST7032i_Init(void) {
    __delay_ms(50); // Wait for LCD power-up

    LCD_Command(0x38); // Function set: 8-bit, 2-line, normal instruction
    LCD_Command(0x39); // Function set: extended instruction
    LCD_Command(0x14); // Internal OSC frequency
    LCD_Command(0x70); // Contrast set low byte
    LCD_Command(0x56); // Power/Icon/Contrast high byte
    LCD_Command(0x6C); // Follower control
    __delay_ms(200); // Wait for voltage stable

    LCD_Command(0x38); // Function set: normal instruction
    LCD_Command(0x0C); // Display ON, Cursor OFF, Blink OFF
    LCD_Command(0x01); // Clear display
    __delay_ms(2); // Wait for clear display
}

// Display string on LCD

void LCD_Print(const char *str) {
    while (*str) {
        LCD_Data(*str++);
    }
}

void __interrupt() isr(void) {
    if (PIR3bits.RCIF) {
        char c = RC1REG;
        if (rx_index < BUFFER_SIZE - 1) {
            rx_buffer[rx_index++] = c;
            if (c == '\n') {
                rx_buffer[rx_index] = '\0';
                data_ready = 1;
                rx_index = 0;
            }
        } else {
            rx_index = 0;
        }
    }
}

void main(void) {
    TRISAbits.TRISA0 = 0; // Set RA0 as output
    ANSELAbits.ANSA0 = 0;
    LATAbits.LATA0 = 0; // Initial state Low

    I2C1_Init();
    __delay_ms(200);
    ST7032i_Init();
    __delay_ms(200);

    LCD_Command(0x02);
    LCD_Print("Display");
    LCD_Command(0xC0); // Move to 2nd line (0x40 address)
    LCD_Print("Ready");
    __delay_ms(1000);

    UART_Init();
    __delay_ms(500);
    BLE_Init();
    __delay_ms(200);
    while (1) {
        if (data_ready) {
            LCD_Command(0x01); // Clear display
            __delay_ms(2);
            LCD_Command(0x02); // Return home
            LCD_Print((const char *) rx_buffer);
            data_ready = 0;
        }
    }
}
