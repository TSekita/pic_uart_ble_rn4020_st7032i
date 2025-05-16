
// PIC16F18857 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
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
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config WRT = OFF        // UserNVM self-write protection bits (Write protection off)
#pragma config SCANE = not_available// Scanner Enable bit (Scanner module is not available for use)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (Program Memory code protection disabled)
#pragma config CPD = OFF        // DataNVM code protection bit (Data EEPROM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#define _XTAL_FREQ 32000000  //Internal oscillator Hz
#define BaudRate 115200      // bps
#define ST7032i_ADDR 0x7C // ST7032i slave address is 0x7C

volatile char uart_buffer[64];
volatile uint8_t uart_index = 0;

void UART_Init(void) {

    TRISCbits.TRISC6 = 0; // TX output
    TRISCbits.TRISC7 = 1; // RX input

    TX1STAbits.BRGH = 1; // High speed
    BAUD1CONbits.BRG16 = 1; // 16-bit baud rate

    uint16_t SP1BRG = (_XTAL_FREQ / (4 * BaudRate)) - 1;
    SP1BRGL = (uint8_t) (SP1BRG & 0xFF);
    SP1BRGH = (uint8_t) ((SP1BRG >> 8) & 0xFF);

    // PPS設定
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0; // PPSアンロック

    RC6PPS = 0x10; // TX
    RXPPS = 0x17; // RX

    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1; // PPSロック

    TX1STAbits.SYNC = 0; // 非同期モード
    RC1STAbits.SPEN = 1; // シリアルポート有効
    TX1STAbits.TXEN = 1; // 送信有効
    RC1STAbits.CREN = 1; // 連続受信有効
}

void __interrupt() ISR(void) {
    if (PIE3bits.RCIE && PIR3bits.RCIF) {
        PIR3bits.RCIF = 0;
        LATAbits.LATA0 = 1;
        // Overrun Error チェック
        if (RC1STAbits.OERR) {
            RC1STAbits.CREN = 0;
            RC1STAbits.CREN = 1;
            return; // データは破棄
        }

        char c = RC1REG;
        if (c == '\r' || c == '\n') {
            uart_buffer[uart_index] = '\0'; // 文字列終端
            uart_index = 0; // 次の文字列のためにリセット
        } else {
            if (uart_index < sizeof (uart_buffer) - 1) {
                uart_buffer[uart_index++] = c;
                uart_buffer[uart_index] = '\0';
            }
        }

    }
}

void UART_SendChar(char c) {
    while (!PIR3bits.TXIF); // 送信バッファ空き待ち
    TX1REG = c;
}

void UART_SendString(const char *str) {
    while (*str) {
        UART_SendChar(*str++);
    }
}

void UART_EnableInterrupt(void) {
    PIR3bits.RCIF = 0;
    PIE3bits.RCIE = 1; // 受信割り込み許可
    INTCONbits.PEIE = 1; // 周辺割り込み許可
    INTCONbits.GIE = 1; // 全体割り込み許可
}

void BLE_SendCmd(const char *cmd) {
    UART_SendString(cmd);
    UART_SendString("\r\n");
}

void BLE_Init(void) {
    __delay_ms(100);
    BLE_SendCmd("SF,1");
    __delay_ms(100);
    BLE_SendCmd("SS,00000001"); // Enable MLDP and Device Info
    BLE_SendCmd("SR,30000800"); // Support MLDP + Auto-enter MLDP
    BLE_SendCmd("SN,MyBLE");
    BLE_SendCmd("R,1"); // Reset module
}

bool BLE_StartAdvertising(void) {
    __delay_ms(1000); // RN4020 再起動後の安定時間
    BLE_SendCmd("A"); // アドバタイズ開始
}

// I2C initialization

void I2C1_Init(void) {
    // PPS: RC3 = SCL, RC4 = SDA
    RC3PPS = 0x14; // SCL output
    RC4PPS = 0x15; // SDA output
    SSP1CLKPPS = 0x13; // SCL input
    SSP1DATPPS = 0x14; // SDA input

    // Configure I2C pins
    TRISCbits.TRISC3 = 1; // input
    TRISCbits.TRISC4 = 1; // input
    ANSC3 = 0; // digital
    ANSC4 = 0; // digital

    // Configure MSSP module as I2C Master, 100kHz
    SSP1CON1 = 0x28; // I2C master mode
    SSP1CON2 = 0x00;
    SSP1ADD = (_XTAL_FREQ / (4 * 100000)) - 1;
}

// Send I2C start condition

void I2C1_Start(void) {
    SSP1CON2bits.SEN = 1;
    while (SSP1CON2bits.SEN);
}

// Send I2C stop condition

void I2C1_Stop(void) {
    SSP1CON2bits.PEN = 1;
    while (SSP1CON2bits.PEN);
}

// Send one byte over I2C

void I2C1_Write(uint8_t data) {
    SSP1BUF = data;
    while (SSP1STATbits.BF); // Wait for buffer empty
    while (SSP1CON2bits.ACKSTAT); // Wait for ACK
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
    __delay_ms(100); // Wait for LCD power-up

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

char UART_ReceiveChar(void) {
    while (!PIR3bits.RCIF);
    return RC1REG;
}

void main(void) {
    TRISAbits.TRISA0 = 0;
    LATAbits.LATA0 = 0;
    ANSELAbits.ANSA0 = 0;
    UART_Init();
    UART_EnableInterrupt();

    BLE_Init();
    __delay_ms(1000); // RN4020の再起動完了待ち（重要）
    if (!BLE_StartAdvertising()) {
        __delay_ms(1000);
    } // 明示的にアドバタイズを開始

    I2C1_Init();
    __delay_ms(200);

    ST7032i_Init();
    __delay_ms(200);

//    while (1) {
//        if (uart_index > 0) {
//            PIE3bits.RCIE = 0;
//
//            LCD_Command(0x01); // LCDクリア
//            __delay_ms(2);
//            LCD_Print(uart_buffer); // LCD表示
//
//            uart_index = 0;
//            uart_buffer[0] = '\0';
//
//            PIE3bits.RCIE = 1;
//        }
//    }
    while (1) {
        char c = UART_ReceiveChar();
        LCD_Data(c);
    }
}
