/* 
 * File:   blinky.c
 * Author: john
 *
 * Created on 10 January 2019, 22:45
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>

// PIC32MX360F512L Configuration Bit Settings
 
// 'C' source line config statements
 
// DEVCFG3
// USERID = No Setting
 
// DEVCFG2
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_4         // System PLL Output Clock Divider (PLL Divide by 4)
#pragma config UPLLIDIV = DIV_4         // 
#pragma config UPLLEN = ON              // 
 
// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Fast RC Osc with PLL)
#pragma config FSOSCEN = OFF            // Secondary Oscillator Enable (Disabled)
#pragma config IESO = OFF               // Internal/External Switch Over (Disabled)
#pragma config POSCMOD = HS            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF            // CLKO Output Signal Active on the OSCO Pin (Enabled)
#pragma config FPBDIV = DIV_1           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/1)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT Disabled (SWDTEN Bit Controls))
 
// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = OFF             // 
#pragma config ICESEL = ICS_PGx2        // ICE/ICD Comm Channel Select (ICE EMUC2/EMUD2 pins shared with PGC2/PGD2)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)
 
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
 
#include <xc.h>
#include <sys/attribs.h>
 
#define LED1        LATEbits.LATE6
#define LED2        LATEbits.LATE7
#define LED3        LATEbits.LATE1
#define LED4        LATAbits.LATA7
#define LED5        LATAbits.LATA6

volatile uint32_t MilliSeconds = 0;
volatile uint32_t SPIword = 0;
volatile uint32_t PhaseAcc = 0;
volatile uint32_t PhaseInc = 1024 * 1024;
uint16_t Sinbuf[4096];

static void dally(const int loops)
{
    volatile int dally;
    
    for (dally = 0; dally < loops; dally++)
            ;
}

static void delayms(const uint32_t interval)
{
    const uint32_t now = MilliSeconds;
    
    while ((MilliSeconds - now) < interval)
        ;
}

static uint32_t millis(void)
{
    /* Arduino-like function to return milliseconds since start-up */
    return (MilliSeconds);
}

void __ISR(_TIMER_4_VECTOR, ipl1) Timer4Handler(void) 
{    
    static int flag = 0;
    int i;
    
    LATAINV = _LATA_LATA4_MASK; // Toggle P7 pin 6 (22050Hz)
    
    LATDCLR = _LATD_LATD9_MASK;   // Assert SS
    
    i = PhaseAcc >> 20;
    PhaseAcc += PhaseInc;
    
    SPI2BUF = (0 << 15) | (1 << 13) | (1 << 12) | Sinbuf[i];
    
    if (flag > 31)
    {
        PR4 = 907;
        flag = 0;
    }
    else
    {
        PR4 = 906;
        flag++;
    }
    
    IFS1CLR = _IFS1_SPI2RXIF_MASK;  // Clear SPI2 interrupt flag
    IEC1SET = _IEC1_SPI2RXIE_MASK;  // Enable SPI2 interrupt
    IFS0CLR = _IFS0_T4IF_MASK;  // Clear Timer 4 interrupt flag
}

void __ISR(_TIMER_1_VECTOR, ipl2) Timer1Handler(void) 
{
    MilliSeconds++;
    
    //LATAINV = _LATA_LATA4_MASK; // Toggle P7 pin 6 (500Hz))
    
    IFS0CLR = _IFS0_T1IF_MASK;  // Clear Timer 1 interrupt flag
}

void __ISR(_SPI_2_VECTOR, ipl3) SPI2Handler(void) 
{
    volatile uint32_t junk;
    
    junk = SPI2BUF;
    LATDSET = _LATD_LATD9_MASK;
    
    IFS1CLR = _IFS1_SPI2RXIF_MASK;  // Clear SPI2 interrupt flag
    IEC1CLR = _IEC1_SPI2RXIE_MASK;  // Disable SPI2 interrupt
}

static void UART1_begin(const int baud)
{
    /* Configure PPS pins */
    RPE5Rbits.RPE5R = 3;    // U1Tx on pin 3, RPE5
    U1RXRbits.U1RXR = 10;   // U1Rx on pin 6, RPC1
    
    /* Configure USART1 */
    U1MODEbits.UEN = 3;     // Use just Rx/Tx; no handshaking
    
    U1STAbits.UTXEN = 1;    // Enable Tx
    U1STAbits.URXEN = 1;    // Enable Rx (unused at present)
    
    U1BRG = (40000000 / (baud * 16)) - 1;
    
    U1MODESET = _U1MODE_ON_MASK;      // Enable USART1
}

static void UART2_begin(const int baud)
{
    /* Configure PPS pins */
    RPG0Rbits.RPG0R = 1;    // U2Tx on pin 90, RPG0
    
    /* Configure USART2 */
    U2MODEbits.UEN = 3;     // Use just Rx/Tx; no handshaking
    
    U2STAbits.UTXEN = 1;    // Enable Tx
    U2STAbits.URXEN = 1;    // Enable Rx (unused at present)
    
    U2BRG = (40000000 / (baud * 16)) - 1;
    
    U2MODESET = _U2MODE_ON_MASK;      // Enable USART2
}

static void UART3_begin(const int baud)
{
    /* Configure PPS pins */
    RPF1Rbits.RPF1R = 1;    // U3Tx on pin 88, RPF1
    
    /* Configure USART3 */
    U3MODEbits.UEN = 3;     // Use just Rx/Tx; no handshaking
    
    U3STAbits.UTXEN = 1;    // Enable Tx
    U3STAbits.URXEN = 1;    // Enable Rx (unused at present)
    
    U3BRG = (40000000 / (baud * 16)) - 1;
    
    U3MODESET = _U3MODE_ON_MASK;      // Enable USART3
}

static void UART4_begin(const int baud)
{
    /* Configure PPS pins */
    RPD4Rbits.RPD4R = 2;    // U4Tx on pin 81, RPD4
    
    /* Configure USART4 */
    U4MODEbits.UEN = 3;     // Use just Rx/Tx; no handshaking
    
    U4STAbits.UTXEN = 1;    // Enable Tx
    U4STAbits.URXEN = 1;    // Enable Rx (unused at present)
    
    U4BRG = (40000000 / (baud * 16)) - 1;
    
    U4MODESET = _U4MODE_ON_MASK;      // Enable USART4
}

static void UART5_begin(const int baud)
{
    /* Configure PPS pins */
    RPD12Rbits.RPD12R = 4;  // U5Tx on pin 79, RPD12
    
    /* Configure USART5 */
    U5MODEbits.UEN = 3;     // Use just Rx/Tx; no handshaking
    
    U5STAbits.UTXEN = 1;    // Enable Tx
    U5STAbits.URXEN = 1;    // Enable Rx (unused at present)
    
    U5BRG = (40000000 / (baud * 16)) - 1;
    
    U5MODESET = _U5MODE_ON_MASK;      // Enable USART5
}

static void ADC_begin(void)
{    
    AD1CON1bits.FORM = 0; // Integer
    AD1CON1bits.SSRC = 7; // Auto convert after sampling
    AD1CON1bits.ASAM = 0; // Sampling begins when SAMP bit is set
    AD1CON1bits.SAMP = 0;
    
    AD1CON2bits.VCFG = 0;  // Vdd/Vss references
    AD1CON2bits.CSCNA = 0; // Do not scan inputs
    AD1CON2bits.ALTS = 0;  // Always use MUX A
    AD1CON2bits.SMPI = 0;  // Interrupt on every conversion
    
    AD1CON3bits.ADRC = 0;   // Peripheral bus clock
    AD1CON3bits.SAMC = 15;  // Auto-sample 15 Tad
    AD1CON3bits.ADCS = 127; // Slow clock
    
    AD1CHSbits.CH0SA = 6; // Mux to AN6
    AD1CHSbits.CH0NA = 0; // Negative input is Vr-    
    
    ANSELBbits.ANSB6 = 1;   // RB6, AN6, pin 26, P1-37 analog
    TRISBbits.TRISB6 = 1;   // RB6, AN6, pin 26, P1-37 input
    
    AD1CON1SET = _AD1CON1_ON_MASK;
}

uint16_t analogRead(const int chan)
{    
    AD1CHSbits.CH0SA = chan;
    
    AD1CON1SET = _AD1CON1_SAMP_MASK;    // Start sampling, then conversion
    
    while (AD1CON1bits.SAMP)        // Wait for sampling to complete
        ;
    
    while (AD1CON1bits.DONE == 0)   // Wait for conversion to complete
        ;
    
    return (ADC1BUF0);
}

static void SPI2_begin(void)
{
        
    /* Configure SPI1 */
    // SCK1 on pin 70 - can't use on this PCB
    //SDI1Rbits.SDI1R = 0;   // SDI1 on RPD3
    //RPC13Rbits.RPC13R = 8; // SDO1 on RPC13
    
    /* Configure SPI2 */
    // SCK2 on pin 10, RG6, P1 pin 32
    SDI2Rbits.SDI2R = 0;   // SDI1 on RPD3, pin 
    RPC13Rbits.RPC13R = 6; // SDO2 on RPC13, pin 73, P7 pin 16
    
    SPI2BRG = 9;            // 2MHz
    SPI2CONbits.MSTEN = 1;  // Master mode
    SPI2CONbits.MODE16 = 1; // 16-bit mode
    SPI2CONbits.MODE32 = 0;
    SPI2CONbits.CKE = 1;
    SPI2CONbits.STXISEL = 0; // Interrupt on Tx complete
    SPI2CONbits.SRXISEL = 3; // Interrupt on Rx full
    
    TRISDbits.TRISD9 = 0;   // P7 pin 12 as output for SS
    
    IPC8bits.SPI2IP = 3;          // SPI2 interrupt priority 3
    IPC8bits.SPI2IS = 1;          // SPI2 interrupt sub-priority 1
    IFS1CLR = _IFS1_SPI2TXIF_MASK;  // Clear SPI2 Tx interrupt flag
    IFS1CLR = _IFS1_SPI2RXIF_MASK;  // Clear SPI2 Rx interrupt flag
    
    SPI2CONbits.ON = 1;
}

void toneT2(const int freq)
{
    if (freq == 0)
    {
        OC2RS = 0;
    }
    else
    {
        const int div = (40000000 / 64) / freq;
        PR2 = div;
        OC2RS = div / 2;
    }
}

void main(void)
{
    char buf[32];
    int i;
    double delta;
    uint16_t ana;
    uint16_t dacx, dacy;
    
    /* Configure tri-state registers*/
    TRISEbits.TRISE6 = 0;   // LED1 as output
    TRISEbits.TRISE7 = 0;   // LED2 as output
    TRISEbits.TRISE1 = 0;   // LED3 as output
    TRISAbits.TRISA7 = 0;   // LED5 as output
    TRISAbits.TRISA6 = 0;   // LED5 as output
    
    TRISAbits.TRISA4 = 0;   // P7 pin 6 as output (timer toggle))
    
    UART1_begin(19200);
    UART2_begin(9600);
    UART3_begin(4800);
    UART4_begin(9600);
    UART5_begin(9600);

    ADC_begin();
    
    SPI2_begin();
    
    RPD8Rbits.RPD8R = 12; // OC1 on P7 pin 10 (LED PWM)
    RPD0Rbits.RPD0R = 11; // OC2 on P7 pin 14 (tone)
    
    /* Configure Timer 2 for tone generation via PWM */
    T2CONbits.TCKPS = 6;        // Timer 2 prescale: 64
    
    TMR2 = 0x00;                // Clear Timer 2 counter
    PR2 = 1420;                 // Divisor for 440Hz
    
    T2CONbits.ON = 1;           // Enable Timer 2
    
    OC2CONbits.OCTSEL = 0;      // Source: Timer 2
    OC2CONbits.OCM = 6;         // PWM mode
    
    OC2RS = 0;                  // Silent
    
    OC2CONbits.ON = 1;          // Enable OC2 PWM
    
    /* Configure Timer 3 for 10-bit PWM */
    T3CONbits.TCKPS = 6;        // Timer 3 prescale: 64
    
    TMR3 = 0x00;                // Clear Timer 3 counter
    PR3 = 1023;                 // PWM range 0..1023 (10 bits)
    
    T3CONbits.ON = 1;           // Enable Timer 3
    
    OC1CONbits.OCTSEL = 1;      // Source: Timer 3
    OC1CONbits.OCM = 6;         // PWM mode
    
    OC1RS = 256;
    
    OC1CONbits.ON = 1;          // Enable OC1 PWM
            
    /* Configure Timer 1 */
    T1CONbits.TCKPS = 0;        // Timer 1 prescale: 1
    
    TMR1 = 0x00;                // Clear Timer 1 counter
    PR1 = 39999;                // Interrupt every 40000 ticks (1ms)
    
    T1CONbits.ON = 1;           // Enable Timer 1
    
    /* Configure Timer 4 */
    T4CONbits.TCKPS = 0;        // Timer 4 prescale: 1
    
    TMR4 = 0x00;                // Clear Timer 4 counter
    PR4 = 906;                  // Interrupt every 907 ticks (44100Hz)
    
    T4CONbits.ON = 1;           // Enable Timer 4
    
    /* Configure interrupts */
    INTCONSET = _INTCON_MVEC_MASK; // Multi-vector mode
    
    IPC1bits.T1IP = 2;          // Timer 1 interrupt priority 2
    IPC1bits.T1IS = 1;          // Timer 1 interrupt sub-priority 1
    IFS0CLR = _IFS0_T1IF_MASK;  // Clear Timer 1 interrupt flag
    IEC0SET = _IEC0_T1IE_MASK;  // Enable Timer 1 interrupt
    
    IPC4bits.T4IP = 1;          // Timer 4 interrupt priority 1
    IPC4bits.T4IS = 1;          // Timer 4 interrupt sub-priority 1
    IFS0CLR = _IFS0_T4IF_MASK;  // Clear Timer 4 interrupt flag
    IEC0SET = _IEC0_T4IE_MASK;  // Enable Timer 4 interrupt
    
    delta = (2.0 * M_PI) / 4096.0;
    
    for (i = 0; i < 4096; i++)
    {
        Sinbuf[i] = (sin(delta * (double)i) * 2047.0) + 2048.0;
    }
    
    __asm__("EI");              // Global interrupt enable
    
    while(1)
    {
        U1TXREG = 'A';
        
        LED1 = 0;
        LED2 = 1;
        LED3 = 1;
        LED4 = 1;
        LED5 = 1;
        
        dacx = 0;
        SPIword = (0 << 15) | (1 << 13) | (1 << 12) | dacx;
        OC1RS = 0;
        toneT2(440);
        
        delayms(500);
        
        ana = analogRead(6);
        
        sprintf(buf, "%dms %d\r\n", millis(), ana);
        
        for (i = 0; buf[i] != '\0'; i++)
        {
            while (U2STAbits.UTXBF) // Wait while Tx buffer full
                ;
            
            U2TXREG = buf[i];
        }
        
        LED1 = 1;
        LED2 = 0;
        
        dacx = 1024;
        SPIword = (0 << 15) | (1 << 13) | (1 << 12) | dacx;
        OC1RS = 128;
        toneT2(0);
        
        delayms(500);
        
        U3TXREG = 'C';
        
        LED2 = 1;
        LED3 = 0;
        
        dacx = 2048;
        SPIword = (0 << 15) | (1 << 13) | (1 << 12) | dacx;
        OC1RS = 256;
        toneT2(880);
        
        delayms(500);
        
        U4TXREG = 'D';
        
        LED3 = 1;
        LED4 = 0;
        
        dacx = 3072;
        SPIword = (0 << 15) | (1 << 13) | (1 << 12) | dacx;
        OC1RS = 512;
        toneT2(0);
        
        delayms(500);
        
        U5TXREG = 'E';
        
        LED4 = 1;
        LED5 = 0;
        
        dacx = 4095;
        SPIword = (0 << 15) | (1 << 13) | (1 << 12) | dacx;
        OC1RS = 1023;
        toneT2(440 * 8);
        
        delayms(500);
    }
}

