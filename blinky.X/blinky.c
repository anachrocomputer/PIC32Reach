/* blinky --- blink five LEDs on PIC32 dev board            2019-01-10 */
/* Copyright (c) 2019 John Honniball. All rights reserved              */

/*
 * Created: 2019-01-10 22:45
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

#define UART_RX_BUFFER_SIZE  (128)
#define UART_RX_BUFFER_MASK (UART_RX_BUFFER_SIZE - 1)
#if (UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK) != 0
#error UART_RX_BUFFER_SIZE must be a power of two and <= 256
#endif

#define UART_TX_BUFFER_SIZE  (128)
#define UART_TX_BUFFER_MASK (UART_TX_BUFFER_SIZE - 1)
#if (UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK) != 0
#error UART_TX_BUFFER_SIZE must be a power of two and <= 256
#endif

struct UART_RX_BUFFER
{
    volatile uint8_t head;
    volatile uint8_t tail;
    uint8_t buf[UART_RX_BUFFER_SIZE];
};

struct UART_TX_BUFFER
{
    volatile uint8_t head;
    volatile uint8_t tail;
    uint8_t buf[UART_TX_BUFFER_SIZE];
};

struct UART_BUFFER
{
    struct UART_TX_BUFFER tx;
    struct UART_RX_BUFFER rx;
};

// UART buffers
static struct UART_BUFFER U4Buf;

volatile uint32_t MilliSeconds = 0;
volatile uint32_t SPIword = 0;

volatile int SPINbytes = 0;
volatile uint8_t *SPIBuf = NULL;
volatile int SPIDummyReads = 0;


/* dally --- CPU busy-loop for crude time delay */

static void dally(const int loops)
{
    volatile int dally;
    
    for (dally = 0; dally < loops; dally++)
        ;
}


/* delayms --- busy-wait delay for given number of milliseconds */

static void delayms(const uint32_t interval)
{
    const uint32_t now = MilliSeconds;
    
    while ((MilliSeconds - now) < interval)
        ;
}


/* millis --- Arduino-like function to return milliseconds since start-up */

static uint32_t millis(void)
{
    return (MilliSeconds);
}


void __ISR(_TIMER_4_VECTOR, ipl4AUTO) Timer4Handler(void) 
{    
    static int flag = 0;
    
    LATAINV = _LATA_LATA4_MASK; // Toggle P7 pin 6 (22050Hz)
    
    LATDCLR = _LATD_LATD9_MASK;   // Assert SS for SPI2
    
    SPI2BUF = SPIword;
    
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

void __ISR(_TIMER_1_VECTOR, ipl2AUTO) Timer1Handler(void) 
{
    MilliSeconds++;
    
    //LATAINV = _LATA_LATA4_MASK; // Toggle P7 pin 6 (500Hz))
    
    IFS0CLR = _IFS0_T1IF_MASK;  // Clear Timer 1 interrupt flag
}

void __ISR(_SPI_2_VECTOR, ipl3AUTO) SPI2Handler(void) 
{
    volatile uint32_t junk;
    
    junk = SPI2BUF;
    LATDSET = _LATD_LATD9_MASK;
    
    IFS1CLR = _IFS1_SPI2RXIF_MASK;  // Clear SPI2 interrupt flag
    IEC1CLR = _IEC1_SPI2RXIE_MASK;  // Disable SPI2 interrupt
}

void __ISR(_SPI_3_VECTOR, ipl1AUTO) SPI3Handler(void) 
{
    volatile uint32_t junk;
    
    if (IFS2 & _IFS2_SPI3RXIF_MASK)
    {
        junk = SPI3BUF;
        SPIDummyReads--;
        
        IFS2CLR = _IFS2_SPI3RXIF_MASK;  // Clear SPI3 Rx interrupt flag
        
        if (SPIDummyReads == 0)
        {
            LATASET = _LATA_LATA0_MASK;

            IEC2CLR = _IEC2_SPI3RXIE_MASK;  // Disable SPI3 Rx interrupt
        }
    }
    else if (IFS2 & _IFS2_SPI3TXIF_MASK)
    {
        SPI3BUF = *SPIBuf++;        // Transmit next byte
        SPINbytes--;
        
        IFS2CLR = _IFS2_SPI3TXIF_MASK;  // Clear SPI3 Tx interrupt flag
        
        if (SPINbytes == 0)
        {
            IEC2CLR = _IEC2_SPI3TXIE_MASK;  // Disable SPI3 Tx interrupt
        }
    }
}

void __ISR(_UART_4_VECTOR, ipl1AUTO) UART4Handler(void) 
{
    if (IFS2bits.U4TXIF)
    {
        if (U4Buf.tx.head != U4Buf.tx.tail) // Is there anything to send?
        {
            const uint8_t tmptail = (U4Buf.tx.tail + 1) & UART_TX_BUFFER_MASK;
            
            U4Buf.tx.tail = tmptail;

            U4TXREG = U4Buf.tx.buf[tmptail];     // Transmit one byte
        }
        else
        {
            IEC2CLR = _IEC2_U4TXIE_MASK;         // Nothing left to send; disable Tx interrupt
        }
        
        IFS2CLR = _IFS2_U4TXIF_MASK;  // Clear UART4 Tx interrupt flag
    }
    
    if (IFS2bits.U4RXIF)
    {
        const uint8_t tmphead = (U4Buf.rx.head + 1) & UART_RX_BUFFER_MASK;
        const uint8_t ch = U4RXREG;   // Read received byte from UART
        
        if (tmphead == U4Buf.rx.tail)   // Is receive buffer full?
        {
             // Buffer is full; discard new byte
        }
        else
        {
            U4Buf.rx.head = tmphead;
            U4Buf.rx.buf[tmphead] = ch;   // Store byte in buffer
        }
        
        IFS2CLR = _IFS2_U4RXIF_MASK;  // Clear UART4 Rx interrupt flag
    }
    
    if (IFS2bits.U4EIF)
    {
        IFS2CLR = _IFS2_U4EIF_MASK;   // Clear UART4 error interrupt flag
    }
}

static void UART1_begin(const int baud)
{
    U1MODEbits.UEN = 3;     // Use just Rx/Tx; no handshaking
    
    U1STAbits.UTXEN = 1;    // Enable Tx
    U1STAbits.URXEN = 1;    // Enable Rx (unused at present)
    
    U1BRG = (40000000 / (baud * 16)) - 1;
    
    U1MODESET = _U1MODE_ON_MASK;      // Enable USART1
}

static void UART2_begin(const int baud)
{
    U2MODEbits.UEN = 3;     // Use just Rx/Tx; no handshaking
    
    U2STAbits.UTXEN = 1;    // Enable Tx
    U2STAbits.URXEN = 1;    // Enable Rx (unused at present)
    
    U2BRG = (40000000 / (baud * 16)) - 1;
    
    U2MODESET = _U2MODE_ON_MASK;      // Enable USART2
}

static void t1ou2(const char ch)
{
    while (U2STAbits.UTXBF) // Wait while Tx buffer full
        ;

    U2TXREG = ch;
}

static void UART3_begin(const int baud)
{
    U3MODEbits.UEN = 3;     // Use just Rx/Tx; no handshaking
    
    U3STAbits.UTXEN = 1;    // Enable Tx
    U3STAbits.URXEN = 1;    // Enable Rx (unused at present)
    
    U3BRG = (40000000 / (baud * 16)) - 1;
    
    U3MODESET = _U3MODE_ON_MASK;      // Enable USART3
}

static void UART4_begin(const int baud)
{
    U4Buf.tx.head = 0;
    U4Buf.tx.tail = 0;
    U4Buf.rx.head = 0;
    U4Buf.rx.tail = 0;

    U4MODEbits.UEN = 3;     // Use just Rx/Tx; no handshaking
    
    U4STAbits.UTXEN = 1;    // Enable Tx
    U4STAbits.URXEN = 1;    // Enable Rx
    
    U4BRG = (40000000 / (baud * 16)) - 1;
    
    IPC9bits.U4IP = 1;          // UART4 interrupt priority 1
    IPC9bits.U4IS = 2;          // UART4 interrupt sub-priority 2
    
    IFS2CLR = _IFS2_U4TXIF_MASK;  // Clear UART4 Tx interrupt flag
    IFS2CLR = _IFS2_U4RXIF_MASK;  // Clear UART4 Rx interrupt flag
    IFS2CLR = _IFS2_U4EIF_MASK;   // Clear UART4 error interrupt flag
    
    IEC2SET = _IEC2_U4RXIE_MASK;  // Enable UART4 Rx interrupt
    IEC2SET = _IEC2_U4EIE_MASK;   // Enable UART4 error interrupt
    
    U4MODESET = _U4MODE_ON_MASK;      // Enable USART4
}


uint8_t UART4RxByte(void)
{
    const uint8_t tmptail = (U4Buf.rx.tail + 1) & UART_RX_BUFFER_MASK;
    
    while (U4Buf.rx.head == U4Buf.rx.tail)  // Wait, if buffer is empty
        ;
    
    U4Buf.rx.tail = tmptail;
    
    return (U4Buf.rx.buf[tmptail]);
}


void UART4TxByte(const uint8_t data)
{
    const uint8_t tmphead = (U4Buf.tx.head + 1) & UART_TX_BUFFER_MASK;
    
    while (tmphead == U4Buf.tx.tail)   // Wait, if buffer is full
        ;

    U4Buf.tx.buf[tmphead] = data;
    U4Buf.tx.head = tmphead;

    IEC2SET = _IEC2_U4TXIE_MASK;       // Enable UART4 Tx interrupt
}


bool UART4RxAvailable(void)
{
    return (U4Buf.rx.head != U4Buf.rx.tail);
}


static void UART5_begin(const int baud)
{
    U5MODEbits.UEN = 3;     // Use just Rx/Tx; no handshaking
    
    U5STAbits.UTXEN = 1;    // Enable Tx
    U5STAbits.URXEN = 1;    // Enable Rx (unused at present)
    
    U5BRG = (40000000 / (baud * 16)) - 1;
    
    U5MODESET = _U5MODE_ON_MASK;      // Enable USART5
}

void _mon_putc(const char ch)
{
    // See: https://microchipdeveloper.com/faq:81
    if (ch == '\n')
    {
        UART4TxByte('\r');
    }
    
    UART4TxByte(ch); // Connect stdout to UART4
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
    
    AD1CON1SET = _AD1CON1_ON_MASK;
}


/* analogRead --- Arduino-like function to read an analog input pin */

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

static void SPI2_begin(const int baud)
{
    SPI2BRG = (20000000 / baud) - 1;
    SPI2CONbits.MSTEN = 1;  // Master mode
    SPI2CONbits.MODE16 = 1; // 16-bit mode
    SPI2CONbits.MODE32 = 0;
    SPI2CONbits.CKE = 1;
    SPI2CONbits.STXISEL = 0; // Interrupt on Tx complete
    SPI2CONbits.SRXISEL = 3; // Interrupt on Rx full
    
    TRISDbits.TRISD9 = 0;   // RD9 pin 69, P7 pin 12 as output for SS
    LATDSET = _LATD_LATD9_MASK;   // De-assert SS for SPI2
    
    IPC8bits.SPI2IP = 3;          // SPI2 interrupt priority 3
    IPC8bits.SPI2IS = 1;          // SPI2 interrupt sub-priority 1
    IFS1CLR = _IFS1_SPI2TXIF_MASK;  // Clear SPI2 Tx interrupt flag
    IFS1CLR = _IFS1_SPI2RXIF_MASK;  // Clear SPI2 Rx interrupt flag
    
    SPI2CONbits.ON = 1;
}

static void SPI3_begin(const int baud)
{    
    SPI3BRG = (20000000 / baud) - 1;
    SPI3CONbits.MSTEN = 1;  // Master mode
    SPI3CONbits.MODE16 = 0; // 8-bit mode
    SPI3CONbits.MODE32 = 0;
    SPI3CONbits.CKE = 1;
    SPI3CONbits.STXISEL = 0; // Interrupt on Tx complete
    SPI3CONbits.SRXISEL = 3; // Interrupt on Rx full
    
    TRISAbits.TRISA0 = 0;   // RA0 pin 17, P1 pin 24 as output for SS
    LATASET = _LATA_LATA0_MASK;   // De-assert SS for SPI3
    
    IPC12bits.SPI3IP = 1;          // SPI3 interrupt priority 1
    IPC12bits.SPI3IS = 1;          // SPI3 interrupt sub-priority 1
    IFS2CLR = _IFS2_SPI3TXIF_MASK;  // Clear SPI3 Tx interrupt flag
    IFS2CLR = _IFS2_SPI3RXIF_MASK;  // Clear SPI3 Rx interrupt flag
    
    SPINbytes = 0;
    SPIDummyReads = 0;
    SPIBuf = NULL;
    
    SPI3CONbits.ON = 1;
}


bool SPIwrite(uint8_t *buf, const int nbytes)
{
    if (SPIDummyReads != 0)     // SPI tranmission still in progress?
    {
        return (false);
    }
    
    if ((nbytes <= 0) || (buf == NULL))
    {
        return (false);
    }
    
    LATACLR = _LATA_LATA0_MASK;   // Assert SS for SPI3
    
    SPIBuf = buf;
    SPINbytes = nbytes;
    SPIDummyReads = nbytes;
    
    SPI3BUF = *SPIBuf++;          // Transmit first byte
    SPINbytes--;
    
    IFS2CLR = _IFS2_SPI3RXIF_MASK;  // Clear SPI3 Rx interrupt flag
    IEC2SET = _IEC2_SPI3RXIE_MASK;  // Enable SPI3 Rx interrupt
    
    if (SPINbytes > 0)
    {
        IFS2CLR = _IFS2_SPI3TXIF_MASK;  // Clear SPI3 Tx interrupt flag
        IEC2SET = _IEC2_SPI3TXIE_MASK;  // Enable SPI3 Tx interrupt
    }
    
    return (true);
}

int SPIbytesPending(void)
{
    return (SPIDummyReads);
}


static void I2C1_begin(const int speed)
{
    if (speed > 200)
    {
        I2C1BRG = 0x2c;     // 400kHz
    }
    else
    {
        I2C1BRG = 0xc2;     // 100kHz
    }
    
    I2C1CONSET = _I2C1CON_ON_MASK;  // Enable I2C1
}


/* I2C1_Start --- send a Start condition and wait for completion */

static void I2C1_Start(void)
{
    I2C1CONSET = _I2C1CON_SEN_MASK; // Initiate start condition

    while (I2C1CONbits.SEN != 0)
        ;
}


/* I2C1_RepeatStart --- send a Repeated Start condition and wait for completion */

static void I2C1_RepeatStart(void)
{
    I2C1CONSET = _I2C1CON_RSEN_MASK; // Initiate repeated start condition

    while (I2C1CONbits.RSEN != 0)
        ;
}


/* I2C1_Transmit --- transmit a byte and wait for completion */

static void I2C1_Transmit(const uint8_t txByte)
{
    I2C1TRN = txByte;               // Transmit one byte

    while (I2C1STATbits.TRSTAT)
        ;
}


/* I2C1_Stop --- send a Stop condition and wait for completion */

static void I2C1_Stop(void)
{
    I2C1CONSET = _I2C1CON_PEN_MASK; // Initiate stop condition

    while (I2C1CONbits.PEN != 0)
        ;
}


/* I2C1_Recover --- recover from a bus collision error by clocking 9 times */

static void I2C1_Recover(void)
{
    int i;
    
    I2C1CONCLR = _I2C1CON_ON_MASK;  // Disable I2C1
    
    TRISAbits.TRISA14 = 1;  // Set SCL1 to input
    TRISAbits.TRISA15 = 1;  // Set SDA1 to input
    
    dally(20);
    
    for (i = 0; i < 9; i++)
    {
        LATAbits.LATA14 = 0;
        TRISAbits.TRISA14 = 0;  // Set SCL1 to output, pulling LOW
        
        dally(12);
        
        LATAbits.LATA14 = 1;    // Allow SCL1 to float HIGH again
        
        dally(12);
    }
    
    dally(10);
    
    I2C1CONSET = _I2C1CON_ON_MASK;  // Re-enable I2C1
    
    dally(10);
    
    I2C1_Stop();
}


/* I2C1_DevicePoll --- poll the I2C bus for a device at a given address */

static bool I2C1_DevicePoll(const uint8_t addr)
{
    bool found;

//    UART4TxByte('[');
//    UART4TxByte('S');
    
    I2C1_Start();
    
//    UART4TxByte('T');

    I2C1_Transmit(addr | 0x01); // Use I2C read address, D0 = 1

    if (I2C1STATbits.ACKSTAT)   // ACKSTAT == 1 => NACK
    {
        found = false;
    }
    else
    {
        found = true;
    }

//    UART4TxByte('P');
    
    I2C1_Stop();

    if (I2C1STATbits.BCL)   // Check for bus collision, which seems to happen
    {                       // at this point with the IMU, for unknown reasons
//        UART4TxByte('C');
        I2C1_Recover();
        return (found);
    }

//    UART4TxByte(']');
    
    return (found);
}


/* I2C1_ReadIMU --- read ID register in MPU9250 IMU via I2C */

static uint8_t I2C1_ReadIMU(const uint8_t i2cAddr, const uint8_t regAddr, const int nBytes, uint8_t buf[])
{
    int i;

    I2C1_Start();

    I2C1_Transmit(i2cAddr & 0xfe);    // IMU write address

    if (I2C1STATbits.ACKSTAT)   // ACKSTAT == 1 => NACK
    {
        I2C1_Stop();
        return (0);
    }

    I2C1_Transmit(regAddr);  // MPU9250 register address

    if (I2C1STATbits.ACKSTAT)   // ACKSTAT == 1 => NACK
    {
        I2C1_Stop();
        return (0);
    }

    I2C1_RepeatStart();

    I2C1_Transmit(i2cAddr | 0x01);    // IMU read address

    if (I2C1STATbits.ACKSTAT)   // ACKSTAT == 1 => NACK
    {
        I2C1_Stop();
        return (0);
    }

    for (i = 0; i < nBytes; i++)
    {
        I2C1CONSET = _I2C1CON_RCEN_MASK; // Initiate receive mode

        while (I2C1STATbits.RBF == 0)
            ;

        buf[i] = I2C1RCV;

        if (i < (nBytes - 1))
        {
            I2C1CONbits.ACKDT = 0;  // Send ACK
        }
        else
        {
            I2C1CONbits.ACKDT = 1;  // Send NACK because this is the last byte to read
        }

        I2C1CONSET = _I2C1CON_ACKEN_MASK; // Send master ACK bit

        while (I2C1CONbits.ACKEN != 0)
            ;
    }

    I2C1_Stop();
    
    if (I2C1STATbits.BCL)   // Check for bus collision, which seems to happen
    {                       // at this point with the IMU, for unknown reasons
//        UART4TxByte('C');
        I2C1_Recover();
        return (nBytes);
    }
    
    return (nBytes);
}


/* I2C1_EERead --- read CAT24C256 EEPROM via I2C */

static int I2C1_EERead(const uint8_t i2cAddr, const uint16_t byteAddr, const int nBytes, uint8_t buf[])
{
    int i;
    
    I2C1_Start();

    I2C1_Transmit(i2cAddr & 0xfe);    // EEPROM write address

    if (I2C1STATbits.ACKSTAT)   // ACKSTAT == 1 => NACK
    {
        I2C1_Stop();
        return (0);
    }

    I2C1_Transmit(byteAddr >> 8);  // Address HI byte

    if (I2C1STATbits.ACKSTAT)   // ACKSTAT == 1 => NACK
    {
        I2C1_Stop();
        return (0);
    }
    
    I2C1_Transmit(byteAddr & 0xff);  // Address LO byte

    if (I2C1STATbits.ACKSTAT)   // ACKSTAT == 1 => NACK
    {
        I2C1_Stop();
        return (0);
    }

    I2C1_RepeatStart();

    I2C1_Transmit(i2cAddr | 0x01);    // EEPROM read address

    if (I2C1STATbits.ACKSTAT)   // ACKSTAT == 1 => NACK
    {
        I2C1_Stop();
        return (0);
    }

    for (i = 0; i < nBytes; i++)
    {
        I2C1CONSET = _I2C1CON_RCEN_MASK; // Initiate receive mode

        while (I2C1STATbits.RBF == 0)
            ;

        buf[i] = I2C1RCV;

        I2C1CONbits.ACKDT = 0;

        I2C1CONSET = _I2C1CON_ACKEN_MASK; // Send master ACK bit

        while (I2C1CONbits.ACKEN != 0)
            ;
    }

    I2C1_Stop();
    
    return (nBytes);
}


/* toneT2 --- generate a tone of the given frequency via Timer 2 and OC2 */

void toneT2(const int freq)
{
    if (freq == 0)
    {
        OC2RS = 0;
    }
    else
    {
        const int div = (40000000 / 64) / freq;
        TMR2 = 0x00;                // Clear Timer 2 counter
        PR2 = div;
        OC2RS = div / 2;
    }
}


/* PPS_begin --- map Peripheral Pin Select to suit dev board */

static void PPS_begin(void)
{
    /* Configure USART1 */
    RPE5Rbits.RPE5R = 3;    // U1Tx on pin 3, RPE5
    U1RXRbits.U1RXR = 10;   // U1Rx on pin 6, RPC1
    
    /* Configure USART2 */
    RPG0Rbits.RPG0R = 1;    // U2Tx on pin 90, RPG0
    U2RXRbits.U2RXR = 12;   // U2Rx on pin 89, RPG1 (5V tolerant)
    
    /* Configure USART3 */
    RPF1Rbits.RPF1R = 1;    // U3Tx on pin 88, RPF1
    U3RXRbits.U3RXR = 4;    // U3Rx on pin 87, RPF0
    
    /* Configure USART4 */
    RPD4Rbits.RPD4R = 2;    // U4Tx on pin 81, RPD4
    U4RXRbits.U4RXR = 6;    // U4Rx on pin 82, RPD5 (5V tolerant)
    
    /* Configure USART5 */
    RPD12Rbits.RPD12R = 4;  // U5Tx on pin 79, RPD12
    U5RXRbits.U5RXR = 0;    // U5Rx on pin 76, RPD1
    
    /* Configure OC pins (PWM) */
    RPD8Rbits.RPD8R = 12; // OC1 on pin 68, P7 pin 10 (LED PWM)
    RPD0Rbits.RPD0R = 11; // OC2 on pin 72, P7 pin 14 (tone)
    
    /* Configure SPI1 */
    // SCK1 on pin 70 RD10 - can't use on this PCB
    //SDI1Rbits.SDI1R = 0;   // SDI1 on RPD3
    //RPC13Rbits.RPC13R = 8; // SDO1 on RPC13
    
    /* Configure SPI2 */
    // SCK2 on pin 10, RG6, P1 pin 32
    SDI2Rbits.SDI2R = 0;   // SDI2 on RPD3, pin 78
    RPC13Rbits.RPC13R = 6; // SDO2 on RPC13, pin 73, P7 pin 16
    
    /* Configure SPI3 */
    // SCK3 on pin 39, RF13, P1 pin 15
    SDI3Rbits.SDI3R = 0;   // SDI3 on RPD2, pin 77
    RPG8Rbits.RPG8R = 14;  // SDO3 on RPG8, pin 12, P1 pin 28
    
    /* Configure SPI4 */
    // SCK4 on pin 48, RD15 blocked by Main current sense
    
    /* Configure I2C1 */
    // SCL1 on RA14, pin 66, P1 pin 19
    // SDA1 on RA15, pin 67, P1 pin 21
    
    /* Configure I2C2 */
    // SCL2 on RA2, pin 58, P7 pin 20
    // SDA2 on RA3, pin 59, P7 pin 22
}


/* TRIS_begin --- switch GPIO pins to input or output as required */

static void TRIS_begin(void)
{
    TRISEbits.TRISE6 = 0;   // LED1 pin 4 as output
    TRISEbits.TRISE7 = 0;   // LED2 pin 5 as output
    TRISEbits.TRISE1 = 0;   // LED3 pin 94 as output
    TRISAbits.TRISA7 = 0;   // LED4 pin 92 as output
    TRISAbits.TRISA6 = 0;   // LED5 pin 91 as output
    
    TRISGbits.TRISG15 = 0;  // U1EN pin 1 as output
    TRISEbits.TRISE2 = 0;   // U2EN pin 98 as output
    TRISGbits.TRISG12 = 0;  // U3EN pin 96 as output
    TRISDbits.TRISD13 = 0;  // U4EN pin 80 as output
    TRISDbits.TRISD11 = 0;  // U5EN pin 71 as output
    
    TRISEbits.TRISE3 = 1;   // U1FAULT pin 99 as input
    TRISGbits.TRISG13 = 1;  // U2FAULT pin 97 as input
    TRISGbits.TRISG14 = 1;  // U3FAULT pin 95 as input
    TRISDbits.TRISD3 = 1;   // U4FAULT pin 78 as input
    TRISDbits.TRISD10 = 1;  // U5FAULT pin 70 as input
    TRISFbits.TRISF5 = 1;   // Main current FAULT pin 50 as input
    TRISFbits.TRISF4 = 1;   // Motor current FAULT pin 49 as input
    
    ANSELEbits.ANSE4 = 1;   // U1 current sense pin 100, RE4, AN21, analog
    ANSELEbits.ANSE0 = 1;   // U2 current sense pin 93, RE0, AN46, analog
    ANSELDbits.ANSD7 = 1;   // U3 current sense pin 84, RD7, AN43, analog
    ANSELDbits.ANSD6 = 1;   // U4 current sense pin 83, RD6, AN42, analog
    ANSELDbits.ANSD2 = 1;   // U5 current sense pin 77, RD2, AN25, analog
    ANSELDbits.ANSD14 = 1;  // Main current sense pin 47, RD14, AN36, analog
    ANSELDbits.ANSD15 = 1;  // Motor current sense pin 48, RD15, AN37, analog (blocks SCK4)

    TRISEbits.TRISE4 = 1;   // U1 current sense pin 100, RE4, AN21, input
    TRISEbits.TRISE0 = 1;   // U2 current sense pin 93, RE0, AN46, input
    TRISDbits.TRISD7 = 1;   // U3 current sense pin 84, RD7, AN43, input
    TRISDbits.TRISD6 = 1;   // U4 current sense pin 83, RD6, AN42, input
    TRISDbits.TRISD2 = 1;   // U5 current sense pin 77, RD2, AN25, input
    TRISDbits.TRISD14 = 1;  // Main current sense pin 47, RD14, AN36, input
    TRISDbits.TRISD15 = 1;  // Motor current sense pin 48, RD15, AN37, input (blocks SCK4)
    
    TRISBbits.TRISB15 = 0;  // HV_EN pin 44 as output
    
    // TODO: Add head LEDs and Add-On ports on daughter board
    
    ANSELBbits.ANSB6 = 1;   // Pot on pin 26, P1-37, RB6, AN6 analog
    TRISBbits.TRISB6 = 1;   // Pot on pin 26, P1-37, RB6, AN6 input
    
    TRISAbits.TRISA4 = 0;   // RA4 pin 60, P7 pin 6 as output (timer toggle)
}


void main(void)
{
    static uint8_t spi[32] = {1, 2, 3, 4, 5, 6, 7, 8};
    static uint8_t hello[] = "Hello, world\r\n";
    uint8_t eebuf[256];
    uint8_t imubuf[32];
    char buf[32];
    int i;
    uint16_t ana;
    uint8_t ch;
    
    /* Set up peripherals to match pin connections on PCB */
    PPS_begin();
    
    /* Configure tri-state registers */
    TRIS_begin();
    
    LATBbits.LATB15 = 0;  // HV_EN pin 44 LOW (6V regulator OFF)
    
    /* Switch off the MOSFETs */
    LATGbits.LATG15 = 0;  // U1EN pin 1 OFF
    LATEbits.LATE2 = 0;   // U2EN pin 98 OFF
    LATGbits.LATG12 = 0;  // U3EN pin 96 OFF
    LATDbits.LATD13 = 0;  // U4EN pin 80 OFF
    LATDbits.LATD11 = 0;  // U5EN pin 71 OFF
    
    UART1_begin(19200);
    UART2_begin(9600);
    UART3_begin(4800);
    UART4_begin(9600);
    UART5_begin(9600);

    ADC_begin();
    
    I2C1_begin(100);
    
    SPI2_begin(2000000);
    SPI3_begin(1000000);
    
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
    
    IPC4bits.T4IP = 4;          // Timer 4 interrupt priority 4
    IPC4bits.T4IS = 1;          // Timer 4 interrupt sub-priority 1
    IFS0CLR = _IFS0_T4IF_MASK;  // Clear Timer 4 interrupt flag
    IEC0SET = _IEC0_T4IE_MASK;  // Enable Timer 4 interrupt
    
    __asm__("EI");              // Global interrupt enable
    
    puts("\nPIC32blinky");
    
    delayms(20);
    
    printf("DEVCFG0 = %08x\n", DEVCFG0);
    printf("DEVCFG1 = %08x\n", DEVCFG1);
    printf("DEVCFG2 = %08x\n", DEVCFG2);
    printf("DEVCFG3 = %08x\n", DEVCFG3);
    
    printf("DEVCFG3.USERID = %x\n", DEVCFG3bits.USERID);
    
    printf("DEVID = %x, VER = %x\n", DEVIDbits.DEVID, DEVIDbits.VER);
    switch (DEVIDbits.DEVID)
    {
    case 0x06A13053:
        puts("PIC32MX250F256L");
        break;
    case 0x06A33053:
        puts("PIC32MX270F512L");
        break;
    case 0x06A15053:
        puts("PIC32MX550F256L");
        break;
    case 0x06A35053:
        puts("PIC32MX570F512L");
        break;
    }
    
    while (1)
    {
        U1TXREG = 'A';
        
        LED1 = 0;
        LED2 = 1;
        LED3 = 1;
        LED4 = 1;
        LED5 = 1;
        
        if (I2C1_DevicePoll(0xA0) == false)  // A0, A2 and D0 will ACK
        {
            LED5 = 0;
        }
        
        SPIword = 0x0000;
        OC1RS = 0;
        toneT2(440);
        SPIwrite(spi, 1);
        
        while (SPIbytesPending() > 0)
            ;
        
        SPIwrite(spi, 2);
        
        while (SPIbytesPending() > 0)
            ;
        
        SPIwrite(spi, 3);
        
        while (SPIbytesPending() > 0)
            ;
        
        SPIwrite(spi, 4);
        
        while (SPIbytesPending() > 0)
            ;
        
        SPIwrite(spi, 5);
        
        delayms(500);
        
        ana = analogRead(6);
        
        if (UART4RxAvailable())
        {
            ch = UART4RxByte();
        }
        else
        {
            ch = '*';
        }
        
        sprintf(buf, "%dms %d %c\r\n", millis(), ana, ch);
        
        for (i = 0; buf[i] != '\0'; i++)
        {
            t1ou2(buf[i]);
        }
        
        LED1 = 1;
        LED2 = 0;
        LED3 = 1;
        LED4 = 1;
        LED5 = 1;
        
        if (I2C1_DevicePoll(0xA2) == false)  // A0, A2 and D0 will ACK
        {
            LED5 = 0;
        }
        
        SPIword = 0xAA55;
        OC1RS = 128;
        toneT2(0);
        SPIwrite(spi, 8);
        
        delayms(500);
        
        U3TXREG = 'C';
        
        LED1 = 1;
        LED2 = 1;
        LED3 = 0;
        LED4 = 1;
        LED5 = 1;
        
        if (I2C1_DevicePoll(0xD0) == false)  // A0, A2 and D0 will ACK
        {
            LED5 = 0;
        }
        
        SPIword = 0x55AA;
        OC1RS = 256;
        toneT2(880);
        SPIwrite(hello, sizeof (hello) - 1);
        
        delayms(500);
        
        printf("%X", 0xDEADBEEF);
        
        LED1 = 1;
        LED2 = 1;
        LED3 = 1;
        LED4 = 0;
        LED5 = 1;
        
        I2C1_EERead(0xA0, 0, 16, eebuf);
        
        SPIword = 0xFF00;
        OC1RS = 512;
        toneT2(0);
        
        delayms(500);
        
        U5TXREG = 'E';
        
        LED1 = 1;
        LED2 = 1;
        LED3 = 1;
        LED4 = 1;
        LED5 = 0;
        
        //I2C1_EERead(0xA2, 0, 16, eebuf);
        if ((I2C1_ReadIMU(0xD0, 117, 1, imubuf) == 1) && (imubuf[0] == 0x71))
        {
            fputs("[I]", stdout);
        }
        else
        {
            fputs("[?]", stdout);
        }
        
        SPIword = 0xFF00;
        OC1RS = 1023;
        toneT2(440 * 8);
        
        delayms(500);
    }
}

