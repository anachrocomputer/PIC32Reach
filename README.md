![Static Badge](https://img.shields.io/badge/MCU-PIC32MX-green "MCU:PIC32MX")
![Static Badge](https://img.shields.io/badge/IDE-MPLAB_X_V6.20-green "IDE:MPLAB_X_V6.20")
![Static Badge](https://img.shields.io/badge/BOARD-Reach_Dev_PCB-green "BOARD:Reach Dev PCB")

# PIC32Reach #

Blink five LEDs on a PIC32MX550F256L dev board from Reach Robotics.
Millisecond timer interrupt generated by Timer 1.
Also sends characters to all five USARTs,
generates PWM on OC1 and OC2,
and accepts analog input from a pot on AN6.
Two SPI channels configured to send data, SPI2 at 2MHz and SPI3 at 1MHz.
SPI2 sends a 16-bit word every 22.6us (44.1kHz).
SPI3 sends variable-length packets at certain times in the main loop.

## Connections ##

| Name | Port | Pin | Activity         |
|------|------|-----|------------------|
| LED1 | RE6  | 4   | LED, active-LOW  |
| LED2 | RE7  | 5   | LED, active-LOW  |
| LED3 | RE1  | 94  | LED, active-LOW  |
| LED4 | RA7  | 91  | LED, active-LOW  |
| LED5 | RA6  | 92  | LED, active-LOW  |
| U1TX | RE5  | 3   | UART1 transmit   |
| U2TX | RG0  | 90  | UART2 transmit   |
| U3TX | RF1  | 88  | UART3 transmit   |
| U4TX | RD4  | 81  | UART4 transmit   |
| U5TX | RD12 | 79  | UART5 transmit   |
| SCK2 | RG6  | 10  | SPI Clock        |
| SDO2 | RC13 | 73  | SPI MOSI         |
| SS2  | RD9  | 69  | SPI CS           |
| SCK3 | RF13 | 39  | SPI Clock        |
| SDO3 | RG8  | 12  | SPI MOSI         |
| SS3  | RA0  | 17  | SPI CS           |
| PWM1 | RD8  | 68  | OC1 PWM output   |
| PWM2 | RD0  | 72  | OC2 PWM output   |
| Pot  | RB6  | 26  | AN6 analog input |

PIC32 pin numbers are for the 100-pin package.

LEDs light when the pin is pulled LOW.

## PIC32 Toolchain ##

MPLAB X V6.20 and 'xc32' V4.60.
These are not quite the latest versions,
but MPLAB X V6.20 is the last version to support my "long obsolete" ICD3 programmer.

## PIC32 Programmer ##

Microchip ICD3.
Other programmers should work, e.g. ICD4 or PICkit4.

