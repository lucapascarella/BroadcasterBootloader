 /*********************************************************************
 *
 *	Hardware specific definitions for:
 *    - PIC32 Ethernet Starter Kit
 *    - PIC32MX795F512L
 *    - Internal 10/100 Ethernet MAC with National DP83848 10/100 PHY
 *
 *********************************************************************
 * FileName:        HardwareProfile.h
 * Dependencies:    Compiler.h
 * Processor:       PIC32
 * Compiler:        Microchip C32 v1.11 or higher
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * Copyright (C) 2002-2010 Microchip Technology Inc.  All rights
 * reserved.
 *
 * Microchip licenses to you the right to use, modify, copy, and
 * distribute:
 * (i)  the Software when embedded on a Microchip microcontroller or
 *      digital signal controller product ("Device") which is
 *      integrated into Licensee's product; or
 * (ii) ONLY the Software driver source files ENC28J60.c, ENC28J60.h,
 *		ENCX24J600.c and ENCX24J600.h ported to a non-Microchip device
 *		used in conjunction with a Microchip ethernet controller for
 *		the sole purpose of interfacing with the ethernet controller.
 *
 * You should refer to the license agreement accompanying this
 * Software for additional information regarding your rights and
 * obligations.
 *
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * MICROCHIP BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE.
 *
 *
 * Author               Date		Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Howard Schlunder		09/16/2010	Regenerated for specific boards
 ********************************************************************/
#ifndef HARDWARE_PROFILE_H
#define HARDWARE_PROFILE_H

#define FIRMWARE_VERSION                    "0.1"
#define HARDWARE_VERSION                    "1.2"


// Clock frequency values
// These directly influence timed events using the Tick module.  They also are used for UART and SPI baud rate generation.
#define GetSystemClock()                (80000000ul)		// Hz
#define GetInstructionClock()           (GetSystemClock()/1)	// Normally GetSystemClock()/4 for PIC18, GetSystemClock()/2 for PIC24/dsPIC, and GetSystemClock()/1 for PIC32.  Might need changing if using Doze modes.
#define GetPeripheralClock()            (GetSystemClock()/1)	// Normally GetSystemClock()/4 for PIC18, GetSystemClock()/2 for PIC24/dsPIC, and GetSystemClock()/1 for PIC32.  Divisor may be different if using a PIC32 since it's configurable.

#define SYS_FREQ                        (GetSystemClock())

#define BUFFER_MAX_SIZE                 (1024ul * 4)

// Some preliminary importance definitions
#ifndef INPUT
#define INPUT                           1
#endif

#ifndef OUTPUT
#define OUTPUT                          0
#endif

/////
////#define Blink12BlueLed()                {LATBbits.LATB4 = 0; TRISBbits.TRISB4=(ReadCoreTimer()%0x0500000)>0x0380000?1:0;}
////#define BlinkFastBlueLed()              {LATBbits.LATB4 = 0; TRISBbits.TRISB4=(ReadCoreTimer()%0x0005000)>0x0002800?1:0;}
// Error indication.
// Switch S3 on USB Starter Kit.
#define ReadSwitchStatus()              (PORTReadBits(IOPORT_D, BIT_11) & BIT_11)
//////

// Hardware I/O pin mappings

// LEDs
#define LED1_TRIS			(TRISDbits.TRISD0)	// Ref LED1, RD0/OC1 Green lower position
#define LED1_IO				(LATDbits.LATD0)
#define LED2_TRIS			(TRISDbits.TRISD3)	// Ref LED2, RD3/OC4 Blue middle position
#define LED2_IO				(LATDbits.LATD3)
#define LED3_TRIS			(TRISBbits.TRISB11)	// Ref LED3, RB11    Red middle position
#define LED3_IO				(LATBbits.LATB11)
#define LED4_TRIS			(TRISDbits.TRISD4)	// Ref LED4, RD4/OC5 Red upper position
#define LED4_IO				(LATDbits.LATD4)

#define greenLED()                      (LATDbits.LATD0)
#define setGreenLED()                   (LATDSET = 0x0001)
#define clearGreenLED()                 (LATDCLR = 0x0001)
#define invertGreenLED()                (LATDINV = 0x0001)

#define blueLED()                       (LATDbits.LATD3)
#define setBlueLED()                    (LATDSET = 0x0008)
#define clearBlueLED()                  (LATDCLR = 0x0008)
#define invertBlueLED()                 (LATDINV = 0x0008)

#define redMiddleLED()                  (LATBbits.LATB11)
#define setRedMiddleLED()               (LATBSET = 0x0800)
#define clearRedMiddleLED()             (LATBCLR = 0x0800)
#define invertRedMiddleLED()            (LATBINV = 0x0800)

#define redLED()                        (LATDbits.LATD4)
#define setRedLED()                     (LATDSET = 0x0010)
#define clearRedLED()                   (LATDCLR = 0x0010)
#define invertRedLED()                  (LATDINV = 0x0010)

#define InitLED()                       ({LED1_TRIS = LED1_IO = LED2_TRIS = LED2_IO = LED3_TRIS = LED3_IO = LED4_TRIS = LED4_IO = 0; AD1PCFG = 0xFFFFFFFF;})
#define BlinkBlueLED()                  (blueLED() = ((ReadCoreTimer() & 0x0400000) != 0))
#define BlinkRedMiddleLED()             (redMiddleLED() = ((ReadCoreTimer() & 0x0100000) != 0))
#define BlinkRedLED()                   (redLED() = ((ReadCoreTimer() & 0x0200000) != 0))
#define BlinkGreenLED()                 (greenLED() = ((ReadCoreTimer() & 0x0400000) != 0))
#define Error()                         do{ setRedLED(); setRedMiddleLED(); } while(0);

// Momentary push buttons
#define BUTTON1_TRIS                    (TRISDbits.TRISD5)	// Ref SW1 Left
#define BUTTON1_IO			(PORTDbits.RD5)
#define BUTTON2_TRIS                    (TRISDbits.TRISD6)	// Ref SW2 Center
#define BUTTON2_IO			(PORTDbits.RD6)
#define BUTTON3_TRIS                    (TRISDbits.TRISD7)	// Ref SW3 Right
#define BUTTON3_IO			(PORTDbits.RD7)
#define BUTTON4_TRIS                    (TRISDbits.TRISD11)	// Ref SW4 Internal
#define BUTTON4_IO			(PORTDbits.RD11)

#define BUTTON_LEFT                     (PORTDbits.RD5)
#define BUTTON_CENTER                   (PORTDbits.RD6)
#define BUTTON_RIGHT                    (PORTDbits.RD7)
#define BUTTON_JUMPR                    (PORTDbits.RD11)

#define BUTTON_PRESSED                  (0u)
#define BUTTON_RELEASED                 (!BUTTON_PRESSED)


// UART configuration (not too important since we don't have a UART
// connector attached normally, but needed to compile if the STACK_USE_UART
// or STACK_USE_UART2TCP_BRIDGE features are enabled.
#define UART_TX_TRIS			(TRISFbits.TRISF5)      // RF4 U2RX or SDA5
#define UART_RX_TRIS			(TRISFbits.TRISF4)      // RF5 U2TX or SCL5

#define UART_TX_O			(LATFbits.LATF5)
#define UART_RX_I			(PORTFbits.RF4)

#define UBRG				U2BRG
#define UMODE				U2MODE
#define USTA				U2STA
#define BusyUART()			BusyUART2()
#define CloseUART()			CloseUART2()
#define ConfigIntUART(a)                ConfigIntUART2(a)
#define DataRdyUART()                   DataRdyUART2()
#define OpenUART(a,b,c)                 OpenUART2(a,b,c)
#define ReadUART()                      ReadUART2()
#define WriteUART(a)                    WriteUART2(a)
#define getsUART(a,b,c)                 getsUART2(a,b,c)
#define putsUART(a)                     putsUART2(a)
#define getcUART()                      getcUART2()
#define putcUART(a)                     do{while(BusyUART()); WriteUART(a); while(BusyUART()); }while(0)
#define putrsUART(a)                    putrsUART2(a)

//// I2C Definitions
//#define I2C_SDA_TRIS			(TRISDbits.TRISD9)      // SDA1
//#define I2C_SCL_TRIS			(TRISDbits.TRISD10)     // SCL1
//#define XEEPROM_CS_TRIS
//// EEPROM Constants
//#define EEPROM_I2C_BUS                  I2C1
//#define I2C_CLOCK_FREQ                  400000

// SST25FV016B => 16Mbit or 2Mbyte
#define SPIFLASH_CS_TRIS                (TRISDbits.TRISD10)
#define SPIFLASH_CS_IO                  (LATDbits.LATD10)

#define SPIFLASH_WP_TRIS                (TRISBbits.TRISB10)
#define SPIFLASH_WP_IO                  (LATBbits.LATB10)

#define SPIFLASH_HOLD_TRIS              (TRISDbits.TRISD9)
#define SPIFLASH_HOLD_IO                (LATDbits.LATD9)


#define SPIFLASH_SCK_TRIS		(TRISGbits.TRISG6)
#define SPIFLASH_SDI_TRIS		(TRISGbits.TRISG7)
#define SPIFLASH_SDI_IO			(LATGbits.LATG7)
#define SPIFLASH_SDO_TRIS		(TRISGbits.TRISG8)

//#define SPIFLASH_SPI_IF			(IFS0bits.SPI1IF)
#define SPIFLASH_SSPBUF			(SPI2BUF)
#define SPIFLASH_SPICON1		(SPI2CON)
#define SPIFLASH_SPICON1bits            (SPI2CONbits)
#define SPIFLASH_SPICON2		(SPI2CON2)
#define SPIFLASH_SPISTAT		(SPI2STAT)
#define SPIFLASH_SPISTATbits            (SPI2STATbits)
#define SPIFLASH_SPIBRG			(SPI2BRG)

//#define SPIFLASH_SCK_TRIS                (TRISGbits.TRISG6)
//#define SPIFLASH_SCK_IO                  (LATDbits.LAT
//
//#define SPIFLASH_CS_TRIS                (TRISDbits.TRISD10)
//#define SPIFLASH_CS_IO                  (LATDbits.LAT
//
//#define SPIFLASH_CS_TRIS                (TRISDbits.TRISD10)
//#define SPIFLASH_CS_IO                  (LATDbits.LAT


// Registers for the SPI module you want to use
#define SPICON1                         SPI2CON
#define SPICON2                         SPI2CON2
#define SPISTAT                         SPI2STAT
#define SPIBUF                          SPI2BUF
#define SPISTAT_RBF                     SPI2STATbits.SPIRBF
#define SPICON1bits                     SPI2CONbits
#define SPISTATbits                     SPI2STATbits
#define SPIENABLE                       SPICON1bits.ON
#define SPIBRG                          SPI2BRG

#define SPI_CHANNEL                     SPI_CHANNEL2

// Tris pins for SCK/SDI/SDO lines
#define SPICLOCK                        TRISGbits.TRISG6
#define SPIIN                           TRISGbits.TRISG7
#define SPIOUT                          TRISGbits.TRISG8
//#define SPIIN_PULLUP                    (CNPUGbits.CNPUG7)


// VS1063
#define MP3_XCS_O                       (LATFbits.LATF1)
#define MP3_XCS_TRIS                    (TRISFbits.TRISF1)

#define MP3_XDCS_O                      (LATFbits.LATF0)
#define MP3_XDCS_TRIS                   (TRISFbits.TRISF0)

#define MP3_XRESET_O                    (LATFbits.LATF3)
#define MP3_XRESET_TRIS                 (TRISFbits.TRISF3)

#define MP3_DREQ_I                      (PORTGbits.RG9)
#define MP3_DREQ_TRIS                   (TRISGbits.TRISG9)

#define MP3_SDI_TRIS                    (SPIIN)
#define MP3_SDO_TRIS                    (SPIOUT)
#define MP3_SCK_TRIS                    (SPICLOCK)

#define MP3_SPIBUF                      (SPIBUF)
#define MP3_SPI_RBF                     (SPISTAT_RBF)

// Define the SPI frequency
//#define SPI_BRG_24MHZ                   (0x00)      // With 80MHz clock
#define SPI_BRG_12MHZ                   (0x02)      // With 80MHz clock -> 13.33MHz
#define SPI_BRG_8MHZ                    (0x04)      // With 80MHz clock
#define SPI_BRG_1MHZ                    (0x27)      // With 80MHz clock



// External National PHY configuration
#define	PHY_RMII				// external PHY runs in RMII mode
//#define	PHY_CONFIG_ALTERNATE	// alternate configuration used
#define	PHY_ADDRESS			0x0	// the address of the LAN8720 PHY

// Note, it is not possible to use a MRF24WB0M Wi-Fi PICtail Plus
// card with this starter kit.  The required interrupt signal, among
// possibly other I/O pins aren't available on the Starter Kit board.

// ******************* MDD File System Required Definitions ********************
// Select your MDD File System interface type
// This library currently only supports a single physical interface layer
// In this example we are going to use the USB so we only need the USB definition
// *****************************************************************************
#define USE_USB_INTERFACE               // USB host MSD library

#endif // #ifndef HARDWARE_PROFILE_H