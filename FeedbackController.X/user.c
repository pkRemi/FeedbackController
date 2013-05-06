/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#if defined(__XC16__)
    #include <xc.h>
#elif defined(__C30__)
    #if defined(__PIC24E__)
    	#include <p24Exxxx.h>
    #elif defined (__PIC24F__)||defined (__PIC24FK__)
	#include <p24Fxxxx.h>
    #elif defined(__PIC24H__)
	#include <p24Hxxxx.h>
    #endif
#endif

#include <stdint.h>          /* For uint32_t definition */
#include <stdbool.h>         /* For true/false definition */
#include <string.h>

#include "user.h"            /* variables/params used by user.c */

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

/* <Initialize variables in user.h and insert code for user algorithms.> */

/* TODO Initialize User Ports/Peripherals/Project here */

void InitApp(void)
{
    /* Setup analog functionality and port direction */
    //TRISA = 0b0000000000010011;   // POR Setting
    //TRISA = 0b0000000000000011;   // RA4 output
    TRISAbits.TRISA4 = 0;           // RA4 output
    /* Initialize peripherals */

    /* Setup of Remappable Pin configuration                                  */
    OSCCONbits.IOLOCK = 0;      //Allowes update of RP registers

    // Serial port UART1
    //RPINR18bits.U1RXR = 11;      // Set UART1 RX to RP11 (maybe not needed for TX)
    RPOR5bits.RP10R = 3 ;       // Set RP10 to UART1 TX (3)

    // SPI1 input
    RPINR20bits.SDI1R = 13;     // SPI1 Data Input MISO RP13 pin 24

    // SPI1 outputs
    //RPOR5bits.RP11R = 9;        // Slave select 1 to RP11 pin 22
    RPOR6bits.RP12R = 7;        // MOSI SDO1 to RP12 pin 23
    RPOR7bits.RP14R = 8;        // SCK1OUT to RP14 pin 25
    TRISBbits.TRISB15 = 0;      // Set RB15 pin 26 to output CE
    _LATB11 = 1;                // Set /CS high
    TRISBbits.TRISB11 = 0;      // /CS for nRF
    OSCCONbits.IOLOCK = 1;      //Locks RP registers
}

void initSerial(void)
{

    // RP pin configurations are set in InitApp
    U1MODEbits.STSEL = 0;               // 1 Stop bit
    U1MODEbits.PDSEL = 0;               // No Parity, 8 data bits
    U1MODEbits.ABAUD = 0;               // Auto-Baud Disabled
    U1BRG = 51;                     // BAUD Rate Setting for 19200
    //U1STABITS.UTXISEL1 = 0;             // 00 Interrupt generated when any character is transferred to the Transmit Shift Register
    U1STAbits.UTXISEL0 = 0;             //
    IEC0bits.U1TXIE = 0;                // Disable UART1 TX Interrupt
    IEC0bits.U1RXIE = 0;                // Disable UART1 RX Interrupt
    U1MODEbits.UARTEN = 1;              // Enable UART
    U1STAbits.UTXEN = 1;                // Enable UART TX
}

void initSPI1(void)
{
    // RP pin configurations are set in InitApp
    SPI1STATbits.SPIEN = 0;     // Disable SPI1 before configuration
    SPI1CON1bits.DISSCK = 0;    // Internal SPIx clock is enabled
    SPI1CON1bits.DISSDO = 0;    // SDOx pin is controlled by the module
    SPI1CON1bits.MODE16 = 0;    // Communication is word-wide (16 bits)
    SPI1CON1bits.SMP = 0;       // Input data sampled at middle of data output time
    SPI1CON1bits.CKE = 1;       // Serial output data changes on transition from active clock state to Idle clock state (see bit 6)
    SPI1CON1bits.SSEN = 0;      // SSx pin used for Slave mode
    SPI1CON1bits.CKP = 0;       // Idle state for clock is a low level; active state is a high level
    SPI1CON1bits.MSTEN = 1;     // Master mode
    SPI1CON1bits.SPRE = 0b111;  // 111 = Secondary prescale 1:1
    SPI1CON1bits.PPRE = 0b01;   // 01 = Primary prescale 16:1 sets SPI to 1MHz
    SPI1CON2bits.FRMEN = 0;     // Framed SPIx support enabled
    SPI1CON2bits.SPIFSD = 0;    // Frame sync pulse output (master)
    SPI1CON2bits.SPIFPOL = 0;   // Frame sync pulse is active-low
    SPI1CON2bits.SPIFE = 1;     // Frame sync pulse precedes first bit clock
    SPI1CON2bits.SPIBEN = 1;    // Enhanced Buffer enabled
    SPI1STATbits.SPIROV = 0;    // No overflow
    SPI1STATbits.SISEL = 0b101; // Interrupt when the last bit is shifted out of SPIxSR, now the transmit is complete
    _LATB11 = 1;                // Set /CS high
    SPI1STATbits.SPIEN = 1;     // Enable SPI1 after configuration
    // Setup interrupt
    IFS0bits.SPI1IF = 0;        // Reset interrupt flag
    IEC0bits.SPI1IE = 1;        // Enable SPI1 interrupt

}

void initnRF(void)
{
    static const char strTX_ADDR[6] = {0x30, 0xa5, 0xd6, 0x65, 0xcb, 0x2a};
    char initString[64] = {0};
    int size = 0;

    // Send powerup and TX mode
    initString[0] = 0x20;
    initString[1] = 0b00001010;
    size = 2;
    sendSPI1string(initString, size);

    // Send TX_ADDR
    size = sizeof(strTX_ADDR);
    memcpy(initString, strTX_ADDR, size);
    sendSPI1string(initString, size);

    // Send RX_ADDR_PO
    initString[0] = 0x2A; // Same address but different register
    sendSPI1string(initString, size);
    
    // Setup retry delay and number of retries
    initString[0] = 0x24;       // Write to SETUP_RETR register
    initString[1] = 0b00100011; // Delay 75us 3 retries
    size = 2;
    sendSPI1string(initString, size);

}
