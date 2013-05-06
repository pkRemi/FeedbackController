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

#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */
#include <libpic30.h>      /* Includes delay definition
#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp              */

/******************************************************************************/
/* Function Prototypes                                                        */
/******************************************************************************/
void sendSPI1(int spidata);
void sendSPI1string(char *spistring, int size);
void sendnRFstring(char *nRFstring, int size);
/* Default interrupt handler */
void __attribute__((interrupt,no_auto_psv)) _DefaultInterrupt(void);
void __attribute__((__interrupt__, __auto_psv__)) _SPI1Interrupt(void)
;

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/
/* i.e. uint16_t <variable_name>; */
char serString[64] = {'H','e','l','l','o',' ','w','o','r','l','d','!','\n'};	//Array to hold page data to UART
bool SPItxInProgress = 0;
/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{

    /* Configure the oscillator for the device */
    ConfigureOscillator();

    /* Initialize IO ports and peripherals */
    InitApp();
    initSerial();
    initSPI1();
    __delay32(1000);
    initnRF();
    //sendSPI1string(serString, 12);
    /* TODO <INSERT USER APPLICATION CODE HERE> */
    int serStringN = 13;
    sendnRFstring(serString, serStringN);
    static const char statusREG[2] = {0x00 , 0xff};
    int i;
    while(1)
    {
        _LATA4 = 1;
        sendSPI1string(statusREG, 2);
        __delay32(160000);
//                for (i = 0; i < serStringN; i = i++)
//        {
//            //while(!U1STAbits.TRMT);
//            //U1TXREG = serString[i];
//            sendSPI1(serString[i]);
//        }
        _LATA4 = 0;
        __delay32(160000);

    }
}
void sendnRFstring(char *nRFstring, int size)
{

    int n=0;
    while(!_LATB11);            // Wait until /CS is set high by interrupt
    while(!SPI1STATbits.SRMPT); // Wait until buffer is empty
    SPItxInProgress = 1;        // Set tx flag so interrupt dont set /CS high before the whole string has been transmitted
    _LATB11 = 0;                // Set /CS to low
    while(!SPI1STATbits.SRMPT); // Wait until buffer is empty
    SPI1BUF = 0xA0;        //Transmit the data from the string
    for(n=0; n<size;n++)
    {
         while(!SPI1STATbits.SRMPT); // Wait until buffer is empty
         SPI1BUF = nRFstring[n];        //Transmit the data from the string
    }
    SPItxInProgress = 0;        // Reset tx flag so interrupt will set /CS high when the rest of the string has been transmitted
    while(!_LATB11);
    __delay32(200); // Delay for >10us
    _LATB15 = 1;
    __delay32(200); // Delay for >10us
    _LATB15 = 0;
    
}
void sendSPI1string(char *spistring, int size)
{
    int n=0;
    while(!_LATB11);            // Wait until /CS is set high by interrupt
    while(!SPI1STATbits.SRMPT); // Wait until buffer is empty
    SPItxInProgress = 1;        // Set tx flag so interrupt dont set /CS high before the whole string has been transmitted
    _LATB11 = 0;                // Set /CS to low
    for(n=0; n<size;n++)
    {
         while(!SPI1STATbits.SRMPT); // Wait until buffer is empty
         SPI1BUF = spistring[n];        //Transmit the data from the string
    }
    SPItxInProgress = 0;        // Reset tx flag so interrupt will set /CS high when the rest of the string has been transmitted
}

void sendSPI1(int spidata)
{
    while(!SPI1STATbits.SRMPT);
    _LATB11 = 0;        // SET CS low
    SPI1BUF = spidata;
    //while(!SPI1STATbits.SRMPT);
    //_LATB11 = 1;
}
/******************************************************************************/
/* Default Interrupt Handler                                                  */
/*                                                                            */
/* This executes when an interrupt occurs for an interrupt source with an     */
/* improperly defined or undefined interrupt handling routine.                */
/******************************************************************************/
void __attribute__((interrupt,no_auto_psv)) _DefaultInterrupt(void)
{
        while(1);
}
void __attribute__((__interrupt__, __auto_psv__)) _SPI1Interrupt(void)
{
    if(!SPItxInProgress)
        _LATB11 = 1;
    IFS0bits.SPI1IF = 0;
    
}

