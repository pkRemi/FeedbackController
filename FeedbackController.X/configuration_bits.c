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

/******************************************************************************/
/* Configuration Bits                                                         */
/*                                                                            */
/* This is not all available configuration bits for all PIC24 devices.        */
/* Refer to the PIC24 device specific .h file in the compiler                 */
/* support\PIC24x\h (x=F,H,E) directory for complete options specific to the  */
/* selected device.  For additional information about what the hardware       */
/* configurations mean in terms of device operation, refer to the device      */
/* datasheet 'Special Features' chapter.                                      */
/*                                                                            */
/* A feature of MPLAB X is the 'Generate Source Code to Output' utility in    */
/* the Configuration Bits window.  Under Window > PIC Memory Views >          */
/* Configuration Bits, a user controllable configuration bits window is       */
/* available to Generate Configuration Bits source code which the user can    */
/* paste into this project.                                                   */
/******************************************************************************/

/* TODO Fill in your configuration bits from the config bits generator here.  */

// PIC24FJ48GA002 Configuration Bit Settings

////#include <p24Fxxxx.h>
////
//// int CONFIG2 __attribute__((space(prog), address(0x83FC))) = 0x7BED ;
//////_CONFIG2(
//////    POSCMOD_XT &         // Primary Oscillator Select (XT Oscillator mode selected)
//////    I2C1SEL_PRI &        // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
//////    IOL1WAY_OFF &        // IOLOCK Protection (IOLOCK may be changed via unlocking seq)
//////    OSCIOFNC_OFF &       // Primary Oscillator Output Function (OSC2/CLKO/RC15 functions as CLKO (FOSC/2))
//////    FCKSM_CSDCMD &       // Clock Switching and Monitor (Clock switching and Fail-Safe Clock Monitor are disabled)
//////    FNOSC_PRIPLL &       // Oscillator Select (Primary Oscillator with PLL module (HSPLL, ECPLL))
//////    SOSCSEL_SOSC &       // Sec Oscillator Select (Default Secondary Oscillator (SOSC))
//////    WUTSEL_LEG &         // Wake-up timer Select (Legacy Wake-up Timer)
//////    IESO_OFF             // Internal External Switch Over Mode (IESO mode (Two-Speed Start-up) disabled)
//////);
//// int CONFIG1 __attribute__((space(prog), address(0x83FE))) = 0x3F7F ;
//////_CONFIG1(
//////    WDTPS_PS32768 &      // Watchdog Timer Postscaler (1:32,768)
//////    FWPSA_PR128 &        // WDT Prescaler (Prescaler ratio of 1:128)
//////    WINDIS_ON &          // Watchdog Timer Window (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
//////    FWDTEN_OFF &         // Watchdog Timer Enable (Watchdog Timer is disabled)
//////    ICS_PGx1 &           // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
//////    GWRP_OFF &           // General Code Segment Write Protect (Writes to program memory are allowed)
//////    GCP_OFF &            // General Code Segment Code Protect (Code protection is disabled)
//////    JTAGEN_OFF           // JTAG Port Enable (JTAG port is disabled)
//////);


// PIC24FJ48GA002 Configuration Bit Settings

#include <p24Fxxxx.h>

 int CONFIG2 __attribute__((space(prog), address(0x83FC))) = 0x7BEE ;
//_CONFIG2(
//    POSCMOD_HS &         // Primary Oscillator Select (HS Oscillator mode selected)
//    I2C1SEL_PRI &        // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
//    IOL1WAY_OFF &        // IOLOCK Protection (IOLOCK may be changed via unlocking seq)
//    OSCIOFNC_OFF &       // Primary Oscillator Output Function (OSC2/CLKO/RC15 functions as CLKO (FOSC/2))
//    FCKSM_CSDCMD &       // Clock Switching and Monitor (Clock switching and Fail-Safe Clock Monitor are disabled)
//    FNOSC_PRIPLL &       // Oscillator Select (Primary Oscillator with PLL module (HSPLL, ECPLL))
//    SOSCSEL_SOSC &       // Sec Oscillator Select (Default Secondary Oscillator (SOSC))
//    WUTSEL_LEG &         // Wake-up timer Select (Legacy Wake-up Timer)
//    IESO_OFF             // Internal External Switch Over Mode (IESO mode (Two-Speed Start-up) disabled)
//);
 int CONFIG1 __attribute__((space(prog), address(0x83FE))) = 0x3F3F ;
//_CONFIG1(
//    WDTPS_PS32768 &      // Watchdog Timer Postscaler (1:32,768)
//    FWPSA_PR128 &        // WDT Prescaler (Prescaler ratio of 1:128)
//    WINDIS_OFF &         // Watchdog Timer Window (Windowed Watchdog Timer enabled; FWDTEN must be 1)
//    FWDTEN_OFF &         // Watchdog Timer Enable (Watchdog Timer is disabled)
//    ICS_PGx1 &           // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
//    GWRP_OFF &           // General Code Segment Write Protect (Writes to program memory are allowed)
//    GCP_OFF &            // General Code Segment Code Protect (Code protection is disabled)
//    JTAGEN_OFF           // JTAG Port Enable (JTAG port is disabled)
//);

