/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#include <p24Fxxxx.h>
#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */
#include <libpic30.h>      /* Includes delay definition                       */
#include <math.h>          /* For double acos(double)                         */
#include <stdio.h>         /*For text to serial port (remove if notneeded     */

#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp              */
#include "i2c.h"           /* I2C functions                                   */
/******************************************************************************/
/* Function Prototypes                                                        */
/******************************************************************************/
void sendnRFstring(unsigned char *nRFstring, int size);
void WriteSPI1(unsigned int data_out);
void LDByteWriteSPI(unsigned char OpCode, unsigned char Data );
void LDCommandWriteSPI(unsigned char OpCode );
unsigned char LDPageWriteSPI( unsigned char OpCode, unsigned char *wrptr, unsigned char strlength );
unsigned char LDByteReadSPI(unsigned char OpCode, unsigned char *rdptr, unsigned char length );
unsigned char LDBytePollnRF();
void getsSPI( unsigned char *rdptr, unsigned char length );
void PutStringSPI( unsigned char *wrptr , unsigned char strlength);
unsigned int ReadSPI1();

void Pcontroller(void);
void Speed2Delay(float speed);
void calcdelay(void);
void readSensorData(void);
void GyroZaverage(void);
void readCompassData(void);
void setRawData2string(void);

/* Default interrupt handler */
void __attribute__((interrupt,no_auto_psv)) _DefaultInterrupt(void);
void __attribute__((__interrupt__, __auto_psv__)) _SPI1Interrupt(void);
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void);

/******************************************************************************/
/* Global Variable Declaration                                                */
/******************************************************************************/
/* i.e. uint16_t <variable_name>; */
unsigned char serString[64] = {'H','e','l','l','o',' ','w','o','r','l','d','!','\n', '\r'};	//Array to hold page data to UART
bool SPItxInProgress = 0;

// Sensor variables
unsigned char ControlByte;	//Control Byte (I2C address) Gyro sensor
unsigned char PageString[64];	//Array to hold page data to/from I2C device
double delaytime = 1500000;
double countpos = 0;
int TemperatureRAW = 100;
float TemperatureC = 11.11;
int accel[3];
int gyro[3];
//unsigned int iaccel;
unsigned char rawsensor[14];
// Test for averaging
int n = 0;
float average = 0;
// Test of stepper motor speed control.
int axdelay = 4000;
int axdir = 0;
unsigned char rawcompass[6];
int compass[3];
float delay = 0;
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
    InitI2C();
    __delay32(1600000); // allow for POR for all devices
    initnRF();
    ControlByte = 0x00D0; // mpu6050 address
    InitMPU6050(ControlByte);
    InitHMC5883L();
    SetupInterrupts();
   
    int serStringN = 14;
    char nRFstatus = 0;
    int i;
    delaytime = 1;
    bool mode = 0;
    while(1)
    {
        //_LATA4 = 1;
        __delay32(160000);
//                for (i = 0; i < serStringN; i = i++)
//        {
//            //while(!U1STAbits.TRMT);
//            //U1TXREG = serString[i];
//        }
        //_LATA4 = 0;
        __delay32(1600);
        if(mode)
        {
            nRFstatus = LDBytePollnRF();
            if(nRFstatus & 0b00010000) //MAX_RT
            {
                //rfStatLED = 0;
                LDByteWriteSPI(0x27, 0b00010000);  //clear MAX_RT
            }
            if(nRFstatus & 0b00100000) //TX_DS
            {
                rfStatLED = 1;
                LDByteWriteSPI(0x27, 0b00100000);  //clear TX_DS
            }
        }
        if(button1)
        {
            rfStatLED = 1;
            __delay32(15000000);
            rfStatLED = 0;
            sendnRFstring( serString, serStringN);
            
            mode = 1;
        }
        else
        {
            //rfStatLED = 0;
        }




 //        _LATD0 = 0;
        __delay32(delaytime);
//        _LATD0 = 1;
        /* Send serial data to PC */
        for (i = 0; i < serStringN; i = i++)
        {
            while(!U1STAbits.TRMT);
            U1TXREG = serString[i];
        }
        __delay32(16000); // Without this delay after U1TXREG, the I2C command acts funny...
        /** Read sensors ******************************************************/
        readSensorData();
        __delay32(1000); // Without this delay, the I2C command acts funny...
        readCompassData();
        /** Feedback computation **********************************************/
        GyroZaverage();
        Pcontroller();
        /** Prepare telemetry *************************************************/
        setRawData2string();
        //serString[21] = delay; // Add the calculated delay between steps (dont work is float!)
        /**** Telemetry string format for command=0x42 ************************/
        /** command, axh, axl, ayh, ayl, azh, azl, th, tl,             9byte **/
        /** gxh, gxl, gyh, gyl, gzh, gzl, cxh, cxl, cyh, cyl, czh, czl,12byte**/
        /**   --Total 21byte                                                 **/
        /** Send telemetry ****************************************************/

        LDByteWriteSPI(0x27, 0b00010000);  // clear MAX_RT (max retries)
        LDCommandWriteSPI(0xE1);           // Flush TX buffer (tx buffer contains last failed transmission)
        LDByteWriteSPI(0x27, 0b00100000);  // clear TX_DS (ACK received)

        sendnRFstring( serString, 21);

        //        serStringN = sprintf(serString, "Sensor reading: %d \n\r", Data);
        TemperatureC = (TemperatureRAW)/340+36.53;
        /* The following sprintf command takes 18.744ms or 187440 instructions!!! */
//        serStringN = sprintf(serString, "Sensor: %04X Accel: %04X %04X %04X Gyro: %04X %04X %04X\n\r",
        serStringN = sprintf(serString, "T: %3.0f Acc: %05d %05d %05d Gyro: %05d %05d %05d Compass: %05d %05d %05d\n\r",
                TemperatureC,
                accel[0], accel[1], accel[2],
                gyro[0], gyro[1], gyro[2],
                compass[0],compass[1], compass[2]);
        __delay32(delaytime);
    }
}



void Pcontroller(void)
{
    // Setpoint, measured value, output - speed
    int axset = 200;
    float speed;
    speed = (axset - accel[0]);
    Speed2Delay(speed);
}
void Speed2Delay(float speed)
{
    // Speed is in steps/sec, max is:
    // 1 rps * 200 step/rev * 32 microstep/step * 2 int/step = 12800
    // Period = PR1 * prescaler * Tcy = 78 * 64 * 100ns = 2ms

    if (fabs(speed) > 12800) // Maximum speed. Avoids overspeed on the motor and potential lockup of the interrupt handler.
        delay = 17;
    else if (fabs(speed) < 10) // Minimum speed. Makes sure that the interrupt is run at least 10 times per sec.
        delay = 21739;
    else
        delay = 1/(speed * 0.0000046);

    axdelay = fabs(delay);
    if (speed >= 0) // Check direction of speed
        axdir = 1;
    else
        axdir = 0;

}
void calcdelay(void)
{
    double tau = 6.2831;
    double cstep = 1*tau/(500);
    countpos = countpos + cstep;
    if (countpos >= tau)
        countpos = 0;
    delaytime = (fabs(cos(countpos))*50000+20000)*1;

}
void setRawData2string(void)
{
    int i;
    int ii = 0;
        serString[ii]= 0x42; // Command for raw data to remote control
        // Put sensor data into 16 bit int variables

    for (i = 0; i < 14; i++)
    {
        ii++;
        serString[ii] = rawsensor[i];
    }
    for(i = 0; i < 6; i++)
    {
        ii++;
        serString[ii] = rawcompass[i];
    }
}
void readSensorData(void)
{

    LDSequentialReadI2C(ControlByte, 0x3B, rawsensor,14); // Read sensor data
    // Put sensor data into 16 bit int variables
    int i;
    int ii = 0;
    for (i = 0; i < 3; i++)
    {
        accel[i] = rawsensor[ii] << 8;
        ii++;
        accel[i] = accel[i] | rawsensor[ii];
        ii++;
    }
    TemperatureRAW = rawsensor[ii] << 8;
    ii++;
    TemperatureRAW = TemperatureRAW | rawsensor[ii];
    ii++;
    for (i = 0; i < 3; i++)
    {
        gyro[i] = rawsensor[ii] << 8;
        ii++;
        gyro[i] = gyro[i] | rawsensor[ii];
        ii++;
    }

}
/********************************************************************
*     Function Name:    readCompassData                             *
*     Parameters:       none.                                       *
*     Description:      Reads Compass sensor data via I2C and       *
*                       stores the values in rawcompass and         *
*                       compass global variables                    *
*                                                                   *
********************************************************************/
void readCompassData(void)
{

    LDSequentialReadI2C(0x3c, 0x03, rawcompass,6); // Read sensor data
    // Put compass data into 16 bit int variables
    int i;
    int ii = 0;
    for (i = 0; i < 3; i++)
    {
        compass[i] = rawcompass[ii] << 8;
        ii++;
        compass[i] = compass[i] | rawcompass[ii];
        ii++;
    }
}
void GyroZaverage(void)
{

      n++;
      average = average + ((gyro[2] - average)/n);
}

/********************************************************************
*     Function Name:    sendnRFstring                               *
*     Parameters:       pointer to string, length of string.        *
*     Description:      Writes a string to the transmit FIFO of nRF *
*                       and toggels the CE line to start            *
*                       transmission.                               *
*                                                                   *
********************************************************************/
void sendnRFstring(unsigned char *nRFstring, int size)
{

    LDPageWriteSPI(0xA0, nRFstring, size);
    __delay32(200); // Delay for >10us
    _LATB15 = 1;
    __delay32(200); // Delay for >10us
    _LATB15 = 0;
    
}

/********************************************************************
*     Function Name:    LDByteReadSPI                               *
*     Parameters:       EE memory control, address, pointer and     *
*                       length bytes.                               *
*     Description:      Reads data Byte from SPI EE memory device.  *
*                       This routine can be used for any SPI        *
*                       EE memory device with 1 byte of address     *
*                                                                   *
********************************************************************/
unsigned char LDByteReadSPI(unsigned char OpCode, unsigned char *rdptr, unsigned char length )
{
  CS_nRF = 0;                             // Select Device
  WriteSPI1( OpCode );                  // Send Read OpCode or register
  getsSPI( rdptr, length );           // read in multiple bytes
  CS_nRF = 1;                             // Deselect Device
  return ( 0 );
}

/********************************************************************
*     Function Name:    LDBytePollnRF                               *
*     Parameters:       None                                        *
*     Description:      Reads the status register from nRF SPI      *
*                       RF tranceiver                               *
*                                                                   *
********************************************************************/
unsigned char LDBytePollnRF()
{
    unsigned char value = 0;
    CS_nRF = 0;                             // Select Device
    //WriteSPI1( 0xFF );                  // Send Read OpCode or register
    value = ReadSPI1();
    CS_nRF = 1;                             // Deselect Device
    return (value);
}

/********************************************************************
*     Function Name:    LDPageWriteSPI                              *
*     Parameters:       Opcode, pointer addr, string length         *
*     Description:      Writes data string to SPI device            *
*                       OpCode is the register that receives the    *
*                       data                                        *
*                                                                   *
********************************************************************/
unsigned char LDPageWriteSPI( unsigned char OpCode, unsigned char *wrptr, unsigned char strlength )
{
  CS_nRF = 0;                           // Select Device
  WriteSPI1 ( OpCode );                 // send OpCode
  PutStringSPI ( wrptr, strlength );    // Write Page to device
  CS_nRF = 1;                           // Deselect Device
  SPI1STATbits.SPITBF = 0;              //Clear Transmit Buffer Full Status bit
  return ( 0 );
}
/********************************************************************
*     Function Name:    LDByteWriteSPI                              *
*     Parameters:       OpCode/register, data.                      *
*     Description:      Writes Data Byte to SPI device              *
*                                                                   *
********************************************************************/
void LDByteWriteSPI(unsigned char OpCode, unsigned char Data )
{
  CS_nRF = 0;                   // Select Device
  WriteSPI1 ( OpCode );         // Send Write OpCode
  WriteSPI1 ( Data );           // Write Byte to device
  CS_nRF = 1;                   // Deselect device
  SPI1STATbits.SPITBF = 0;      //Clear Transmit Buffer Full Status bit
}
/********************************************************************
*     Function Name:    LDCommandWriteSPI                           *
*     Parameters:       OpCode/register.                            *
*     Description:      Writes Command to SPI device                *
*                                                                   *
********************************************************************/
void LDCommandWriteSPI(unsigned char OpCode )
{
  CS_nRF = 0;                   // Select Device
  WriteSPI1 ( OpCode );         // Send Write OpCode
  CS_nRF = 1;                   // Deselect device
  SPI1STATbits.SPITBF = 0;      //Clear Transmit Buffer Full Status bit
}
/********************************************************************
*     Function Name:    PutStringSPI                                *
*     Return Value:     void                                        *
*     Parameters:       address of write string storage location    *
*                       and length of string                        *
*     Description:      This routine writes a string to the SPI bus.*
********************************************************************/

void PutStringSPI( unsigned char *wrptr , unsigned char strlength)
{
  unsigned int x;
  unsigned int dummy;
  for (x = 0; x < strlength; x++ )     // transmit data until PageSize
  {
     SPI1BUF = *wrptr++;               // initiate SPI bus cycle
     //while( !SPI1STATbits.SPITBF );    // wait until 'BF' bit is set (this sometimes hangs)
     while(!SPI1STATbits.SPIRBF);
     dummy = SPI1BUF;
  }
}


/********************************************************************
*     Function Name : WriteSPI1                                     *
*     Description   : This routine writes a single byte/word to     *
*                     the SPI bus.                                  *
*     Parameters    : Single data byte/word for SPI bus             *
*     Return Value  : None                                          *
********************************************************************/

void WriteSPI1(unsigned int data_out)
{
    if (SPI1CON1bits.MODE16)          /* word write */
        SPI1BUF = data_out;
    else
        SPI1BUF = data_out & 0xff;    /*  byte write  */
    while(SPI1STATbits.SPITBF);
    while(!SPI1STATbits.SPIRBF);
    data_out = SPI1BUF;               //Avoiding overflow when reading
}

/******************************************************************************
*     Function Name :   ReadSPI1                                              *
*     Description   :   This function will read single byte/ word  from SPI   *
*                       bus. If SPI is configured for byte  communication     *
*                       then upper byte of SPIBUF is masked.                  *
*     Parameters    :   None                                                  *
*     Return Value  :   contents of SPIBUF register                           *
******************************************************************************/

unsigned int ReadSPI1()
{
    int spiin = 0x42;
    SPI1STATbits.SPIROV = 0;
  SPI1BUF = 0xFF;                  // initiate bus cycle , was 00 changed to FF
  while(!SPI1STATbits.SPIRBF);
  //__delay32(1000);
  /* Check for Receive buffer full status bit of status register*/
  if (SPI1STATbits.SPIRBF)
  {
      SPI1STATbits.SPIROV = 0;

      if (SPI1CON1bits.MODE16)
      {
          spiin = SPI1BUF;
          return (spiin);           /* return word read */

      }
      else
          spiin = SPI1BUF;
          return (spiin & 0xff);    /* return byte read */
  }
  return -1;                  		/* RBF bit is not set return error*/
}

/********************************************************************
*     Function Name:    getsSPI                                     *
*     Return Value:     void                                        *
*     Parameters:       address of read string storage location and *
*                       length of string bytes to read              *
*     Description:      This routine reads a string from the SPI    *
*                       bus.  The number of bytes to read is deter- *
*                       mined by parameter 'length'.                *
********************************************************************/
void getsSPI( unsigned char *rdptr, unsigned char length )
{
  while ( length )                  // stay in loop until length = 0
  {
    *rdptr++ = ReadSPI1();          // read a single byte
    length--;                       // reduce string length count by 1
  }
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
//    if(!SPItxInProgress)
//        _LATB11 = 1;
//    IFS0bits.SPI1IF = 0;
    
}
// Timer 1 interrupt service routine toggles LED on RD1
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void)
{

    // Toggle LED on RD1
    _LATA4 = 1 - _LATA4;
    PR1 = axdelay;
    //_LATD3 = axdir;
    // Clear Timer 1 interrupt flag
    _T1IF = 0;


}

