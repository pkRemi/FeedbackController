/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

/* Device header file */
#include <p24Fxxxx.h>
#include "system.h"        /* System funct/params, like osc/peripheral config */
#include <stdint.h>        /* Includes uint16_t definition                    */
#include <stdbool.h>       /* Includes true/false definition                  */
#include <libpic30.h>      /* Includes delay definition                       */
#include <math.h>          /* For double acos(double)                         */
#include <stdio.h>         /*For text to serial port (remove if notneeded     */

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
unsigned char readnRFbyte(unsigned char OpCode );
unsigned char LDBytePollnRF();
void getsSPI( unsigned char *rdptr, unsigned char length );
void PutStringSPI( unsigned char *wrptr , unsigned char strlength);
unsigned int ReadSPI1();
void WriteSPI2(unsigned int data_out);
void CalibrateGyro(void);   /* Finding Gyro Offsett                           */
//void AverageFilter(void);
int FeedbackControlLoop(int gyro, int gos, int ggain, int accel, int aos, int again);
int FeedbackControlLoopPI(int gyro, int ggain, int integrator );
int saturate(int value, int max);
void speedFilter(void);

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
float angle = 0;
signed int xSpeed = 0;
signed int ySpeed = 0;
float GyroOffset[3] = {0,0,0};

signed long GyroGain[3] = {40,40,40};
signed long AccelGain[3] = {16,16,16};
signed int xspeedFilter[16] = {0};
signed int yspeedFilter[16] = {0};
unsigned int speedFilterPoint = 7;
unsigned int speedFilterLength = 8;
signed int accelMax = 600;
signed int gMax = 600;
float xgIntegrator = 0;
float ygIntegrator = 0;
signed int igain = 1;

volatile double AngleOffset[2];
volatile double gyroXangle, gyroYangle; // Angle calculate using the gyro
volatile double compAngleX, compAngleY; // Calculate the angle using a complementary filter
volatile double compAngleXlast = 0.0;
volatile double compAngleYlast = 0.0;
volatile double accXangle, accYangle;
unsigned int axFilterPoint = 0;
signed int axFilterAveraged = 0;
volatile unsigned char enableSteppers = 0;
int fastSample = 15;
volatile unsigned int buttonState = 0;

#define PI 3.1415926
#define RAD_TO_DEG 57.29577951
#define SAMPLE_TIME 0.01
/******************************************************************************/
/* Main Program                                                               */
/******************************************************************************/

int16_t main(void)
{

    unsigned int buttonCounter = 20000;
    int i;

    /* Configure the oscillator for the device */
    ConfigureOscillator();
    /* Initialize IO ports and peripherals */
    InitApp();
    //initSerial();
    initSPI1();
    initSPI2();
    InitI2C();
    __delay32(1600000); // allow for POR for all devices
    initnRF();
    ControlByte = 0x00D0; // mpu6050 address
    InitMPU6050(ControlByte);
//    InitHMC5883L();
    __delay_ms(500);
    /** Init control loop *****************************************************/
    readSensorData();
    accXangle = (atan2(accel[0], accel[2])*RAD_TO_DEG);
    accYangle = (atan2(accel[1], accel[2])*RAD_TO_DEG);
    gyroXangle = accXangle;
    gyroYangle = accYangle;
    compAngleX = accXangle;
    compAngleY = accYangle;
    AngleOffset[0] = accXangle;
    AngleOffset[1] = accYangle;

    CalibrateGyro();    // Finding the gyro zero-offset
    SetupInterrupts();
   
//////    int serStringN = 14;
//////    char nRFstatus = 0;
////////    int i;
//////    delaytime = 1;
//////    bool mode = 0;
    while(1)
    {
        /** Read Button RB7 for enable steppers *******************************/
        if (buttonState == 0 && PORTBbits.RB7 && !buttonCounter)
        {
            buttonState = 1;
            buttonCounter = 20000;
            enableSteppers = 0;
        }
        if (buttonState == 1 && !PORTBbits.RB7 && !buttonCounter)
        {
            enableSteppers = 2;
            __delay32(40000000);
            buttonState = 2;
            buttonCounter = 20000;
            enableSteppers = 1;
        }
        if (buttonState == 2 && PORTBbits.RB7 && !buttonCounter)
        {
            buttonState = 3;
            buttonCounter = 20000;
            enableSteppers = 0;
        }
        if (buttonState == 3 && !PORTBbits.RB7 && !buttonCounter)
        {
            buttonState = 0;
            buttonCounter = 20000;
            enableSteppers = 0;
        }
        if (buttonCounter)
        {
            buttonCounter--;
        }
        for (i=0;i<100;i++)
        {
            i=i;
        }
//        __delay32(100);
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
    serString[5] = 0;
    serString[6] = 0;
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
unsigned char readnRFbyte(unsigned char OpCode )
{
    char rdata;
  CS_nRF = 0;                             // Select Device
  WriteSPI1( OpCode );                  // Send Read OpCode or register
  rdata = ReadSPI1();           // read in one bytes
  CS_nRF = 1;                             // Deselect Device
  return ( rdata );
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
/********************************************************************
*     Function Name : WriteSPI2                                     *
*     Description   : This routine writes a single byte/word to     *
*                     the SPI bus.                                  *
*     Parameters    : Single data byte/word for SPI bus             *
*     Return Value  : None                                          *
********************************************************************/

void WriteSPI2(unsigned int data_out)
{
    if (SPI2CON1bits.MODE16)          /* word write */
        SPI2BUF = data_out;
    else
        SPI2BUF = data_out & 0xff;    /*  byte write  */
    while(SPI2STATbits.SPITBF);
    while(!SPI2STATbits.SPIRBF);
    data_out = SPI2BUF;               //Avoiding overflow when reading
}
void CalibrateGyro(void)
{
    int i;
    int ii;
    GyroOffset[0] = 0;
    GyroOffset[1] = 0;
    GyroOffset[2] = 0;
    AngleOffset[0] = 0;
    AngleOffset[1] = 0;


    for (i=0;i<1000;i++)
    {
        /** Read sensors ******************************************************/
        readSensorData();
        __delay32(5000); // Without this delay, the I2C command acts funny...
        accXangle = (atan2(accel[0], accel[2])*RAD_TO_DEG);
        accYangle = (atan2(accel[1], accel[2])*RAD_TO_DEG);
        for (ii=0;ii<3;ii++)
        {
            GyroOffset[ii] = GyroOffset[ii] + gyro[ii];
        }
        AngleOffset[0] += accXangle;
        AngleOffset[1] += accYangle;
    }
    for (ii=0;ii<3;ii++)
    {
        GyroOffset[ii] = GyroOffset[ii]/1000;
    }
    AngleOffset[0] = AngleOffset[0]/1000.0;
    AngleOffset[1] = AngleOffset[1]/1000.0;
    gyroXangle = AngleOffset[0];
    gyroYangle = AngleOffset[1];
    compAngleX = AngleOffset[0];
    compAngleY = AngleOffset[1];
}
int FeedbackControlLoop(int gyro, int gos, int ggain, int accel, int aos, int again)
{
    int output;
    output = (((gyro - gos)*ggain) + ((accel - aos)* again))/16;
    return(output);
}
int FeedbackControlLoopPI(int gyro, int ggain, int integrator )
{
    int output;
    output = ((gyro * ggain)+integrator * igain);
}
//void AverageFilter(void)
//{
//    int i;
//    long temp = 0;
//    axFilterAverage[axFilterPoint] = accel[0];
//    axFilterPoint++;
//    if (axFilterPoint > 31)
//        axFilterPoint = 0;
//    for (i=0;i<32;i++)
//    {
//        temp = temp + axFilterAverage[i];
//    }
//    axFilterAveraged = temp/32;
//}
void speedFilter(void)
{
    int i;
    long xtemp = 0;
    long ytemp = 0;
    xspeedFilter[speedFilterPoint] = xSpeed;
    yspeedFilter[speedFilterPoint] = ySpeed;
    if (speedFilterPoint)
        speedFilterPoint--;
    else
        speedFilterPoint = speedFilterLength-1;

    for (i=0;i<speedFilterLength;i++)
    {
        xtemp = xtemp + xspeedFilter[i];
        ytemp = ytemp + yspeedFilter[i];
    }
    xSpeed = xtemp/speedFilterLength;
    ySpeed = ytemp/speedFilterLength;
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
int saturate(int value, int max)
{
    if (value>max)
        return (max);
    else if (value < -max)
        return (-max);
    else
        return(value);
}
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt(void)
{
    unsigned int *chptr;           // For sending a float value
    unsigned int command;          // For sending commands to Axis Converter
    double gyroXrate, gyroYrate;
    char nRFstatus;
    unsigned char rfCommands[32];            // Commands received from remote
    unsigned char nRFregisters[10];
    int iii;
    // Toggle LED
    _LATA4 = 1;
    /** Read sensors ******************************************************/
    readSensorData();
    __delay32(1000); // Without this delay, the I2C command acts funny...
    accXangle = ((atan2(accel[0], accel[2]))*RAD_TO_DEG)-AngleOffset[0];
    accYangle = ((atan2(accel[1], accel[2]))*RAD_TO_DEG)-AngleOffset[1];
    gyroXrate = ((double)gyro[0]-(double)GyroOffset[0])/131;
    gyroYrate = -(((double)gyro[1]-(double)GyroOffset[1])/131);

    /** Feedback Loop *****************************************************/
    compAngleX = (0.99*(compAngleX+(gyroYrate*(double)SAMPLE_TIME)))+(0.01*accXangle); // Calculate the angle using a Complimentary filter
    compAngleY = (0.99*(compAngleY+(gyroXrate*(double)SAMPLE_TIME)))+(0.01*accYangle);
    ySpeed = -((compAngleY)*(double)50+(compAngleY-compAngleYlast)*(double)20);
    xSpeed = -((compAngleX)*(double)50+(compAngleX-compAngleXlast)*(double)20);
    if (xSpeed<0)
        xSpeed = -(xSpeed*xSpeed);
    else
        xSpeed = xSpeed*xSpeed;
    if (ySpeed<0)
        ySpeed = -(ySpeed*ySpeed);
    else
        ySpeed = ySpeed*ySpeed;
    
    compAngleXlast = compAngleX;
    compAngleYlast = compAngleY;
//    xSpeed = gyroXrate *1000;
//    ySpeed = gyroYrate *1000;

    /** Build command string **********************************************/
    command = 0b1000000000000000;
    if (enableSteppers == 1)
        command = command | 0b0100000000000000;
    /* Send data to Axis Controller ***************************************/
    chptr = (unsigned char *) &angle;  // For sending a float value
    CS_Axis = 0;
    /* Command: MSB
     * 15: 0 = NO Speed Values
     * 14: 1 = Enable Stepper Driver
     * 13:
     */
    WriteSPI2(command);                 // Command: MSB
    WriteSPI2(xSpeed);                  // Send x-velocity vector
    WriteSPI2(ySpeed);                  // Send y-velocity vector
    WriteSPI2(*chptr++);                // Sending first part of float value
    WriteSPI2(*chptr);                // Sending second part of float value
    CS_Axis = 1;

    /** Prepare telemetry *************************************************/
    setRawData2string();
    /**** Telemetry string format for command=0x42 ************************/
    /** command, axh, axl, ayh, ayl, azh, azl, th, tl,             9byte **/
    /** gxh, gxl, gyh, gyl, gzh, gzl, cxh, cxl, cyh, cyl, czh, czl,12byte**/
    /** motorStatus                                                 1byte**/
    /**   --Total 22byte                                                 **/

    serString[16] = xSpeed & 0xFF; // Temporary sending additional telemetry
    serString[15] = xSpeed >> 8;
    serString[18] = ySpeed & 0xFF;
    serString[17] = ySpeed >> 8;
//        serString[18] = iax & 0xFF;
//        serString[17] = iax >> 8;
//        serString[20] = igy & 0xFF;
//        serString[19] = igy >> 8;

    serString[21] = enableSteppers;


    /** Receive RF commands if any ********************************************/
//    LDByteWriteSPI(0x20, 0b00001010);
    nRFstatus = LDBytePollnRF();
    for(iii=0; iii<10;iii++) // For debug
    {
        nRFregisters[iii] = readnRFbyte(iii);
    }
    if (nRFstatus & 0b01000000)  // Data Ready RX FIFO
    {
    //    _LATB4 = 1;
////        Delay10TCYx(100);
        LDByteReadSPI(0x61, rfCommands, 21);    // Read RX FIFO
        LDByteWriteSPI(0x27 , 0b01000000);        // Clear RX FIFO interrupt bit
        //ByteWriteSPI(0xE2 , 0xFF); //Flush RX FIFO
        if(rfCommands[0] == 0x82)
        {
            if(enableSteppers)
            {
                enableSteppers = 0;
                buttonState = 0;
                //_LATB4 = 0;
            } else {
                enableSteppers = 1;
                buttonState = 2;
                //_LATB4 = 1;
            }
        } else if (rfCommands[0] == 0x82)
        {
            _LATB4 = 1;
        }
//        //LATDbits.LATD3 = 0;
//
    }
//    LDByteWriteSPI(0x20, 0b00001010);   //
//    LDByteWriteSPI(0x26, 0b00000010);   // Set RC_CH to reset retransmit counter
    LDCommandWriteSPI(0xE2);            // Flush RX buffer (not during ACK)
    /** Send telemetry ********************************************************/
//    _LATB15 = 0;                        // Set CE low to stop receiving data
//    LDByteWriteSPI(0x20 , 0b00001010);  // Set to send mode
//    __delay32(100);
    LDByteWriteSPI(0x27, 0b00010000);   // clear MAX_RT (max retries)
    LDCommandWriteSPI(0xE1);            // Flush TX buffer (tx buffer contains last failed transmission)
    LDByteWriteSPI(0x27, 0b00100000);   // clear TX_DS (ACK received)
    sendnRFstring( serString, 22);
    /** Set nRF to recive mode ************************************************/
//    __delay32(1000);
//    LDByteWriteSPI(0x27, 0b00100000);   // clear TX_DS (ACK received)
//    LDByteWriteSPI(0x20 , 0b00001011); // Set to receive mode
//    _LATB15 = 1;     // Set CE high to start receiving data
    /* Done with interrupt ****************************************************/
    _LATA4 = 0;
    _T1IF = 0;  // Clear Timer 1 interrupt flag

}
