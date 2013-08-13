/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/
#define rfStatLED   LATBbits.LATB4
#define CS_nRF       LATBbits.LATB11
#define CS_Axis     LATBbits.LATB2
#define button1     PORTBbits.RB7
/* TODO Application specific user parameters used in user.c may go here */

/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/

/* TODO User level functions prototypes (i.e. InitApp) go here */

void InitApp(void);         /* I/O and Peripheral Initialization              */
void initSerial(void);      /* UART1 init                                     */
void initSPI1(void);        /* SPI1 init                                      */
void initSPI2(void);        /* SPI2 init                                      */
void initnRF(void);         /* nRF wireless tranceiver                        */
void InitMPU6050(unsigned char I2Caddr); /* initialization of MPU6050         */
void InitHMC5883L(void);    /* initialization of HMC5883L                     */
