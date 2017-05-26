#define HARDWARE            "Dual motor controll"
#define HARDWARE_desc       ""
#define VERSION             ""
#define URL                 "https://github.com/KlaatuBaradaNikto/telepresence_base_PIC18"


/******************************************************************************/
/*Files to Include                                                            */
/******************************************************************************/
#include <xc.h>             /* XC8 General Include File */


/******************************************************************************/
/*Personality settings                                                        */
/******************************************************************************/
//#define __PIC18F46K22
//#define __PIC18F45K22
//#define __PIC18F44K22
//#define __PIC18F43K22
//#define __PIC18F25K22
#define __PIC18F25K50

// Board and MCU personality settings
#define __NoXtal
#define __NoMotDualCCP
#define __NoLEDbank
#define __LBDMTHqb
//#define __LBRSEv11

// Specific motor driver being used
//#define __TB6612FNG				// Toshiba dual 1 Amp driver chip. Data sheet: https://toshiba.semicon-storage.com/info/docget.jsp?did=10660&prodName=TB6612FNG
#define __PO24v13					// Pololu 24v13 sngle motor driver. Data sheet: https://www.pololu.com/product/2992


#define I2C_buffSize           8
#define default_I2C_addr	0x10	// Default address

/******************************************************************************/
// Motor operation settings
/******************************************************************************/
#ifdef __LBRSEv11
    #define MOTen   LATBbits.LB4
    #define DIR1	LATCbits.LC5    // A = motor 1
    #define nSLP1	LATBbits.LB1
    #define DIR2	LATCbits.LC0    // B = motor 2
    #define nSLP2	LATBbits.LB0

    #define nFALT1  PORTBbits.RB3
    #define nFALT2  PORTBbits.RB2

    #define UsrSw   PORTBbits.RB5
#endif
#ifdef __LBDMTHqb
    #define MOTen   LATAbits.LA1    // Change back to LA5
    #define DIR1	LATAbits.LA4    // A = motor 1
    #define nSLP1	LATCbits.LC0
    #define DIR2	LATCbits.LC6    // B = motor 2
    #define nSLP2	LATCbits.LC7

    #define nFALT1  PORTAbits.RA2
    #define nFALT2  PORTAbits.RA3

    #define UsrSw   PORTBbits.RB6

#endif

#ifdef __PIC18F25K50
    #define RBIE IOCIE
    #define RBIF IOCIF
    #define CCPTMRS0 CCPTMRS
    #define SSP2BUF SSPBUF
    #define SSP2STAT SSPSTAT
    #define SSP2STATbits SSPSTATbits
    #define RW2 RW
    #define SSP2CON2bits SSPCON2bits
#endif

/******************************************************************************/
// Macros
/******************************************************************************/
#define I2C_address 	I2C_data[0]		// I2C_data[0] = device address
#define I2C_command 	I2C_data[1]		// I2C_data[1] = Command byte
#define I2C_byteCount 	I2C_data[2]		// I2C_data[2] = number of bytes to follow
#define I2C_motSelect 	I2C_data[3]		// I2C_data[3] = Motor # (0 for both, 1, or 2)
#define I2C_param1		I2C_data[4] 	// I2C_data[4] = parameter value1
#define I2C_param2		I2C_data[5] 	// I2C_data[5] = parameter value2
#define I2C_param3		I2C_data[6] 	// I2C_data[6] = parameter value3
#define I2C_param4		I2C_data[7] 	// I2C_data[7] = parameter value4


// State machine states
#define smBattRelay     0x00
#define smMotorEnable   0x01
#define smSystemIdle    0x02
#define smFullStop      0x03
#define smBrake         0x04
#define smDirection     0x05
#define smSetSpeedSM    0x06
#define smSetSpeedLAP   0x07
#define smSpin          0x08

#define smChangeAddress 0xF0


/******************************************************************************/
/*Global variables                                                           */
/******************************************************************************/

struct i2cv {
	char address;
	char command_byte;
    char bytes_following;
    char motor_number;
	char data1;
	char data2;
	char data3;
	char data4;
} I2C_dataset;

struct motCondition {
	unsigned char mot1dir;
	unsigned char mot1dir;
	unsigned char mot1CCPRL;
	unsigned char mot1CCPRL;
	unsigned char mot1CCPRH;
	unsigned char mot1CCPRH;
}
	

bit     SleepFlag;

char    I2C_addr;
int     I2C_good = 0;
int     I2C_bitCount = 0;
int     I2C_length = 0;
char    I2C_data[I2C_buffSize];
//char    I2C_status;
//char    I2C_value;
//char    I2C_chars[2];
//char    I2Cspeed[2];


// These would be better if local as pointers to structs, but that'll come later
int Usr1, Usr2;                // Usr1 holds the value of the Usr1 switch the most recent time it was checked. Usr2, the same for switch Usr2

