#include <xc.h>         /* XC8 General Include File */

/******************************************************************************/
/* User Level #define Macros                                                  */
/******************************************************************************/
/* TODO Application specific user parameters used in user.c may go here */

/******************************************************************************/
/* User Function Prototypes                                                   */
/******************************************************************************/

/* TODO User level functions prototypes (i.e. InitApp) go here */

// System functions
void InitApp(void);         /* I/O and Peripheral Initialization */
void delayLong(unsigned long x);

// EE Prom functions
void Write_b_eep( unsigned int badd,unsigned char bdata );
unsigned char Read_b_eep( unsigned int badd );
void Busy_eep ( void );
void setNewAddress(unsigned char newAddress); 	// Set new board address


// Analog functions
int ReadADC0(char ADCnum);

// Base I2C functions
int write_I2C_byte(char byte_out);
char read_I2C_byte(char ackornot);	// ackornot: 1 = respond with an ACK, 0 = don't

// Motor control functions
// #### Motor number is always 0x00 for a single motor or single servo system
// #### In a two motor system, motor number is 0x00, 0x01 or 0x02 for the specific motor, (0x00 = both motors)
void MCUidle();									// Idle the MCU, keep peripherals running
void battCut(unsigned char onOff);
void enable(unsigned char enDis);
void stop_now(int mot);
void fwd(int mot, unsigned char pwm_val);
void rev(int mot, unsigned char pwm_val);
void set_dir(int mot, unsigned char direction);
void set_pwm(int mot, unsigned char pwm_val);
//void adj(int mot, unsigned char speed_adj);
//void accel_to(int mot, unsigned char speed, unsigned char rate);
//void decel_to(int mot, unsigned char speed, unsigned char rate);
//void spin(unsigned char direction, unsigned char speed);
