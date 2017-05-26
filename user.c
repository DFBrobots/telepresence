/******************************************************************************/
/* Files to Include                                                           */
/******************************************************************************/

#include <xc.h>        /* XC8 General Include File */
#include "globals.h"
#include "user.h"
#include "system.h"

/******************************************************************************/
/* User Functions                                                             */
/******************************************************************************/

/* <Initialize variables in user.h and insert code for user algorithms.> */

void InitApp(void)
{
    // Start all ports as cleared digital outputs
    TRISA = 0x00;
    TRISB = 0x00;
    TRISC = 0x00;

    LATA  = 0x00;
    LATB  = 0x00;
    LATC  = 0x00;

    ANSELA = 0x00;
    ANSELB = 0x00;
    ANSELC = 0x00;

    // Ports set to input for input switches and interrupts
    TRISBbits.TRISB6 = 1;   		// RB6 is input: UsrSw
    TRISBbits.TRISB5 = 1;   		// RB5 is input: UsrSw
    TRISBbits.TRISB3 = 1;   		// RB3 is input: !Fault 1
    TRISBbits.TRISB2 = 1;   		// RB2 is input: !Fault 2

    // UsrSw is normally-open active low. It needs to be pull-up
    WPUBbits.WPUB6 = 1;             // Weak pull-up for RB6 / UsrSW
    WPUBbits.WPUB5 = 1;             // Weak pull-up for RB5 / UsrSW
    WPUBbits.WPUB3 = 1;             // Weak pull-up for RB3 / !Fault 1
    WPUBbits.WPUB2 = 1;             // Weak pull-up for RB2 / !Fault 2

    INTCON2bits.nRBPU = 0;    		// 0 = enable pull-ups on PORTB, provided individual WPUB bits are set

    // Ports to be set to analog input
    ANSELA  |= 1 << 0;      		// AN0 is current sense 2
    ANSELA  |= 1 << 1;      		// AN1 is current sense 1

#ifdef __LBDMTHqb

    // Set I2C pins as inputs
    TRISBbits.TRISB0 = 1;        	// RB0 = SDA
    TRISBbits.TRISB1 = 1;        	// RB1 = SCL

    //SSP2ADD = 0x9F;         		// 100 KHz I2C clock with 64 MHz system clock
    SSP1STATbits.SMP = 1;   		// Slew off
    SSP1CON1bits.SSPEN = 1; 		// Enable I2C
    SSP1CON1 = 0b00110110;

    SSP1STAT = 0x80;
    // SSPEN enabled; WCOL no_collision; CKP disabled; SSPM 7 Bit Polling; SSPOV no_overflow; 

    SSP1CON1 = 0b00110110;
    // ACKEN disabled; GCEN disabled; PEN disabled; ACKDT acknowledge; RSEN disabled; RCEN disabled; SEN enabled; 
    //SSP1CON1bits.CKP = 1;
    SSP1CON2 = 0x01;
    // SBCDE disabled; BOEN disabled; SCIE disabled; PCIE disabled; DHEN disabled; SDAHT 100ns; AHEN disabled; 
    SSP1CON3 = 0x00;
    // MSK0 127; 
    SSP1MSK = (0x7F << 1);  // adjust UI mask for R/nW bit            

    
	// To do: First time processing. Read address in EEPROM. If it's blank, then set it to the default address.
    I2C_addr    = default_I2C_addr;	// To do: Read address from EEPROM
    SSP1ADD     = I2C_addr << 1;
    
	
    // Set up PWM
	TRISCbits.TRISC5 = 1;		// Disable pin for PWM1 while setting up for PWM
	TRISCbits.TRISC0 = 1;		// Disable pin for PWM2 while setting up for PWM
	
	CCPTMRS0 = 0b00000000; 		// Set up timer 2 as the PWM resource for both CCP1 and CCP2
	PR2 = 0xFF;					// Set PWM period to 20KHz
	CCP1CON = 0b00001100;		// Configure PWM 1
	CCP2CON = 0b00001100; 		// Configure PWM 2
	
	CCPR1L = 0x00;				// Clear the duty cycle LSBs to start out with a duty cycle of 0
	CCPR2L = 0x00;				// Clear the duty cycle LSBs to start out with a duty cycle of 0
	
	TMR1IF = 0;					// Clear the timer interrupt flag
	TMR2IF = 0;
	
	T2CONbits.T2CKPS1 = 1;		// Timer prescale to 4
	TMR2ON = 1;					// Turn on timer 2
	
	TRISCbits.TRISC5 = 0;		// Enable pin for PWM1 while setting up for PWM
	TRISCbits.TRISC0 = 0;		// Enable pin for PWM2 while setting up for PWM
	
   
    // Set up interrupts
    IPEN  = 0;              	// Disable Interrupt priorities, as they are not needed
    INTCON = 0b00000000;    	// Start with everything cleared

    IOCBbits.IOCB5 = 1;     	// Enable IOC (Interrrupt on change) on pin RB5
    IOCBbits.IOCB6 = 1;     	// Enable IOC (Interrrupt on change) on pin RB5

    SSPIE = 1;					// Enable I2C interrupt
    PEIE = 1;               	// Enable peripheral interrupts

    RBIE = 1;               	// Enable Port B int on change: RB4 and RB5 are Usr 1 and Usr 2
    
    // Temporary code
    TRISAbits.TRISA1 = 0;
    // End of temporary code
#endif
    
#ifdef __LBRSEv11
    // Ports set to input for input switches and interrupts
    TRISBbits.TRISB5 = 1;   		// RB5 is input: UsrSw
    TRISBbits.TRISB3 = 1;   		// RB3 is input: !Fault 1
    TRISBbits.TRISB2 = 1;   		// RB2 is input: !Fault 2

    // Ports to be set to analog input
    ANSELA  |= 1 << 0;      		// AN0 is current sense 2
    ANSELA  |= 1 << 1;      		// AN1 is current sense 1
    
    // UsrSw is normally-open active low. It needs to be pull-up
    WPUBbits.WPUB5 = 1;             // Weak pull-up for RB5 / UsrSW
    WPUBbits.WPUB3 = 1;             // Weak pull-up for RB3 / !Fault 1
    WPUBbits.WPUB2 = 1;             // Weak pull-up for RB2 / !Fault 2

    INTCON2bits.nRBPU = 0;    		// 0 = enable pull-ups on PORTB, provided individual WPUB bits are set

    // Set I2C pins as inputs
    TRISCbits.TRISC4 = 1;        	// RC4 = SDA
    TRISCbits.TRISC3 = 1;        	// RC3 = SCL

    //SSP2ADD = 0x9F;         		// 100 KHz I2C clock with 64 MHz system clock
    SSP1STATbits.SMP = 1;   		// Slew off
    SSP1CON1bits.SSPEN = 1; 		// Enable I2C
    SSP1CON1 = 0b00110110;

    SSP1STAT = 0x80;
    // SSPEN enabled; WCOL no_collision; CKP disabled; SSPM 7 Bit Polling; SSPOV no_overflow; 

    SSP1CON1 = 0b00110110;
    // ACKEN disabled; GCEN disabled; PEN disabled; ACKDT acknowledge; RSEN disabled; RCEN disabled; SEN enabled; 
    //SSP1CON1bits.CKP = 1;
    SSP1CON2 = 0x01;
    // SBCDE disabled; BOEN disabled; SCIE disabled; PCIE disabled; DHEN disabled; SDAHT 100ns; AHEN disabled; 
    SSP1CON3 = 0x00;
    // MSK0 127; 
    SSP1MSK = (0x7F << 1);  // adjust UI mask for R/nW bit            

    
	// To do: First time processing. Read address in EEPROM. If it's blank, then set it to the default address.
    I2C_addr    = default_I2C_addr;	// To do: Read address from EEPROM
    SSP1ADD     = I2C_addr << 1;
    
	
    // Set up PWM
	TRISCbits.TRISC5 = 1;		// Disable pin for PWM1 while setting up for PWM
	TRISCbits.TRISC0 = 1;		// Disable pin for PWM2 while setting up for PWM
	
	CCPTMRS0 = 0b00000000; 		// Set up timer 2 as the PWM resource for both CCP1 and CCP2
	PR2 = 0xFF;					// Set PWM period to 20KHz
	CCP1CON = 0b00001100;		// Configure PWM 1
	CCP2CON = 0b00001100; 		// Configure PWM 2
	
	CCPR1L = 0x00;				// Clear the duty cycle LSBs to start out with a duty cycle of 0
	CCPR2L = 0x00;				// Clear the duty cycle LSBs to start out with a duty cycle of 0
	
	TMR1IF = 0;					// Clear the timer interrupt flag
	TMR2IF = 0;
	
	T2CONbits.T2CKPS1 = 1;		// Timer prescale to 4
	TMR2ON = 1;					// Turn on timer 2
	
	TRISCbits.TRISC5 = 0;		// Enable pin for PWM1 while setting up for PWM
	TRISCbits.TRISC0 = 0;		// Enable pin for PWM2 while setting up for PWM
	
   
    // Set up interrupts
    IPEN  = 0;              	// Disable Interrupt priorities, as they are not needed
    INTCON = 0b00000000;    	// Start with everything cleared

    IOCBbits.IOCB5 = 1;     	// Enable IOC (Interrrupt on change) on pin RB5

    SSPIE = 1;					// Enable I2C interrupt
    PEIE = 1;               	// Enable peripheral interrupts
    RBIE = 1;               	// Enable Port B int on change: RB4 and RB5 are Usr 1 and Usr 2
#endif
}


// Delay time in multiples of 10 ms
void delayLong(unsigned long x) {
    unsigned long i;
    for (i = 0; i < x; i++) {
        __delay_ms(10);	// System library function
    }
}

void Write_b_eep( unsigned int badd,unsigned char bdata ) {
}

unsigned char Read_b_eep( unsigned int badd ) {
    return(0);
}

void Busy_eep ( void ) {
}

// Set new board address
void setNewAddress(unsigned char newAddress) { 
}


// Idle the MCU, keep peripherals running
void MCUidle() {
    OSCCONbits.IDLEN = 1;       // Enable MCU idle
    OSCCONbits.SCS1 = 1;        // Switch to RC_IDLE mode
}



// Read ADC 0
// To do: use the parameter to select which ADC to use
int ReadADC0(char ADCnum) {
    int ADCvalue;
    GO = 1;
    while (GO);
    ADCvalue = (ADRESH << 2) | (ADRESL >> 6);
    return (ADCvalue);
}



int write_I2C_byte(char byte_out) {
	SSP2BUF = byte_out;
        while (SSP2STATbits.RW2);       // Wait for write to complete
	while (SSP2CON2bits.ACKSTAT);   // Wait for acknowledge from slave
    
    return(0);
}


char read_I2C_byte(char ackornot) {
char dist;

    SSP2CON2bits.RCEN = 1;              // Set read mode
    while (SSP2CON2bits.RCEN);          // Wait until read is complete

    dist = SSP2BUF;                     // Read I2C data buffer

    if (ackornot == 1) {                // Only acknowledge if not the last char
        SSP2CON2bits.ACKDT = 0;         // Set acknowledge
        SSP2CON2bits.ACKEN = 1;         // Send acknowledge bit
        while (SSP2CON2bits.ACKEN);     // Wait for acknowledge to end
    }

    return(dist);
}



// Will be used if there's a master battery relay
void battCut(unsigned char onOff) {
    
}

// Typically used for a digital enable/disable control, like a disable/enable input on a driver chip
void enable(unsigned char enDis) {
    if (enDis == 1) {
        MOTen = 1;
    } else {
        MOTen = 0;
    }
}


void stop_now(int mot) {
    if (mot == 0) { 			// Both motors
        CCPR1H = 0x00;
		CCPR2H = 0x00;
     } else if (mot == 1) {		// Motor 1
        CCPR1H = 0x00;
    } else if (mot == 2) {		// Motor 2
        CCPR2H = 0x00;
    } else {
        
    }
}

// Set to drive to forward first, then set the speed
// To do: put in some checking for too big of a speed change, or a direction reversal
void fwd(int mot, unsigned char pwm_val) {
	DIR1 = 1;	// Direction, 1 = forward, 2 = reverse
	DIR2 = 1;
    if (mot == 0) { 			// Both motors
        CCPR1H = pwm_val;
		CCPR2H = pwm_val;
     } else if (mot == 1) {		// Motor 1
        CCPR1H = pwm_val;
    } else if (mot == 2) {		// Motor 2
        CCPR2H = pwm_val;
    } else {
        
    }
}

// Set to drive to reverse first, then set the speed
// To do: put in some checking for too big of a speed change, or a direction reversal
void rev(int mot, unsigned char pwm_val) {
	DIR1 = 0;	// Direction, 1 = forward, 2 = reverse
	DIR2 = 0;
    if (mot == 0) { 			// Both motors
        CCPR1H = pwm_val;
		CCPR2H = pwm_val;
     } else if (mot == 1) {		// Motor 1
        CCPR1H = pwm_val;
    } else if (mot == 2) {		// Motor 2
        CCPR2H = pwm_val;
    } else {
        
    }
}

// 1 = forward, 0 = reverse
void set_dir(int mot, unsigned char direction) {
    if (mot == 0) { 			// Both motors
        DIR1 = direction;
        DIR2 = direction;
     } else if (mot == 1) {		// Motor 1
        DIR1 = direction;
    } else if (mot == 2) {		// Motor 2
         DIR2 = direction;
    } else {
        
    }
}

// Just set the PWM value, regardless of the direction
// To do: put in some checking for too big of a speed change
void set_pwm(int mot, unsigned char pwm_val) {
    if (mot == 0) { 			// Both motors
        CCPR1H = pwm_val;
		CCPR2H = pwm_val;
     } else if (mot == 1) {		// Motor 1
        CCPR1H = pwm_val;
    } else if (mot == 2) {		// Motor 2
        CCPR2H = pwm_val;
    } else {
        
    }
}

/*
void adj(int mot, unsigned char speed_adj) {
}

// Adjusts to motor speed gradually. mot = motor, speedDelta = amount to change speed, rate = speed of change
void accel_to(int mot, unsigned char speed, unsigned char rate) {
}

// Adjusts to motor speed gradually. mot = motor, speedDelta = amount to change speed, rate = speed of change
void decel_to(int mot, unsigned char speed, unsigned char rate) {
}

void spin(unsigned char direction, unsigned char speed) {
}
*/