/******************************************************************************/
/*Files to Include                                                            */
/******************************************************************************/

#include <xc.h>        /* XC8 General Include File */
#include "globals.h"
#include "user.h"
#include "system.h"


/******************************************************************************/
/* Interrupt Routines                                                         */
/******************************************************************************/

/* Interrupt priorities have been disabled. All are equal */

    /*
     * The MCU stays in its main loop, or stays idling until called to service an interrupt.
     * The ISR routine reads the I2C buss and fills up the I2C buffer
     * The ISR also sets I2C_good, which allows execution of the state machine in main()
     * 
     * The I2C buffer is used even if a non-I2C interrupt is called.
     * The buffer is filled as though an I2C command came in
    */

void interrupt isr(void) {
    if (INTCONbits.PEIE == 1 && PIE1bits.SSP1IE == 1 && PIR1bits.SSP1IF == 1) {     // If I2C int enabled and flagged
        while (!(SSP1STATbits.I2C_START));   // Wait until start bit detected. To do: add a time-out error escape
        SSP1CON1bits.CKP = 0;                // Hold clock until ready again
        I2C_byteInCount = 1;				 // Counter for the number of bytes the master said it would send
        SSPIF = 0;
        I2C_address = SSP1BUF;               // Save address
        SSP1CON1bits.CKP = 1;                // Release the clock
        while (!(SSP1STATbits.I2C_STOP)) {   // Repeat until "P" indicates STOP bit. To do: add a time-out error escape
            if (SSPIF){
                SSP1CON1bits.CKP = 0;        // Hold clock until ready again
                SSPIF = 0;
                I2C_data[I2C_byteInCount++] = SSPBUF;
                SSP1CON1bits.CKP = 1;        // Release the clock
            }
        }
        I2C_length = I2C_byteInCount - 1;      // May need to know the length outside of the ISR
        SSPIF = 0;                          // Clear flags
        SSP1STATbits.BF = 0;
        I2C_good = 1;                       // Indicate a good I2C receive
    } else if(RBIE && RBIF) {				// Port B interrupt on change
        // At the moment, for debugging purposes, this is being used to simulate the motor enable command (MOTen)
        Usr1 = !UsrSw;
        I2C_command = 0x01;					// Enable command
        I2C_byteCount = 0x02;					// Two bytes to follow
        I2C_motSelect = 0x00;					// 0x00 = both motors
        if(!UsrSw) {						// Set parameter value, 1 = enable, 0 = disable
            I2C_param1 = 1;
        } else {
            I2C_param1 = 0;
        }
        RBIF = 0;							// Clear the interrupt on change flag
        I2C_good = 1;           			// Simulate a good I2C receive
    }
}

