// ##########################################################################
// ##########################################################################
// I2C Motor driver
//
// Written by Duane Benson
// 9/01/16
//
// For MCU: PIC18FXXK22 family, or similar PIC18F
// 
//
// Edit history:
//
//
// ********************************************
// ####
// #### Project specific notes
// ####
// #### Motor number is always 0x00 for a single motor or single servo system, or for global operations
// #### In a two motor system, motor number is 0x00, 0x01 or 0x02 for the specific motor, (0x00 = both motors)
// ####
// #### 0x10 = default I2C address for PWM motor controller.
// #### 0x12 = Each subsequent PWM motor controller I2C address will be one step up
// ####
// #### 0x30 = default I2C address for servo.
// #### 0x32 = Each subsequent servo I2C address will be one step up
// ####
// #### I2C commad and data structure:
// #### I2C_data[0] = address
// #### I2C_data[1] = command byte
// #### I2C_data[2] = number of bytes to follow (not always needed, but reserved just in case)
// #### I2C_data[3] = motor number: 
// #### I2C_data[4] = parameter value
// #### I2C_data[5] = parameter value
// #### I2C_data[6] = parameter value
// #### I2C_data[7] = parameter value
// ####
// #### I2C commands:
// #### Not all drivers have all commands implemented
// #### 0x00 = battery relay cut off. 0 = off, 1 = on.                  [Addr] [0x00] [0x02] [motor] [0 or 1]
// #### 0x01 = motor enable, 0 = disable, 1 = enable.                   [Addr] [0x01] [0x02] [motor] [0 or 1]
// #### 0x02 = System sleep or idle, 0 = sleep/idle, 1 = wake.          [Addr] [0x02] [0x02] [0x00] [0 or 1]
// #### 0x03 = full stop                                                [Addr] [0x03] [0x01] [motor]
// #### 0x04 = brake, 0 - 255                                           [Addr] [0x04] [0x02] [motor] [Power Level]
// #### 0x05 = direction, 0 = reverse, 1 = forward                      [Addr] [0x05] [0x02] [motor] [Direction]
// #### 0x06 = set speed value (sign magnitude). 0 - 255                [Addr] [0x06] [0x02] [motor] [Power Level]
// #### 0x07 = set speed value (locked anti-phase). 0 - 127 - 255       [Addr] [0x07] [0x02] [motor] [Power Level]
// #### 0x08 = Spin, value1 = dir 1 = right, 2 = left, value2 = speed   [Addr] [0x08] [0x03] [motor] [Direction] [Power Level]
// #### 0x09 = 
// #### 0x0A = 
// #### 0x10 = set R/C servo mode, 0 = analog, 1 = digital
// #### 0x11 = set R/C servo position, 0 = (-), 1 = (+), value 0 to 255 (127 = center)
// #### 0x12 = set R/C servo degrees, 0 = (-), 1 = (+), degree value 0 to 45
// #### 0x13 = center RC servo
// #### 0x14 = Spin right, speed, time in seconds
// #### 0x15 = Spin left, speed, time in seconds
// ####
// ####
// #### 0xF0 = change device I2C address                       [original addr] [0xF0] [0x02] [0x00] [new addr]
// ####
// #### --------------------------------
// ####
// ####



/******************************************************************************/
/* Include files                                                          */
/******************************************************************************/

#include <xc.h>        /* XC8 General Include File */
#include "globals.h"
#include "system.h"        /* System funct/params, like osc/peripheral config */
#include "user.h"          /* User funct/params, such as InitApp */
#include "configuration_bits.c"


void main(void)
{
    // Configure the oscillator for the device
    ConfigureOscillator();

    // Initialize I/O and Peripherals for application
    InitApp();

    // I2C_data[0] = device address
    // I2C_data[1] = Command byte
    // I2C_data[2] = number of bytes to follow
    // I2C_data[3] = Motor # (0 for both, 1, or 2)
    // I2C_data[4] = parameter value1
    // I2C_data[5] = parameter value2
    // I2C_data[6] = parameter value3
    // I2C_data[7] = parameter value4
    
    /*
     * The MCU stays in this loop until called to service an interrupt.
     * The ISR routine reads the I2C buss and fills up the I2C buffer
     * The ISR also sets I2C_good, which directs execution of the state machine
     * Global interrupts are disabled inside the state machine
     * State machine calls the motor control routines
     * After return from the routine, I2C_good is cleared and global interrupts are enabled
	 * The MCU then goes back to idle
    */
    GIE = 1;	// Enable global interrupts
    while (1) {
        if (I2C_good == 1) {
            GIE = 0;
            if (command == smBattRelay) {  // Battery cut off
                // [value1] 0 = off, 1 = on
                battCut(I2C_data[4]);
            } else if (command == smMotorEnable) {  // Enable
                // [Motor], [value1] 0 = disable / 1 = enable
                enable(I2C_data[4]);
            } else if (command == smSystemIdle) { // Idle MCU
			
            } else if (command == smFullStop) { // Stop
                // [Motor] 1, 2, or 3 for both
                stop_now(I2C_data[3]);
            } else if (command == smBrake) { // Braking
			
            } else if (command == smDirection) { // Set direction
                // [Motor], [value1] FWD = 1, REV = 0
//                set_dir(I2C_data[3], I2C_data[4]);
            } else if (command == smSetSpeedSM) { // Set speed 
                // [Motor], [value1] PWM value 0 to 255, 
                // Optional: [value2] = two PWM LSB bits, right justified
//                set_pwm(I2C_data[3], I2C_data[4]);
            } else if (command == smSpin) { // Spin
                // [value1] spin direction: 1 = right, 2 = left, [value2] = speed
//                spin(I2C_data[4], I2C_data[5]);
            } else if (command == smChangeAddress) { // Set new I2C address
                // [value1] new address
                //setNewAddress(I2C_data[4]);  
            } else {
                stop_now(0x00);
            }
            I2C_good = 0;	// Clear the I2C data flag
            GIE = 1;		// Re enable global interrupts
        }
		MCUidle();		// Send this MCU only to idle mode. Peripherals, like PWM, ned to stay on
    }
}

