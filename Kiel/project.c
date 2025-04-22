// Created by: Dhanvin Tandon
// Date: March 31, 2025

// === INCLUDES ===
#include <stdint.h>
#include "tm4c1294ncpdt.h"            // Header for TM4C1294 microcontroller
#include "SysTick.h"                   // System tick timer functions
#include "PLL.h"                       // Phase-Locked Loop configuration           
#include "Lights.h"                    // LED control functions
#include "VL53L1X_api.h"               // Time-of-Flight (ToF) sensor API
#include "uart.h"                      // UART communication functions

// === COMMENTED OUT GLOBAL VARIABLES FROM MILESTONE 1 ===
/*
uint8_t motorRunning = 0; // 0 = stopped, 1 = running
uint8_t motorState = 0;   // 0 = stopped, 1 = running clockwise, 2 = running counter-clockwise
uint8_t motorAngle = 1;   // 1 = 11.25°, 4 = 45° (in steps)
uint8_t homeFlag = 0;     // 0 = not homing, 1 = homing
uint32_t currentPosition = 0; // Current position in steps (0-512 for 0-360 degrees)
uint8_t homePosition = 0;   // Home position (0 degrees)
uint8_t stepCounter = 0;    // Counter for steps to track angle progress
*/

// === I2C CONSTANTS ===
// I2C control register bit definitions
#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable

#define TRIES                   5           // Number of receive attempts before giving up
#define STEPS                   3          // Number of vertical steps/planes for 3D scanning (matches Python STEPS constant)

// Global variable to track stepper motor angle (0-2047 for 0-360 degrees)
volatile int angle = 0;

// === INTERRUPT CONTROL FUNCTIONS ===
// Enable interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}

// Low power wait for interrupt
void WaitForInt(void)
{    __asm("    wfi\n");
}

// === PORT INITIALIZATION FUNCTIONS ===
// Initialize Port N for LEDs D1, D2
void PortN_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;                 // Activate the clock for Port N
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R12) == 0){};        // Allow time for clock to stabilize
		
	GPIO_PORTN_DIR_R |= 0x03;                                 // Enable PN0 and PN1 as outputs													
	GPIO_PORTN_DEN_R |= 0x03;                                 // Enable PN0 and PN1 as digital pins
	GPIO_PORTN_AFSEL_R &= ~0x03;                              // Disable alt function on PN0 and PN1
	GPIO_PORTN_AMSEL_R &= ~0x03;                              // Disable analog function on PN0 and PN1
	
	// Flash LED to signal initialization completed
	GPIO_PORTN_DATA_R ^= 0x03;
	SysTick_Wait10ms(10);
	GPIO_PORTN_DATA_R ^= 0x03;
	return;
}

// Initialize Port F for LEDs D3, D4
void PortF_Init(void){
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;                 	// Activate the clock for Port F
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R5) == 0){};          // Allow time for clock to stabilize
		
	GPIO_PORTF_DIR_R |= 0x11;                                  // Enable PF0 and PF4 as outputs
	GPIO_PORTF_DEN_R |= 0x11;                                  // Enable PF0 and PF4 as digital pins
	GPIO_PORTF_AFSEL_R &= ~0x11;                               // Disable alt function on PF0 and PF4
	GPIO_PORTF_AMSEL_R &= ~0x11;                               // Disable analog function on PF0 and PF4
		
	// Flash LED to signal initialization completed
	GPIO_PORTF_DATA_R ^= 0x11;
	SysTick_Wait10ms(10);
	GPIO_PORTF_DATA_R ^= 0x11;
	return;
}

// Initialize Port M for PM0 & PM1 as pushbutton inputs
void PortM_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                 // Activate the clock for Port M
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){};      // Allow time for clock to stabilize
		
	GPIO_PORTM_DIR_R |= 0x03;                                 // Enable PM0 and PM1 as inputs 
  GPIO_PORTM_DEN_R |= 0x03;                                  // Enable PM0 and PM1 as digital pins
	GPIO_PORTM_PDR_R |= 0x03;                                  // Enable Pull-Down resistors for active high input for PM0 & PM1
	GPIO_PORTM_AFSEL_R &= ~0x03;                               // Disable alt function on PM0 & PM1
	GPIO_PORTM_AMSEL_R &= ~0x03;                               // Disable analog function on PM0 & PM1
	return;
}

// Initialize Port J for PJ0 & PJ1 as pushbutton inputs
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;                  // Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};         // Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x03;                                 // Make PJ0 and PJ1 inputs 
  GPIO_PORTJ_DEN_R |= 0x03;                                  // Enable digital I/O on PJ0 and PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000FF;                          // Configure PJ0 and PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x03;                               // Disable analog functionality on PJ0 and PJ1		
	GPIO_PORTJ_PUR_R |= 0x03;                                  // Enable weak pull up resistor on PJ0 and PJ1
}

// Initialize interrupt for GPIO Port J (for pushbuttons)
void PortJ_Interrupt_Init(void){
	GPIO_PORTJ_IS_R = 0;                                       // PJ1 is Edge-sensitive 
	GPIO_PORTJ_IBE_R = 0;                                      // PJ1 is not triggered by both edges 
	GPIO_PORTJ_IEV_R = 0;                                      // PJ1 is falling edge event 
	GPIO_PORTJ_ICR_R = 0x02;                                   // Clear interrupt flag by setting proper bit in ICR register
	GPIO_PORTJ_IM_R = 0x02;                                    // Arm interrupt on PJ1 by setting proper bit in IM register
    
	NVIC_EN1_R = 0x00080000;                                   // Enable interrupt 51 in NVIC (which is in Register EN1)
	
	NVIC_PRI12_R = 0xA0000000;                                 // Set interrupt priority to 5

	EnableInt();                                                // Enable Global Interrupt
}

// Initialize I2C for communicating with the VL53L1X ToF sensor
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;                      // Activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;                    // Activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};                     // Wait for Port B clock to be ready

  GPIO_PORTB_AFSEL_R |= 0x0C;                                 // Enable alt function on PB2,3 (SCL and SDA)
  GPIO_PORTB_ODR_R |= 0x08;                                   // Enable open drain on PB3 only (SDA)

  GPIO_PORTB_DEN_R |= 0x0C;                                   // Enable digital I/O on PB2,3
//  GPIO_PORTB_AMSEL_R &= ~0x0C;                               // Disable analog functionality on PB2,3

  // Configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    // TED
  I2C0_MCR_R = I2C_MCR_MFE;                                   // Master function enable
  I2C0_MTPR_R = 0b0000000000000101000000000111011;            // Configure for 100 kbps clock (with glitch suppression)
//  I2C0_MTPR_R = 0x3B;                                        // Configure for 100 kbps clock
        
}

// Delay used for stepper motor control timing
#define DELAY 200

// Initialize Port H for stepper motor control
void PortH_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                  // Activate clock for Port H
	while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R7) == 0){};       // Allow time for clock to stabilize
  
	GPIO_PORTH_DIR_R |= 0x0F;                                  // Enable PH0-PH3 as outputs for stepper motor control
	GPIO_PORTH_DEN_R |= 0x0F;                                  // Enable PH0-PH3 as digital pins
	GPIO_PORTH_AFSEL_R &= ~0x0F;                               // Disable alt function on PH0-PH3
	GPIO_PORTH_AMSEL_R &= ~0x0F;                               // Disable analog function PH0-PH3
}

// Spin stepper motor counter-clockwise by specified number of steps
// Updates the global angle variable to track position
void spinCCW(uint32_t steps, int* angle){													
	for(uint32_t i=0; i< steps/4; i++){
		// Standard 4-step sequence for stepper motor control (half-stepping)
		GPIO_PORTH_DATA_R = 0b00000011;      // Step 1: Energize coils 0 and 1
		SysTick_Wait10us(DELAY);											
		GPIO_PORTH_DATA_R = 0b00001001;      // Step 2: Energize coils 0 and 3
		SysTick_Wait10us(DELAY);
		GPIO_PORTH_DATA_R = 0b00001100;      // Step 3: Energize coils 2 and 3
		SysTick_Wait10us(DELAY);
		GPIO_PORTH_DATA_R = 0b00000110;      // Step 4: Energize coils 1 and 2
		SysTick_Wait10us(DELAY);
		*angle = (*angle-4)%2048;            // Update angle position (4 steps = 11.25°)
	}
	
	GPIO_PORTH_DATA_R = 0b00000000;          // Turn off all coils when done
}

// Spin stepper motor clockwise by specified number of steps
// Updates the global angle variable to track position
void spinCW(uint32_t steps, int* angle){													
	for(uint32_t i=0; i< steps/4; i++){
		// Same steps as CCW but in reverse order
		GPIO_PORTH_DATA_R = 0b00000011;      // Step 1: Energize coils 0 and 1
		SysTick_Wait10us(DELAY);											
		GPIO_PORTH_DATA_R = 0b00000110;      // Step 2: Energize coils 1 and 2
		SysTick_Wait10us(DELAY);
		GPIO_PORTH_DATA_R = 0b00001100;      // Step 3: Energize coils 2 and 3
		SysTick_Wait10us(DELAY);
		GPIO_PORTH_DATA_R = 0b00001001;      // Step 4: Energize coils 0 and 3
		SysTick_Wait10us(DELAY);
		*angle = (*angle+4)%2048;            // Update angle position (4 steps = 11.25°)
	}
	
	GPIO_PORTH_DATA_R = 0b00000000;          // Turn off all coils when done
}

// === COMMENTED OUT CODE FOR FIRST MILESTONE ===
/*
void updateMotorPosition(uint8_t direction) {
    // direction: 1 for clockwise, 2 for counter-clockwise
    
    if (direction == 1) {
        // Update position counter (clockwise increases)
        currentPosition = (currentPosition + 1) % 512; // 512 steps for full 360 degrees rotation
    } else {
        // Update position counter (counter-clockwise decreases)
        if (currentPosition == 0)
            currentPosition = 512; // Wrap around from 0 to 512
        else
            currentPosition--;
    }
    
    // Update step counter and check if we should blink LED3
    stepCounter = (stepCounter + 1) % motorAngle;
    if (stepCounter == 0 && motorState != 0) {
        // Blink LED3 (PF4) when the step count matches the selected angle
        GPIO_PORTF_DATA_R |= 0x01; // Turn on LED3 (PF0)
        SysTick_Wait10us(500);     // Small delay for visible blink
        GPIO_PORTF_DATA_R &= ~0x01; // Turn off LED3 (PF4)
    }
}

// Function to handle Button 0 (PJ0) press
void handleButton0() {
    if (motorState == 0) {
        // Motor is stopped, start it clockwise
        motorState = 1; // Set state to running clockwise
        GPIO_PORTN_DATA_R |= 0x02; // Turn on LED0 (PN0)
        
        // Update LED2 based on the current angle setting
        if (motorAngle == 1) {
            GPIO_PORTF_DATA_R |= 0x10; // Turn on LED2 (PF0) for 11.25°
        } else {
            GPIO_PORTF_DATA_R &= ~0x10; // Turn off LED2 (PF0) for 45°
        }
    } else if (motorState == 1 || motorState == 2) {
        // Motor is running, stop it
        motorState = 0; // Set state to stopped
        GPIO_PORTN_DATA_R &= ~0x02; // Turn off LED0 (PN0)
        GPIO_PORTH_DATA_R = 0x00; // Turn off motor control signals
        
        // Turn off LEDs 2 and 3 when motor stops
        GPIO_PORTF_DATA_R &= ~0x11; // Turn off LED2 (PF0) and LED3 (PF4)
			
    }
}

// Function to handle Button 1 (PJ1) press
void handleButton1() {
    if (motorState == 1) {
        // Motor is running clockwise, reverse to counter-clockwise
        motorState = 2; // Set state to running counter-clockwise
        GPIO_PORTN_DATA_R &= ~0x01; // Turn off LED1 (PN1)
    } else if (motorState == 2) {
        // Motor is running counter-clockwise, reverse to clockwise
        motorState = 1; // Set state to running clockwise
        GPIO_PORTN_DATA_R |= 0x01; // Turn on LED1 (PN1)
    }
}

// Function to handle Button 2 (PM1) press - Toggle angle
void handleButton2() {
    if (motorAngle == 16) {
        // Change from 11.25° to 45°
        motorAngle = 64; // 4 steps = 45°
        GPIO_PORTF_DATA_R &= ~0x10; // Turn off LED2 (PF0) for 45°
    } else {
        // Change from 45° to 11.25°
        motorAngle = 16; // 1 step = 11.25°
        GPIO_PORTF_DATA_R |= 0x10; // Turn on LED2 (PF0) for 11.25°
    }
    
    // Reset step counter when angle changes
    stepCounter = 0;
}

// Function to handle Button 3 (PM0) - Home button
void handleButton3() {
    // Save previous motor state
    uint8_t prevMotorState = motorState;
    
    // Override motor state - stop normal operation
    motorState = 0;
    
    // Set home flag to indicate homing operation
    homeFlag = 1;
    
    // Turn on LED indicator for homing operation
    GPIO_PORTN_DATA_R |= 0x02; // Use LED1 (PN1) as homing indicator
    
    // Calculate shortest path to home
    int16_t stepsToHome;
    uint8_t homeDirection;
    
    if (currentPosition == homePosition) {
        // Already at home position
        homeFlag = 0;
        // Turn motor off and clear indicators
        GPIO_PORTN_DATA_R &= ~0x03; // Turn off LED0 and LED1
        GPIO_PORTF_DATA_R &= ~0x11; // Turn off LED2 and LED3
        GPIO_PORTH_DATA_R = 0x00;   // Turn off motor control signals
        return;
    }
    
    if (currentPosition > homePosition) {
        stepsToHome = currentPosition - homePosition;
        if (stepsToHome <= 256) {
            homeDirection = 2; // Counter-clockwise
        } else {
            homeDirection = 1; // Clockwise
            stepsToHome = 512 - stepsToHome;
        }
    } else { // currentPosition < homePosition
        stepsToHome = homePosition - currentPosition;
        if (stepsToHome <= 256) {
            homeDirection = 1; // Clockwise
        } else {
            homeDirection = 2; // Counter-clockwise
            stepsToHome = 512 - stepsToHome;
        }
    }
    
    // Move to home position
    while (currentPosition != homePosition && homeFlag) {
        if (homeDirection == 1) {
            spinCW(4, &angle); // Adjust number as needed
            updateMotorPosition(1); // Update position tracking for clockwise movement
        } else {
            spinCCW(4, &angle); // Adjust number as needed
            updateMotorPosition(2); // Update position tracking for counter-clockwise movement
        }
    }
    
    // After reaching home, turn everything off
    motorState = 0; // Ensure motor state is set to off
    GPIO_PORTN_DATA_R &= ~0x03; // Turn off LED0 and LED1
    GPIO_PORTF_DATA_R &= ~0x11; // Turn off LED2 and LED3
    GPIO_PORTH_DATA_R = 0x00;   // Turn off motor control signals
    
    // Reset home flag
    homeFlag = 0;
} */
// Interrupt handler for Port J (buttons)
// This is called when PJ1 button is pressed
// The purpose is to return the scanner to its home position (angle = 0)
void GPIOJ_IRQHandler(void){
  if((GPIO_PORTJ_DATA_R & 0b0010) == 0){  // Check if PJ1 was pressed (active low)
			while(1){
					if(angle>0){
						// Keep rotating clockwise until back at home position (angle = 0)
						// This ensures the scanner returns to a consistent starting position
						spinCW(4, &angle);
					}
					else {
						break;  // Exit once we reach home position
					}
			}
	}
	
	GPIO_PORTJ_ICR_R = 0x02;  // Acknowledge the interrupt to prevent re-triggering
}

// Take a single 360-degree measurement with Time-of-Flight (ToF) sensor
// This function performs a complete scan at one position
// Takes 32 measurements at 11.25° intervals for a full 360° rotation
// Stores distance values in the provided distances array for later processing
int takeMeasurement(uint16_t distances[32], uint16_t dev, uint8_t ready){
	int status = 0;
	uint16_t Distance;
	uint32_t steps = 64;  // 64 steps = 11.25° rotation (with stepper motor configuration)
	                       // (32 measurements × 11.25° = 360° full rotation)
	
	// Initialize and start the ToF sensor VL53L1X
	VL53L1X_ClearInterrupt(dev);  // Clear any pending interrupts on the sensor
	status = VL53L1X_SensorInit(dev);  // Initialize the ToF sensor
	status = VL53L1X_StartRanging(dev);  // Begin the ranging/measurement mode

	// Take 32 measurements (full 360° rotation) at even intervals		
	for (int i = 0; i < 32; i++) {
		
			// Wait for sensor data to be ready before taking measurement
			// This ensures we have valid data from the sensor
			while (ready == 0) {
					VL53L1X_CheckForDataReady(dev, &ready);
					VL53L1_WaitMs(dev, 5);  // Small delay to prevent CPU hogging
			}
			ready = 0;  // Reset data ready flag for next reading

			spinCCW(steps, &angle);  // Move stepper motor 11.25° counter-clockwise
			                         // This positions the sensor for the next measurement point

			// Get distance measurement from the ToF sensor and store it in the array
			VL53L1X_GetDistance(dev, &Distance);
			distances[i] = Distance;  // Store the distance value in mm
			VL53L1X_ClearInterrupt(dev);  // Clear the interrupt to prepare for next reading

			SysTick_Wait10ms(1);  // Short delay between measurements for stability
	}

	// Return to home position after completing the scan by rotating back clockwise
	// This ensures consistent positioning for the next scan
	spinCW(steps * 32, &angle); 
	VL53L1X_StopRanging(dev);  // Stop the ranging operation to conserve power
	
	return status;  // Return the status code from sensor initialization
}

// Take multiple measurements at different positions to build a 3D point cloud
// This function coordinates the entire 3D scanning process
// STEPS constant defines how many x positions to take measurements at
// Coordinates with the STEPS constant in both C and Python for data transfer
void takeAllMeasurements(uint16_t allDistances[STEPS][32]){
	uint16_t dev = 0x29;  // I2C address of VL53L1X ToF sensor (default address)
	uint8_t dataReady = 0;  // Flag to track when sensor data is ready

	// Wait for sensor to boot up completely before attempting communication
	while (dataReady == 0) {
			VL53L1X_BootState(dev, &dataReady);
			SysTick_Wait10ms(10);  // 100ms delay between boot status checks
	}
	
	// Take measurements for each step to build the 3D scan
	for(int i = 0; i<STEPS; i++){
		// Wait for user to press button PJ0 before starting next scan
		// This allows the user to manually move the device to the next vertical position
		// The scan is paused until the user indicates they're ready for the next level
		while (1) {
				if ((GPIO_PORTJ_DATA_R & 0b0001) == 0) {	
					break;  // Exit wait loop when button is pressed
				}
		}	
		LED3(1);  // Turn on LED3 (PF0) to provide visual feedback that scanning is in progress
		takeMeasurement(allDistances[i], dev, dataReady);  // Take full 360° scan at this vertical position
		LED3(0);  // Turn off LED3 (PF0) to indicate scan at this level is complete
	}
}

// Send all collected measurements to the computer via UART
// This function handles the communication protocol with the Python script
// The Python script sends 's' character to request data in chunks
// Data is sent in a specific format expected by the visualization software
void sendMeasurements(uint16_t allDistances[STEPS][32]){
	int input = 0;  // Store character received from UART/Python
	uint16_t* distanceMeasurements;  // Pointer to current array of distance measurements
	UART_printf(printf_buffer); // Flush print buffer to ensure clean communication
	LED0(1);  // Turn on LED0(PN1) to indicate the system is now in data transmission mode
	
	while(1){  // Continuous loop to allow repeated data requests from Python
			// For each x step in the 3D scan
			for (int step = 0; step<STEPS; step++){
				// Wait for signal from Python script to send next x step data
				// This ensures the Python receiver is ready before we send data
				while (1) {
					input = UART_InChar();  // Read a character from UART
					if (input == 's') {  // 's' is the signal to start sending data
							break;  // Exit wait loop when signal received
					}
				}	
				LED2(1);  // Turn on LED2 (PF4) to indicate active data transmission
				distanceMeasurements = allDistances[step];  // Get pointer to current vertical step data
				
				// Send each step's data in 4 chunks of 8 measurements each (total 32 measurements)
				// This chunking approach matches the Python script's receiving pattern
				// and helps avoid buffer overflows or timing issues
				for (int i = 0; i < 4; i++) {
					// Wait for signal to send next chunk of data
					// Python script requests each chunk individually for better synchronization
					while (1) {
						input = UART_InChar();
							if (input == 's') {
									break;  // Exit wait loop when signal received
							}
						}
						// Format data as CSV (Comma Separated Values) and send via UART
						// Each line contains 8 distance measurements in millimeters
						sprintf(printf_buffer,"%u,%u,%u,%u,%u,%u,%u,%u\n", 
							distanceMeasurements[8*i], distanceMeasurements[8*i+1], distanceMeasurements[8*i+2], distanceMeasurements[8*i+3], 
							distanceMeasurements[8*i+4], distanceMeasurements[8*i+5], distanceMeasurements[8*i+6], distanceMeasurements[8*i+7]);
						UART_printf(printf_buffer);  // Send the formatted string via UART
					}
			}
			
			LED2(0);  // Turn off LED2 (PF4) when transmission of all data is complete
		}
}

/*
void test(void){
    while(1){
    GPIO_PORTM_DATA_R ^= 0x01;
    SysTick_Wait(6000);
    GPIO_PORTM_DATA_R ^= 0x01;
    SysTick_Wait(6000);
    }
}

                Target: 1 kHz Square Wave
                P = 1ms
                Program to wait 1ms/2 = 0.5 ms

                Terms of clock cycles is:
                0.0005s×12,000,000cycles
                s=6,000cycles.
        */
		
// Main program - Coordinates the entire 3D scanning process
int main(void){
	// Initialize all necessary hardware components for the scanner
	PLL_Init();        // Configure system clock for proper timing
	SysTick_Init();    // Initialize SysTick timer for delay functions
	PortJ_Init();      // Initialize buttons on Port J for user input
	PortM_Init();      // Initialize buttons on Port M for additional control
	PortN_Init();      // Initialize LEDs on Port N for user feedback
	PortF_Init();      // Initialize LEDs on Port F for additional status indicators
	PortH_Init();      // Initialize stepper motor control pins on Port H for rotation
	I2C_Init();        // Initialize I2C bus for communication with ToF sensor
  UART_Init();       // Initialize UART for serial communication with PC/Python script
	PortJ_Interrupt_Init(); // Initialize interrupts for button input (emergency stop/home)
	
	// Turn off all LEDs to start with a clean visual state
	LEDs(0);
	
	// test(); // uncomment to show bus speed on AD3
	
	// Array to store all distance measurements from the 3D scan
	// [STEPS][32]: STEPS vertical positions, 32 measurements per rotation (11.25° increments)
	// This creates a 3D point cloud with STEPS × 32 measurement points
	uint16_t allDistances[STEPS][32];
	
	// Perform the complete 3D scan by taking measurements at each vertical position
	takeAllMeasurements(allDistances);
	
	// Send all collected data to PC via UART for visualization and processing
	// This function communicates with a Python script running on the PC
	sendMeasurements(allDistances);
	
}