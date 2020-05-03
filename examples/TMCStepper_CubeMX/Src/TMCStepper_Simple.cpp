
#include <TMCStepper.h>
#include <source/TMC_Pin.h>
#include "main.h"

TMCStepper_n::OutputPin step({DRV_STEP_GPIO_Port, DRV_STEP_Pin});

SPIClass SPI(SPI2);
TMC2130Stepper driver(SPI, {DRV_EN_GPIO_Port, DRV_EN_Pin}, 0.5);

SW_SPIClass SWSPI({DRV_EN_GPIO_Port, DRV_EN_Pin}, {DRV_EN_GPIO_Port, DRV_EN_Pin}, {DRV_EN_GPIO_Port, DRV_EN_Pin});
TMC2130Stepper driver_sw(SWSPI, {DRV_EN_GPIO_Port, DRV_EN_Pin}, 0.5);

extern "C"
void initDriver() {
	LL_GPIO_ResetOutputPin(DRV_EN_GPIO_Port, DRV_EN_Pin);		// Enable driver in hardware

									// Enable one according to your setup
	driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
							  // UART: Init SW UART (if selected) with default 115200 baudrate
	driver.toff(5);                 // Enables driver in software
	driver.rms_current(600);        // Set motor RMS current
	driver.microsteps(16);          // Set microsteps to 1/16th

	driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
	//driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2208/2209/2224
	driver.pwm_autoscale(true);     // Needed for stealthChop
}

bool shaft = false;

extern "C"
void runMotor() {
	// Run 5000 steps and switch direction in software
	for (uint16_t i = 5000; i>0; i--) {
		step = true;
		LL_mDelay(160);
		//step = LOW;
		LL_mDelay(160);
	}
	shaft = !shaft;
	driver.shaft(shaft);

	//step ? step = HIGH : step = LOW;
}
