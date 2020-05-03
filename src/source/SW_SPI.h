#pragma once

#include <stdint.h>
#include "TMC_Pin.h"

class SW_SPIClass {
	public:
		SW_SPIClass(TMCStepper_n::PinDef mosi, TMCStepper_n::PinDef miso, TMCStepper_n::PinDef sck);
		void init();
		void begin() {};
		uint8_t transfer(uint8_t ulVal);
		uint16_t transfer16(uint16_t data);
		void endTransaction() {};
	private:
		const TMCStepper_n::PinDef mosi_pin, sck_pin, miso_pin;
};
