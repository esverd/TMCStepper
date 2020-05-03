
#pragma once

#include <stdint.h>
#if defined(ARDUINO) && ARDUINO >= 100
	#include <Arduino.h>
#elif defined(USE_FULL_LL_DRIVER)
	#include <stm32f4xx_ll_gpio.h>
#elif defined(TARGET_LPC1768)
	#include <time.h>
	#include <gpio.h>
#endif

#include "TMC_platforms.h"

namespace TMCStepper_n {

#if defined(ARDUINO_ARCH_AVR)

	typedef uint8_t PinDef;

	class TMCPin {
	public:
		explicit TMCPin(const uint8_t _pin) :
			port(portOutputRegister(digitalPinToPort(_pin))),
			bitMask(digitalPinToBitMask(_pin))
			{}

		void mode(const uint8_t mode) const {
			volatile uint8_t *reg = portModeRegister(*port);
			volatile uint8_t *out = portOutputRegister(*port);

			uint8_t oldSREG = SREG;
			cli();

			switch(mode) {
				case OUTPUT:
					*reg |= bitMask;
					break;
				case INPUT:
					*reg &= ~bitMask;
					*out &= ~bitMask;
					break;
				default: break;
			}
			SREG = oldSREG;
		}

		bool read() const {
			return *port & bitMask;
		}

		operator bool() const {
			return read();
		}

	protected:
		volatile uint8_t* const port = nullptr;

		uint8_t const bitMask;
	};

	class OutputPin : public TMCPin {
	public:
		OutputPin(const uint8_t _pin) : TMCPin(_pin) {}

		void write(const bool state) const {
			if (state)
				*port |= bitMask;
			else
				*port &= ~bitMask;
		}
	};

	typedef TMCPin InputPin;

#elif defined(ARDUINO_ARCH_SAM)

	typedef uint32_t PinDef;

	class TMCPin {
	public:
		explicit TMCPin(const uint32_t _pin) :
			pin(_pin)
			{}

		void mode(const uint8_t mode) const {
			switch(mode) {
				case OUTPUT:
					pinMode(pin, OUTPUT);
					break;
				case INPUT:
					pinMode(pin, INPUT);
					break;
				default: break;
			}
		}

		bool read() const {
			return PIO_Get( g_APinDescription[pin].pPort, PIO_INPUT, g_APinDescription[pin].ulPin );
		}

		operator bool() const {
			return read();
		}

		operator uint32_t() const {
			return pin;
		}

	protected:
		uint32_t const pin;
	};

	class OutputPin : public TMCPin {
	public:
		OutputPin(const uint32_t _pin) : TMCPin(_pin) {}

		void write(const bool state) const {
			if (state)
				g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
			else
				g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
		}
	};

	typedef TMCPin InputPin;

#elif defined(TARGET_LPC1768)

	typedef uint16_t PinDef;

	class TMCPin {
	public:
		explicit TMCPin(const uint16_t _pin) : pin(_pin) {}

		void mode(const uint8_t mode) const {
			switch(mode) {
				case OUTPUT:
					LPC176x::gpio_set_output(pin);
					break;
				case INPUT:
					LPC176x::gpio_set_input(pin);
					break;
				default: break;
			}
		}

		bool read() const {
			LPC176x::delay_ns(pinDelay);
			LPC176x::gpio_get(pin);
			LPC176x::delay_ns(pinDelay);
			return out;
		}

		operator bool() const {
			return read();
		}

		operator uint16_t() const {
			return pin;
		}

	protected:
		uint16_t const pin;

		static constexpr uint8_t pinDelay = 60;
	};

	class OutputPin : public TMCPin {
	public:
		OutputPin(const uint16_t _pin) : TMCPin(_pin) {}

		void write(const bool state) const {
			LPC176x::delay_ns(pinDelay);

			if (state)
				LPC176x::gpio_set(pin);
			else
				LPC176x::gpio_clear(pin);

			LPC176x::delay_ns(pinDelay);
		}
	};

	typedef TMCPin InputPin;

#elif defined(ARDUINO)

	typedef uint8_t PinDef;

	class TMCPin {
		explicit TMCPin(const uint8_t _pin) : pin(_pin) {}

		void mode(const uint8_t mode) const {
			switch(mode) {
				case OUTPUT:
					pinMode(pin, OUTPUT);
					break;
				case INPUT:
					pinMode(pin, INPUT);
					break;
				default: break;
			}
		}

		bool read() const {
			return digitalRead(pin);
		}

		operator bool() const {
			return read();
		}

		operator uint8_t() const {
			return pin;
		}

	protected:
		uint8_t const pin;
	};

	class OutputPin : public TMCPin {
	public:
		OutputPin(const uint8_t _pin) : TMCPin(_pin) {}

		void write(const bool state) const {
			digitalWrite(pin, state);
		}
	};

	typedef TMCPin InputPin;

#endif

}
