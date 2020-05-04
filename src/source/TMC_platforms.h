#pragma once

#if defined(ARDUINO_ARCH_AVR)
	#include <SPI.h>
#endif

typedef SPIClass HW_SPI_TYPE;

#ifndef HIGH
	#define HIGH 1
#endif
#ifndef LOW
	#define LOW 0
#endif
#ifndef INPUT
	#define INPUT  0x00
#endif
#ifndef OUTPUT
	#define OUTPUT 0x01
#endif
#if !defined(MSBFIRST) && !defined(ARDUINO_ARCH_SAM)
	#define MSBFIRST 1
#endif
#ifndef SPI_MODE3
	#define SPI_MODE3 0
#endif
