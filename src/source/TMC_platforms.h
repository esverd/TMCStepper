#pragma once

#if defined(USE_HAL_DRIVER) || defined(HAL_SPI_MODULE_ENABLED)
	#include <stdint.h>
	#include <stm32f4xx_ll_utils.h>
	#include <stm32f4xx_ll_spi.h>
	#include <stm32f4xx_ll_usart.h>
	#include <main.h>
	#if defined(HAL_SPI_MODULE_ENABLED)
		struct SPISettings {
			SPISettings(uint32_t, uint8_t, uint8_t) {}
		};

		struct SPIClass {
			explicit SPIClass(SPI_TypeDef *const spi = nullptr) : hspi(spi) {}

			uint8_t transfer(const uint8_t data) const {
				LL_SPI_TransmitData8(hspi, data);
				return LL_SPI_ReceiveData8(hspi);
			}
			static void beginTransaction(SPISettings) {}
			static void endTransaction() {}

			operator bool() const { return hspi; }

		private:
			SPI_TypeDef * const hspi;
		};

		struct Stream {
			static uint8_t write(uint8_t data) {
				LL_USART_TransmitData8(USART2, data);
				return 1;
			}
			static uint8_t read() {
				return LL_USART_ReceiveData8(USART2);
			}
			static uint8_t available() {
				return !LL_USART_IsActiveFlag_RXNE(USART2);
			}
		};

		typedef SPIClass HW_SPI_TYPE;

		inline void delay(uint32_t ms) {
			LL_mDelay(ms);
		}
	#elif defined(USE_FULL_LL_DRIVER)
		#include <stdint.h>
		#include <stm32f4xx_ll_utils.h>
		#include <stm32f4xx_ll_spi.h>
		#include <stm32f4xx_ll_usart.h>

		struct SPISettings {
			SPISettings(uint32_t, uint8_t, uint8_t) {}
		};

		struct SPIClass {
			explicit SPIClass(SPI_TypeDef *const spi = nullptr) : hspi(spi) {}

			uint8_t transfer(const uint8_t data) const {
				LL_SPI_TransmitData8(hspi, data);
				return LL_SPI_ReceiveData8(hspi);
			}
			void transfer(uint8_t *buf, uint8_t count) {
				while(count --> 0) {
					transfer(*buf);
				}
			}

			static void beginTransaction(SPISettings) {}
			static void endTransaction() {}

			operator bool() const { return hspi; }

		private:
			SPI_TypeDef * const hspi;
		};

		struct Stream {
			static uint8_t write(uint8_t data) {
				LL_USART_TransmitData8(USART2, data);
				return 1;
			}
			static uint8_t read() {
				return LL_USART_ReceiveData8(USART2);
			}
			static uint8_t available() {
				return !LL_USART_IsActiveFlag_RXNE(USART2);
			}
		};

		typedef SPIClass HW_SPI_TYPE;

		inline void delay(uint32_t ms) {
			LL_mDelay(ms);
		}
	#endif
#elif defined(ARDUINO_ARCH_AVR)
	#include <SPI.h>
	typedef SPIClass HW_SPI_TYPE;
#else
	typedef SPIClass HW_SPI_TYPE;
#endif

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
