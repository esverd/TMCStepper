#pragma once

#if defined(USE_HAL_DRIVER) || defined(USE_FULL_LL_DRIVER)
	#include <stdint.h>
	#include <spi.h>
	#include <usart.h>

	#if defined(HAL_SPI_MODULE_ENABLED)
		struct SPISettings {
			SPISettings(uint32_t, uint8_t, uint8_t) {}
		};

		struct SPIClass {
			explicit SPIClass(SPI_HandleTypeDef &spi) : hspi(&spi) {}

			void transfer(uint8_t *buf, uint8_t count) const {
				HAL_SPI_TransmitReceive(hspi, buf, buf, count, timeout);
			}
			static void beginTransaction(SPISettings) {}
			static void endTransaction() {}

			operator bool() const { return hspi; }

		private:
			SPI_HandleTypeDef * const hspi;
			static constexpr uint32_t timeout = 1000;
		};

		typedef SPIClass HW_SPI_TYPE;

		inline void delay(uint32_t ms) {
			HAL_Delay(ms);
		}
	#elif defined(USE_FULL_LL_DRIVER)
		struct SPISettings {
			SPISettings(uint32_t, uint8_t, uint8_t) {}
		};

		struct SPIClass {
			explicit SPIClass(SPI_TypeDef *const spi) : hspi(spi) {}

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

		typedef SPIClass HW_SPI_TYPE;

		inline void delay(uint32_t ms) {
			LL_mDelay(ms);
		}
	#endif

	#if defined(HAL_UART_MODULE_ENABLED)
		struct Stream {
			Stream(UART_HandleTypeDef *const handle) : huart(handle) {}

			uint8_t write(uint8_t data) {
				uint8_t bytesWritten = 0;
				if (HAL_UART_Transmit(huart, &data, 1, timeout) == HAL_OK) {
					bytesWritten++;
				}
				return bytesWritten;
			}
			uint8_t read() {
				uint8_t data{};
				HAL_UART_Receive(huart, &data, 1, timeout);
				return data;
			}
			uint8_t available() {
				return __HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE);
			}
			static constexpr uint32_t timeout = 1000;
		private:
			UART_HandleTypeDef * const huart;
		};
	#elif defined(USE_FULL_LL_DRIVER)
		struct Stream {
			Stream(USART_TypeDef *const handle) : huart(handle) {}

			uint8_t write(uint8_t data) {
				LL_USART_TransmitData8(huart, data);
				return 1;
			}
			uint8_t read() {
				return LL_USART_ReceiveData8(huart);
			}
			uint8_t available() {
				return LL_USART_IsActiveFlag_RXNE(huart);
			}
		private:
			USART_TypeDef * const huart;
		};
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
