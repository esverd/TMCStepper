#include "SW_SPI.h"

using namespace TMCStepper_n;

SW_SPIClass::SW_SPIClass(PinDef mosi, PinDef miso, PinDef sck) :
  mosi_pin(mosi),
  sck_pin(sck),
  miso_pin(miso)
  {}

void SW_SPIClass::init() {
  OutputPin mosi(mosi_pin), sck(sck_pin);
  InputPin miso(miso_pin);

  mosi.mode(OUTPUT);
  sck.mode(OUTPUT);
  miso.mode(INPUT);
  sck.write(HIGH);
}

uint8_t SW_SPIClass::transfer(uint8_t ulVal) {
  uint8_t value = 0;

  OutputPin mosi(mosi_pin), sck(sck_pin);
  InputPin miso(miso_pin);

  sck.write(LOW);

  for (uint8_t i=7 ; i>=1 ; i--) {
    // Write bit
    mosi.write(!!(ulVal & (1 << i)) ? HIGH : LOW);
    // Start clock pulse
    sck.write(HIGH);
    // Read bit
    value |= ( miso ? 1 : 0) << i;
    // Stop clock pulse
    sck.write(LOW);
  }

  mosi.write(!!(ulVal & (1 << 0)) ? HIGH : LOW);
  sck.write(HIGH);
  value |= ( miso ? 1 : 0) << 0;

  return value;
}

uint16_t SW_SPIClass::transfer16(uint16_t data) {
  uint16_t returnVal = 0x0000;
  returnVal |= transfer((data>>8)&0xFF) << 8;
  returnVal |= transfer(data&0xFF) & 0xFF;
  return returnVal;
}
