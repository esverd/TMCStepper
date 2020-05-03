#pragma once

#include <stdint.h>
#include "TMC_Pin.h"

class SSwitch {
  public:
    SSwitch(const TMCStepper_n::PinDef pin1, const TMCStepper_n::PinDef pin2, const uint8_t address);
    void active();
  private:
    TMCStepper_n::PinDef p1;
    TMCStepper_n::PinDef p2;
    const uint8_t addr;
};
