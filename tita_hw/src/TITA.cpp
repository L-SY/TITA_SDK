//
// Created by lsy on 24-8-16.
//

#include "tita_hw/TITA.h"

namespace tita_hw
{

void MotionControlBoard::read(std::vector<canfd_frame> read_buffe)  {
  std::cout << "Reading data from MotionControlBoard: " << name << std::endl;
  for (const auto& peripheral : peripherals) {
    peripheral->read(read_buffe);
  }
}

} // namespace tita_hw