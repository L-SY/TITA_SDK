#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include "tita_hw/CANBus.h"
#include "tita_hw/TITA.h"

int main()
{
  tita_hw::CanBus canBus("can0", 95);
  tita_hw::IMU imu("imu");

  const std::chrono::milliseconds interval(100); // 10 Hz

  while (true)
  {
    std::string msg = "imu";

    std::cout << msg << std::endl;

    {
      std::lock_guard<std::mutex> guard(canBus.mutex_);
      imu.read(canBus.read_buffer_);
      canBus.read_buffer_.clear();
    }

    std::this_thread::sleep_for(interval);
  }

  return 0;
}
