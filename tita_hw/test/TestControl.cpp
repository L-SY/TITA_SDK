//
// Created by lsy on 24-10-12.
//
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include "tita_hw/CANBus.h"
#include "tita_hw/TITA.h"

int main()
{
    tita_hw::CanBus canBus("can0", 95);
    // tita_hw::IMU imu("imu",false);
    tita_hw::RemoteControl rc("rc",true);
    const std::chrono::milliseconds interval(100); // 10 Hz
    std::string msg = "Ready enter loop";
    std::cout << msg << std::endl;
    while (true)
    {
        std::lock_guard<std::mutex> guard(canBus.mutex_);
        // imu.read(canBus.read_buffer_);
        rc.read(canBus.read_buffer_);
        // canBus.read_buffer_.clear();
        std::this_thread::sleep_for(interval);
    }

    return 0;
}
