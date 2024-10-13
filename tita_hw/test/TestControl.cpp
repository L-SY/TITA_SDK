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

    tita_hw::RobotCommand robotCommand("robot_command", true);

    const std::chrono::milliseconds interval(100); // 10 Hz
    std::string msg = "Ready to enter loop";
    std::cout << msg << std::endl;

    tita_hw::RemoteProcedureCall rpc("robot_rpc", true);

    uint32_t timestamp = 0;  // 模拟时间戳

    // rpc.setStandMode(timestamp, 3);  // 3 表示 TRANSFORM_UP 模式
    // canBus.write(&rpc.internalFrame);
    while (true)
    {
        std::lock_guard<std::mutex> guard(canBus.mutex_);
        robotCommand.forwardVel = 0.0f;  // 设定前进速度
        robotCommand.yawVel = 0.0f;      // 设定偏航速度
        robotCommand.pitchPos = 0.0f;    // 设定俯仰位置
        robotCommand.rollPos = 0.0f;     // 设定滚转位置
        robotCommand.heightPos = 0.2f;   // 设定高度位置
        robotCommand.forwardAccel = 1.0f;  // 设定前进加速度
        robotCommand.yawAccel = 0.8f;      // 设定偏航加速度

        //robotCommand.write();

        //canBus.write(&robotCommand.internalFrame);

        // canBus.read_buffer_.clear();
        std::this_thread::sleep_for(interval);
    }
    return 0;
}
