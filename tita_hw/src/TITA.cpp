//
// Created by lsy on 24-8-16.
//

#include "tita_hw/TITA.h"

namespace tita_hw
{

void Motor::read(std::vector<canfd_frame> read_buffer)
{
  bool debug = false;
  if (debug)
  {
    std::cout << "Reading data from Motor: " << name << std::endl;
    std::cout << "Timestamp: " << timestamp << ", Position: " << position << ", KP: " << kp
              << ", Velocity: " << velocity << ", KD: " << kd << ", Torque: " << torque << std::endl;
  }
  for (const auto& frame_stamp : read_buffer)
  {

  }
}

void IMU::read(std::vector<canfd_frame> read_buffer)
{
  bool debug = false;
  if (debug)
  {
    std::cout << "Reading data from IMU: " << name << std::endl;
    std::cout << "Timestamp: " << timestamp << ", Accel: [" << accel[0] << ", " << accel[1] << ", " << accel[2] << "]"
              << std::endl;
    std::cout << "Gyro: [" << gyro[0] << ", " << gyro[1] << ", " << gyro[2] << "]" << std::endl;
    std::cout << "Quaternion: [" << quaternion[0] << ", " << quaternion[1] << ", " << quaternion[2] << ", "
              << quaternion[3] << "]" << std::endl;
  }
  for (const auto& frame_stamp : read_buffer)
  {

  }
}

void RemoteControl::read(std::vector<canfd_frame> read_buffer)
{
  bool debug = false;
  if (debug)
  {
    std::cout << "Reading data from RemoteControl: " << name << std::endl;
    std::cout << "Timestamp: " << timestamp << ", Forward: " << forward << ", Roll: " << roll << ", Pitch: " << pitch
              << ", Yaw: " << yaw << ", Height: " << height << ", Mode: " << mode << ", Speed: " << speed
              << ", Jump: " << jump << ", Status: " << status << std::endl;
  }

}
void MotionControlBoard::read(std::vector<canfd_frame> read_buffe)
{
  std::cout << "Reading data from MotionControlBoard: " << name << std::endl;
  for (const auto& peripheral : peripherals)
  {
    peripheral->read(read_buffe);
  }
}

}  // namespace tita_hw