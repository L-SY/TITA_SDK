//
// Created by lsy on 24-8-16.
//

#include "tita_hw/TITA.h"

namespace tita_hw
{

void Motor::read(std::vector<canfd_frame> read_buffer)
{
  if (debug_)
  {
    std::cout << "Reading data from Motor: " << name_ << std::endl;
    std::cout << "Timestamp: " << timestamp << ", Position: " << position << ", KP: " << kp
              << ", Velocity: " << velocity << ", KD: " << kd << ", Torque: " << torque << std::endl;
  }
  for (const auto& frame_stamp : read_buffer)
  {
  }
}

void IMU::read(std::vector<canfd_frame> read_buffer)
{
  for (const auto& frame_stamp : read_buffer)
  {
    if (frame_stamp.can_id == 0x118 || frame_stamp.can_id == 0x119)
    {
      std::memcpy(&timestamp, frame_stamp.data, sizeof(timestamp));
      std::memcpy(accel.data(), frame_stamp.data + sizeof(timestamp), 3 * sizeof(float));
      std::memcpy(gyro.data(), frame_stamp.data + sizeof(timestamp) + 3 * sizeof(float), 3 * sizeof(float));
      std::memcpy(quaternion.data(), frame_stamp.data + sizeof(timestamp) + 6 * sizeof(float), 4 * sizeof(float));
      std::memcpy(&temperature, frame_stamp.data + sizeof(timestamp) + 10 * sizeof(float), sizeof(float));

      if (debug_)
      {
        std::cout << "Timestamp: " << timestamp << std::endl;
        std::cout << "Accel: [" << accel[0] << ", " << accel[1] << ", " << accel[2] << "]" << std::endl;
        std::cout << "Gyro: [" << gyro[0] << ", " << gyro[1] << ", " << gyro[2] << "]" << std::endl;
        std::cout << "Quaternion: [" << quaternion[0] << ", " << quaternion[1] << ", " << quaternion[2] << ", " << quaternion[3] << "]" << std::endl;
        std::cout << "Temperature: " << temperature << std::endl;
      }
    }
  }
  std::cout << "IMU read: "<< std::endl;
}

void RemoteControl::read(std::vector<canfd_frame> read_buffer)
{
  if (debug_)
  {
    std::cout << "Reading data from RemoteControl: " << name_ << std::endl;
    std::cout << "Timestamp: " << timestamp << ", Forward: " << forward << ", Roll: " << roll << ", Pitch: " << pitch
              << ", Yaw: " << yaw << ", Height: " << height << ", Mode: " << mode << ", Speed: " << speed
              << ", Jump: " << jump << ", Status: " << status << std::endl;
  }

}
void MotionControlBoard::read(std::vector<canfd_frame> read_buffe)
{
  std::cout << "Reading data from MotionControlBoard: " << name_ << std::endl;
  for (const auto& peripheral : peripherals)
  {
    peripheral->read(read_buffe);
  }
}

}  // name_space tita_hw