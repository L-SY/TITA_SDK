//
// Created by lsy on 24-8-16.
//

#include "tita_hw/TITA.h"
#include <iomanip>

namespace tita_hw
{
void RobotCommand::validateAndClamp()
{
    // 使用 std::clamp 简化边界检查
    forwardVel = std::clamp(forwardVel, 0.0f, 0.25f);
    yawVel = std::clamp(yawVel, 0.0f, 0.4f);
    pitchPos = std::clamp(pitchPos, -0.942f, 0.942f);
    pitchVel = std::clamp(pitchVel, 0.0f, 1.2f);
    rollPos = std::clamp(rollPos, -0.2f, 0.2f);
    heightPos = std::clamp(heightPos, 0.11f, 0.375f);
    heightVel = std::clamp(heightVel, 0.0f, 0.5f);
}

void RobotCommand::write()
{
  validateAndClamp();

  std::memcpy(internalFrame.data, &timestamp, sizeof(timestamp));
  std::memcpy(internalFrame.data + sizeof(timestamp), &forwardVel, sizeof(forwardVel));
  std::memcpy(internalFrame.data + sizeof(timestamp) + sizeof(forwardVel), &yawVel, sizeof(yawVel));
  std::memcpy(internalFrame.data + sizeof(timestamp) + 2 * sizeof(float), &pitchPos, sizeof(pitchPos));
  std::memcpy(internalFrame.data + sizeof(timestamp) + 3 * sizeof(float), &pitchVel, sizeof(pitchVel));
  std::memcpy(internalFrame.data + sizeof(timestamp) + 4 * sizeof(float), &rollPos, sizeof(rollPos));
  std::memcpy(internalFrame.data + sizeof(timestamp) + 5 * sizeof(float), &heightPos, sizeof(heightPos));
  std::memcpy(internalFrame.data + sizeof(timestamp) + 6 * sizeof(float), &heightVel, sizeof(heightVel));
  std::memcpy(internalFrame.data + sizeof(timestamp) + 7 * sizeof(float), &forwardAccel, sizeof(forwardAccel));
  std::memcpy(internalFrame.data + sizeof(timestamp) + 8 * sizeof(float), &yawAccel, sizeof(yawAccel));

  if (debug_)
  {
    std::cout << name_ << " Command Sent:" << std::endl;
    std::cout << "  Timestamp: " << timestamp << std::endl;
    std::cout << "  Forward Velocity: " << forwardVel << std::endl;
    std::cout << "  Yaw Velocity: " << yawVel << std::endl;
    std::cout << "  Pitch Position: " << pitchPos << std::endl;
    std::cout << "  Pitch Velocity: " << pitchVel << std::endl;
    std::cout << "  Roll Position: " << rollPos << std::endl;
    std::cout << "  Height Position: " << heightPos << std::endl;
    std::cout << "  Height Velocity: " << heightVel << std::endl;
    std::cout << "  Forward Acceleration: " << forwardAccel << std::endl;
    std::cout << "  Yaw Acceleration: " << yawAccel << std::endl;
  }
}


void Motor::read(std::vector<canfd_frame> read_buffer)
{
    int motor_index = -1;

    if (name_ == "M-L0")
      motor_index = 0;
    else if (name_ == "M-L1")
      motor_index = 1;
    else if (name_ == "M-L2")
      motor_index = 2;
    else if (name_ == "M-L3")
      motor_index = 3;
    else if (name_ == "M-R0")
      motor_index = 4;
    else if (name_ == "M-R1")
      motor_index = 5;
    else if (name_ == "M-R2")
      motor_index = 6;
    else if (name_ == "M-R3")
      motor_index = 7;

    if (motor_index == -1) {
      std::cerr << "Error: Invalid motor name_." << std::endl;
      return;
    }

  for (const auto& frame_stamp : read_buffer)
  {
    if ((motor_index >= 0 && motor_index <= 3 && frame_stamp.can_id == 0x108) ||
        (motor_index >= 4 && motor_index <= 7 && frame_stamp.can_id == 0x109))
    {
      int actual_motor_index = motor_index % 4;  // 每个帧最多有4个motor，所以需要计算在帧中的实际偏移

      std::memcpy(&timestamp, frame_stamp.data + actual_motor_index * (sizeof(uint32_t) + 3 * sizeof(float)), sizeof(uint32_t));
      std::memcpy(&position, frame_stamp.data + actual_motor_index * (sizeof(uint32_t) + 3 * sizeof(float)) + sizeof(uint32_t), sizeof(float));
      std::memcpy(&velocity, frame_stamp.data + actual_motor_index * (sizeof(uint32_t) + 3 * sizeof(float)) + sizeof(uint32_t) + sizeof(float), sizeof(float));
      std::memcpy(&torque, frame_stamp.data + actual_motor_index * (sizeof(uint32_t) + 3 * sizeof(float)) + sizeof(uint32_t) + 2 * sizeof(float), sizeof(float));
    }

    if (debug_)
    {
      std::cout << name_ << " Data: " << std::endl;
      std::cout << "  Timestamp: " << timestamp << std::endl;
      std::cout << "  Position: " << position << std::endl;
      std::cout << "  Velocity: " << velocity << std::endl;
      std::cout << "  Torque: " << torque << std::endl;
    }
  }
}

void Motor::write()
{
  // TODO(4yang): Add single motor cmd
  std::cout << name_ << " Data: " << std::endl;
}

void IMU::read(std::vector<canfd_frame> read_buffer)
{
  for (const auto& frame_stamp : read_buffer)
  {
    if (frame_stamp.can_id == 0x118)
    {
      std::cout << std::dec << std::endl; // Reset to decimal for further output
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
}

void RemoteControl::read(std::vector<canfd_frame> read_buffer)
{
  for (const auto& frame_stamp : read_buffer)
  {
    if (frame_stamp.can_id == 0x12D)
    {
      std::memcpy(&timestamp, frame_stamp.data, sizeof(timestamp));
      std::memcpy(&forward, frame_stamp.data + sizeof(timestamp), sizeof(forward));
      std::memcpy(&yaw, frame_stamp.data + sizeof(timestamp) + sizeof(float), sizeof(yaw));
      std::memcpy(&pitch, frame_stamp.data + sizeof(timestamp) + 2*sizeof(float), sizeof(pitch));
      std::memcpy(&roll, frame_stamp.data + sizeof(timestamp) + 3*sizeof(float) , sizeof(roll));
      std::memcpy(&height, frame_stamp.data + sizeof(timestamp) + 4*sizeof(forward) , sizeof(height));
      std::memcpy(&split, frame_stamp.data + sizeof(timestamp) + 5*sizeof(forward) , sizeof(split));
      std::memcpy(&tilt, frame_stamp.data + sizeof(timestamp) + 6*sizeof(forward) , sizeof(tilt));
      std::memcpy(&forward_accel, frame_stamp.data + sizeof(timestamp) + 7*sizeof(forward), sizeof(forward_accel));
      std::memcpy(&yaw_accel, frame_stamp.data + sizeof(timestamp) + 8*sizeof(forward), sizeof(yaw_accel));
    }
  }
if (debug_)
  {
    std::cout << "Reading data from RemoteControl: " << name_ << std::endl;
    std::cout << "Timestamp: " << timestamp << std::endl;
    std::cout << "Forward: " << forward << std::endl;
    std::cout << "Yaw: " << yaw << std::endl;
    std::cout << "Pitch: " << pitch << std::endl;
    std::cout << "Roll: " << roll << std::endl;
    std::cout << "Height: " << height << std::endl;
    std::cout << "Split: " << split << std::endl;
    std::cout << "Tilt: " << tilt << std::endl;
    std::cout << "Forward_accel: " << forward_accel << std::endl;
    std::cout << "Yaw_accel: " << yaw_accel << std::endl;

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