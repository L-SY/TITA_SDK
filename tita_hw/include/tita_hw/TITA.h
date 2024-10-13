//
// Created by lsy on 24-8-16.
//

#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include "tita_hw/SocketCAN.h"
#include "tita_hw/CANBus.h"

namespace tita_hw
{

class Peripheral
{
public:
  std::string name_;
  bool debug_;

  Peripheral(const std::string& name, bool debug = false)
    : name_(name), debug_(debug)
  {}

  virtual void read(std::vector<canfd_frame> read_buffer){}
  virtual void write(canfd_frame canfd_frame){}
  virtual ~Peripheral() = default;
};

class Motor : public Peripheral
{
public:
  // for read
  uint32_t timestamp;
  float position;
  float velocity;
  float torque;

  // for write
  uint32_t timestampCmd;
  float kpCmd;
  float kdCmd;
  float torqueCmd;
  float positionCmd;
  float velocityCmd;


  Motor(const std::string& name, bool debug = false,
        uint32_t timestamp = 0, float position = 0.0, float velocity = 0.0, float torque = 0.0,
        uint32_t timestampCmd = 0, float kpCmd = 0.0, float kdCmd = 0.0,
        float torqueCmd = 0.0, float positionCmd = 0.0, float velocityCmd = 0.0)
      : Peripheral(name, debug),
        timestamp(timestamp), position(position), velocity(velocity), torque(torque),
        timestampCmd(timestampCmd), kpCmd(kpCmd), kdCmd(kdCmd),
        torqueCmd(torqueCmd), positionCmd(positionCmd), velocityCmd(velocityCmd)
  {}

  void read(std::vector<canfd_frame> read_buffer) override;
  void write(canfd_frame canfd_frame) override;
};

class IMU : public Peripheral
{
public:
  uint32_t timestamp;
  std::vector<float> accel;
  std::vector<float> gyro;
  std::vector<float> quaternion;
  float temperature;

  IMU(const std::string& name, bool debug = false, uint32_t timestamp = 0)
    : Peripheral(name,debug), timestamp(timestamp)
  {
    accel.resize(3);
    gyro.resize(3);
    quaternion.resize(4);
  }

  void read(std::vector<canfd_frame> read_buffer) override;
};

class RemoteControl : public Peripheral
{
public:
  uint32_t timestamp;
  float forward;
  float yaw;
  float pitch;
  float roll;
  float height;
  float split;
  float tilt;
  float forward_accel;
  float yaw_accel ;

  RemoteControl(const std::string& name, bool debug = false, uint32_t timestamp = 0, float forward = 0, float yaw = 0, float pitch = 0,
                float roll = 0, float height = 0, float split = 0, float tilt = 0, float forward_accel = 0, float yaw_accel = 0)
    : Peripheral(name,debug)
    , timestamp(timestamp)
    , forward(forward)
    , yaw(yaw)
    , pitch(pitch)
    , roll(roll)
    , height(height)
    , split(split)
    , tilt(tilt)
    , forward_accel(forward_accel)
    , yaw_accel(yaw_accel)
  {
  }

  void read(std::vector<canfd_frame> read_buffer) override;
};

class Board
{
public:
  std::string name_;
  int canfd_id;
  std::vector<std::shared_ptr<Peripheral>> peripherals;

  Board(const std::string& name, int canfd_id) : name_(name), canfd_id(canfd_id)
  {
  }

  virtual void read(std::vector<canfd_frame> read_buffer) = 0;

  virtual ~Board() = default;

  void addPeripheral(std::shared_ptr<Peripheral> peripheral)
  {
    peripherals.push_back(peripheral);
  }
};

class MotionControlBoard : public Board
{
public:
  MotionControlBoard(const std::string& name, int canfd_id) : Board(name, canfd_id)
  {
    for (int i = 0; i < 8; ++i)
    {
      std::string motor_name;
      if (i < 4)
        motor_name = "M-L" + std::to_string(i);
      else
        motor_name = "M-R" + std::to_string(i - 4);

      addPeripheral(std::make_shared<Motor>(motor_name));
    }
    addPeripheral(std::make_shared<IMU>("IMU"));
    addPeripheral(std::make_shared<RemoteControl>("RemoteControl"));
  }

  void read(std::vector<canfd_frame> read_buffer) override;
};

class TITA
{
public:
  std::vector<std::shared_ptr<Board>> boards_;
  CanBus canBus_;

  void addBoard(std::shared_ptr<Board> board)
  {
    boards_.push_back(board);
  }

  void readAllBoards()
  {
    for (const auto& board : boards_)
    {
      board->read(canBus_.read_buffer_);
    }
  }
};
}  // namespace tita_hw