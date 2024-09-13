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

  virtual void read(std::vector<canfd_frame> read_buffer) = 0;
  virtual void write(canfd_frame canfd_frame){}
  virtual ~Peripheral() = default;
};

class Motor : public Peripheral
{
public:
  uint32_t timestamp;
  float position;
  float kp;
  float velocity;
  float kd;
  float torque;

  Motor(const std::string& name, bool debug = false, uint32_t timestamp = 0, float position = 0.0, float kp = 0.0, float velocity = 0.0,
        float kd = 0.0, float torque = 0.0)
    : Peripheral(name,debug), timestamp(timestamp), position(position), kp(kp), velocity(velocity), kd(kd), torque(torque)
  {}

  void read(std::vector<canfd_frame> read_buffer) override;
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
  int forward;
  int roll;
  int pitch;
  int yaw;
  int height;
  int mode;
  int speed;
  int jump;
  int status;

  RemoteControl(const std::string& name, bool debug = false, uint32_t timestamp = 0, int forward = 0, int roll = 0, int pitch = 0,
                int yaw = 0, int height = 0, int mode = 0, int speed = 0, int jump = 0, int status = 0)
    : Peripheral(name,debug)
    , timestamp(timestamp)
    , forward(forward)
    , roll(roll)
    , pitch(pitch)
    , yaw(yaw)
    , height(height)
    , mode(mode)
    , speed(speed)
    , jump(jump)
    , status(status)
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
      addPeripheral(std::make_shared<Motor>("Motor" + std::to_string(i + 1)));
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