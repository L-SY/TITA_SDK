//
// Created by lsy on 24-8-16.
//

#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include "tita_hw/SocketCAN.h"

namespace tita_hw
{
class Peripheral {
public:
  std::string name;

  Peripheral(const std::string& name) : name(name) {}

  virtual void read() = 0;

  virtual ~Peripheral() = default;
};

class Motor : public Peripheral {
public:
  int speed;

  Motor(const std::string& name, int speed = 0)
    : Peripheral(name), speed(speed) {}

  void read() override {
    std::cout << "Reading data from Motor: " << name << ", Speed: " << speed << std::endl;
  }
};

class IMU : public Peripheral {
public:
  float orientation;

  IMU(const std::string& name, float orientation = 0.0)
    : Peripheral(name), orientation(orientation) {}

  void read() override {
    std::cout << "Reading data from IMU: " << name << ", Orientation: " << orientation << std::endl;
  }
};

class RemoteControl : public Peripheral {
public:
  int signalStrength;

  RemoteControl(const std::string& name, int signalStrength = 0)
    : Peripheral(name), signalStrength(signalStrength) {}

  void read() override {
    std::cout << "Reading data from RemoteControl: " << name << ", Signal Strength: " << signalStrength << std::endl;
  }
};


class Board {
public:
  std::string name;
  int canfd_id;
  std::vector<std::shared_ptr<Peripheral>> peripherals;

  Board(const std::string& name, int canfd_id)
    : name(name), canfd_id(canfd_id) {}

  virtual void read(SocketCAN socketCan) = 0;

  virtual ~Board() = default;

  void addPeripheral(std::shared_ptr<Peripheral> peripheral) {
    peripherals.push_back(peripheral);
  }
};

class MotionControlBoard : public Board {
public:
  MotionControlBoard(const std::string& name, int canfd_id)
    : Board(name, canfd_id) {}

  void read(SocketCAN socketCan) override {
    std::cout << "Reading data from MotionControlBoard: " << name << std::endl;
    for (const auto& peripheral : peripherals) {
      peripheral->read();
    }
  }
};


class RobotAssets {
public:
  std::vector<std::shared_ptr<Board>> boards_;
  SocketCAN socketCan_;

  void addBoard(std::shared_ptr<Board> board) {
    boards_.push_back(board);
  }

  void readAllBoards() {
    for (const auto& board : boards_) {
      board->read(socketCan_);
    }
  }
};
} // namespace tita_hw