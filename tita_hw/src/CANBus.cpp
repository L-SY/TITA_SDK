//
// Created by lsy on 24-8-16.
//

#include <iostream>
#include "tita_hw/CANBus.h"
#include <chrono>
#include <functional>
#include "boost/bind/bind.hpp"
using namespace boost::placeholders;

namespace tita_hw
{
CanBus::CanBus(const std::string& bus_name, int thread_priority)
  : bus_name_(bus_name)
{
  // Initialize device at can_device, false for no loop back.
  while (!socket_can_.open(bus_name, boost::bind(&CanBus::frameCallback, this, _1), thread_priority) )
    std::this_thread::sleep_for(std::chrono::seconds(1));

  std::cout << "Successfully connected to " << bus_name << std::endl;
}

void CanBus::write(canfd_frame* frame)
{
  socket_can_.write(frame);
}

void CanBus::frameCallback(const canfd_frame& frame)
{
  std::lock_guard<std::mutex> guard(mutex_);
  read_buffer_.push_back(frame);
}
}// namespace tita_hw