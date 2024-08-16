//
// Created by lsy on 24-8-16.
//

#pragma once

#include "SocketCAN.h"

#include <chrono>
#include <mutex>
#include <thread>
#include <vector>

namespace tita_hw
{
class CanBus
{
public:
  /** \brief
   * Initialize device at can_device, retry if fail. Set up header of CAN frame.
   *
   * \param bus_name Bus's name(example: can0).
   * \param data_ptr Pointer which point to CAN data.
   */
  CanBus(const std::string& bus_name, int thread_priority);
  /** \brief Read active data from read_buffer_ to data_ptr_, such as position, velocity, torque and so on. Clear
   * read_buffer_ after reading.
   */
  void read();
  /** \brief Write commands to can bus.
   *
   */
  void write();

  void write(canfd_frame* frame);


  const std::string bus_name_;
  std::vector<canfd_frame> read_buffer_;

private:
  /** \brief This function will be called when CAN bus receive message. It push frame which received into a vector: read_buffer_.
   *
   * @param frame The frame which socketcan receive.
   */
  void frameCallback(const canfd_frame& frame);

  SocketCAN socket_can_;

  mutable std::mutex mutex_;
};
} // namespace tita_hw