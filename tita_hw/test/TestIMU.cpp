//
// Created by lsy on 24-8-23.
//

#include "tita_hw/CANBus.h"
#include "tita_hw/TITA.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu");
  ros::NodeHandle n;
  tita_hw::CanBus canBus("can0",95);
  tita_hw::IMU imu("imu");
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("imu", 1000);
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "imu";
    chatter_pub.publish(msg);
    std::lock_guard<std::mutex> guard(canBus.mutex_);
    imu.read(canBus.read_buffer_);
    canBus.read_buffer_.clear();
    ros::spinOnce();
    loop_rate.sleep();
  }
}
