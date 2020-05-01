// This file is licensed under MIT-0 (https://github.com/aws/mit-0)
// which can be found in the 'LICENSE' file in this repository.

#ifndef FAKE_ROBOT__LASER_LISTENER_HPP_
#define FAKE_ROBOT__LASER_LISTENER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace fake_robot
{

class LaserListenerNode : public rclcpp::Node
{
public:
  explicit LaserListenerNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  size_t received_message_count_ = 0;
};

}  // namespace fake_robot

#endif  // FAKE_ROBOT__LASTER_LISTENER_HPP_
