// Copyright (c) 2020 Amazon.com, Inc. or its affiliates
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

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

#endif  // FAKE_ROBOT__LASER_LISTENER_HPP_
