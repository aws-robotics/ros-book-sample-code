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

#include <memory>
#include <utility>

#include "fake_robot/laser_listener.hpp"

namespace fake_robot
{

LaserListenerNode::LaserListenerNode(const rclcpp::NodeOptions & options)
: Node("fake_laser", options)
{
  // Use the SensorData QoS profile with a KeepLast initializer reflecting our chosen history depth
  auto qos_request = rclcpp::QoS(rclcpp::KeepLast(5));
  auto callback =
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) -> void
    {
      RCLCPP_INFO(
        this->get_logger(), "Received laser scan %d with %d points",
        received_message_count_++, msg->ranges.size());
    };
  rclcpp::SubscriptionOptions subscription_options;

  subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", qos_request, callback, subscription_options);
}

}  // namespace fake_robot

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<fake_robot::LaserListenerNode>(options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fake_robot::LaserListenerNode)
