#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "fake_robot/fake_laser_listener.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "libstatistics_collector/collector/generate_statistics_message.hpp"

#include <functional>
#include <iostream>

namespace fake_robot {

FakeLaserListenerNode::FakeLaserListenerNode(const rclcpp::NodeOptions &options)
: Node("fake_laser_listener", options)
{

  auto callback = [this](sensor_msgs::msg::LaserScan::UniquePtr msg) {
    this->on_message_received(*msg);
  };

  rclcpp::QoS qos_profile(rclcpp::KeepLast(5));
  subscription_ = create_subscription<sensor_msgs::msg::LaserScan,
    std::function<void(sensor_msgs::msg::LaserScan::UniquePtr)>>(
      "scan",
      qos_profile,
      callback);

  publisher_ = create_publisher<statistics_msgs::msg::MetricsMessage>(std::string("/system_metrics"), qos_profile);

  window_start_ = now();

  publish_timer_ = this->create_wall_timer(
    std::chrono::seconds(5), [this]() {
      this->publish_message();
      this->window_start_ = this->now();
    });
}

void FakeLaserListenerNode::publish_message()
{
  const auto msg = libstatistics_collector::collector::GenerateStatisticMessage(
    get_name(),
    GetMetricName(),
    GetMetricUnit(),
    window_start_,
    now(),
    libstatistics_collector::collector::Collector::GetStatisticsResults());
  publisher_->publish(msg);
}

void FakeLaserListenerNode::FakeLaserListenerNode::on_message_received(const sensor_msgs::msg::LaserScan & msg)
{
  AcceptData(msg.scan_time);
}

bool FakeLaserListenerNode::FakeLaserListenerNode::SetupStart()
{
  return true;
}

bool FakeLaserListenerNode::SetupStop()
{
  return true;
}

std::string FakeLaserListenerNode::GetMetricName() const
{
  return "scan_time";
}

std::string FakeLaserListenerNode::GetMetricUnit() const
{
  return "seconds";
}

}  // namespace fake_robot

RCLCPP_COMPONENTS_REGISTER_NODE(fake_robot::FakeLaserListenerNode)
