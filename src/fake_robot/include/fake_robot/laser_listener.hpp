// This file is licensed under MIT-0 (https://github.com/aws/mit-0)
// which can be found in the 'LICENSE' file in this repository.

#ifndef FAKE_ROBOT__LASER_LISTENER_HPP_
#define FAKE_ROBOT__LASER_LISTENER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "libstatistics_collector/collector/collector.hpp"

namespace fake_robot
{

class LaserListenerNode : public rclcpp::Node, public libstatistics_collector::collector::Collector
{
public:
  explicit LaserListenerNode(const rclcpp::NodeOptions & options);

private:
  void publish_message();

  void on_message_received(const sensor_msgs::msg::LaserScan & msg);

  bool SetupStart() override;

  bool SetupStop() override;

  std::string GetMetricName() const override;

  std::string GetMetricUnit() const override;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<statistics_msgs::msg::MetricsMessage>::SharedPtr publisher_;
  rclcpp::Time window_start_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  size_t received_message_count_ = 0;
};

}  // namespace fake_robot

#endif  // FAKE_ROBOT__LASTER_LISTENER_HPP_
