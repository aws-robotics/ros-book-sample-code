#ifndef FAKE_ROBOT__FAKE_LASER_LISTENER_HPP_
#define FAKE_ROBOT__FAKE_LASER_LISTENER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "libstatistics_collector/collector/collector.hpp"
#include <string>

namespace fake_robot
{
class FakeLaserListenerNode : public rclcpp::Node, public libstatistics_collector::collector::Collector
{
public:
  FakeLaserListenerNode(const rclcpp::NodeOptions & options);

private:
  void publish_message();

  void on_message_received(const sensor_msgs::msg::LaserScan & msg);

  bool SetupStart() override;

  bool SetupStop() override;

  std::string GetMetricName() const override;

  std::string GetMetricUnit() const override;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<statistics_msgs::msg::MetricsMessage>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Time window_start_;
};

}  // namespace fake_robot

#endif  // FAKE_ROBOT__FAKE_LASER_LISTENER_HPP_
