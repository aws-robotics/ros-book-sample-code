// This file is licensed under MIT-0 (https://github.com/aws/mit-0)
// which can be found in the 'LICENSE' file in this repository.

#ifndef FAKE_ROBOT__FAKE_LASER_HPP_
#define FAKE_ROBOT__FAKE_LASER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace fake_robot
{

// Defining some math constants here to avoid external dependencies for this demo
const double PI = 3.141592659;
const uint DEGREES_IN_CIRCLE = 360;
const double RADS_IN_CIRCLE = PI * 2.0;
const double DEG_TO_RAD = RADS_IN_CIRCLE / DEGREES_IN_CIRCLE;

class FakeLaserNode : public rclcpp::Node
{
public:
  explicit FakeLaserNode(const rclcpp::NodeOptions & options);

private:
  void publish_timer_callback();
  void create_arbitrary_fake_laser_scan();

  std::unique_ptr<sensor_msgs::msg::LaserScan> message_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t publish_count_ = 0;

  // The below properties of this fake 2D LIDAR are arbitrary for this demo,
  // but are comparable to e.g. Turtlebot 3 or common robotic vacuums with attached scanners.
  const double scan_period_ = 0.2;
  const uint points_per_scan_ = DEGREES_IN_CIRCLE;
  const double angle_min_ = 0.0;
  const double angle_max_ = RADS_IN_CIRCLE;
  const double angle_increment_ = 1.0 * DEG_TO_RAD;
  const double min_range_ = 0.1;
  const double max_range_ = 5.0;

  // The following settings dictate the shape of the fake laser scan, they are arbitrary.
  const double base_radius_ = 1.0;
  const double wobble_range_ = 0.1;
  const double wobble_speed_ = 5.0;
  const double num_wobbles_ = 7.0;
};

}  // namespace fake_robot

#endif  // FAKE_ROBOT__FAKE_LASER_HPP_
