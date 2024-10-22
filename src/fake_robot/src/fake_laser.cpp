// This file is licensed under MIT-0 (https://github.com/aws/mit-0)
// which can be found in the 'LICENSE' file in this repository.

#include "fake_robot/fake_laser.hpp"

#include <chrono>
#include <cmath>

namespace fake_robot
{

FakeLaserNode::FakeLaserNode(const rclcpp::NodeOptions & options)
: Node("fake_laser", options)
{
  // Use the default QoS profile to start
  rclcpp::QoS qos_offer(rclcpp::KeepLast(5));
  rclcpp::PublisherOptions publisher_options;
  publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
    "scan", qos_offer, publisher_options);
  timer_ = create_wall_timer(
    std::chrono::milliseconds(200), std::bind(&FakeLaserNode::publish_timer_callback, this));
}

void FakeLaserNode::publish_timer_callback()
{
  publish_count_++;
  create_arbitrary_fake_laser_scan();
  RCLCPP_INFO(get_logger(), "Publishing laser scan");
  publisher_->publish(std::move(message_));
}

void FakeLaserNode::create_arbitrary_fake_laser_scan()
{
  message_ = std::make_unique<sensor_msgs::msg::LaserScan>();
  message_->header.frame_id = "base_link";
  message_->header.stamp = get_clock()->now();
  message_->angle_min = angle_min_;
  message_->angle_max = angle_max_;
  message_->angle_increment = angle_increment_;
  message_->time_increment = scan_period_ / points_per_scan_;
  message_->scan_time = scan_period_;
  message_->range_min = min_range_;
  message_->range_max = max_range_;

  // This loop fills the scan points as a circle with a little "wobble" over time
  // so that we can see movement in RViz. The actual shape doesn't have meaning.
  message_->ranges.reserve(points_per_scan_);
  double wobble_size = wobble_range_ * std::cos(DEG_TO_RAD * publish_count_ * wobble_speed_);
  for (size_t i = 0; i < points_per_scan_; i++) {
    double wobble = wobble_size * std::sin(i * DEG_TO_RAD * num_wobbles_);
    message_->ranges.push_back(base_radius_ + wobble);
  }
}

}  // namespace fake_robot

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<fake_robot::FakeLaserNode>(options);

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
RCLCPP_COMPONENTS_REGISTER_NODE(fake_robot::FakeLaserNode)
