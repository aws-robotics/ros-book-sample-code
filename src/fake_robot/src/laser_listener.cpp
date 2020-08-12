// This file is licensed under MIT-0 (https://github.com/aws/mit-0)
// which can be found in the 'LICENSE' file in this repository.

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
      this->on_message_received(*msg);
    };
  rclcpp::SubscriptionOptions subscription_options;

  subscription_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", qos_request, callback, subscription_options);

  publisher_ = create_publisher<statistics_msgs::msg::MetricsMessage>(std::string("/system_metrics"), qos_request);

  window_start_ = now();

  publish_timer_ = this->create_wall_timer(
    std::chrono::seconds(10), [this]() {
      this->publish_message();
      this->window_start_ = this->now();
    });
}

void LaserListenerNode::publish_message()
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

void LaserListenerNode::LaserListenerNode::on_message_received(const sensor_msgs::msg::LaserScan & msg)
{
  RCLCPP_INFO(this->get_logger(), "Received laser scan %d with %d points",
              received_message_count_++, msg.ranges.size());
  AcceptData(msg.scan_time);
}

bool LaserListenerNode::SetupStart()
{
  return true;
}

bool LaserListenerNode::SetupStop()
{
  return true;
}

std::string LaserListenerNode::GetMetricName() const
{
  return "scan_time";
}

std::string LaserListenerNode::GetMetricUnit() const
{
  return "seconds";
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
