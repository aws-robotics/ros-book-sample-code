// This file is licensed under MIT-0 (https://github.com/aws/mit-0)
// which can be found in the 'LICENSE' file in this repository.

#include "fake_robot/laser_listener.hpp"

namespace fake_robot
{

LaserListenerNode::LaserListenerNode(const rclcpp::NodeOptions & options)
: Node("fake_laser", options)
{
  // Use the SensorData QoS profile with a KeepLast initializer reflecting our chosen history depth
  auto qos_request = rclcpp::SensorDataQoS(rclcpp::KeepLast(5));
  qos_request.deadline(std::chrono::milliseconds(300));

  auto callback =
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) -> void
    {
      RCLCPP_INFO(this->get_logger(), "Received laser scan %d with %d points",
        received_message_count_++, msg->ranges.size());
    };

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.event_callbacks.incompatible_qos_callback =
    [this](rclcpp::QOSRequestedIncompatibleQoSInfo & event) -> void
    {
      RCLCPP_INFO(
        get_logger(),
        "My QoS request was incompatible with a publisher's offer. "
        "This has happened %d times", event.total_count);
    };
  subscription_options.event_callbacks.deadline_callback =
    [this](rclcpp::QOSDeadlineRequestedInfo & event) -> void
    {
      RCLCPP_INFO(
        get_logger(),
        "The publisher missed its deadline for sending a message. "
        "This has happened %d times.", event.total_count);
    };
  subscription_options.event_callbacks.liveliness_callback =
    [this](rclcpp::QOSLivelinessChangedInfo & event) -> void
    {
      RCLCPP_INFO(
        get_logger(),
        "The number of live publishers on /scan has changed. "
        "There are now %d live publishers.", event.alive_count);
    };

  subscription_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", qos_request, callback, subscription_options);
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
