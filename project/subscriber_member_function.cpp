#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber"), last_message_time_(this->now())
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
     
    check_timer_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalSubscriber::check_for_messages, this));
  }

private:
  void topic_callback(const std_msgs::msg::String & msg)
  {
    last_message_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
 
  void check_for_messages()
  {
    if ((this->now() - last_message_time_).seconds() > 1.0)
    {
      RCLCPP_WARN(this->get_logger(), "Warning: No message received!");
    }
  }

  rclcpp::TimerBase::SharedPtr check_timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
 
  rclcpp::Time last_message_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
