#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class NodeB : public rclcpp::Node {
public:
    NodeB() : Node("node_b"), last_message_time_(this->now()) {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("topic_b_to_a", 10);
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "topic_a_to_b", 10, std::bind(&NodeB::messageCallback, this, std::placeholders::_1));
        check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), std::bind(&NodeB::checkForMessages, this));
    }

private:

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr check_timer_;
    rclcpp::Time last_message_time_;

    void messageCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        last_message_time_ = this->now();
        int response = (msg->data == 1)? 0 : 1;
        auto message = std_msgs::msg::Int32();
        message.data = response;
        RCLCPP_INFO(this->get_logger(), "Received %d, response %d", msg->data, message.data);
        publisher_->publish(message);
    }

    void checkForMessages() {
        if ((this->now() - last_message_time_).seconds() > 1.0) {
            RCLCPP_WARN(this->get_logger(), "Warning: No messages received from node_a!");
        }
    }


};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeB>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
