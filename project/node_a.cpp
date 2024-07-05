#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class NodeA : public rclcpp::Node {
public:
    NodeA() : Node("node_a"), last_message_time_(this->now()) {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("topic_b_to_a", 10);
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "topic_a_to_b", 10, std::bind(&NodeA::messageCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), std::bind(&NodeA::publishMessage, this));
        check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2000), std::bind(&NodeA::checkForMessages, this));
    }

private:
    void publishMessage() {
        auto message = std_msgs::msg::Int32();
        message.data = 1;
        publisher_->publish(message);
    }

    void messageCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        last_message_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Received from node_b: %d", msg->data);
    }

    void checkForMessages() {
        if ((this->now() - last_message_time_).seconds() > 2.0) {
            RCLCPP_WARN(this->get_logger(), "Warning: No messages received from node_b!");
        }
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr check_timer_;
    rclcpp::Time last_message_time_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeA>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
