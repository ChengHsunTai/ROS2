#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class NodeB : public rclcpp::Node {
public:
    NodeB() : Node("node_b"), last_message_time_(this->now()), last_data_(2), count(0) {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("topic_b_to_a", 10);
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "topic_a_to_b", 10, std::bind(&NodeB::messageCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&NodeB::PublishMessage, this));
        check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), std::bind(&NodeB::checkForMessages, this));
    }

private:

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr check_timer_;
    rclcpp::Time last_message_time_;
    int last_data_;
    int count;
   
    void PublishMessage (){
    if (count % 2 == 1){
    SendMessage(1);
    }else {
    SendMessage(0);
    }
    count++;
    if (count == 10)
    count = 0;
    }

    void messageCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        last_message_time_ = this->now();
        if (last_data_ == msg->data){
        RCLCPP_INFO(this->get_logger(), "repeatedly received %d .member_b may be crash!", msg->data);
        }else{
        RCLCPP_INFO(this->get_logger(), "Received %d", msg->data);
        }
    last_data_ = msg->data;
    }

    void checkForMessages() {
        if ((this->now() - last_message_time_).seconds() > 1.0) {
            RCLCPP_WARN(this->get_logger(), "Warning: No messages received from node_a!");
        }
    }
   
    void SendMessage(int data){
    auto message = std_msgs::msg::Int32();
    message.data = data;
    publisher_->publish(message);
    }

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeB>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
