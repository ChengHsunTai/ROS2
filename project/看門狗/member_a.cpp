#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class NodeA : public rclcpp::Node {
public:
    NodeA() : Node("node_a"), last_message_time_(this->now()), last_data_(2) {
        publisher_ = this->create_publisher<std_msgs::msg::Int32>("topic_a_to_b", 10);
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "topic_b_to_a", 10, std::bind(&NodeA::messageCallback, this, std::placeholders::_1));
        check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), std::bind(&NodeA::checkForMessages, this));
           
        send_message(1);
    }

private:
   
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr check_timer_;
    rclcpp::Time last_message_time_;
    int last_data_;

    void messageCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        last_message_time_ = this->now();
        int response = (msg->data == 1)? 1 : 0;
        if (last_data_ == msg->data){
        RCLCPP_WARN(this->get_logger(), "Node B may be crashed!");        
        }else{
        RCLCPP_INFO(this->get_logger(), "Received %d, response %d", msg->data, response);
        }
        last_data_ = msg->data;
        send_message(response);
    }

    void checkForMessages() {
        if ((this->now() - last_message_time_).seconds() > 1.0) {
            RCLCPP_WARN(this->get_logger(), "Warning: No messages received from node_b!");
        }
    }
   
    void send_message(int data){
    auto message = std_msgs::msg::Int32();
    message.data = data;
    publisher_->publish(message);
    }


};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeA>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
