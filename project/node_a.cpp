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
