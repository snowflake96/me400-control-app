#include <chrono>
#include <functional>
#include "cannon_package/sensor_node.hpp"

using namespace std::chrono_literals;



KeyInputNode::KeyInputNode() : Node("keyinput_node")
{
    RCLCPP_INFO(this->get_logger(), "Key Input Node has started. Press 'w' or 's' to publish.");
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("key_topic", 10);
    timer_ = this->create_wall_timer(2000ms, std::bind(&KeyInputNode::publish_keyboard_data, this));
}

char c = 'w';

void KeyInputNode::publish_keyboard_data()
{
    auto message = std_msgs::msg::Float64();
        // std::cout << "Enter a message: ";
        // std::cin >> c;

    switch(c)
    {
        case 'w':
            message.data = 1000;
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Published: 1000");
            c = 's';
            break;
        case 's':
            message.data = 2000;
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Published: 2000");
            c = 'w';
            break;
        default:
            message.data = 1500;
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Published: 1500");
            break;
    }
}