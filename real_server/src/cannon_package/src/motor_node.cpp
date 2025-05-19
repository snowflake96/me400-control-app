/*
Servo node is subscribed to the 'motor_command' topic and sends motor commands to the PCA9685 servo driver
*/

#include "cannon_package/motor_node.hpp"

MotorNode::MotorNode() : Node("servo_node"), driver_(std::make_unique<PCA9685Driver>("/dev/i2c-1", 0x40)) {
  // Initialize PCA9685 on I²C bus 1 (0x40), set 50 Hz PWM
  RCLCPP_INFO(this->get_logger(), "Initializing PCA9685 at 50 Hz...");
  driver_->setFrequency(50.0f);

  // Subscribe to motor_command (Vector3: x→ch0, y→ch1, z→ch2)
  servo_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
    "servo_command", 10,
    std::bind(&MotorNode::servo_callback, this, std::placeholders::_1)
  );

  launch_sub_ = create_subscription<std_msgs::msg::Float64>(
    "launch_command", 10,
    std::bind(&MotorNode::launch_callback, this, std::placeholders::_1)
  );

  esc_sub_ = create_subscription<std_msgs::msg::Float64>(
    "esc_command", 10,
    std::bind(&MotorNode::esc_callback, this, std::placeholders::_1)
  );
}

void MotorNode::servo_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
  driver_->setServoSpeed(CH_PITCH, msg->x); // pitch
  driver_->setServoSpeed(CH_YAW, msg->y); // yaw
  RCLCPP_INFO(get_logger(), "Sent servo command – Pitch:  %f,  Yaw:  %f", msg->x, msg->y);
}

void MotorNode::launch_callback(const std_msgs::msg::Float64::SharedPtr msg) {
  driver_->setServoPosition(CH_TRIGGER, msg->data);
  RCLCPP_INFO(get_logger(), "Sent trigger command: %f", msg->data);
}

void MotorNode::esc_callback(const std_msgs::msg::Float64::SharedPtr msg) {
  constexpr static float offset = 0.005;
  driver_->setESC(CH_ESC_L, msg->data);
  driver_->setESC(CH_ESC_R, msg->data + offset);
  RCLCPP_INFO(get_logger(), "Sent ESC command: %f", msg->data);
}