/*
Servo node is subscribed to the 'motor_command' topic and sends motor commands to the PCA9685 servo driver
*/

#include "cannon_package/motor_node.hpp"

MotorNode::MotorNode() : Node("motor_node"), driver_(std::make_unique<PCA9685Driver>("/dev/i2c-1", 0x40)) {
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

  motor_offset_sub_ = create_subscription<std_msgs::msg::Float64>(
    "motor_offset", 10,
    std::bind(&MotorNode::motor_offset_callback, this, std::placeholders::_1)
  );

}

void MotorNode::servo_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
  driver_->setServoSpeed(CH_PITCH, msg->x, 1000.f, 1690.f, 1950.f, 2600.f); // pitch
  driver_->setServoSpeed(CH_YAW, msg->y, 1000.f, 1740.f, 1880.f, 2600.f); // yaw
  // RCLCPP_INFO(get_logger(), "Sent servo command – Pitch:  %f,  Yaw:  %f", msg->x, msg->y);
}

void MotorNode::launch_callback(const std_msgs::msg::Float64::SharedPtr msg) {
  driver_->setServoPosition(CH_TRIGGER, msg->data);
  // RCLCPP_INFO(get_logger(), "Sent trigger command: %f", msg->data);
}

void MotorNode::esc_callback(const std_msgs::msg::Float64::SharedPtr msg) {
  update_motor_speed(msg->data);
  // RCLCPP_INFO(get_logger(), "Sent ESC command: %f", msg->data);
}

void MotorNode::motor_offset_callback(const std_msgs::msg::Float64::SharedPtr msg){
  motor_offset_ = msg->data;
  update_motor_speed(last_throttle_);
  // RCLCPP_INFO(get_logger(), "Set L/R motor offset to : %f", msg->data);
}

void MotorNode::update_motor_speed(const double throttle){
  double left_throttle = throttle, right_throttle = throttle;

  if(motor_offset_ > 0.0){
    right_throttle += motor_offset_;
  }
  else if(motor_offset_ < 0.0){
    left_throttle -= motor_offset_;
  }


  driver_->setESC(CH_ESC_R, right_throttle);
  driver_->setESC(CH_ESC_L, left_throttle);
  last_throttle_ = throttle;
}