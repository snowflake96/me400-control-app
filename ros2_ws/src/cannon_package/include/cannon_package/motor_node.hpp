#pragma once
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/float64.hpp"
#include "pca9685_driver.hpp"

class MotorNode : public rclcpp::Node {
public:
  explicit MotorNode();

private:
  void servo_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  void launch_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void esc_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void motor_offset_callback(const std_msgs::msg::Float64::SharedPtr msg);
  
  void update_motor_speed(const double throttle);
  
  constexpr static uint8_t CH_PITCH = 1;
  constexpr static uint8_t CH_YAW = 0;
  constexpr static uint8_t CH_ESC_L = 4;
  constexpr static uint8_t CH_ESC_R = 5;
  constexpr static uint8_t CH_TRIGGER = 3;

  inline static double motor_offset_ = 0.0; // L/R motor offset
  double last_throttle_ = 0.0;

  std::unique_ptr<PCA9685Driver> driver_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr servo_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr launch_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr esc_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr motor_offset_sub_;
};