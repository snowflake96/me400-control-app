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
  
  constexpr static uint8_t CH_PITCH = 11;
  constexpr static uint8_t CH_YAW = 15;
  constexpr static uint8_t CH_TRIGGER = 2;
  constexpr static uint8_t CH_ESC_L = 0;
  constexpr static uint8_t CH_ESC_R = 1;

  std::unique_ptr<PCA9685Driver> driver_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr servo_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr launch_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr esc_sub_;
};