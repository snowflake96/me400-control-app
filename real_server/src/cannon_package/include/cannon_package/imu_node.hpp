#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "imu.hpp"
#include <chrono>

class ImuNode : public rclcpp::Node
{
public:
  ImuNode();
  ~ImuNode() = default;

private:
  // Callback to read and publish sensor data.
  void publish_sensor_data();

  // Simulate reading sensor data (replace with actual sensor interfacing code).
  double read_sensor();

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  MPU6050 sensor_;
  
  // Complementary filter variables
  double prev_pitch_;  // Previous pitch angle in degrees
  double alpha_;       // Complementary filter constant (0-1)
  std::chrono::steady_clock::time_point last_reading_time_;  // Time of last sensor reading
};

class KeyInputNode : public rclcpp::Node
{
public:
    KeyInputNode();
    ~KeyInputNode() = default;
private:
    void publish_keyboard_data();
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};