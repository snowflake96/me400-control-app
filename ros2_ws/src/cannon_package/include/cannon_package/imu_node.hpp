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
  struct Tilt{
    double roll;
    double pitch;
  };

  // Callback to read and publish sensor data.
  void publish_sensor_data();
  // Simulate reading sensor data (replace with actual sensor interfacing code).
  Tilt read_sensor();

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  MPU6050 sensor_;
  
  // Complementary filter variables
  double prev_roll_{};   // Previous roll angle in degrees
  double prev_pitch_ = 60.0;  // Previous pitch angle in degrees
  double alpha_;       // Complementary filter constant (0-1)
  std::chrono::steady_clock::time_point last_reading_time_;  // Time of last sensor reading
};