#pragma once
#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "imu.hpp"

class SensorNode : public rclcpp::Node
{
public:
  SensorNode();
  ~SensorNode() = default;

private:
  // Callback to read and publish sensor data.
  void publish_sensor_data();

  // Simulate reading sensor data (replace with actual sensor interfacing code).
  double read_sensor();

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  MPU6050 sensor_;
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