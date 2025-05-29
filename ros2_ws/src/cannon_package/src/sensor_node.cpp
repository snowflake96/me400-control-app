#include <chrono>
#include <functional>
#include "cannon_package/sensor_node.hpp"

#define TEST

using namespace std::chrono_literals;

SensorNode::SensorNode() : Node("sensor_node"), sensor_("/dev/i2c-1", 0x68)
{
  RCLCPP_INFO(this->get_logger(), "Sensor Node Initialized");
  
  // Create a publisher on the "sensor_data" topic with a queue size of 10.
  publisher_ = this->create_publisher<std_msgs::msg::Float64>("gyro_topic", 10);

  // Create a timer to call publish_sensor_data() every 10 milliseconds (100Hz)
  timer_ = this->create_wall_timer(10ms, std::bind(&SensorNode::publish_sensor_data, this));

#ifndef TEST
  /// Initialize the sensor ///
  if (!sensor_.initialize()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize the IMU (MPU6050).");
  }

  // Optionally adjust sensor settings.
  // Set accelerometer sensitivity: 0 = ±2g, 1 = ±4g, 2 = ±8g, 3 = ±16g.
  if (!sensor_.setAccelerometerRange(1)) {  // Example: set to ±4g.
      RCLCPP_ERROR(this->get_logger(), "Failed to set accelerometer range.");
  }

  // Set gyroscope range (0: ±250 °/s, 1: ±500 °/s, 2: ±1000 °/s, 3: ±2000 °/s).
  if (!sensor_.setGyroRange(0)) {  // Example: set to ±250 °/s.
    RCLCPP_ERROR(this->get_logger(), "Failed to set gyroscope range.");
  }

  // Set sample rate divider (sample_rate = gyro_rate / (1 + divider)).
  if (!sensor_.setSampleRateDivider(9)) {  // Example: if gyro_rate is 1kHz, sample rate becomes 100Hz.
      RCLCPP_ERROR(this->get_logger(), "Failed to set sample rate divider.");
  }

  if (!sensor_.setDLPF(1)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set DLFP");
  }
#endif

  RCLCPP_INFO(this->get_logger(), "IMU initialized. Starting to read sensor data...");
}

void SensorNode::publish_sensor_data()
{
  auto message = std_msgs::msg::Float64();
#ifndef TEST
  message.data = read_sensor();
#endif
  // Log every 2 seconds
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Publishing sensor data: %.2f", message.data);
  publisher_->publish(message);
}

// Read all 6 axes for now
double SensorNode::read_sensor()
{
  // AccelData accel;
  // if(!sensor_.readAccelerometer(accel)) {
  //     RCLCPP_ERROR(this->get_logger(), "Failed to read accelerometer data.");
  // }

  // Implement getAccelkConversionFactor() if required

  GyroData gyro;
#ifndef TEST
  if(!sensor_.readGyroscope(gyro)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read gyroscope data.");
  }
#endif

  // Convert from raw gyro value to rad/s.
  double gyro_z_rad = static_cast<double>(gyro.z) * sensor_.getGyroConversionFactor();

  return gyro_z_rad;
}
