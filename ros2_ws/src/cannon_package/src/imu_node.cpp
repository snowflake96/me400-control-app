#include <chrono>
#include <functional>
#include "cannon_package/imu_node.hpp"

// #define TEST

using namespace std::chrono_literals;

ImuNode::ImuNode() : Node("imu_node"), sensor_("/dev/i2c-1", 0x68), alpha_(0.98), last_reading_time_(std::chrono::steady_clock::now())
{
  RCLCPP_INFO(this->get_logger(), "Initializing IMU Node...");
  
  // Create a publisher on the "sensor_data" topic with a queue size of 10.
  publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("imu_topic", 10);

  /// Initialize the sensor ///
  if (!sensor_.initialize()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize the IMU (MPU6050).");
  }

  // Optionally adjust sensor settings.
  // Set accelerometer sensitivity: 0 = ±2g, 1 = ±4g, 2 = ±8g, 3 = ±16g.
  if (!sensor_.setAccelerometerRange(0)) {  // Example: set to ±2g.
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

  if (!sensor_.setDLPF(static_cast<uint8_t>(MPU6050::DLPFConfig::BW_94HZ))) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set DLFP");
  }

  // Get all servo params
  this->declare_parameter("gyro_offsets.x", 0.0);
  this->declare_parameter("gyro_offsets.y", 0.0);
  this->declare_parameter("gyro_offsets.z", 0.0);
  this->declare_parameter("log_gyro", false);

  gyro_offsets.x = this->get_parameter("gyro_offsets.x").as_double();
  gyro_offsets.y = this->get_parameter("gyro_offsets.y").as_double();
  gyro_offsets.z = this->get_parameter("gyro_offsets.z").as_double();
  log_gyro = this->get_parameter("log_gyro").as_bool();
  
  RCLCPP_INFO(this->get_logger(), "IMU initialized Successfully. Starting to read sensor data...");

  // Create a timer to call publish_sensor_data() every 10 milliseconds (100Hz)
  timer_ = this->create_wall_timer(10ms, std::bind(&ImuNode::publish_sensor_data, this));
}

void ImuNode::publish_sensor_data()
{
  static size_t counter = 0;

  Tilt tilt = read_sensor();
  
  // Publish at 50Hz
  if(++counter == 2){
    counter = 0;
    auto msg = geometry_msgs::msg::Vector3();
    msg.x = tilt.roll;
    msg.y = tilt.pitch;
    publisher_->publish(msg);
  }
}

ImuNode::Tilt ImuNode::read_sensor()
{
  static constexpr double rad_to_deg = 180.0 / M_PI;

  // Calculate time difference since last reading
  auto current_time = std::chrono::steady_clock::now();
  double dt = std::chrono::duration<double>(current_time - last_reading_time_).count();
  last_reading_time_ = current_time;

  AccelData accel{};
  GyroData gyro{};
  if(!sensor_.readAccelGyro(accel, gyro)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read IMU data.");
      return {prev_roll_, prev_pitch_};  // Return previous pitch if read fails
  }

  // Get the acceleration in m/s^2
  double accel_scale = sensor_.getAccelConversionFactor();
  double accel_x = static_cast<double>(accel.x) * accel_scale;
  double accel_y = static_cast<double>(accel.y) * accel_scale;
  double accel_z = static_cast<double>(accel.z) * accel_scale * -1;

  // Convert gyro to degrees per second  
  double gyro_scale = sensor_.getGyroConversionFactor();
  double gyro_x_deg = static_cast<double>(gyro.x) * gyro_scale - gyro_offsets.x;
  double gyro_y_deg = static_cast<double>(gyro.y) * gyro_scale - gyro_offsets.y;
  
  if(log_gyro){
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200, "Gyro values (deg/s) - X: %.3f, Y: %.3f", gyro_x_deg, gyro_y_deg);
  }

  // Calculate roll from accelerometer
  double accel_roll = atan2(-accel_y, std::hypot(accel_x, accel_z));
  double accel_roll_deg = accel_roll * rad_to_deg;

  // Calculate pitch from accelerometer
  double accel_pitch = atan2(accel_x, std::hypot(accel_y, accel_z));
  double accel_pitch_deg = accel_pitch * rad_to_deg;

  // Complementary filter for both pitch and roll using actual dt
  double roll_deg = alpha_ * (prev_roll_ + gyro_x_deg * dt) + (1.0 - alpha_) * accel_roll_deg;
  double pitch_deg = alpha_ * (prev_pitch_ + gyro_y_deg * dt) + (1.0 - alpha_) * accel_pitch_deg;
  
  // Store current pitch for next iteration
  prev_roll_ = roll_deg;
  prev_pitch_ = pitch_deg;

  return {roll_deg, pitch_deg};
}
