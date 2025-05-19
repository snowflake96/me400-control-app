/*
Motor control node that processes state information and computes a PID output.
Created by: Jisang You on 2021-03-10
*/
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <cmath> // std::round, std::isnan
#include <limits>
#include <algorithm>
#include <string>
#include <thread>
#include <chrono>
#include <format>
#include <cstring>
#include <string_view>
#include <atomic>
#include "pid.hpp"
#include "server.hpp"

class MasterNode : public rclcpp::Node { 
public:
  MasterNode();
  ~MasterNode();
  
private:
  // Handler function to process received data packets from client
  void server_handler(const DataPacket& packet, int client_fd);
  // Callback invoked upon receiving target values from the "target_topic" (in autonoumous mode).
  void bbox_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg);
  // Callback for receiving pitch angle from IMU
  void pitch_callback(const std_msgs::msg::Float64::SharedPtr msg);
  // Sends servo commands to the PCA9685 driver
  void send_servo_command(double x, double y);
  // Sends launch servo commands to the PCA9685 driver
  void send_launch_command(bool launch);
  // Sends esc commands to the PCA9685 driver
  void send_esc_command(double speed);
  // Callback that updates the pid gains when a new message is received on "pid_gains_pitch".
  void set_pitch_gains_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  // Callback that updates the pid gains when a new message is received on "pid_gains_yaw".
  void set_yaw_gains_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  // Callback invoked upon limit switch trigger
  void switch_pressed_callback(const std_msgs::msg::Int32::SharedPtr msg);
  // Callback invoked upon limit switch release
  void switch_released_callback(const std_msgs::msg::Int32::SharedPtr msg);
  // Function to set the offset
  void set_offset(const double pitch);
  // Reset
  void reset();
  // Accelerate motor to default speed
  void accelerate_to_default_speed();

  enum class DrivingMode : uint8_t {
    Autonomous = 0, // Auto aim and fire
    Manual = 1, // Fully manual
    AutoAim = 2 // Auto aim, manual fire/esc control/set offset
  } mode_{DrivingMode::Manual};

  enum class State : uint8_t {
    Running = 0,
    Stopped = 1
  } state_{State::Stopped};

  enum class Switch : uint8_t {
    Left=0,
    Right,
    Front,
    Back
  };

  struct SwitchState{
    bool left, right, front, back;
  } switch_{};

  struct Target{
    std::atomic<double> x, y;
  } target_{};

  struct LaunchThreshold{
    std::atomic<double> EPS;
    std::atomic<uint8_t> N;
  } launch_threshold_{0.01, 10};

  bool found_offset_ = false;
  
  size_t pitch_counter_{0};
  size_t launch_counter_{0};

  double current_pitch_{0.0};
  double current_speed_{0.0};

  static constexpr double DEFAULT_SPEED = 0.2; // Default ESC value (for autonomous mode)
  
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr servo_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr launch_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr esc_command_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr bbox_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pitch_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pid_gains_pitch_subscription_, pid_gains_yaw_subscription_; 
  // rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr switch_pressed_subscription_, switch_released_subscription_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  
  PID pid_pitch_, pid_yaw_;
  
  Server server_; // default port number is 12345
  std::thread server_thread_; // Thread for running the server
};

