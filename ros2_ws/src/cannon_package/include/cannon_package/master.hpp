/*
Motor control node that processes state information and computes a PID output.
Created by: Jisang You on 2021-03-10
*/
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <cmath> // std::round
#include <algorithm>
#include <unordered_map>
#include <string>
#include <thread>

#include "pid.hpp"
#include "motor.hpp"
#include "server.hpp"

class MotorController : public rclcpp::Node { 
public:
  MotorController();
  ~MotorController();
  
private:
  // Handler function to process received data packets from client
  void server_handler(DataPacket packet);
  // Function to update the command and compute the angular velocity target.
  void update_command(double throttle, double target);
  // Callback invoked upon receiving target values from the "target_topic" (in autonoumous mode).
  void target_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  // Callback invoked upon receiving target values from the "bbox".
  void bbox_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  // Callback for inner angular velocity feedback on receiving msg from "gyro_topic".
  void gyro_callback(const std_msgs::msg::Float64::SharedPtr msg);
  // Callback that updates the pid gains when a new message is received on "pid_gains".
  void set_position_gains_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  // Callback that updates the pid gains when a new message is received on "pid_gains".
  void set_velocity_gains_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);
  // Callback that updates the pid parameters when a new message is received on "pid_params".
  void set_params_callback(const geometry_msgs::msg::Vector3::SharedPtr msg);

  enum class CONTROL_MODE : uint8_t {
    POSITION = 0,
    VELOCITY = 1
  } control_mode_ = CONTROL_MODE::POSITION;

  enum class DRIVING_MODE : uint8_t {
    AUTONOMOUS = 0,
    MANUAL = 1
  } driving_mode_ = DRIVING_MODE::MANUAL;

  // PID gains and parameters subscriptions
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr target_subscription_, bbox_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gyro_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr pid_gains_position_subscription_, pid_gains_velocity_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr params_subscription_;
  std::unordered_map<std::string, Motor> motors_;
  PID pid_position, pid_velocity;
  double throttle_, angular_velocity_target_;
  
  Server server_; // default port number is 12345
  std::thread server_thread_; // Thread for running the server
};

