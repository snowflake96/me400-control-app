/*
Motor control node that processes state information and computes a PID output.
Created by: Jisang You on 2021-03-10
*/

#include "cannon_package/main_node.hpp"

MotorController::MotorController() : Node("motor_controller")
{
  RCLCPP_INFO(this->get_logger(), "Motor Node Initialized");

  // Subscribe to "target_topic" with a queue size of 10.
  target_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    "target_topic", 10,
    std::bind(&MotorController::target_callback, this, std::placeholders::_1)
  );

  // // Subscribe to the "bbox" topic with a queue size of 10.
  bbox_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    "bbox", 10,
    std::bind(&MotorController::bbox_callback, this, std::placeholders::_1)
  );

  // Subscribe to the "pid_params" topic with a queue size of 10.
  gyro_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
    "gyro_topic", 10,
    std::bind(&MotorController::gyro_callback, this, std::placeholders::_1)
  );

  // Subscribe to the "pid_gains" topic with a queue size of 10.
  pid_gains_position_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    "pid_gains_position", 10,
    std::bind(&MotorController::set_position_gains_callback, this, std::placeholders::_1)
  );

  // Subscribe to the "pid_gains" topic with a queue size of 10.
  pid_gains_velocity_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    "pid_gains_velocity", 10,
    std::bind(&MotorController::set_velocity_gains_callback, this, std::placeholders::_1)
  );

  // Subscribe to the "pid_params" topic with a queue size of 10.
  params_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    "pid_params_topic", 10,
    std::bind(&MotorController::set_params_callback, this, std::placeholders::_1)
  );

  // Initialize motors (Choose from HW_PWM1, HW_PWM2, SW_PWM1, SW_PWM2)
  motors_.emplace("servo", 18);
  for(auto& motor : motors_){
    motor.second.start();
    // std::thread t(&Motor::Run, &(motor.second)); // Start the motor in a separate thread
    // t.detach(); // Detach the thread to run independently
    // motor.second.stop();
  }

  // Launch server in a separate thread. Pass the handler function to process received data packets.
  RCLCPP_INFO(this->get_logger(), "Starting server thread...");
  auto handler = std::bind(&MotorController::server_handler, this, std::placeholders::_1);
  server_thread_ = std::thread(&Server::launchServer, &server_, handler);
}

MotorController::~MotorController() {
  server_.stop();
  // Ensure proper cleanup of the server thread
  if (server_thread_.joinable()) {
    RCLCPP_INFO(this->get_logger(), "Shutting down server thread...");
    server_thread_.join();
  }
}

// Handler function to process received data packets from client
void MotorController::server_handler(DataPacket packet){
  
  const char* log = nullptr;

  /* The default mode is MANUAL and POSITION */
  switch (packet.type) {
      case DataPacket::Type::COMMAND:
          driving_mode_ = DRIVING_MODE::MANUAL;
          update_command(packet.data.vector3.x, packet.data.vector3.y);
          log = "Set to MANUAL";
          break;
      case DataPacket::Type::AUTONOMOUS:
          driving_mode_ = DRIVING_MODE::AUTONOMOUS;
          log = "Set to AUTONOMOUS";
          break;
      case DataPacket::Type::POSITION_CONTROL:
          control_mode_ = CONTROL_MODE::POSITION;
          log = "Set to POSITION CONTROL";
          break;
      case DataPacket::Type::VELOCITY_CONTROL:
          control_mode_ = CONTROL_MODE::VELOCITY;
          log = "Set to VELOCITY CONTROL";
          break;
      case DataPacket::Type::START:
          // for(auto& motor : motors_) motor.second.start();
          log = "Started motors";
          break;
      case DataPacket::Type::STOP:
          for(auto& motor : motors_) motor.second.stop();
          log = "Stopped motors";
          break;
      default:
          log = "Unknown packet type received";
  }
  RCLCPP_INFO(this->get_logger(), "%s", log);
}

void MotorController::update_command(double throttle, double target){
  // Extract the throttle value from the message and clamp it between 0 and 200.
  throttle_ = std::clamp(throttle, 0.0, 200.0);

  // Compute the angular velocity target based on the control mode
  if(control_mode_ == CONTROL_MODE::POSITION){
    angular_velocity_target_ = pid_position.compute(target, 0.0);
  }
  else if(control_mode_ == CONTROL_MODE::VELOCITY){
    angular_velocity_target_ = target;
  }
}

// Callback invoked upon receiving target values from the "target_topic" (in autonoumous mode).
void MotorController::target_callback(const geometry_msgs::msg::Vector3::SharedPtr msg){
  if(driving_mode_ == DRIVING_MODE::AUTONOMOUS){
    update_command(msg->x, msg->y);
  }
}

// Callback invoked upon receiving target values from the "bbox".
void MotorController::bbox_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
  // Compute the duty cycle for the motors
  if (msg->y == 0) {
    motors_.at("servo").stop();
    return;
  }

  double target_y = 180;
  // double control_signal = pid_velocity.compute(target_y, msg->data->y);
  double error = (target_y) - (msg->y);

  // Send the control signal to the motor.
  if (std::abs(error) < 1) {
    motors_.at("servo").stop();
  } else {
    if (error > 0) {
      std::cout << "CCW" << std::endl;
      std::cout << motors_.at("servo").isRunning() << std::endl;
      motors_.at("servo").ToCCW();
    } else {
      motors_.at("servo").ToCW();
    }
  }
  // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void MotorController::gyro_callback(const std_msgs::msg::Float64::SharedPtr msg) {
  // Compute the duty cycle for the motors
  double control_signal = pid_velocity.compute(angular_velocity_target_, msg->data);
  // double left_motor_duty_cycle = std::clamp(static_cast<int>(std::round(throttle_ + control_signal)), 0, 255);
  // double right_motor_duty_cycle = std::clamp(static_cast<int>(std::round(throttle_ - control_signal)), 0, 255);

  // Send the control signal to the motor.
  // motors_.at("left").setDutyCycle(left_motor_duty_cycle);
  // motors_.at("right").setDutyCycle(right_motor_duty_cycle);
  // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

// Callback that updates the pid gains when a new message is received on "pid_gains".
void MotorController::set_position_gains_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
  double kp = msg->x;  // Proportional gain
  double ki = msg->y;  // Integral gain
  double kd = msg->z;  // Derivative gain
  pid_position.setGains(kp, ki, kd);

  RCLCPP_INFO(this->get_logger(), "Received Position PID gains - Kp: %f, Ki: %f, Kd: %f", kp, ki, kd);
}

// Callback that updates the pid gains when a new message is received on "pid_gains".
void MotorController::set_velocity_gains_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
  double kp = msg->x;  // Proportional gain
  double ki = msg->y;  // Integral gain
  double kd = msg->z;  // Derivative gain
  pid_velocity.setGains(kp, ki, kd);

  RCLCPP_INFO(this->get_logger(), "Received Velocity PID gains - Kp: %f, Ki: %f, Kd: %f", kp, ki, kd);
}

// Callback that updates the pid parameters when a new message is received on "pid_params".
void MotorController::set_params_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
  double cutoff_freq = msg->x;  // Cutoff frequency in Hz
  double sampling_time = msg->y;  // Sampling time in seconds
  double integral_limit = msg->z;  // Integral limit

  pid_position.setParams(cutoff_freq, sampling_time, integral_limit);

  RCLCPP_INFO(this->get_logger(), "Received PID parameters - Cutoff frequency: %f, Sampling time: %f, Integral limit: %f",
              cutoff_freq, sampling_time, integral_limit);
}

