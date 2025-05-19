/*
Master node that processes state information and computes motor commands.
Created by: Jisang You on 2021-03-10
*/

#include "cannon_package/master.hpp"
using namespace std::chrono_literals;

MasterNode::MasterNode() : Node("master_node")
{
  using namespace std::placeholders;

  RCLCPP_INFO(this->get_logger(), "Initializing Master Node...");

  // Publisher for motor commands
  servo_command_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("servo_command", 10);
  launch_command_pub_ = this->create_publisher<std_msgs::msg::Float64>("launch_command", 10);
  esc_command_pub_ = this->create_publisher<std_msgs::msg::Float64>("esc_command", 10);
 
  // Subscribe to the "bbox" topic with a queue size of 1 to prevent build up of messages while sleeping
  bbox_subscription_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
    "bbox", 1, std::bind(&MasterNode::bbox_callback, this, _1)
  );

  // Subscribe to the pitch angle from IMU
  pitch_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
    "pitch_topic", 10,
    std::bind(&MasterNode::pitch_callback, this, _1)
  );

  // Stop all motors
  state_ = State::Running;
  send_launch_command(false); // Retract the launch servo initially
  send_esc_command(0.0); // Stop esc
  send_servo_command(0.0, 0.0); // Stop servos
  state_ = State::Stopped;

  // Launch server in a separate thread. Pass the handler function to process received data packets.
  RCLCPP_INFO(this->get_logger(), "Starting server thread...");
  auto handler = std::bind(&MasterNode::server_handler, this, _1, _2);
  server_thread_ = std::thread(&Server::launchServer, &server_, handler);
}

MasterNode::~MasterNode() {
  server_.stop();
  // Ensure proper cleanup of the server thread
  if (server_thread_.joinable()) {
    RCLCPP_INFO(this->get_logger(), "Shutting down server thread...");
    server_thread_.join();
  }
}

// Handler function to process received data packets from client
void MasterNode::server_handler(const DataPacket &packet, int client_fd) {
  const auto &v       = packet.data.vector3;
  const auto  w       = packet.data.w;
  const auto  boolean = packet.data.boolean;
  const auto &threshold = packet.data.threshold;

  /* Starting mode is AutoAim and Stopped */

  std::string log_msg;

  switch (packet.type) {
    case DataPacket::Type::ServoCommand:
      if (mode_ == DrivingMode::Manual) {
        send_servo_command(v.x, v.y);
        log_msg = std::format( "Sent servo command – x: {:.3f}, y: {:.3f}", v.x, v.y);
      } 
      else{
        log_msg = "Switch to manual mode to send servo command";
      }
      break;

    case DataPacket::Type::TriggerCommand:
      if (mode_ == DrivingMode::Manual || mode_ == DrivingMode::AutoAim) {
        send_launch_command(boolean);
        log_msg = std::format( "Sent trigger command: {}", boolean ? "push" : "retract");
      }
      else{
        log_msg = "Switch to manual or autoaim mode to send trigger command";
      }
      break;

    case DataPacket::Type::EscCommand:
      if (mode_ == DrivingMode::Manual || mode_ == DrivingMode::AutoAim) {
        send_esc_command(w);
        log_msg = std::format( "Sent ESC command – w: {:.3f}", w);
      }
      else{
        log_msg = "Switch to manual or autoaim mode to send ESC command";
      }
      break;

    case DataPacket::Type::TunePitch:
      pid_pitch_.setGains(v.x, v.y, v.z); // v.z is not used
      log_msg = std::format("Set pitch gains – Kp: {:.3f}, Ki: {:.3f}", v.x, v.y);
      break;

    case DataPacket::Type::TuneYaw:
      pid_yaw_.setGains(v.x, v.y, v.z); // v.z is not used
      log_msg = std::format("Set yaw gains – Kp: {:.3f}, Ki: {:.3f}", v.x, v.y);
      break;

    case DataPacket::Type::Start:
      if (state_ == State::Stopped) {
        state_ = State::Running;
        reset();
        log_msg = "Started motors";
      }
      else{
        log_msg = "Motors already running";
      }
      break;

    case DataPacket::Type::Stop:
      if(state_ == State::Running){
        send_servo_command(0.0, 0.0);
        send_esc_command(0.0);
        state_ = State::Stopped;
        log_msg = "Stopped motors";
      }
      else{
        log_msg = "Motors already stopped";
      }
      break;

    case DataPacket::Type::SetAutonomous:
      if(mode_ != DrivingMode::Autonomous){
        mode_ = DrivingMode::Autonomous;
        reset();
        log_msg = "Set to autonomous mode";
      }
      else{
        log_msg = "Already in autonomous mode";
      }
      break;

    case DataPacket::Type::SetManual:
      if(mode_ != DrivingMode::Manual){
        mode_ = DrivingMode::Manual;
        log_msg = "Set to manual mode";
      }
      else{
        log_msg = "Already in manual mode";
      }
      break;

    case DataPacket::Type::SetAutoAim:
      if(mode_ != DrivingMode::AutoAim){
        mode_ = DrivingMode::AutoAim;
        log_msg = "Set to automatic aim mode";
      }
      else{
        log_msg = "Already in autoaim mode";
      }
      break;

    case DataPacket::Type::SetOffset:
      if(mode_ == DrivingMode::Manual || mode_ == DrivingMode::AutoAim){
        target_.x = v.x;
        target_.y = v.y;
        log_msg = std::format("Set offset to x: {:.3f}, y: {:.3f}", v.x, v.y);
      }
      else{
        log_msg = "Switch to manual or autoaim mode to set offset";
      }
      break;

    case DataPacket::Type::SetPitchIntegralLimit:
      pid_pitch_.setIntegralLimit(w);
      log_msg = std::format("Set pitch integral limit to: {:.3f}", w);
      break;

    case DataPacket::Type::SetYawIntegralLimit:
      pid_yaw_.setIntegralLimit(w);
      log_msg = std::format("Set yaw integral limit to: {:.3f}", w);
      break;
    
    case DataPacket::Type::SetLaunchThreshold:
      launch_threshold_.EPS = threshold.EPS;
      launch_threshold_.N = threshold.N;
      log_msg = std::format("Set launch threshold to EPS: {:.3f}, N: {}", threshold.EPS, threshold.N);
      break;
    
    case DataPacket::Type::Query: {
      DataPacket packet{};
      const char* mode = nullptr;
      switch(mode_){
        case DrivingMode::Autonomous:
          packet.type = DataPacket::Type::SetAutonomous;
          mode = "autonomous";
          break;
        case DrivingMode::Manual:
          packet.type = DataPacket::Type::SetManual;
          mode = "manual";
          break;
        case DrivingMode::AutoAim:
          packet.type = DataPacket::Type::SetAutoAim;
          mode = "autoaim";
          break;
      }
      server_.sendDataToClient(packet, client_fd);

      const char* state = nullptr;
      switch(state_){
        case State::Running:
          packet.type = DataPacket::Type::Start;
          state = "running";
          break;
        case State::Stopped:
          packet.type = DataPacket::Type::Stop;
          state = "stopped";
          break;
      }
      server_.sendDataToClient(packet, client_fd);
      log_msg = std::format("Mode: {} State: {}", mode, state);
      break;
    }

    default:
      log_msg = "Unknown packet type received";
  }
  
  RCLCPP_INFO(this->get_logger(), "%s", log_msg.c_str());
  server_.sendLogToClient(log_msg, client_fd);
}

// Function to reset all parameters, control, and motor speed to default (keep mode and state same)
void MasterNode::reset(){
  // Reset params to default
  launch_counter_ = 0;
  pitch_counter_ = 0;
  found_offset_ = false;
  // Reset control to default
  pid_pitch_.resetIntegral();
  pid_yaw_.resetIntegral();
  // Reset motor speed to default if running
  if(state_ == State::Running){
    if(current_speed_ < DEFAULT_SPEED){
      accelerate_to_default_speed();
    }
    else{
      send_esc_command(DEFAULT_SPEED);
    }
  }
}

void MasterNode::accelerate_to_default_speed(){
  // Accelerate at 0.5 throttle for 300ms before setting to default speed
  send_esc_command(0.5); 
  timer_ = this->create_wall_timer(300ms, [this]() { timer_->cancel(); send_esc_command(DEFAULT_SPEED); });
}

// Function to compute and set the offset from the pitch angle to the bell
void MasterNode::set_offset(const double pitch){
  target_.x = 0.0;
  target_.y = 0.0 * pitch; // Find polynomial equation from data

  std::string log = std::format("Offset found - x: {:.3f}, y: {:.3f}", target_.x.load(), target_.y.load());
  RCLCPP_INFO(this->get_logger(), log.c_str());
  server_.sendLogToClient(log);

  DataPacket packet{};
  packet.type = DataPacket::Type::SetOffset;
  packet.data.vector3.x = target_.x;
  packet.data.vector3.y = target_.y;
  server_.sendDataToClient(packet);
}

// Callback invoked upon receiving target values from the "bbox".
void MasterNode::bbox_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
  static LowPassFilter lpf_x(10.0), lpf_y(10.0); // LPF with cut-off frequency of 10Hz
  static size_t nan_counter=0;

  // Note: NaN propagates
  double x1 = 2.0 * msg->x - 1.0;  // x1 normalized to [-1,1]
  double y1 = 2.0 * msg->y - 1.0;  // y1 normalized to [-1,1]
  double x2 = 2.0 * msg->z - 1.0;  // x2 normalized to [-1,1]
  double y2 = 2.0 * msg->w - 1.0;  // y2 normalized to [-1,1]

  // Forward the bbox corners to client (top-left and bottom-right)
  DataPacket packet{};
  packet.type = DataPacket::Type::BboxPos;
  packet.data.double_array[0] = x1;
  packet.data.double_array[1] = y1;
  packet.data.double_array[2] = x2;
  packet.data.double_array[3] = y2;
  server_.sendDataToClient(packet);
  RCLCPP_INFO(this->get_logger(), "Sending bbox: x1: %f, y1: %f, x2: %f, y2: %f", x1, y1, x2, y2);

  // Return here if in manual control mode or stopped
  if(mode_ == DrivingMode::Manual || state_ == State::Stopped){
    return;
  }

  // If no bounding box is detected for 10 consecutive times, stop servo
  if(std::isnan(x1)){
    if(++nan_counter >= 10){
      send_servo_command(0.0, 0.0);
      pid_pitch_.resetIntegral();
      pid_yaw_.resetIntegral();
      if(nan_counter == 10){
        const char* log_msg = "No detection for 5 consecutive times. Disabling servos.";
        RCLCPP_INFO(this->get_logger(), log_msg);
        server_.sendLogToClient(log_msg);
      }
    }
    return;
  }
  else if(nan_counter > 0){
    nan_counter = 0;
    const char* log_msg = "New detection. Re-enabling servos.";
    RCLCPP_INFO(this->get_logger(), log_msg);
    server_.sendLogToClient(log_msg);
  }

  // Calculate center coordinates from bounding box
  double center_x = (x1 + x2) / 2.0;  // Average of left and right
  double center_y = (y1 + y2) / 2.0;  // Average of top and bottom

  // Pass to LPF
  center_x = lpf_x.filter(center_x);
  center_y = lpf_x.filter(center_y);

  // Compute and send control signal to servos
  double control_signal_pitch = pid_pitch_.compute(target_.y, center_y);
  double control_signal_yaw = pid_yaw_.compute(target_.x, center_x);
  send_servo_command(control_signal_pitch, control_signal_yaw);
  
  if(mode_ == DrivingMode::Autonomous){
    // Find the offset
    if(!found_offset_){
      if(pitch_counter_ > 5){
        static constexpr double deg_to_rad = M_PI / 180.0;
        static constexpr double rad_to_deg = 180.0 / M_PI;
        static constexpr double FOV_Y = 41.0; // vertical FOV in degrees
        double bell_angle = std::atan(center_y * std::tan(0.5 * FOV_Y * deg_to_rad)) * rad_to_deg; // Bell angle in camera frame
        double pitch_to_bell = current_pitch_ + bell_angle;
        set_offset(pitch_to_bell);
        found_offset_ = true;
      }
    }
    // Execute launch command
    else{
      double p_error = pid_pitch_.getLastError();
      double y_error = pid_yaw_.getLastError();
      if(p_error < launch_threshold_.EPS && y_error < launch_threshold_.EPS){
        // If the error is less than EPS for N consecutive times, launch
        if(++launch_counter_ > launch_threshold_.N){
          const char* log = "Launching cannon!";
          RCLCPP_INFO(this->get_logger(), log);
          server_.sendLogToClient(log);
          // Launch
          send_launch_command(true); // Push
          timer_ = this->create_wall_timer(500ms, [this]() { 
            timer_->cancel(); 
             // Retract and accelerate after 500ms
            send_launch_command(false);
            accelerate_to_default_speed();
          }); 
          launch_counter_ = 0;
        }
      }
      else{
        launch_counter_ = 0;  
      }
      RCLCPP_INFO(this->get_logger(), "Counter: %zu", launch_counter_);
    }
  }
}

// Add the callback implementation at the end of the file
void MasterNode::pitch_callback(const std_msgs::msg::Float64::SharedPtr msg) {
  ++pitch_counter_;
  current_pitch_ = msg->data;
  DataPacket packet;
  packet.type = DataPacket::Type::PitchAngle;
  packet.data.w = current_pitch_;
  server_.sendDataToClient(packet);
}

// p: pitch, y: yaw
void MasterNode::send_servo_command(double p, double y){
  if(state_ == State::Stopped) return;

  // // Limit motion if limit switch is pressed (CHECK MIN AND MAX)
  // if(switch_.front) p = std::min(p, 0.0);
  // if(switch_.back) p = std::max(p, 0.0);
  // if(switch_.left) y = std::min(y, 0.0);
  // if(switch_.right) y = std::max(y, 0.0);

  auto msg = geometry_msgs::msg::Vector3();
  msg.x = p;
  msg.y = y;
  servo_command_pub_->publish(msg);
}

void MasterNode::send_launch_command(bool launch){
  if(state_ == State::Stopped) return;

  static constexpr double push = 170.0;
  static constexpr double retract = 80.0;

  auto msg = std_msgs::msg::Float64();
  msg.data = launch ? push : retract;
  launch_command_pub_->publish(msg);
}

void MasterNode::send_esc_command(double speed){
  if(state_ == State::Stopped) return;

  auto msg = std_msgs::msg::Float64();
  msg.data = speed;
  esc_command_pub_->publish(msg);
  current_speed_ = speed;
}


// // Subscribe to the "pid_gains" topic with a queue size of 10.
// pid_gains_yaw_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
//   "pid_gains_yaw", 10,
//   std::bind(&MasterNode::set_yaw_gains_callback, this, _1)
// );

// // Limit switch node subscriptions - unused
// switch_pressed_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
//   "limit_switch_pressed", 10,
//   std::bind(&MasterNode::switch_pressed_callback, this, _1)
// );
// switch_released_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
//   "limit_switch_released", 10,
//   std::bind(&MasterNode::switch_released_callback, this, _1)
// );

// // Callback that updates the pid gains when a new message is received on "pid_gains".
// void MasterNode::set_pitch_gains_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
//   double kp = msg->x;  // Proportional gain
//   double ki = msg->y;  // Integral gain
//   double kd = msg->z;  // Derivative gain

//   pid_pitch_.setGains(kp, ki, kd);
//   RCLCPP_INFO(this->get_logger(), "Received pitch PID gains - Kp: %f, Ki: %f, Kd: %f", kp, ki, kd);
// }

// // Callback that updates the pid gains when a new message is received on "pid_gains".
// void MasterNode::set_yaw_gains_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
//   double kp = msg->x;  // Proportional gain
//   double ki = msg->y;  // Integral gain
//   double kd = msg->z;  // Derivative gain

//   pid_yaw_.setGains(kp, ki, kd);
//   RCLCPP_INFO(this->get_logger(), "Received yaw PID gains - Kp: %f, Ki: %f, Kd: %f", kp, ki, kd);
// }

// // Subscribe to the "pid_gains" topic with a queue size of 10.
// pid_gains_pitch_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
//   "pid_gains_pitch", 10,
//   std::bind(&MasterNode::set_pitch_gains_callback, this, _1)
// );

// // Callback invoked upon limit switch trigger
// void MasterNode::switch_pressed_callback(const std_msgs::msg::Int32::SharedPtr msg){
//   Switch lswitch = static_cast<Switch>(msg->data);
//   auto nan = std::numeric_limits<float>::quiet_NaN();
//   switch(lswitch){
//     case Switch::Left:
//       send_servo_command(nan, 0.0);
//       switch_.left = true;
//       break;
//     case Switch::Right:
//       send_servo_command(nan, 0.0);
//       switch_.right = true;
//       break;
//     case Switch::Front:
//       send_servo_command(0.0, nan);
//       switch_.front = true;
//       break;
//     case Switch::Back:
//       send_servo_command(0.0, nan);
//       switch_.back = true;
//       break;
//   }
// }

// // Callback invoked upon limit switch release
// void MasterNode::switch_released_callback(const std_msgs::msg::Int32::SharedPtr msg){
//   Switch lswitch = static_cast<Switch>(msg->data);
//   switch(lswitch){
//     case Switch::Left:
//       switch_.left = false;
//       break;
//     case Switch::Right:
//       switch_.right = false;
//       break;
//     case Switch::Front:
//       switch_.front = false;
//       break;
//     case Switch::Back:
//       switch_.back = false;
//       break;
//   }
// }