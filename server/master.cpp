/*
Master node that processes state information and computes motor commands.
Created by: Jisang You on 2021-03-10
*/

#include "cannon_package/master.hpp"
using namespace std::chrono_literals;

MasterNode::MasterNode() : Node("master_node")
{
  RCLCPP_INFO(this->get_logger(), "Initializing Master Node...");

  // Publisher for motor commands
  servo_command_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("servo_command", 10);
  launch_command_pub_ = this->create_publisher<std_msgs::msg::Float64>("launch_command", 10);
  esc_command_pub_ = this->create_publisher<std_msgs::msg::Float64>("esc_command", 10);

  // Limit switch node subscriptions
  switch_pressed_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "limit_switch_pressed", 10,
    std::bind(&MasterNode::switch_pressed_callback, this, std::placeholders::_1)
  );
  switch_released_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    "limit_switch_released", 10,
    std::bind(&MasterNode::switch_released_callback, this, std::placeholders::_1)
  );

  // Subscribe to the "bbox" topic with a queue size of 10.
  bbox_subscription_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
    "bbox", 10,
    std::bind(&MasterNode::bbox_callback, this, std::placeholders::_1)
  ); // mouse_position or bbox //

  // Subscribe to the pitch angle from IMU
  pitch_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
    "pitch_topic", 10,
    std::bind(&MasterNode::pitch_callback, this, std::placeholders::_1)
  );

  // Subscribe to the "pid_gains" topic with a queue size of 10.
  pid_gains_pitch_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    "pid_gains_pitch", 10,
    std::bind(&MasterNode::set_pitch_gains_callback, this, std::placeholders::_1)
  );

  // Subscribe to the "pid_gains" topic with a queue size of 10.
  pid_gains_yaw_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    "pid_gains_yaw", 10,
    std::bind(&MasterNode::set_yaw_gains_callback, this, std::placeholders::_1)
  );

  // Launch server in a separate thread. Pass the handler function to process received data packets.
  RCLCPP_INFO(this->get_logger(), "Starting server thread...");
  auto handler = std::bind(&MasterNode::server_handler, this, std::placeholders::_1);
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

// Callback invoked upon limit switch trigger
void MasterNode::switch_pressed_callback(const std_msgs::msg::Int32::SharedPtr msg){
  Switch lswitch = static_cast<Switch>(msg->data);
  auto nan = std::numeric_limits<float>::quiet_NaN();
  switch(lswitch){
    case Switch::Left:
      send_servo_command(nan, 0.0);
      switch_.left = true;
      break;
    case Switch::Right:
      send_servo_command(nan, 0.0);
      switch_.right = true;
      break;
    case Switch::Front:
      send_servo_command(0.0, nan);
      switch_.front = true;
      break;
    case Switch::Back:
      send_servo_command(0.0, nan);
      switch_.back = true;
      break;
  }
}

// Callback invoked upon limit switch release
void MasterNode::switch_released_callback(const std_msgs::msg::Int32::SharedPtr msg){
  Switch lswitch = static_cast<Switch>(msg->data);
  switch(lswitch){
    case Switch::Left:
      switch_.left = false;
      break;
    case Switch::Right:
      switch_.right = false;
      break;
    case Switch::Front:
      switch_.front = false;
      break;
    case Switch::Back:
      switch_.back = false;
      break;
  }
}

// Handler function to process received data packets from client
void MasterNode::server_handler(DataPacket packet){
  const char* log = nullptr;
  const auto& v = packet.data.vector3;
  const auto& w = packet.data.w;
  const auto& boolean = packet.data.boolean;
  
  /* Starting with AutoAim and Stopped */

  switch (packet.type) {
      case DataPacket::Type::ServoCommand:
          if(mode_ == DrivingMode::Manual){
            send_servo_command(v.x, v.y);
            RCLCPP_INFO(this->get_logger(), "Sent servo command - x: %f, y: %f, z: %f", v.x, v.y, v.z);
          }
          else{
            log = "Switch to manual mode to send servo command";
          }
          break;
      case DataPacket::Type::TriggerCommand:
          if(mode_ == DrivingMode::Manual || mode_ == DrivingMode::AutoAim){
            send_launch_command(packet.data.boolean);
            RCLCPP_INFO(this->get_logger(), "Sent trigger comand: %s", boolean ? "up" : "down");
          }
          else{
            log = "Switch to manual or autoaim mode to send trigger command";
          }
          break;
      case DataPacket::Type::EscCommand: // WORKS IN ANY MODE (including autonomous mode)
          send_esc_command(w);
          RCLCPP_INFO(this->get_logger(), "Sent ESC command - x: %f", w);
          break;
      case DataPacket::Type::TunePitch:
          pid_pitch_.setGains(v.x, v.y, v.z); // x: Kp, y: Ki, z: Kd
          RCLCPP_INFO(this->get_logger(), "Received pitch PID gains - Kp: %f, Ki: %f, Kd: %f", v.x, v.y, v.z);
          break;
      case DataPacket::Type::TuneYaw:
          pid_yaw_.setGains(v.x, v.y, v.z); // x: Kp, y: Ki, z: Kd
          RCLCPP_INFO(this->get_logger(), "Received yaw PID gains - Kp: %f, Ki: %f, Kd: %f", v.x, v.y, v.z); 
          break;
      case DataPacket::Type::Start:
          if(state_ == State::Stopped){
            state_ = State::Running;
            send_esc_command(DEFAULT_SPEED);
            log = "Started motors";
          }
          break;
      case DataPacket::Type::Stop:
          send_servo_command(0.0, 0.0);
          send_esc_command(0.0);
          state_ = State::Stopped;
          log = "Stopped motors";
          break;
      case DataPacket::Type::SetAutonomous:
          mode_ = DrivingMode::Autonomous;
          send_esc_command(DEFAULT_SPEED);
          log = "Set to autonomous mode";
          break;
      case DataPacket::Type::SetManual:
          mode_ = DrivingMode::Manual;
          log = "Set to manual mode";
          break;
      case DataPacket::Type::SetAutoAim:
          mode_ = DrivingMode::AutoAim;
          log = "Set to automatic aim mode";
          break;
      case DataPacket::Type::SetOffset:
          target_.x = v.x;
          target_.y = v.y;
          RCLCPP_INFO(this->get_logger(), "Received offset - x: %f, y: %f", v.x, v.y); 
          break;
      case DataPacket::Type::SetPitchIntegralLimit:
          pid_pitch_.setIntegralLimit(w);
          RCLCPP_INFO(this->get_logger(), "Setting pitch integral limit to: %f", w); 
          break;
      case DataPacket::Type::SetYawIntegralLimit:
          pid_yaw_.setIntegralLimit(w);
          RCLCPP_INFO(this->get_logger(), "Setting yaw integral limit to: %f", w); 
          break;
      default:
          log = "Unknown packet type received";
  }
  
  if(log){
    RCLCPP_INFO(this->get_logger(), "%s", log);
  }
}

// Callback invoked upon receiving target values from the "bbox".
void MasterNode::bbox_callback(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
  static constexpr double EPS = 0.01; // Tune this
  static constexpr size_t N = 10; // Tune this
  static size_t launch_counter=0;
  static size_t nancount=0;

  // NaN propagates
  double x1 = 2.0 * msg->x - 1.0;  // x1 normalized to [-1,1]
  double y1 = 2.0 * msg->y - 1.0;  // y1 normalized to [-1,1]
  double x2 = 2.0 * msg->z - 1.0;  // x2 normalized to [-1,1]
  double y2 = 2.0 * msg->w - 1.0;  // y2 normalized to [-1,1]

  // Forward the bbox corners to client (top-left and bottom-right)
  DataPacket packet;
  packet.type = DataPacket::Type::BboxPos;
  packet.data.double_array[0] = x1;
  packet.data.double_array[1] = y1;
  packet.data.double_array[2] = x2;
  packet.data.double_array[3] = y2;
  RCLCPP_INFO(this->get_logger(), "%f, %f, %f, %f", x1, y1, x2, y2);
  server_.sendDataToClients(packet);
  
  // Return here if in manual control mode or stopped
  if(mode_ == DrivingMode::Manual || state_ == State::Stopped){
    launch_counter = 0;
    nancount = 0;
    return;
  }

  // AutoAim and Autonomous Modes Proceed Onward

  // If no bounding box is detected for 3 consecutive times, stop servo
  if(std::isnan(x1)){
    if(++nancount > 3){
      send_servo_command(0.0, 0.0); // If 3 or more consecutive inferences did not detect a bell, then turn off servo
    }
    return;
  }
  else{
    nancount = 0;
  }

  // Calculate center coordinates from bounding box
  double center_x = (x1 + x2) / 2.0;  // Average of left and right
  double center_y = (y1 + y2) / 2.0;  // Average of top and bottom

  // Compute and send control signal to servos
  double control_signal_pitch = pid_pitch_.compute(target_.y, center_y);
  double control_signal_yaw = pid_yaw_.compute(target_.x, center_x);
  send_servo_command(control_signal_pitch, control_signal_yaw); // Sign change moved to servo driver
  
  // This part is skipped in AutoAim mode
  if(mode_ == DrivingMode::Autonomous && pid_pitch_.getLastError() < EPS && pid_yaw_.getLastError() < EPS){
    RCLCPP_INFO(this->get_logger(), "Counter: %zu", launch_counter);
    // If the error is less than EPS for N consecutive times, launch
    if(++launch_counter > N){
      RCLCPP_INFO(this->get_logger(), "----------  LAUNCHING CANNON !  ------------");
      send_launch_command(true); // Push
      rclcpp::sleep_for(2s); // Wait for the servo to move to the new position
      send_launch_command(false); // Retract
      launch_counter = 0;
    }
  }
  else{
    launch_counter = 0;
  }
}

// Add the callback implementation at the end of the file
void MasterNode::pitch_callback(const std_msgs::msg::Float64::SharedPtr msg) {
  // Store the current pitch angle
  current_pitch_ = msg->data;
  // Forward the bbox corners to client (top-left and bottom-right)
  DataPacket packet;
  packet.type = DataPacket::Type::PitchAngle;
  packet.data.w = current_pitch_;
  server_.sendDataToClients(packet);
  // Log the pitch angle every 0.5 seconds
  // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Current pitch angle: %.2f degrees", current_pitch_);
}

// p: pitch, y: yaw
void MasterNode::send_servo_command(double p, double y){
  if(state_ == State::Stopped) return;

  // Limit motion if limit switch is pressed (CHECK MIN AND MAX)
  if(switch_.front) p = std::min(p, 0.0);
  if(switch_.back) p = std::max(p, 0.0);
  if(switch_.left) y = std::min(y, 0.0);
  if(switch_.right) y = std::max(y, 0.0);

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
}

// Callback that updates the pid gains when a new message is received on "pid_gains".
void MasterNode::set_pitch_gains_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
  double kp = msg->x;  // Proportional gain
  double ki = msg->y;  // Integral gain
  double kd = msg->z;  // Derivative gain

  pid_pitch_.setGains(kp, ki, kd);
  RCLCPP_INFO(this->get_logger(), "Received pitch PID gains - Kp: %f, Ki: %f, Kd: %f", kp, ki, kd);
}

// Callback that updates the pid gains when a new message is received on "pid_gains".
void MasterNode::set_yaw_gains_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
  double kp = msg->x;  // Proportional gain
  double ki = msg->y;  // Integral gain
  double kd = msg->z;  // Derivative gain

  pid_yaw_.setGains(kp, ki, kd);
  RCLCPP_INFO(this->get_logger(), "Received yaw PID gains - Kp: %f, Ki: %f, Kd: %f", kp, ki, kd);
}