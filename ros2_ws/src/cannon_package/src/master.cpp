/*
Master node that processes state information and computes motor commands.
Created by: Jisang You on 2021-03-10
*/

#include "cannon_package/master.hpp"
using namespace std::chrono_literals;

MasterNode::MasterNode() : Node("master_node"), mode_{Mode::Manual}, state_{State::Stopped}, server_(12345)
{
  RCLCPP_INFO(this->get_logger(), "Initializing Master Node");

  // Publisher for motor commands
  servo_command_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("servo_command", 10);
  launch_command_pub_ = this->create_publisher<std_msgs::msg::Float64>("launch_command", 10);
  esc_command_pub_ = this->create_publisher<std_msgs::msg::Float64>("esc_command", 10);
  motor_offset_pub_ = this->create_publisher<std_msgs::msg::Float64>("motor_offset", 10);

  using namespace std::placeholders;
  
  // Subscribe to the "bbox" topic with a queue size of 1 to prevent build up of messages while sleeping
  bbox_subscription_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
    "bbox", 1, std::bind(&MasterNode::bbox_callback, this, _1)
  );

  // Subscribe to the roll, pitch angles from IMU
  imu_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
    "imu_topic", 10, std::bind(&MasterNode::imu_callback, this, _1)
  );

  init_parameters_from_yaml();

  auto stop_motors = [this](){
    state_ = State::Running;
    send_launch_command(false); // Retract the launch servo initially
    send_esc_command(0.0); // Stop esc
    send_servo_command(0.0, 0.0); // Stop servos
    state_ = State::Stopped;
  };

  // Stop all motors to start
  stop_motors();
  // Stop motors if all clients disconnect
  server_.setFatalCallback(stop_motors);
  // Launch server in a separate thread. Pass the handler function to process received data packets.
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

void MasterNode::init_parameters_from_yaml(){
    // Declare all parameters
    this->declare_parameter("use_interpolation", false);
    this->declare_parameter("pid_pitch.kp", 1.0);
    this->declare_parameter("pid_pitch.ki", 0.001);
    this->declare_parameter("pid_pitch.integral_limit", 1.0);
    this->declare_parameter("pid_yaw.kp", 1.0);
    this->declare_parameter("pid_yaw.ki", 0.001);
    this->declare_parameter("pid_yaw.integral_limit", 1.0);
    this->declare_parameter("trigger.push", 170.0);
    this->declare_parameter("trigger.retract", 55.0);
    this->declare_parameter("cutoff_frequency", 10.0);
    this->declare_parameter("stop_throttle", 0.0);
    this->declare_parameter("motor_offset", 0.0);
    this->declare_parameter("default_speed", 0.2);
    this->declare_parameter("N", 10);
    this->declare_parameter("EPS", 0.005);
    this->declare_parameter("max_consecutive_nans", 5);

    try {
        // Whether to use cubic interpolation
        USE_INTERPOLATION = this->get_parameter("use_interpolation").as_bool();

        // Load PID parameters
        pid_pitch_.setGains(
            this->get_parameter("pid_pitch.kp").as_double(),
            this->get_parameter("pid_pitch.ki").as_double(),
            0.0
        );
        pid_pitch_.setIntegralLimit(this->get_parameter("pid_pitch.integral_limit").as_double());
        
        pid_yaw_.setGains(
            this->get_parameter("pid_yaw.kp").as_double(),
            this->get_parameter("pid_yaw.ki").as_double(),
            0.0
        );
        pid_yaw_.setIntegralLimit(this->get_parameter("pid_yaw.integral_limit").as_double());

        // Trigger angles
        push_ = this->get_parameter("trigger.push").as_double();
        retract_ = this->get_parameter("trigger.retract").as_double();

        // Set filter cutoff frequency
        double cutoff_freq = this->get_parameter("cutoff_frequency").as_double();
        lpf_x.setCutoffFrequency(cutoff_freq);
        lpf_y.setCutoffFrequency(cutoff_freq);

        // Load other parameters
        stop_throttle_ = this->get_parameter("stop_throttle").as_double();
        motor_offset_ = this->get_parameter("motor_offset").as_double();
        DEFAULT_SPEED_ = this->get_parameter("default_speed").as_double();

        launch_threshold_.N = this->get_parameter("N").as_int(); 
        launch_threshold_.EPS = this->get_parameter("EPS").as_double(); 
        pid_pitch_.setIntegralThreshold(5 * launch_threshold_.EPS.load());
        pid_yaw_.setIntegralThreshold(5 * launch_threshold_.EPS.load());
        max_consecutive_nans_ = this->get_parameter("max_consecutive_nans").as_int();
        
        RCLCPP_INFO(this->get_logger(), "Successfully loaded parameters:");
        RCLCPP_INFO(this->get_logger(), "  use_interpolation: %s", USE_INTERPOLATION ? "true" : "false");
        auto [pitch_kp, pitch_ki] = pid_pitch_.getPIgains();
        auto [yaw_kp, yaw_ki] = pid_yaw_.getPIgains();
        RCLCPP_INFO(this->get_logger(), "  PID pitch - kp: %.3f, ki: %.3f, integral_limit: %.3f", pitch_kp, pitch_ki, pid_pitch_.getIntegralLimit());
        RCLCPP_INFO(this->get_logger(), "  PID yaw - kp: %.3f, ki: %.3f, integral_limit: %.3f", yaw_kp, yaw_ki, pid_yaw_.getIntegralLimit());
        RCLCPP_INFO(this->get_logger(), "  Trigger angles - push: %.1f, retract: %.1f", push_, retract_);
        RCLCPP_INFO(this->get_logger(), "  Cutoff frequency: %.1f Hz", lpf_x.getCutoffFrequency());
        RCLCPP_INFO(this->get_logger(), "  Motor params - stop_throttle: %.2f, offset: %.2f, default_speed: %.2f", stop_throttle_, motor_offset_, DEFAULT_SPEED_);
        RCLCPP_INFO(this->get_logger(), "  Launch threshold - N: %d, EPS: %.3f, max_consecutive_nans: %d", launch_threshold_.N.load(), launch_threshold_.EPS.load(), max_consecutive_nans_);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load parameters: %s", e.what());
    }
}

// Handler function to process received data packets from client
void MasterNode::server_handler(const DataPacket &packet, int client_fd) {
  const auto &v       = packet.data.vector3;
  const auto  w       = packet.data.w;
  const auto  boolean = packet.data.boolean;
  const auto &threshold = packet.data.threshold;

  /* Starting mode is Manual and Stopped */

  std::string log_msg;

  switch (packet.type) {
    case DataPacket::Type::ServoCommand:
      if (mode_ == Mode::Manual) {
        send_servo_command(v.x, v.y);
        log_msg = std::format( "Sent servo command – x: {:.3f}, y: {:.3f}", v.x, v.y);
      } 
      else{
        log_msg = "Switch to manual mode to send servo command";
      }
      break;

    case DataPacket::Type::TriggerCommand:
      if (mode_ == Mode::Manual || mode_ == Mode::AutoAim) {
        send_launch_command(boolean);
        log_msg = std::format( "Sent trigger command: {}", boolean ? "push" : "retract");
      }
      else{
        log_msg = "Switch to manual or autoaim mode to send trigger command";
      }
      break;

    case DataPacket::Type::EscCommand:
      if (mode_ == Mode::Manual || mode_ == Mode::AutoAim) {
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
      if(mode_ != Mode::Autonomous){
        mode_ = Mode::Autonomous;
        reset();
        log_msg = "Set to autonomous mode";
      }
      else{
        log_msg = "Already in autonomous mode";
      }
      break;

    case DataPacket::Type::SetManual:
      if(mode_ != Mode::Manual){
        mode_ = Mode::Manual;
        log_msg = "Set to manual mode";
      }
      else{
        log_msg = "Already in manual mode";
      }
      break;

    case DataPacket::Type::SetAutoAim:
      if(mode_ != Mode::AutoAim){
        mode_ = Mode::AutoAim;
        log_msg = "Set to automatic aim mode";
      }
      else{
        log_msg = "Already in autoaim mode";
      }
      break;

    case DataPacket::Type::SetOffset:
      if(mode_ == Mode::Manual || mode_ == Mode::AutoAim){
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
      launch_threshold_.EPS = std::abs(threshold.EPS); 
      launch_threshold_.N = threshold.N;
      pid_pitch_.setIntegralThreshold(5 * launch_threshold_.EPS.load());
      pid_yaw_.setIntegralThreshold(5 * launch_threshold_.EPS.load());
      log_msg = std::format("Set launch threshold to EPS: {:.3f}, N: {}", launch_threshold_.EPS.load(), launch_threshold_.N.load());
      break;
    
    case DataPacket::Type::Query:
      log_msg = handle_client_query(client_fd);
      break;

    case DataPacket::Type::MotorOffset: {
      auto message = std_msgs::msg::Float64();
      message.data = w;
      motor_offset_pub_->publish(message);
      motor_offset_ = w;
      log_msg = std::format("Set motor offset to: {:.3f}", w);
      break;
    }

    case DataPacket::Type::SetCutoffFrequency:
      // Set the cutoff frequency for both x and y bbox positions
      lpf_x.setCutoffFrequency(w);
      lpf_y.setCutoffFrequency(w);
      log_msg = std::format("Set LPF cutoff frequency to: {:.3f}Hz", lpf_x.getCutoffFrequency());
      break;
    
    case DataPacket::Type::SetStopThrottle:
      stop_throttle_ = std::clamp(w, -0.1, 0.1);
      send_esc_command(current_speed_); // Update the throttle
      log_msg = std::format("Set stop throttle to: {:.3f}", stop_throttle_);
      break;
    
    case DataPacket::Type::SetMaxConsecutiveNans:
      max_consecutive_nans_ = packet.data.uint32;
      log_msg = std::format("Set max consecutive nans to: {}", max_consecutive_nans_);
      break;
    
    case DataPacket::Type::SetDefaultSpeed:
      DEFAULT_SPEED_ = std::clamp(w, 0.0, 0.3);
      log_msg = std::format("Set default speed to: {:.3f}", DEFAULT_SPEED_);
      break;

    default:
      log_msg = "Unknown packet type received";
  }
  
  RCLCPP_INFO(this->get_logger(), "%s", log_msg.c_str());
  server_.sendLogToClient(log_msg, client_fd);
}

// Helper function to handle query from client and return logging message
std::string MasterNode::handle_client_query(int client_fd){
  DataPacket packet{};
  packet.type = DataPacket::Type::CurrentState;
  auto& state_info = packet.data.state;

  const char* mode = nullptr;
  switch(mode_){
    case Mode::Autonomous:
      state_info.mode = DataPacket::Type::SetAutonomous;
      mode = "autonomous";
      break;
    case Mode::Manual:
      state_info.mode = DataPacket::Type::SetManual;
      mode = "manual";
      break;
    case Mode::AutoAim:
      state_info.mode = DataPacket::Type::SetAutoAim;
      mode = "autoaim";
      break;
  }

  const char* state = nullptr;
  switch(state_){
    case State::Running:
      state_info.motor_state = DataPacket::Type::Start;
      state = "running";
      break;
    case State::Stopped:
      state_info.motor_state = DataPacket::Type::Stop;
      state = "stopped";
      break;
  }

  // Other state variables
  state_info.launch_counter = launch_counter_;
  state_info.N = launch_threshold_.N;
  state_info.EPS = launch_threshold_.EPS;
  state_info.target_x = target_.x;
  state_info.target_y = target_.y;
  state_info.stop_throttle = stop_throttle_;
  state_info.motor_offset = motor_offset_;
  state_info.default_speed = DEFAULT_SPEED_;
  state_info.cutoff_freq = lpf_x.getCutoffFrequency(); // lpf_x and lpf_y use the same cutoff frequency

  server_.sendDataToClient(packet, client_fd);
  return std::format("Mode: {}  -  State: {}", mode, state);
}

// Function to reset all parameters, control, and motor speed to default (keep mode and state same)
void MasterNode::reset(){
  // Reset params to default
  launch_counter_ = 0;
  found_offset_ = false;
  // Reset control to default
  pid_pitch_.reset();
  pid_yaw_.reset();
  lpf_x.reset();
  lpf_y.reset();
  // Reset motor speed to default if running
  if(state_ == State::Running && mode_ == Mode::Autonomous){
    if(current_speed_ < DEFAULT_SPEED_){
      accelerate_to_default_speed();
    }
    else{
      send_esc_command(DEFAULT_SPEED_);
    }
  }
}

void MasterNode::accelerate_to_default_speed(){
  // Accelerate at 0.5 throttle for 200ms before setting to default speed
  send_esc_command(0.5); 
  timer_ = this->create_wall_timer(200ms, [this]() { timer_->cancel(); send_esc_command(DEFAULT_SPEED_); });
}

// Function to compute and set the offset from the pitch angle to the bell
void MasterNode::set_offset_from_pitch(const double pitch){

  target_.x = 0.0;
  target_.y = spline_.interpolate(pitch);

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
  static size_t nan_counter=0;

  // Normalize and convert to cartersian coordinates
  double x1 = 2.0 * msg->x - 1.0;  // x1 normalized to [-1,1]
  double y1 = -(2.0 * msg->y - 1.0);  // y1 normalized to [-1,1]
  double x2 = 2.0 * msg->z - 1.0;  // x2 normalized to [-1,1]
  double y2 = -(2.0 * msg->w - 1.0);  // y2 normalized to [-1,1]

  // Forward the bbox corners to client (top-left and bottom-right)
  DataPacket packet{};
  packet.type = DataPacket::Type::BboxPos;
  packet.data.double_array[0] = x1;
  packet.data.double_array[1] = y1;
  packet.data.double_array[2] = x2;
  packet.data.double_array[3] = y2;
  server_.sendDataToClient(packet);
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Bbox: x1: %.3f, y1: %.3f, x2: %.3f, y2: %.3f", x1, y1, x2, y2);

  // Return here if in manual control mode or stopped
  if(mode_ == Mode::Manual || state_ == State::Stopped) return; 

  // If no bounding box is detected for max_consecutive_nans_ times, stop servo
  if(std::isnan(x1)){
    if(++nan_counter == max_consecutive_nans_){
      send_servo_command(0.0, 0.0); // Stop the servos
      reset(); // This resets both the launch counter and the found_offset flag
      std::string log_msg = std::format("No detection for {} consecutive times. Stopping servos.", max_consecutive_nans_);
      RCLCPP_INFO(this->get_logger(), log_msg.c_str());
      server_.sendLogToClient(log_msg);
    }
    return;
  }
  else if(nan_counter > 0){
    nan_counter = 0;
    const char* log_msg = "New bbox detection. Re-enabling servos.";
    RCLCPP_INFO(this->get_logger(), log_msg);
    server_.sendLogToClient(log_msg);
  }

  // Calculate center coordinates from bounding box
  double center_x = (x1 + x2) * 0.5;  // Average of left and right
  double center_y = (y1 + y2) * 0.5;  // Average of top and bottom

  // Pass to LPF
  center_x = lpf_x.filter(center_x);
  center_y = lpf_y.filter(center_y);

  // Send filtered bbox center to client
  packet.type = DataPacket::Type::FilteredBbox;
  packet.data.vector3.x = center_x;
  packet.data.vector3.y = center_y;
  server_.sendDataToClient(packet);
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Filtered BBOX - x: %.3f  y: %.3f", center_x, center_y);

  // Compute and send control signal to servos
  double control_signal_pitch = pid_pitch_.compute(center_y, target_.y);
  double control_signal_yaw = pid_yaw_.compute(center_x, target_.x);
  send_servo_command(control_signal_pitch, control_signal_yaw);

  double psignal, isignal;
  psignal = pid_pitch_.getLastPsignal(); isignal = pid_pitch_.getLastIsignal();
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Pitch PID - P: %.3f  I: %.3f  Total output: %.3f", psignal, isignal, control_signal_pitch);
  psignal = pid_yaw_.getLastPsignal(); isignal = pid_yaw_.getLastIsignal();
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Yaw PID - P: %.3f  I: %.3f  Total output: %.3f", psignal, isignal, control_signal_yaw);

  if(mode_ == Mode::Autonomous){
    double p_error = pid_pitch_.getLastError();
    double y_error = pid_yaw_.getLastError();
    if(USE_INTERPOLATION && !found_offset_){
      static constexpr double deg_to_rad = M_PI / 180.0;
      static constexpr double rad_to_deg = 180.0 / M_PI;
      static constexpr double FOV_Y = 41.0; // vertical FOV in degrees
      double bell_angle = std::atan(center_y * std::tan(0.5 * FOV_Y * deg_to_rad)) * rad_to_deg; // Bell angle in camera frame
      double pitch_to_bell = tilt_.pitch + bell_angle;
      set_offset_from_pitch(pitch_to_bell);
      found_offset_ = true;
    }
    // Execute launch command
    else if(std::abs(p_error) < launch_threshold_.EPS && std::abs(y_error) < launch_threshold_.EPS){
      if(++launch_counter_ > launch_threshold_.N){
        const char* log = "Launching cannon!";
        RCLCPP_INFO(this->get_logger(), log);
        server_.sendLogToClient(log);
        // Launch
        send_launch_command(true); // Push
        // Retract and accelerate after 500ms
        timer_ = this->create_wall_timer(400ms, [this]() { 
          timer_->cancel(); 
          send_launch_command(false);
          accelerate_to_default_speed();
        }); 
        launch_counter_ = 0;
      }
    }
    // Bounding box outside of margin from target so reset counter
    else{
      launch_counter_ = 0;  
    }

    // Send the launch counter to the client
    packet.type = DataPacket::Type::LaunchCounter;
    packet.data.uint32 = launch_counter_;
    server_.sendDataToClient(packet);
    RCLCPP_INFO_STREAM(this->get_logger(), "Launch counter: " << launch_counter_);
  }
}

// Add the callback implementation at the end of the file
void MasterNode::imu_callback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
  tilt_.roll = msg->x;
  tilt_.pitch = msg->y;

  DataPacket packet;
  packet.type = DataPacket::Type::Tilt;
  packet.data.vector3.x = tilt_.roll;
  packet.data.vector3.y = tilt_.pitch;
  server_.sendDataToClient(packet);
}

void MasterNode::send_servo_command(double p, double y){
  if(state_ == State::Stopped) return;

  auto msg = geometry_msgs::msg::Vector3();
  msg.x = -p; // Invert pitch
  msg.y = -y; // Invert yaw
  servo_command_pub_->publish(msg);
}

void MasterNode::send_launch_command(bool launch){
  if(state_ == State::Stopped) return;

  auto msg = std_msgs::msg::Float64();
  msg.data = launch ? push_ : retract_;
  launch_command_pub_->publish(msg);
}

void MasterNode::send_esc_command(double speed){
  if(state_ == State::Stopped) return;

  auto msg = std_msgs::msg::Float64();
  msg.data = stop_throttle_ + speed;
  esc_command_pub_->publish(msg);
  current_speed_ = speed;
}