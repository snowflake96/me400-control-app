#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "cannon_package/master.hpp"
#include "cannon_package/motor_node.hpp"
#include "cannon_package/imu_node.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  // Create both nodes
  auto master_node = std::make_shared<MasterNode>();
  auto motor_node  = std::make_shared<MotorNode>();
  auto imu_node  = std::make_shared<ImuNode>();

  // Use MultiThreadedExecutor for concurrent callbacks
  rclcpp::executors::MultiThreadedExecutor exec( rclcpp::ExecutorOptions(), /*num_threads=*/3);
  exec.add_node(master_node);
  exec.add_node(motor_node);
  exec.add_node(imu_node);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}
