#include "rclcpp/rclcpp.hpp"
#include "cannon_package/motor_node.hpp"
#include "cannon_package/sensor_node.hpp"

int main(int argc, char** argv){

    rclcpp::init(argc, argv);

    // Create the nodes using the factory functions from node1.cpp and node2.cpp
    // auto sensor_node = std::make_shared<SensorNode>();
    // auto keyinput_node = std::make_shared<KeyInputNode>();
    auto master_node = std::make_shared<MotorController>();

    auto executor = rclcpp::executors::SingleThreadedExecutor();
    executor.add_node(master_node);

    RCLCPP_INFO(master_node->get_logger(), "Spinning executor");
    executor.spin();

    rclcpp::shutdown();
    return 0;
}