cmake_minimum_required(VERSION 3.8)
project(cannon_package)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-Wno-psabi)

# === Dependencies ===
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# === C++ Executable ===
add_executable(main
  src/main.cpp
  src/master.cpp
  src/motor_node.cpp
  src/imu_node.cpp
)

target_include_directories(main PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(main
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# === Install C++ Executable ===
install(TARGETS main
  DESTINATION lib/${PROJECT_NAME}
)

# === Install Python Modules ===
ament_python_install_package(${PROJECT_NAME})

# === Install Python Modules ===
install(PROGRAMS
  scripts/yolo_node.py
  # scripts/mouse.py
  DESTINATION lib/${PROJECT_NAME}
)

# === Install Launch Directory ===
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# === Install Templates/Static (optional for web assets) ===
install(DIRECTORY
  resource/templates
  resource/static
  resource/model
  DESTINATION share/${PROJECT_NAME}
)

# === Testing ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
