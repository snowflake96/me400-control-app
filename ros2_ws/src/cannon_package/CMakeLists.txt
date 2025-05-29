cmake_minimum_required(VERSION 3.8)
project(cannon_package)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Option to use pigpio or pigpiod_if2 ===
option(USE_PIGPIOD_IF2 "Use pigpiod_if2 (daemon mode) instead of pigpio (direct mode)" ON)

# === Dependencies ===
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# === pigpio vs pigpiod_if2 ===
if(USE_PIGPIOD_IF2)
  message(STATUS "Using pigpiod_if2 (daemon mode)")
  find_library(PIGPIO_LIB pigpiod_if2)
else()
  message(STATUS "Using pigpio (direct mode)")
  find_library(PIGPIO_LIB pigpio)
endif()

if(NOT PIGPIO_LIB)
  message(FATAL_ERROR "Requested pigpio library not found")
endif()

# === Include directories ===
# include_directories(
#   include
#   /usr/local/include  # Optional — only needed if headers are installed there
# )

# === C++ Executable ===
add_executable(main
  src/main.cpp
  src/motor_node.cpp
  src/sensor_node.cpp
  src/imu.cpp
  src/keyinput.cpp
)

target_include_directories(main PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  /usr/local/include  # only if pigpio.h is here
)

# # Add defines and link libraries depending on mode
# if (USE_PIGPIOD_IF2)
#     target_compile_definitions(main PRIVATE USE_DAEMON)
# endif()

target_link_libraries(main ${PIGPIO_LIB} i2c lgpio gpiod)

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
  scripts/web_node.py
  scripts/yolo_node.py
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
