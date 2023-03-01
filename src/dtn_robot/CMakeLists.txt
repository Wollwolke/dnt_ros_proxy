cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(dtn_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(dtn_robot_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Control Interface
add_executable(control_interface src/control_interface.cpp)
target_include_directories(control_interface PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(control_interface
  "dtn_robot_interfaces"
  "geometry_msgs"
  "nav2_msgs"
  "rclcpp"
  "rclcpp_action")

# DHT fake sensor
add_executable(dht_fake src/dht_fake_sensor.cpp)
target_include_directories(dht_fake PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(dht_fake
  "rclcpp"
  "sensor_msgs")

# Status Publisher
add_executable(status_pub src/status_publisher.cpp)
target_include_directories(status_pub PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(status_pub
  "rclcpp"
  "diagnostic_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "sensor_msgs")

install(TARGETS
  status_pub
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  dht_fake
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  control_interface
  DESTINATION lib/${PROJECT_NAME})

# Install files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
