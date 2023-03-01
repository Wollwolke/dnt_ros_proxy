cmake_minimum_required(VERSION 3.8)
project(dtn_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)

# Msg generation
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PointArray.msg"
  "srv/FollowWaypoints.srv"
  DEPENDENCIES geometry_msgs # Dependency of PointArray.msg
)

ament_package()


