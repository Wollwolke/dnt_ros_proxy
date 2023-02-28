cmake_minimum_required(VERSION 3.8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(dtn_proxy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

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

file(GLOB SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/ros/*.cpp)

add_executable(dtnproxy ${SOURCES})
ament_target_dependencies(dtnproxy rclcpp rosbag2_cpp rosidl_typesupport_introspection_cpp rosidl_typesupport_cpp example_interfaces)

target_include_directories(dtnproxy PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/external/cpp-httplib
  ${CMAKE_CURRENT_SOURCE_DIR}/external/toml11
  ${CMAKE_CURRENT_SOURCE_DIR}/external/websocketpp)

# nlohmann json
set(JSON_Install OFF CACHE INTERNAL "")
set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/json)
target_link_libraries(dtnproxy nlohmann_json::nlohmann_json)

# ### Generate C bindings for bp7-rs
# set(CARGO_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR}/bp7)
# target_include_directories(dtnproxy PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/generated/include/)

# include(ExternalProject)

# ExternalProject_Add(
# bp7
# DOWNLOAD_COMMAND ""
# CONFIGURE_COMMAND ""
# BUILD_COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${CARGO_TARGET_DIR} cargo build
# COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${CARGO_TARGET_DIR} cargo build --release
# COMMAND cbindgen -c ../../cbindgen.toml -o ${CMAKE_CURRENT_BINARY_DIR}/generated/include/bp7.hpp
# BINARY_DIR "${PROJECT_SOURCE_DIR}/external/bp7-rs"
# INSTALL_COMMAND ""
# LOG_BUILD ON)

# add_dependencies(dtnproxy bp7)

# target_link_libraries(
# dtnproxy
# debug "${CARGO_TARGET_DIR}/debug/libbp7.a"
# optimized "${CARGO_TARGET_DIR}/release/libbp7.a"
# )
# ### End Generate C bindings for bp7-rs
install(TARGETS
  dtnproxy
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
