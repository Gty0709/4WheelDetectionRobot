cmake_minimum_required(VERSION 3.8)
project(perception_pkg)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── C++ dependencies ──
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

# ── C++ executables ──
# odom_to_tf
add_executable(odom_to_tf_cpp src_cpp/odom_to_tf_node.cpp)
target_include_directories(odom_to_tf_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(odom_to_tf_cpp
  rclcpp nav_msgs tf2_ros)

# cmd_vel_guard
add_executable(cmd_vel_guard_cpp src_cpp/cmd_vel_guard_node.cpp)
target_include_directories(cmd_vel_guard_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cmd_vel_guard_cpp
  rclcpp geometry_msgs)

# save_map_snapshot
add_executable(save_map_snapshot_cpp src_cpp/save_map_snapshot_node.cpp)
target_include_directories(save_map_snapshot_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(save_map_snapshot_cpp
  rclcpp nav_msgs geometry_msgs std_msgs std_srvs tf2_ros)
target_link_libraries(save_map_snapshot_cpp yaml-cpp)

# auto_explore
add_executable(auto_explore_cpp src_cpp/auto_explore_node.cpp)
target_include_directories(auto_explore_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(auto_explore_cpp
  rclcpp geometry_msgs sensor_msgs)

# patrol_mapper
add_executable(patrol_mapper_cpp src_cpp/patrol_mapper_node.cpp)
target_include_directories(patrol_mapper_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(patrol_mapper_cpp
  rclcpp nav_msgs geometry_msgs std_srvs)

# save_map_trigger
add_executable(save_map_trigger_cpp src_cpp/save_map_trigger_node.cpp)
target_include_directories(save_map_trigger_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(save_map_trigger_cpp
  rclcpp std_srvs)

# slider_teleop (terminal-based)
add_executable(slider_teleop_cpp src_cpp/slider_teleop_node.cpp)
target_include_directories(slider_teleop_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(slider_teleop_cpp
  rclcpp geometry_msgs std_msgs std_srvs)
target_link_libraries(slider_teleop_cpp ncurses)

# ── Install C++ executables ──
install(TARGETS
  odom_to_tf_cpp
  cmd_vel_guard_cpp
  save_map_snapshot_cpp
  auto_explore_cpp
  patrol_mapper_cpp
  slider_teleop_cpp
  save_map_trigger_cpp
  DESTINATION lib/${PROJECT_NAME})

# ── Install C++ headers ──
install(DIRECTORY include/
  DESTINATION include)

# ── Install Python scripts (unchanged) ──
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS
    scripts/auto_explore.py
    scripts/patrol_mapper.py
    scripts/odom_to_tf.py
    scripts/slider_teleop.py
    scripts/cmd_vel_guard.py
    scripts/save_map_snapshot.py
    scripts/save_map_trigger.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
