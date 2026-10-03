cmake_minimum_required(VERSION 3.8)
project(navigation_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(navigation_pkg_lib SHARED
  src_cpp/waypoint_io.cpp
  src_cpp/manhattan_tsp.cpp
  src_cpp/collision_check.cpp
  src_cpp/trajectory_tracker.cpp
  src_cpp/obstacle_detect.cpp
  src_cpp/mission_plot.cpp
  src_cpp/mission_recorder.cpp
  src_cpp/sim_pose_sync.cpp)
target_include_directories(navigation_pkg_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(navigation_pkg_lib
  rclcpp nav_msgs tf2_ros tf2_geometry_msgs gazebo_msgs)
target_link_libraries(navigation_pkg_lib
  yaml-cpp
  tf2_geometry_msgs::tf2_geometry_msgs
  ${OpenCV_LIBS})

macro(add_nav_exec name src)
  add_executable(${name} ${src})
  target_include_directories(${name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(${name} navigation_pkg_lib)
endmacro()

add_nav_exec(send_goal_cpp src_cpp/send_goal_node_node.cpp)
ament_target_dependencies(send_goal_cpp
  rclcpp rclcpp_action nav2_msgs action_msgs)

add_nav_exec(playback_mission_cpp src_cpp/playback_mission_node.cpp)
ament_target_dependencies(playback_mission_cpp
  rclcpp nav_msgs geometry_msgs visualization_msgs tf2_ros)

add_nav_exec(waypoint_patrol_cpp src_cpp/waypoint_patrol_node.cpp)
ament_target_dependencies(waypoint_patrol_cpp
  rclcpp rclcpp_action nav2_msgs nav_msgs geometry_msgs visualization_msgs
  tf2_ros tf2_geometry_msgs action_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_manhattan_tsp_cpp test/test_manhattan_tsp.cpp)
  if(TARGET test_manhattan_tsp_cpp)
    target_link_libraries(test_manhattan_tsp_cpp navigation_pkg_lib)
    ament_target_dependencies(test_manhattan_tsp_cpp rclcpp)
  endif()
  ament_add_gtest(test_collision_check_cpp test/test_collision_check.cpp)
  if(TARGET test_collision_check_cpp)
    target_link_libraries(test_collision_check_cpp navigation_pkg_lib)
    ament_target_dependencies(test_collision_check_cpp rclcpp)
  endif()
endif()

install(
  TARGETS navigation_pkg_lib send_goal_cpp playback_mission_cpp waypoint_patrol_cpp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY include/
  DESTINATION include)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/send_goal.py
    scripts/waypoint_patrol.py
    scripts/playback_mission.py
    scripts/analyze_navigation_run.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(navigation_pkg_lib)
ament_export_dependencies(
  rclcpp rclcpp_action nav_msgs nav2_msgs geometry_msgs visualization_msgs
  tf2_ros tf2_geometry_msgs yaml-cpp OpenCV gazebo_msgs)

ament_package()
