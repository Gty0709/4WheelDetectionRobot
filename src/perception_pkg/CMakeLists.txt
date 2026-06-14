cmake_minimum_required(VERSION 3.8)
project(perception_pkg)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

    install(
  PROGRAMS
    scripts/auto_explore.py
    scripts/patrol_mapper.py
    scripts/odom_to_tf.py
    scripts/slider_teleop.py
    scripts/cmd_vel_guard.py
    scripts/save_map_snapshot.py
    scripts/save_map_trigger.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
