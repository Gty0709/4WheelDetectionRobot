cmake_minimum_required(VERSION 3.8)
project(navigation_pkg)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

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

ament_package()
