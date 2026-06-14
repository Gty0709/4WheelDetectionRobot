cmake_minimum_required(VERSION 3.8)
project(detection_pkg)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

install(
  DIRECTORY detection_pkg
  DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
)

install(
  DIRECTORY configs config launch simmodel simresult models
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES yolo26m.pt
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/train_sim_yolo26.py
    scripts/probe_batch_size.py
    scripts/clip_detector_node.py
    scripts/clip_detector_wrapper.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
