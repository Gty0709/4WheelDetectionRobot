cmake_minimum_required(VERSION 3.8)
project(detection_pkg)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(USE_TENSORRT "Build with NVIDIA TensorRT inference" OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(DETECTION_PKG_LIB_SOURCES
  src_cpp/backprojection.cpp
  src_cpp/motion_compensation.cpp
  src_cpp/waypoint_track.cpp
  src_cpp/waypoint_reid.cpp
  src_cpp/embedding_reid.cpp
  src_cpp/map_overlay.cpp
  src_cpp/trt_engine_builder.cpp
  src_cpp/trt_inference.cpp
  src_cpp/trt_osnet.cpp
)

add_library(detection_pkg_lib SHARED ${DETECTION_PKG_LIB_SOURCES})
target_include_directories(detection_pkg_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(detection_pkg_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  tf2_ros
  message_filters
  ament_index_cpp)
target_link_libraries(detection_pkg_lib
  yaml-cpp
  ${OpenCV_LIBS})

if(USE_TENSORRT)
  find_package(CUDAToolkit REQUIRED)
  find_path(TENSORRT_INCLUDE_DIR NvInfer.h
    HINTS
      /usr/local/cuda/include
      /usr/local/cuda-12.6/targets/x86_64-linux/include
      /usr/include/x86_64-linux-gnu
  )
  find_library(NVINFER_LIB nvinfer
    HINTS /usr/local/cuda/lib64 /usr/lib/x86_64-linux-gnu)
  find_library(NVONNXPARSER_LIB nvonnxparser
    HINTS /usr/local/cuda/lib64 /usr/lib/x86_64-linux-gnu)

  if(NOT TENSORRT_INCLUDE_DIR OR NOT NVINFER_LIB OR NOT NVONNXPARSER_LIB)
    message(FATAL_ERROR "USE_TENSORRT=ON but TensorRT libraries/headers not found")
  endif()

  target_compile_definitions(detection_pkg_lib PUBLIC DETECTION_PKG_USE_TENSORRT)
  target_include_directories(detection_pkg_lib PUBLIC ${TENSORRT_INCLUDE_DIR})
  target_link_libraries(detection_pkg_lib
    CUDA::cudart
    ${NVINFER_LIB}
    ${NVONNXPARSER_LIB})
endif()

add_executable(clip_detector_cpp src_cpp/clip_detector_node.cpp)
target_include_directories(clip_detector_cpp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(clip_detector_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  tf2_ros
  message_filters
  ament_index_cpp)
target_link_libraries(clip_detector_cpp detection_pkg_lib)

if(USE_TENSORRT)
  add_executable(build_trt_engines src_cpp/build_trt_engines.cpp)
  target_include_directories(build_trt_engines PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_link_libraries(build_trt_engines detection_pkg_lib)
endif()

set(_DETECTION_INSTALL_TARGETS detection_pkg_lib clip_detector_cpp)
if(USE_TENSORRT)
  list(APPEND _DETECTION_INSTALL_TARGETS build_trt_engines)
endif()
install(
  TARGETS ${_DETECTION_INSTALL_TARGETS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_python_install_package(${PROJECT_NAME})

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

ament_export_include_directories(include)
ament_export_libraries(detection_pkg_lib)
ament_export_dependencies(
  rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs visualization_msgs
  tf2_ros message_filters ament_index_cpp yaml-cpp OpenCV)

ament_package()
