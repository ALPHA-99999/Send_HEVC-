cmake_minimum_required(VERSION 3.16)
project(send_lb_capture_preprocess LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(capture_preprocess_node
  src/capture_preprocess_node.cpp
)

target_include_directories(capture_preprocess_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(capture_preprocess_node ${OpenCV_LIBS} Threads::Threads)

ament_target_dependencies(capture_preprocess_node
  rclcpp
  sensor_msgs
  cv_bridge
)

install(TARGETS capture_preprocess_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
