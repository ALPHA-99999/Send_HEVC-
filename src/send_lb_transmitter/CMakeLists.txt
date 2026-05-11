cmake_minimum_required(VERSION 3.16)
project(send_lb_transmitter LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

add_executable(transmitter_node
  src/transmitter_node.cpp
)

ament_target_dependencies(transmitter_node
  rclcpp
  sensor_msgs
)

target_link_libraries(transmitter_node Threads::Threads)

install(TARGETS transmitter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
