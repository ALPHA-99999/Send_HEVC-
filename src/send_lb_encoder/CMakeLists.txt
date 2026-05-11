cmake_minimum_required(VERSION 3.16)
project(send_lb_encoder LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(AVCODEC REQUIRED IMPORTED_TARGET libavcodec)
pkg_check_modules(AVUTIL REQUIRED IMPORTED_TARGET libavutil)
pkg_check_modules(SWSCALE REQUIRED IMPORTED_TARGET libswscale)

add_executable(encoder_node
  src/encoder_node.cpp
)

target_include_directories(encoder_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(encoder_node
  ${OpenCV_LIBS}
  PkgConfig::AVCODEC
  PkgConfig::AVUTIL
  PkgConfig::SWSCALE
)

ament_target_dependencies(encoder_node
  rclcpp
  sensor_msgs
  cv_bridge
)

install(TARGETS encoder_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
