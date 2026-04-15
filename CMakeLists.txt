cmake_minimum_required(VERSION 3.15)
project(CameraStreamer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(AVCODEC REQUIRED IMPORTED_TARGET libavcodec)
pkg_check_modules(AVUTIL REQUIRED IMPORTED_TARGET libavutil)
pkg_check_modules(SWSCALE REQUIRED IMPORTED_TARGET libswscale)

add_executable(CameraStreamer
    main.cpp
    CameraStreamer.cpp
)

add_executable(LowBandwidthStreamerApp
    low_bandwidth_main.cpp
    LowBandwidthStreamer.cpp
)

add_executable(BufferedLowBandwidthStreamerApp
    buffered_low_bandwidth_main.cpp
    BufferedLowBandwidthStreamer.cpp
)

add_executable(mytest TEST.cpp)

foreach(target CameraStreamer LowBandwidthStreamerApp BufferedLowBandwidthStreamerApp mytest)
    target_include_directories(${target} PRIVATE ${OpenCV_INCLUDE_DIRS})
endforeach()

target_link_libraries(CameraStreamer PRIVATE
    ${OpenCV_LIBS}
    PkgConfig::AVCODEC
    PkgConfig::AVUTIL
    PkgConfig::SWSCALE
)

target_link_libraries(LowBandwidthStreamerApp PRIVATE
    ${OpenCV_LIBS}
    PkgConfig::AVCODEC
    PkgConfig::AVUTIL
    PkgConfig::SWSCALE
)

target_link_libraries(BufferedLowBandwidthStreamerApp PRIVATE
    ${OpenCV_LIBS}
    PkgConfig::AVCODEC
    PkgConfig::AVUTIL
    PkgConfig::SWSCALE
)

target_link_libraries(mytest PRIVATE ${OpenCV_LIBS})

if(UNIX AND NOT APPLE)
    target_compile_options(CameraStreamer PRIVATE -pthread)
    target_compile_options(LowBandwidthStreamerApp PRIVATE -pthread)
    target_compile_options(BufferedLowBandwidthStreamerApp PRIVATE -pthread)
    target_link_options(CameraStreamer PRIVATE -pthread)
    target_link_options(LowBandwidthStreamerApp PRIVATE -pthread)
    target_link_options(BufferedLowBandwidthStreamerApp PRIVATE -pthread)
endif()

enable_testing()
add_test(NAME MyTest COMMAND mytest)
