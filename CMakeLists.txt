cmake_minimum_required(VERSION 3.15)
project(send_lb LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(AVCODEC REQUIRED IMPORTED_TARGET libavcodec)
pkg_check_modules(AVUTIL REQUIRED IMPORTED_TARGET libavutil)
pkg_check_modules(SWSCALE REQUIRED IMPORTED_TARGET libswscale)

option(ENABLE_HIKROBOT "Enable Hikrobot camera support if the SDK is available" ON)
set(HIKROBOT_LOCAL_INCLUDE "${CMAKE_SOURCE_DIR}/third_party/hikrobot/include")
set(HIKROBOT_LOCAL_LIB "${CMAKE_SOURCE_DIR}/third_party/hikrobot/lib/amd64/libMvCameraControl.so")
set(LIBUSB_LOCAL_INCLUDE "${CMAKE_SOURCE_DIR}/third_party/libusb/include")

add_executable(send_lb
    main.cpp
    lb_streamer.cpp
    lb_streamer_utils.cpp
    lb_streamer_roi.cpp
    lb_streamer_encode.cpp
)

set_target_properties(send_lb PROPERTIES
    OUTPUT_NAME "send_lb"
)

target_include_directories(send_lb PRIVATE ${OpenCV_INCLUDE_DIRS})

target_link_libraries(send_lb PRIVATE
    ${OpenCV_LIBS}
    PkgConfig::AVCODEC
    PkgConfig::AVUTIL
    PkgConfig::SWSCALE
)

if(ENABLE_HIKROBOT AND EXISTS "${HIKROBOT_LOCAL_INCLUDE}/MvCameraControl.h" AND EXISTS "${HIKROBOT_LOCAL_LIB}")
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(HIKROBOT_LIBUSB REQUIRED libusb-1.0)

    target_sources(send_lb PRIVATE hikrobot_camera.cpp)
    target_compile_definitions(send_lb PRIVATE HAVE_HIKROBOT_SDK=1)
    target_include_directories(send_lb PRIVATE
        ${HIKROBOT_LOCAL_INCLUDE}
        ${LIBUSB_LOCAL_INCLUDE}
    )
    target_compile_options(send_lb PRIVATE ${HIKROBOT_LIBUSB_CFLAGS_OTHER})
    target_link_libraries(send_lb PRIVATE
        ${HIKROBOT_LOCAL_LIB}
        ${HIKROBOT_LIBUSB_LIBRARIES}
    )

    # 让运行时优先从仓库里的本地海康库找符号。
    set_target_properties(send_lb PROPERTIES
        BUILD_RPATH "${CMAKE_SOURCE_DIR}/third_party/hikrobot/lib/amd64"
    )
endif()

if(UNIX AND NOT APPLE)
    target_compile_options(send_lb PRIVATE -pthread)
    target_link_options(send_lb PRIVATE -pthread)
endif()
