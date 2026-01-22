cmake_minimum_required(VERSION 3.10)
project(handeye_calibration_eye_in_hand LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

add_executable(handeye_calibration_eye_in_hand
    handeye_calibration_eye_in_hand.cpp
)

target_include_directories(handeye_calibration_eye_in_hand PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${realsense2_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_directories(handeye_calibration_eye_in_hand PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

target_link_libraries(handeye_calibration_eye_in_hand PRIVATE
    ${OpenCV_LIBS}
    realsense2
    fairino
)

set_target_properties(handeye_calibration_eye_in_hand PROPERTIES
    BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib"
    INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/lib"
)
