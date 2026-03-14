cmake_minimum_required(VERSION 3.16.3)

project(sp_vision LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

option(SP_VISION_BUILD_AUTO_BUFF "Build the auto_buff module" OFF)
option(SP_VISION_BUILD_OMNIPERCEPTION "Build the omniperception module" OFF)

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
if(NOT OpenVINO_DIR)
  set(
    OpenVINO_DIR
    "/opt/intel/openvino_2024.6.0/runtime/cmake/"
    CACHE PATH "Path to the OpenVINO CMake package"
  )
endif()
find_package(OpenVINO REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})


add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
if(SP_VISION_BUILD_AUTO_BUFF)
  add_subdirectory(tasks/auto_buff)
endif()

if(SP_VISION_BUILD_OMNIPERCEPTION)
  add_subdirectory(tasks/omniperception)
endif()

function(sp_vision_add_binary target source)
  add_executable(${target} ${source})
  target_link_libraries(${target} PRIVATE ${ARGN})
endfunction()

#################### src ####################
sp_vision_add_binary(gkdhero src/gkdhero.cpp ${OpenCV_LIBS} fmt::fmt auto_aim tools io)
sp_vision_add_binary(gkdhero_debug src/gkdhero_debug.cpp ${OpenCV_LIBS} fmt::fmt auto_aim tools io)
sp_vision_add_binary(gkdinfantry src/gkdinfantry.cpp ${OpenCV_LIBS} fmt::fmt auto_aim tools io)
sp_vision_add_binary(
  gkdinfantry_debug
  src/gkdinfantry_debug.cpp
  ${OpenCV_LIBS}
  fmt::fmt
  auto_aim
  tools
  io
)
sp_vision_add_binary(
  gkdinfantry_MPC
  src/gkdinfantry_MPC.cpp
  ${OpenCV_LIBS}
  fmt::fmt
  auto_aim
  tools
  io
)
sp_vision_add_binary(
  gkdinfantry_debug_MPC
  src/gkdinfantry_debug_MPC.cpp
  ${OpenCV_LIBS}
  fmt::fmt
  auto_aim
  tools
  io
)

################## calibration ##################
sp_vision_add_binary(capture calibration/capture.cpp ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
sp_vision_add_binary(
  calibrate_camera
  calibration/calibrate_camera.cpp
  ${OpenCV_LIBS}
  fmt::fmt
  yaml-cpp
  tools
)
sp_vision_add_binary(
  calibrate_handeye
  calibration/calibrate_handeye.cpp
  ${OpenCV_LIBS}
  fmt::fmt
  yaml-cpp
  tools
)
sp_vision_add_binary(
  calibrate_robotworld_handeye
  calibration/calibrate_robotworld_handeye.cpp
  ${OpenCV_LIBS}
  fmt::fmt
  yaml-cpp
  tools
)
sp_vision_add_binary(split_video calibration/split_video.cpp ${OpenCV_LIBS} fmt::fmt tools)
sp_vision_add_binary(record calibration/record.cpp ${OpenCV_LIBS} fmt::fmt tools io)

################## tests ##################
sp_vision_add_binary(
  camera_detect_test
  tests/camera_detect_test.cpp
  ${OpenCV_LIBS}
  fmt::fmt
  auto_aim
  tools
  io
)
sp_vision_add_binary(camera_test tests/camera_test.cpp ${OpenCV_LIBS} fmt::fmt tools io)
sp_vision_add_binary(
  camera_thread_test
  tests/camera_thread_test.cpp
  ${OpenCV_LIBS}
  fmt::fmt
  auto_aim
  tools
  io
)
sp_vision_add_binary(
  handeye_test
  tests/handeye_test.cpp
  ${OpenCV_LIBS}
  fmt::fmt
  yaml-cpp
  auto_aim
  tools
  io
)
