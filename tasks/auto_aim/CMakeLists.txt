cmake_minimum_required(VERSION 3.16)

if(NOT OpenVINO_DIR)
    set(
        OpenVINO_DIR
        "/opt/intel/openvino_2024.6.0/runtime/cmake"
        CACHE PATH "Path to the OpenVINO CMake package"
    )
endif()

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_subdirectory(planner/tinympc)

add_library(auto_aim OBJECT 
    armor.cpp
    classifier.cpp 
    detector.cpp
    solver.cpp
    aimer.cpp
    target.cpp
    tracker.cpp
    voter.cpp
    shooter.cpp
    yolo.cpp
    yolos/yolov5.cpp
    yolos/yolov8.cpp
    yolos/yolo11.cpp
    multithread/commandgener.cpp
    multithread/mt_detector.cpp
    planner/planner.cpp
)

target_link_libraries(auto_aim PUBLIC
    Eigen3::Eigen
    fmt::fmt
    io
    nlohmann_json::nlohmann_json
    openvino::runtime
    tinympcstatic
    yaml-cpp
)
