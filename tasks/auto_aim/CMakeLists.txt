cmake_minimum_required(VERSION 3.16)

if(NOT OpenVINO_DIR)
    foreach(
        _sp_vision_openvino_dir
        IN ITEMS
        "$ENV{OpenVINO_DIR}"
        "$ENV{OPENVINO_DIR}"
        "/opt/intel/openvino_2024.6.0/runtime/cmake"
        "/opt/intel/openvino/runtime/cmake"
    )
        if(_sp_vision_openvino_dir AND EXISTS "${_sp_vision_openvino_dir}")
            set(
                OpenVINO_DIR
                "${_sp_vision_openvino_dir}"
                CACHE PATH "Path to the OpenVINO CMake package" FORCE
            )
            break()
        endif()
    endforeach()
endif()

find_package(OpenVINO REQUIRED COMPONENTS Runtime)

add_subdirectory(planner/tinympc)

add_library(auto_aim OBJECT 
    armor.cpp
    classifier.cpp 
    detector.cpp
    solver.cpp
    aimer.cpp
    center_aimer.cpp
    center_shooter.cpp
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
