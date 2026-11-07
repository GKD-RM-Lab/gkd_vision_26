find_package(Threads REQUIRED)

add_library(runtime_parameter_support STATIC
  parameter_channel.cpp
  ../../tasks/auto_aim/runtime_params.cpp)
target_include_directories(runtime_parameter_support PUBLIC
  "${PROJECT_SOURCE_DIR}" ${OpenCV_INCLUDE_DIRS})
target_link_libraries(runtime_parameter_support PUBLIC
  Eigen3::Eigen ${OpenCV_LIBS} yaml-cpp Threads::Threads nlohmann_json::nlohmann_json
  PRIVATE spdlog::spdlog fmt::fmt)
