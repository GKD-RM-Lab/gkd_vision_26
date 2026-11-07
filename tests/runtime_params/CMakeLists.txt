# Exercise the real tracker/target and UDP implementation without camera/model/hardware.
add_executable(runtime_params_test
  ../runtime_params_test.cpp
  ../../tasks/auto_aim/tracker.cpp
  ../../tasks/auto_aim/target.cpp
  ../../tasks/auto_aim/solver.cpp
  ../../tasks/auto_aim/armor.cpp
  ../../tasks/auto_aim/aimer.cpp
  ../../tasks/auto_aim/center_aimer.cpp
  ../../tools/extended_kalman_filter.cpp
  ../../tools/math_tools.cpp
  ../../tools/trajectory.cpp
  ../../tools/logger.cpp
  ../../io/gkdcontrol/send_control.cpp)
target_include_directories(runtime_params_test PRIVATE "${PROJECT_SOURCE_DIR}")
target_link_libraries(runtime_params_test PRIVATE runtime_parameter_support spdlog::spdlog fmt::fmt)
target_compile_definitions(runtime_params_test PRIVATE
  RUNTIME_TEST_CONFIG="${PROJECT_SOURCE_DIR}/configs/gkdhero_fiexed.yaml")
add_test(NAME runtime_params_test COMMAND runtime_params_test)
set_tests_properties(runtime_params_test PROPERTIES TIMEOUT 30)
