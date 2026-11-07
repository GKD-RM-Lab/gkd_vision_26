find_package(Threads REQUIRED)

add_library(video_streaming STATIC video_packet.cpp video_streamer.cpp)
target_include_directories(video_streaming PUBLIC "${PROJECT_SOURCE_DIR}" ${OpenCV_INCLUDE_DIRS})
target_link_libraries(video_streaming PUBLIC ${OpenCV_LIBS} yaml-cpp Threads::Threads
  PRIVATE spdlog::spdlog fmt::fmt)

if(SP_VISION_ENABLE_VIDEO)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(VIDEO_FFMPEG REQUIRED IMPORTED_TARGET libavcodec libavutil libswscale)
  target_sources(video_streaming PRIVATE h265_encoder.cpp)
  target_compile_definitions(video_streaming PUBLIC SP_VISION_ENABLE_VIDEO=1)
  target_link_libraries(video_streaming PUBLIC PkgConfig::VIDEO_FFMPEG)
endif()

option(SP_VISION_BUILD_VIDEO_TESTS "Build standalone video encoding/packet/transport tests" ON)
if(SP_VISION_BUILD_VIDEO_TESTS)
  add_executable(video_streamer_test
    ../../tests/video_streamer_test.cpp
    ../../tools/logger.cpp
    ../gkdcontrol/send_control.cpp
    ../gkdcontrol/video_sender.cpp)
  target_link_libraries(video_streamer_test PRIVATE video_streaming spdlog::spdlog fmt::fmt)
  add_test(NAME video_streamer_test COMMAND video_streamer_test)
  set_tests_properties(video_streamer_test PROPERTIES TIMEOUT 45)
endif()
