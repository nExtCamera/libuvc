cmake_minimum_required(VERSION 3.0)
project(cmake_test VERSION 0.0.1 LANGUAGES CXX)

# Prepare doctest for other targets to use
find_package(Doctest REQUIRED)
# add it globally
include_directories(${DOCTEST_INCLUDE_DIR})


find_package(LibUSB)
include_directories(${LibUSB_INCLUDE_DIRS})

# Make test executable
add_executable(uvc_tests
    mock_uvc.cpp
    test_uvc_init.cpp
    #test_streaming.cpp
    test_video_mjpeg.cpp
    test_video_frame_based.cpp
)

target_compile_features(uvc_tests PRIVATE cxx_std_17)
target_link_libraries(uvc_tests
    PRIVATE
    LibUVC::UVC
    ${SDL2_LIBRARIES}
    JPEG::JPEG
)

# copy test file
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/testorig.jpg
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
#file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/frame_50.jpg
#     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# add tests to CTest
include(doctest)
doctest_discover_tests(uvc_tests)
