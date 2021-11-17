cmake_minimum_required(VERSION 3.0)
project(examples LANGUAGES CXX)

option(BUILD_EXAMPLE "Build example program" OFF)
option(BUILD_EXAMPLE_SDL "Build SDL example program" ON)
option(BUILD_EXAMPLE_OPENCV "Build OpenCV example program" OFF)


if(BUILD_EXAMPLE)
  add_executable(example example.c)
  find_package(Threads)
  target_link_libraries(example
    PRIVATE
      LibUVC::UVC
      Threads::Threads
  )
endif()

if(BUILD_EXAMPLE_OPENCV)
  # OpenCV defines targets with transitive dependencies not with namespaces but using opencv_ prefix. 
  # This targets provide necessary include directories and linked flags.
  find_package(OpenCVPkg REQUIRED
    COMPONENTS
      opencv_core
      opencv_highgui
  )

  add_executable(example_opencv test.c)
  target_link_libraries(example_opencv
    PRIVATE
      LibUVC::UVC
      opencv_core
      opencv_highgui
  )
endif()

if(BUILD_EXAMPLE_SDL)
    find_package(SDL2 REQUIRED)
    include_directories(${SDL2_INCLUDE_DIRS})

    add_executable(example_sdl example_sdl.cpp)
    target_compile_features(example_sdl PRIVATE cxx_std_17)
    target_link_libraries(example_sdl
        PRIVATE
        LibUVC::UVC
        ${SDL2_LIBRARIES}
    )
endif()

