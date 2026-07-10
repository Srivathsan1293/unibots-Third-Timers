cmake_minimum_required(VERSION 3.14)
project(apriltag_native)

set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_WRAPPER OFF CACHE BOOL "" FORCE)

include(FetchContent)
FetchContent_Declare(
        apriltag
        GIT_REPOSITORY https://github.com/AprilRobotics/apriltag.git
        GIT_TAG        master
)
FetchContent_MakeAvailable(apriltag)

add_library(apriltag_native SHARED apriltag_native.cpp)
target_link_libraries(apriltag_native PRIVATE apriltag log)
target_include_directories(apriltag_native PRIVATE
        ${apriltag_SOURCE_DIR}
        ${apriltag_SOURCE_DIR}/common
)