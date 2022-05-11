# On Ubuntu 18.04, get the latest CMake from https://apt.kitware.com/.
cmake_minimum_required(VERSION 3.18)

project(RobustVision_New LANGUAGES C CXX)

# The options need to be the same as Open3D's default
# If Open3D is configured and built with custom options, you'll also need to
# specify the same custom options.
option(STATIC_WINDOWS_RUNTIME "Use static (MT/MTd) Windows runtime" ON)
if(STATIC_WINDOWS_RUNTIME)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

# Find installed Open3D, which exports Open3D::Open3D
find_package(Open3D REQUIRED PATHS /home/ubuntu-z2/open3d_install)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(realsense2 REQUIRED)

add_executable(RobustVision_New)
target_sources(RobustVision_New PRIVATE main.cpp)
target_link_libraries(RobustVision_New
    Open3D::Open3D
    ${OpenCV_LIBS}
     Eigen3::Eigen              #OPEN3D
    ${realsense2_LIBRARY}
    )
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${OpenIGTLink_INCLUDE_DIRS}
    )
