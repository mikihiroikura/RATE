# Required CMake version and project name.
cmake_minimum_required(VERSION 3.0.0)
project(QuickGui VERSION 1.0.0 LANGUAGES CXX)

# Required language standard.
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# Find required packages.
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)

add_subdirectory(lib)

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    lib
    include
)

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE ImGui ImPlot glog)
target_include_directories(${PROJECT_NAME} INTERFACE
    include/
    lib/
    lib/imgui
    lib/implot)


add_executable(${PROJECT_NAME}_main main.cpp)
target_link_libraries(${PROJECT_NAME}_main ${PROJECT_NAME})