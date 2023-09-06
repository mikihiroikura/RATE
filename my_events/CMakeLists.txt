cmake_minimum_required(VERSION 3.0.2)
project(my_events)

# Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dvs_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(Ceres REQUIRED)

catkin_package()

include_directories(
  include
  include/cv-plot/CvPlot/inc
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
)

add_executable(ev_feature_detection src/feature_detection.cpp)

target_link_libraries(ev_feature_detection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  Eigen3::Eigen
)

