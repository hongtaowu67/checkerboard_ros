cmake_minimum_required(VERSION 2.8.3)

project(checkerboard_ros)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    cv_bridge
    image_geometry
    image_transport
    message_filters
    tf
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
#  CATKIN_DEPENDS roscpp tf opencv cv_bridge message_filters
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ./include/
)

add_executable(capture_checkerboard src/capture_checkerboard.cpp)
target_link_libraries(capture_checkerboard ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


