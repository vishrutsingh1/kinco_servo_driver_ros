cmake_minimum_required(VERSION 3.0.2)
project(base_motor)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
)

catkin_package(

)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/kinco_drive.cpp src/ros_wrapper.cpp src/main.cpp)


target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

