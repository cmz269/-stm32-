cmake_minimum_required(VERSION 3.0.2)
project(car_controller)



find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg serial tf nav_msgs)




generate_messages(DEPENDENCIES std_msgs)
## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(car_controller src/car_controller.cpp)
target_link_libraries(car_controller ${catkin_LIBRARIES})
add_dependencies(car_controller car_controller_generate_messages_cpp)

add_executable(car_listener src/car_listener.cpp)
target_link_libraries(car_listener ${catkin_LIBRARIES})
add_dependencies(car_listener car_controller_generate_messages_cpp)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
