cmake_minimum_required(VERSION 3.0.2)
project(z1_arm_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  interactive_markers
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  pluginlib
  roscpp
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES z1_arm_test
#  CATKIN_DEPENDS cmake_modules interactive_markers moveit_core moveit_ros_perception moveit_ros_planning_interface pluginlib roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# For test_random.cpp
add_executable(test_random_node src/test_random.cpp)
add_dependencies(test_random_node z1_arm_test_generate_messages_cpp)
target_link_libraries(test_random_node ${catkin_LIBRARIES})

# For test_custom.cpp
add_executable(test_custom_node src/test_custom.cpp)
add_dependencies(test_custom_node z1_arm_test_generate_messages_cpp)
target_link_libraries(test_custom_node ${catkin_LIBRARIES})

# For add_collision_object.cpp
add_executable(add_collision_object src/add_collision_object.cpp)
add_dependencies(add_collision_object z1_arm_test_generate_messages_cpp)
target_link_libraries(add_collision_object ${catkin_LIBRARIES})