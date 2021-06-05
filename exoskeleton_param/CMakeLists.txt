cmake_minimum_required(VERSION 3.0.2)
project(exoskeleton_param)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  roscpp
  std_msgs
  tf
  message_generation
)

add_message_files(
  FILES
  JointState_self.msg
)

#generate_messages必须在catkin_package前面
generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(

CATKIN_DEPENDS message_runtime

)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_listener src/tf_listener.cpp)
target_link_libraries(tf_listener ${catkin_LIBRARIES})

add_executable(joint_state src/joint_state.cpp)
target_link_libraries(joint_state ${catkin_LIBRARIES})