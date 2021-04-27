cmake_minimum_required(VERSION 3.5)
project(sensor_module)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
# compile
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(Piezoelectric src/Force/Piezoelectric.cpp )
target_link_libraries(Piezoelectric  ${catkin_LIBRARIES})
ament_target_dependencies(Piezoelectric rclcpp std_msgs)

add_executable(MPU6050 src/IMU/MPU6050.cpp )
target_link_libraries(MPU6050  ${catkin_LIBRARIES})
ament_target_dependencies(MPU6050 rclcpp std_msgs)

add_executable(Human_Left_Thigh src/IMU/Human_Left_Thigh.cpp )
target_link_libraries(Human_Left_Thigh  ${catkin_LIBRARIES})
ament_target_dependencies(Human_Left_Thigh rclcpp std_msgs)
add_executable(Human_Left_Calf src/IMU/Human_Left_Calf.cpp )
target_link_libraries(Human_Left_Calf  ${catkin_LIBRARIES})
ament_target_dependencies(Human_Left_Calf rclcpp std_msgs)
add_executable(Human_Right_Thigh src/IMU/Human_Right_Thigh.cpp )
target_link_libraries(Human_Right_Thigh  ${catkin_LIBRARIES})
ament_target_dependencies(Human_Right_Thigh rclcpp std_msgs)
add_executable(Human_Right_Calf src/IMU/Human_Right_Calf.cpp )
target_link_libraries(Human_Right_Calf  ${catkin_LIBRARIES})
ament_target_dependencies(Human_Right_Calf rclcpp std_msgs)

add_executable(Exoskeleton_Left_Thigh src/IMU/Exoskeleton_Left_Thigh.cpp )
target_link_libraries(Exoskeleton_Left_Thigh  ${catkin_LIBRARIES})
ament_target_dependencies(Exoskeleton_Left_Thigh rclcpp std_msgs)
add_executable(Exoskeleton_Left_Calf src/IMU/Exoskeleton_Left_Calf.cpp )
target_link_libraries(Exoskeleton_Left_Calf  ${catkin_LIBRARIES})
ament_target_dependencies(Exoskeleton_Left_Calf rclcpp std_msgs)
add_executable(Exoskeleton_Right_Thigh src/IMU/Exoskeleton_Right_Thigh.cpp )
target_link_libraries(Exoskeleton_Right_Thigh  ${catkin_LIBRARIES})
ament_target_dependencies(Exoskeleton_Right_Thigh rclcpp std_msgs)
add_executable(Exoskeleton_Right_Calf src/IMU/Exoskeleton_Right_Calf.cpp )
target_link_libraries(Exoskeleton_Right_Calf  ${catkin_LIBRARIES})
ament_target_dependencies(Exoskeleton_Right_Calf rclcpp std_msgs)


add_executable(Sensor_Human src/Sensor_Fusion/Sensor_Human.cpp )
target_link_libraries(Sensor_Human  ${catkin_LIBRARIES})
ament_target_dependencies(Sensor_Human rclcpp std_msgs)
add_executable(Sensor_Exoskeleton src/Sensor_Fusion/Sensor_Exoskeleton.cpp )
target_link_libraries(Sensor_Exoskeleton  ${catkin_LIBRARIES})
ament_target_dependencies(Sensor_Exoskeleton rclcpp std_msgs)

install(TARGETS
Human_Left_Thigh
Human_Left_Calf
Human_Right_Thigh
Human_Right_Calf

Exoskeleton_Left_Thigh
Exoskeleton_Left_Calf
Exoskeleton_Right_Thigh
Exoskeleton_Right_Calf

Piezoelectric
Sensor_Human
Sensor_Exoskeleton
DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
