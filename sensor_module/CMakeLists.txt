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

add_executable(MPU6050 src/IMU/MPU6050.cpp )
target_link_libraries(MPU6050  ${catkin_LIBRARIES})
ament_target_dependencies(MPU6050 rclcpp std_msgs)
add_executable(MPU6050_Thigh src/IMU/MPU6050_Thigh.cpp )
target_link_libraries(MPU6050_Thigh  ${catkin_LIBRARIES})
ament_target_dependencies(MPU6050_Thigh rclcpp std_msgs)
add_executable(Exoskeleton_Thigh src/IMU/Exoskeleton_Thigh.cpp )
target_link_libraries(Exoskeleton_Thigh  ${catkin_LIBRARIES})
ament_target_dependencies(Exoskeleton_Thigh rclcpp std_msgs)
add_executable(MPU6050_Calf src/IMU/MPU6050_Calf.cpp )
target_link_libraries(MPU6050_Calf  ${catkin_LIBRARIES})
ament_target_dependencies(MPU6050_Calf rclcpp std_msgs)
add_executable(Exoskeleton_Calf src/IMU/Exoskeleton_Calf.cpp )
target_link_libraries(Exoskeleton_Calf  ${catkin_LIBRARIES})
ament_target_dependencies(Exoskeleton_Calf rclcpp std_msgs)

add_executable(Piezoelectric src/Force/Piezoelectric.cpp )
target_link_libraries(Piezoelectric  ${catkin_LIBRARIES})
ament_target_dependencies(Piezoelectric rclcpp std_msgs)

add_executable(Sensor src/Sensor.cpp )
target_link_libraries(Sensor  ${catkin_LIBRARIES})
ament_target_dependencies(Sensor rclcpp std_msgs)

install(TARGETS
MPU6050_Thigh
MPU6050_Calf
Piezoelectric
Sensor
DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
