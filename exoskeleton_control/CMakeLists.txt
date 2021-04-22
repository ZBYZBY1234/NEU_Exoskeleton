cmake_minimum_required(VERSION 3.5)
project(exoskeleton_control)

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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_library(EciDemo113
    src/Eci/EciDemo113.cpp
)
add_library(EciDemoCommon
    src/Eci/EciDemoCommon.c
)
add_library(Shmfifo
    src/Eci/shmfifo.c
)
target_link_libraries(EciDemo113 ${catkin_LIBRARIES})

ament_target_dependencies(EciDemoCommon ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
ament_target_dependencies(EciDemo113 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(EciDemo src/Eci/EciDemo113.cpp src/Eci/LinuxEciDemo.cpp)
target_link_libraries(EciDemo  EciDemoCommon ${catkin_LIBRARIES} "/usr/lib/libeci113DriverLinux.so")

add_executable(Eci_keyboard src/Eci/EciDemo113.cpp src/Eci/Eci_keyboard.cpp)
target_link_libraries(Eci_keyboard EciDemoCommon ${catkin_LIBRARIES} "/usr/lib/libeci113DriverLinux.so")

add_executable(Test  src/Test.cpp)
# target_link_libraries(Angle_Feedback EciDemoCommon "/usr/lib/libeci113DriverLinux.so")
ament_target_dependencies(Test ${catkin_LIBRARIES} rclcpp std_msgs)

add_executable(Angle_Feedback  src/Eci/Angle_Feedback.cpp)
ament_target_dependencies(Angle_Feedback  rclcpp std_msgs)
target_link_libraries(Angle_Feedback EciDemoCommon "/usr/lib/libeci113DriverLinux.so")

add_executable(Server src/Server.c)
target_link_libraries(Server ${catkin_LIBRARIES})

add_executable(Client src/Client.c)
target_link_libraries(Client ${catkin_LIBRARIES})

add_executable(Clientcpp src/Client.cpp)
target_link_libraries(Clientcpp ${catkin_LIBRARIES})

add_executable(Servercpp src/Server.cpp)
target_link_libraries(Servercpp ${catkin_LIBRARIES})

add_executable(Socket_Client src/Socket_Client.cpp)
target_link_libraries(Socket_Client ${catkin_LIBRARIES})

add_executable(Socket_Server src/Socket_Server.cpp)
target_link_libraries(Socket_Server ${catkin_LIBRARIES})

install(TARGETS
EciDemo
Eci_keyboard
Angle_Feedback

DESTINATION lib/${PROJECT_NAME}
)

add_executable(Exoskeleton_kinematics src/exoskeleton_kinematics.cpp)
ament_target_dependencies(Exoskeleton_kinematics ${catkin_LIBRARIES} rclcpp std_msgs)

add_executable(Exoskeleton_dynamics src/exoskeleton_dynamics.cpp)
ament_target_dependencies(Exoskeleton_dynamics ${catkin_LIBRARIES} rclcpp std_msgs)

add_executable(Admittance_control_Software src/Admittance_control_Software.cpp)
ament_target_dependencies(Admittance_control_Software ${catkin_LIBRARIES} rclcpp std_msgs sensor_msgs)

add_executable(Position_control src/Position_control.cpp)
ament_target_dependencies(Position_control ${catkin_LIBRARIES} rclcpp std_msgs sensor_msgs)

install(TARGETS
Exoskeleton_kinematics
Exoskeleton_dynamics
Admittance_control_Software
Position_control
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
