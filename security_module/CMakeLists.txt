cmake_minimum_required(VERSION 3.5)
project(security_module)

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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(Joint_Record src/Joint_Record.cpp)
ament_target_dependencies(Joint_Record rclcpp std_msgs sensor_msgs)

add_executable(Joint_Record_25 src/Joint_Record_25.cpp)
ament_target_dependencies(Joint_Record_25 rclcpp std_msgs sensor_msgs)

add_executable(Joint_Input src/Joint_Input.cpp)
ament_target_dependencies(Joint_Input rclcpp std_msgs)
install(
    TARGETS
    Joint_Record
    Joint_Record_25
    Joint_Input
    DESTINATION lib/${PROJECT_NAME}
)
ament_package()
