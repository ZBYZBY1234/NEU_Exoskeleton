# cmake_minimum_required(VERSION 3.0.2)
# project(powered_exoskeleton)
# # Set c++11 cmake flags
# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

# set(CMAKE_C_FLAGS "-Wall -Wno-unused-result -Wno-unknown-pragmas -Wno-unused-variable -Wno-return-type -Wfatal-errors -fPIC ${CMAKE_C_FLAGS}")
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   std_msgs
# )

#  catkin_package()

# include_directories( 
#   include 
#   ${catkin_INCLUDE_DIRS}
# )

# # add_library(test1
   
# #    src/test.cpp
# # )
# add_library(EciDemo113
#    src/EciDemo113.cpp
# )

# add_library(EciDemoCommon
#    src/EciDemoCommon.c
# )

# # target_link_libraries(test1 ${catkin_LIBRARIES})
# target_link_libraries(EciDemo113 ${catkin_LIBRARIES})

# # add_dependencies(test1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(EciDemoCommon ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(EciDemo113 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )

# # Origin
# add_executable(${PROJECT_NAME} src/EciDemo113.cpp src/LinuxEciDemo.cpp  )
# target_link_libraries(${PROJECT_NAME}  EciDemoCommon ${catkin_LIBRARIES} )#"/usr/lib/libeci113DriverLinux.so")


# install(TARGETS ${PROJECT_NAME}
# ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# install(DIRECTORY include/${PROJECT_NAME}/
# DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# FILES_MATCHING PATTERN "*.h"
# PATTERN ".svn" EXCLUDE
# )