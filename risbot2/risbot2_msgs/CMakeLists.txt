################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(risbot2_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(action_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
set(msg_files
)

set(srv_files
)

set(action_files
)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${msg_files}
#   ${srv_files}
#   ${action_files}
#   DEPENDENCIES action_msgs std_msgs
#   ADD_LINTER_TESTS
# )
ament_export_dependencies(rosidl_default_runtime)
ament_package()
