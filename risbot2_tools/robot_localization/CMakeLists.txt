cmake_minimum_required(VERSION 3.5)
project(robot_localization)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message(STATUS "${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(Boost REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

set(library_name rl_lib)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/FromLL.srv"
  "srv/GetState.srv"
  "srv/SetDatum.srv"
  "srv/SetPose.srv"
  "srv/SetUTMZone.srv"
  "srv/ToggleFilterProcessing.srv"
  "srv/ToLL.srv"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    geographic_msgs
    diagnostic_msgs
  ADD_LINTER_TESTS
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

add_library(
  ${library_name}
  src/ekf.cpp
  src/ukf.cpp
  src/filter_base.cpp
  src/filter_utilities.cpp
  src/navsat_transform.cpp
  src/robot_localization_estimator.cpp
  src/ros_filter.cpp
  src/ros_filter_utilities.cpp
  src/ros_robot_localization_listener.cpp)
target_include_directories(${library_name} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${library_name} PUBLIC
  Boost::boost
  "${cpp_typesupport_target}"
  Eigen3::Eigen
  angles::angles
  ${diagnostic_msgs_TARGETS}
  diagnostic_updater::diagnostic_updater
  ${geographic_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  message_filters::message_filters
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2::tf2
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
)
target_link_libraries(${library_name} PRIVATE
  ${GeographicLib_LIBRARIES}
  yaml-cpp::yaml-cpp
)

add_executable(
  ekf_node
  src/ekf_node.cpp
)
target_link_libraries(ekf_node PRIVATE
  ${library_name}
  rclcpp::rclcpp
)

add_executable(
  ukf_node
  src/ukf_node.cpp
)
target_link_libraries(ukf_node PRIVATE
  ${library_name}
  rclcpp::rclcpp
)

add_executable(
  navsat_transform_node
  src/navsat_transform_node.cpp
)
target_link_libraries(navsat_transform_node PRIVATE
  ${library_name}
  rclcpp::rclcpp
)

add_executable(
  robot_localization_listener_node
  src/robot_localization_listener_node.cpp
)
target_link_libraries(robot_localization_listener_node PRIVATE
  ${library_name}
  rclcpp::rclcpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_find_gtest()

  #### FILTER BASE TESTS ####
  ament_add_gtest(filter_base-test test/test_filter_base.cpp)
  target_link_libraries(filter_base-test ${library_name})

  #### DIAGNOSTICS TESTS ####
  ament_add_gtest_executable(test_filter_base_diagnostics_timestamps
                    test/test_filter_base_diagnostics_timestamps.cpp)
  target_link_libraries(test_filter_base_diagnostics_timestamps ${library_name})

  target_link_libraries(test_filter_base_diagnostics_timestamps "${cpp_typesupport_target}")

  add_dependencies(test_filter_base_diagnostics_timestamps ekf_node)
  ament_add_test(test_filter_base_diagnostics_timestamps
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 300
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/test_filter_base_diagnostics_timestamps.launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  )

  #### EKF TESTS ######
  ament_add_gtest(test_ekf test/test_ekf.cpp)
  target_link_libraries(test_ekf ${library_name})

  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(test_ekf "${cpp_typesupport_target}")

  ament_add_gtest_executable(
    test_ekf_node_interfaces
    test/test_se_node_interfaces.cpp)
  target_link_libraries(
    test_ekf_node_interfaces
    ${library_name})

  rosidl_get_typesupport_target(
    cpp_typesupport_target
    "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(
    test_ekf_node_interfaces
    "${cpp_typesupport_target}")

  add_dependencies(
    test_ekf_node_interfaces
    ekf_node)
  ament_add_test(
    test_ekf_node_interfaces
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 300
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/test_se_node_interfaces.launch.py"
    ENV NODE_TYPE=ekf
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")

  #### UKF TESTS #####
  ament_add_gtest(test_ukf test/test_ukf.cpp)
  target_link_libraries(test_ukf ${library_name})

  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(test_ukf "${cpp_typesupport_target}")

  ament_add_gtest_executable(
    test_ukf_node_interfaces
    test/test_se_node_interfaces.cpp)
  target_link_libraries(
    test_ukf_node_interfaces
    ${library_name})

  rosidl_get_typesupport_target(
    cpp_typesupport_target
    "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(
    test_ukf_node_interfaces
    "${cpp_typesupport_target}")

  add_dependencies(
    test_ukf_node_interfaces
    ukf_node)
  ament_add_test(
    test_ukf_node_interfaces
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 300
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/test_se_node_interfaces.launch.py"
    ENV NODE_TYPE=ukf
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")

  #### DATA TESTS #####
#  ament_add_gtest(test_ekf_localization_node_bag1 test/test_localization_node_bag_pose_tester.cpp)
#  target_link_libraries(test_ekf_localization_node_bag1 ${library_name})

#  ament_add_gtest(test_ekf_localization_node_bag2 test/test_localization_node_bag_pose_tester.cpp)
#  target_link_libraries(test_ekf_localization_node_bag2 ${library_name})

#  ament_add_gtest(test_ekf_localization_node_bag3 test/test_localization_node_bag_pose_tester.cpp)
#  target_link_libraries(test_ekf_localization_node_bag3 ${library_name})

#  ament_add_gtest(test_ukf_localization_node_bag1 test/test_localization_node_bag_pose_tester.cpp)
#  target_link_libraries(test_ukf_localization_node_bag1 ${library_name})

#  ament_add_gtest(test_ukf_localization_node_bag2 test/test_localization_node_bag_pose_tester.cpp)
#  target_link_libraries(test_ukf_localization_node_bag2 ${library_name})

#  ament_add_gtest(test_ukf_localization_node_bag3 test/test_localization_node_bag_pose_tester.cpp)
#  target_link_libraries(test_ukf_localization_node_bag3 ${library_name})

  #### RLE/RLL TESTS #####
  ament_add_gtest_executable(test_robot_localization_estimator
                  test/test_robot_localization_estimator.cpp)
  target_link_libraries(test_robot_localization_estimator ${library_name})
  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(test_robot_localization_estimator "${cpp_typesupport_target}")
  ament_add_test(test_robot_localization_estimator
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 100
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/test_robot_localization_estimator.launch.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  )

  #### NAVSAT CONVERSION TESTS ####
  ament_add_gtest(test_navsat_conversions test/test_navsat_conversions.cpp)
  target_include_directories(test_navsat_conversions PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")
  target_link_libraries(test_navsat_conversions ${library_name} ${GeographicLib_LIBRARIES})

  ament_add_gtest_executable(test_ros_robot_localization_listener
                  test/test_ros_robot_localization_listener.cpp)
  target_link_libraries(test_ros_robot_localization_listener ${library_name})
  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(test_ros_robot_localization_listener "${cpp_typesupport_target}")
  ament_add_gtest_executable(test_ros_robot_localization_listener_publisher
                  test/test_ros_robot_localization_listener_publisher.cpp)
  target_link_libraries(test_ros_robot_localization_listener_publisher ${library_name})
  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(test_ros_robot_localization_listener_publisher "${cpp_typesupport_target}")
  ament_add_test(test_ros_robot_localization_listener
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 100
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros_robot_localization_listener.launch.py"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  )

  ament_cppcheck(LANGUAGE "c++")
  ament_cpplint()
  ament_lint_cmake()
  ament_uncrustify()

  install(
    TARGETS
    filter_base-test
    test_filter_base_diagnostics_timestamps
    test_ekf
    test_ekf_node_interfaces
    test_ukf
    test_ukf_node_interfaces
    #test_ekf_localization_node_bag1
    #test_ekf_localization_node_bag2
    #test_ekf_localization_node_bag3
    test_robot_localization_estimator
    test_navsat_conversions
    test_ros_robot_localization_listener
    test_ros_robot_localization_listener_publisher
    #test_ukf_localization_node_bag1
    #test_ukf_localization_node_bag2
    #test_ukf_localization_node_bag3
    DESTINATION lib/${PROJECT_NAME}
  )

  # Install test launch config files.
  install(DIRECTORY
    test
    DESTINATION share/${PROJECT_NAME}
    USE_SOURCE_PERMISSIONS
  )
endif()

install(
  TARGETS
  navsat_transform_node
  ekf_node
  ukf_node
  robot_localization_listener_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${library_name}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

# Install params config files.
install(DIRECTORY
  params
  launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})
ament_export_dependencies(
  angles
  Boost
  diagnostic_msgs
  diagnostic_updater
  Eigen3
  geographic_msgs
  geometry_msgs
  message_filters
  nav_msgs
  rclcpp
  rosidl_default_runtime
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
)
ament_export_targets(${PROJECT_NAME})
ament_package()
