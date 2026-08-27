cmake_minimum_required(VERSION 3.8)
project(scan_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

option(SCAN_PLANNER_BUILD_OPEN_LOOP_CONTROLLER
  "Build the legacy open-loop controller executable" ON)
option(SCAN_PLANNER_BUILD_SIMULATION_NODES
  "Build Go2-only kinematic simulation executables" ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(omni_robot_interfaces REQUIRED)
find_package(path_searching REQUIRED)
find_package(plan_env REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(scan_planner_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(traj_utils REQUIRED)
find_package(visualization_msgs REQUIRED)

if(CMAKE_SYSROOT)
  set(_scan_planner_numeric_search_paths
    "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/openblas-pthread"
    "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/lapack"
    "${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/blas")
  find_library(SCAN_PLANNER_LAPACK_LIBRARY
    NAMES lapack
    PATHS ${_scan_planner_numeric_search_paths}
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH)
  find_library(SCAN_PLANNER_BLAS_LIBRARY
    NAMES openblas blas
    PATHS ${_scan_planner_numeric_search_paths}
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH)
else()
  find_library(SCAN_PLANNER_LAPACK_LIBRARY NAMES lapack)
  find_library(SCAN_PLANNER_BLAS_LIBRARY NAMES openblas blas)
endif()
set(SCAN_PLANNER_NUMERIC_LIBS "")
if(SCAN_PLANNER_LAPACK_LIBRARY AND SCAN_PLANNER_BLAS_LIBRARY)
  list(APPEND SCAN_PLANNER_NUMERIC_LIBS
    "${SCAN_PLANNER_LAPACK_LIBRARY}"
    "${SCAN_PLANNER_BLAS_LIBRARY}")
  if(CMAKE_SYSROOT)
    list(APPEND SCAN_PLANNER_NUMERIC_LIBS
      "-Wl,-rpath-link,${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu"
      "-Wl,-rpath-link,${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/openblas-pthread")
  endif()
endif()

set(scan_planner_dependencies
  ament_index_cpp bspline_opt builtin_interfaces geometry_msgs nav_msgs omni_robot_interfaces
  path_searching plan_env rclcpp rclcpp_action scan_planner_msgs sensor_msgs std_msgs
  tf2 tf2_geometry_msgs tf2_ros traj_utils visualization_msgs)

add_library(trajectory_acceptance_validator STATIC
  src/trajectory_acceptance_validator.cpp)
target_include_directories(trajectory_acceptance_validator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(trajectory_acceptance_validator bspline_opt)
target_link_libraries(trajectory_acceptance_validator Eigen3::Eigen)

add_library(reference_route STATIC
  src/local_route_progress_map.cpp
  src/merge_reference_builder.cpp
  src/reference_route.cpp
  src/route_progress_tracker.cpp)
target_include_directories(reference_route PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(reference_route Eigen3::Eigen)

add_executable(scan_planner_node
  src/scan_planner_node.cpp
  src/scan_replan_fsm.cpp
  src/follow_route_server.cpp
  src/planner_manager.cpp)
target_include_directories(scan_planner_node PRIVATE include)
ament_target_dependencies(scan_planner_node ${scan_planner_dependencies})
target_link_libraries(scan_planner_node
  reference_route trajectory_acceptance_validator
  Eigen3::Eigen ${SCAN_PLANNER_NUMERIC_LIBS})

set(scan_planner_runtime_targets
  scan_planner_node
  closed_loop_controller
  global_path_publisher)

if(SCAN_PLANNER_BUILD_OPEN_LOOP_CONTROLLER)
  list(APPEND scan_planner_runtime_targets open_loop_controller)
endif()

if(SCAN_PLANNER_BUILD_SIMULATION_NODES)
  list(APPEND scan_planner_runtime_targets
    go2_kinematic_sim
    go2_gait_publisher)
endif()

foreach(target IN LISTS scan_planner_runtime_targets)
  if(target STREQUAL "scan_planner_node")
    continue()
  endif()
  add_executable(${target} src/${target}.cpp)
  target_include_directories(${target} PRIVATE include)
  ament_target_dependencies(${target}
    bspline_opt builtin_interfaces geometry_msgs nav_msgs rclcpp scan_planner_msgs sensor_msgs
    std_msgs tf2 tf2_geometry_msgs tf2_ros)
  target_link_libraries(${target} Eigen3::Eigen)
endforeach()

install(TARGETS ${scan_planner_runtime_targets}
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS reference_route trajectory_acceptance_validator
  EXPORT export_scan_planner_algorithms
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME} OPTIONAL)
install(FILES launch/default.rviz DESTINATION share/${PROJECT_NAME}/rviz OPTIONAL)
install(PROGRAMS scripts/keypoint_recorder.py
  DESTINATION lib/${PROJECT_NAME} OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_reference_path_tracker
    test/test_reference_path_tracker.cpp)
  target_include_directories(test_reference_path_tracker PRIVATE include)
  target_link_libraries(test_reference_path_tracker reference_route Eigen3::Eigen)
  ament_add_gtest(test_reference_route
    test/test_reference_route.cpp)
  target_include_directories(test_reference_route PRIVATE include)
  target_link_libraries(test_reference_route reference_route Eigen3::Eigen)
  ament_add_gtest(test_route_progress_tracker
    test/test_route_progress_tracker.cpp)
  target_include_directories(test_route_progress_tracker PRIVATE include)
  target_link_libraries(test_route_progress_tracker reference_route Eigen3::Eigen)
  ament_add_gtest(test_local_route_progress_map
    test/test_local_route_progress_map.cpp)
  target_include_directories(test_local_route_progress_map PRIVATE include)
  target_link_libraries(test_local_route_progress_map reference_route Eigen3::Eigen)
  ament_add_gtest(test_merge_reference_builder
    test/test_merge_reference_builder.cpp)
  target_include_directories(test_merge_reference_builder PRIVATE include)
  target_link_libraries(test_merge_reference_builder reference_route Eigen3::Eigen)
  ament_add_gtest(test_yaw_command_filter
    test/test_yaw_command_filter.cpp)
  target_include_directories(test_yaw_command_filter PRIVATE include)
  ament_add_gtest(test_local_trajectory_tracker
    test/test_local_trajectory_tracker.cpp)
  target_include_directories(test_local_trajectory_tracker PRIVATE include)
  target_link_libraries(test_local_trajectory_tracker Eigen3::Eigen)
  ament_add_gtest(test_pure_pursuit_recovery_sweep
    test/test_pure_pursuit_recovery_sweep.cpp)
  target_include_directories(test_pure_pursuit_recovery_sweep PRIVATE include)
  target_link_libraries(test_pure_pursuit_recovery_sweep Eigen3::Eigen)
  ament_add_gtest(test_path_tracking_controller
    test/test_path_tracking_controller.cpp)
  target_include_directories(test_path_tracking_controller PRIVATE include)
  ament_add_gtest(test_path_command_governor
    test/test_path_command_governor.cpp)
  target_include_directories(test_path_command_governor PRIVATE include)
  ament_add_gtest(test_bspline_message_validator
    test/test_bspline_message_validator.cpp)
  target_include_directories(test_bspline_message_validator PRIVATE include)
  ament_target_dependencies(test_bspline_message_validator
    geometry_msgs scan_planner_msgs std_msgs)
  ament_add_gtest(test_planner_heartbeat_gate
    test/test_planner_heartbeat_gate.cpp)
  target_include_directories(test_planner_heartbeat_gate PRIVATE include)
  ament_target_dependencies(test_planner_heartbeat_gate builtin_interfaces)
  ament_add_gtest(test_planning_safety_foundation
    test/test_planning_safety_foundation.cpp)
  target_include_directories(test_planning_safety_foundation PRIVATE include)
  ament_add_gtest(test_trajectory_acceptance_validator
    test/test_trajectory_acceptance_validator.cpp)
  target_include_directories(test_trajectory_acceptance_validator PRIVATE include)
  ament_target_dependencies(test_trajectory_acceptance_validator bspline_opt)
  target_link_libraries(test_trajectory_acceptance_validator
    trajectory_acceptance_validator Eigen3::Eigen)
  add_launch_test(test/test_planner_startup.py TIMEOUT 30)
  add_launch_test(test/test_waypoint_parameters.py TIMEOUT 30)
  ament_add_pytest_test(test_legacy_zsibot_opt_in
    test/test_legacy_zsibot_opt_in.py
    TIMEOUT 30)
  ament_add_pytest_test(test_omni_node_contract
    test/test_omni_node_contract.py
    TIMEOUT 30)
  ament_add_pytest_test(test_s100_build_contract
    test/test_s100_build_contract.py
    TIMEOUT 30)
endif()

ament_export_targets(export_scan_planner_algorithms HAS_LIBRARY_TARGET)
ament_export_dependencies(bspline_opt Eigen3)

ament_package()
