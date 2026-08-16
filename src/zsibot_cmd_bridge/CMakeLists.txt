cmake_minimum_required(VERSION 3.8)
project(zsibot_cmd_bridge)

message(WARNING
  "zsibot_cmd_bridge is deprecated. Use the product-level unified robot bridge; "
  "this package is retained only for temporary migration or bench testing.")

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

option(
  ZSIBOT_ENABLE_DEPRECATED_SDK_TARGETS
  "Build deprecated binaries that link and own the vendor SDK"
  OFF)

add_executable(zsibot_cmd_udp_client src/zsibot_cmd_udp_client.cpp)
ament_target_dependencies(zsibot_cmd_udp_client geometry_msgs rclcpp)
set(ZSIBOT_INSTALL_TARGETS zsibot_cmd_udp_client)

if(ZSIBOT_ENABLE_DEPRECATED_SDK_TARGETS)
  message(WARNING
    "Building deprecated vendor SDK owners zsibot_cmd_bridge and zsibot_sdk_proxy. "
    "Never run them alongside the unified robot bridge.")

  set(ZSIBOT_MODEL "zsl-1w" CACHE STRING "ZsiBot model: zsl-1 or zsl-1w")
  set_property(CACHE ZSIBOT_MODEL PROPERTY STRINGS zsl-1 zsl-1w)
  set(ZSIBOT_SDK_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../zsibot_sdk"
    CACHE PATH "Path to the ZsiBot SDK root")

  if(ZSIBOT_MODEL STREQUAL "zsl-1")
    set(ZSIBOT_LIB_BASENAME "mc_sdk_zsl_1")
    add_compile_definitions(ZSIBOT_MODEL_ZSL_1)
  elseif(ZSIBOT_MODEL STREQUAL "zsl-1w")
    set(ZSIBOT_LIB_BASENAME "mc_sdk_zsl_1w")
    add_compile_definitions(ZSIBOT_MODEL_ZSL_1W)
  else()
    message(FATAL_ERROR "Unsupported ZSIBOT_MODEL: ${ZSIBOT_MODEL}")
  endif()

  if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64")
    set(ZSIBOT_ARCH "x86_64")
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(ZSIBOT_ARCH "aarch64")
  else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
  endif()

  set(ZSIBOT_LIB_DIR "${ZSIBOT_SDK_ROOT}/lib/${ZSIBOT_MODEL}/${ZSIBOT_ARCH}")
  find_library(ZSIBOT_SDK_LIB
    NAMES "${ZSIBOT_LIB_BASENAME}_${ZSIBOT_ARCH}"
    PATHS "${ZSIBOT_LIB_DIR}"
    NO_CMAKE_FIND_ROOT_PATH
    NO_DEFAULT_PATH
    REQUIRED)

  add_executable(zsibot_cmd_bridge src/zsibot_cmd_bridge.cpp)
  target_include_directories(zsibot_cmd_bridge PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${ZSIBOT_SDK_ROOT}/include"
    "${ZSIBOT_SDK_ROOT}/include/${ZSIBOT_MODEL}")
  ament_target_dependencies(zsibot_cmd_bridge geometry_msgs rclcpp)
  target_link_libraries(zsibot_cmd_bridge "${ZSIBOT_SDK_LIB}")
  set_target_properties(zsibot_cmd_bridge PROPERTIES
    BUILD_RPATH "${ZSIBOT_LIB_DIR}"
    INSTALL_RPATH "$ORIGIN")

  add_executable(zsibot_sdk_proxy src/zsibot_sdk_proxy.cpp)
  target_include_directories(zsibot_sdk_proxy PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${ZSIBOT_SDK_ROOT}/include"
    "${ZSIBOT_SDK_ROOT}/include/${ZSIBOT_MODEL}")
  target_link_libraries(zsibot_sdk_proxy "${ZSIBOT_SDK_LIB}")
  set_target_properties(zsibot_sdk_proxy PROPERTIES
    BUILD_RPATH "${ZSIBOT_LIB_DIR}"
    INSTALL_RPATH "$ORIGIN")

  list(APPEND ZSIBOT_INSTALL_TARGETS zsibot_cmd_bridge zsibot_sdk_proxy)
  install(FILES "${ZSIBOT_SDK_LIB}" DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS
    "Deprecated vendor SDK targets are disabled; building UDP compatibility client only.")
endif()

install(TARGETS ${ZSIBOT_INSTALL_TARGETS}
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
