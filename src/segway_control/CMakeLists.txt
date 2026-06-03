cmake_minimum_required(VERSION 3.8)
project(segway_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# ── Dépendances ───────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ── imu_filter_node ───────────────────────────────────────────────────────────
add_executable(imu_filter_node
  src/imu_filter_node.cpp
)

target_include_directories(imu_filter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(imu_filter_node
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
)

# ── balance_controller ────────────────────────────────────────────────────────
add_executable(balance_controller
  src/balance_controller.cpp
)

target_include_directories(balance_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(balance_controller
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  imu_filter_node
  balance_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
