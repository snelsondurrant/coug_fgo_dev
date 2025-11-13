cmake_minimum_required(VERSION 3.8)
project(coug_fgo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(GTSAM REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geodesy REQUIRED)

add_executable(factor_graph
  src/factor_graph_node.cpp
  src/utils/conversion_utils.cpp
)

ament_target_dependencies(factor_graph
  rclcpp  
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_include_directories(factor_graph PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${GTSAM_INCLUDE_DIRS}
  /usr/local/include/gtsam/3rdparty/Eigen
)

target_link_libraries(factor_graph
  ${GTSAM_LIBRARIES}
  gtsam
  gtsam_unstable
)

add_executable(navsat_preprocessor
  src/navsat_preprocessor_node.cpp
)

ament_target_dependencies(navsat_preprocessor
  rclcpp
  sensor_msgs
  nav_msgs
  geographic_msgs
  tf2_geometry_msgs
  geodesy
)

target_include_directories(navsat_preprocessor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS
  factor_graph
  navsat_preprocessor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()