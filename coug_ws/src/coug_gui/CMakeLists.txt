### SET CMAKE POLICIES ###
cmake_minimum_required(VERSION 3.10...3.17)

if(${CMAKE_VERSION} VERSION_LESS 3.12)
    cmake_policy(VERSION ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION})
endif()

if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(${CMAKE_VERSION} VERSION_EQUAL "3.11.0" OR ${CMAKE_VERSION} VERSION_GREATER "3.11.0")
  cmake_policy(SET CMP0072 NEW)
endif()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
### END CMAKE POLICIES ###

project(coug_gui
  LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(map_msgs REQUIRED)
find_package(mapviz REQUIRED)
find_package(marti_common_msgs REQUIRED)
find_package(marti_nav_msgs REQUIRED)
find_package(marti_sensor_msgs REQUIRED)
find_package(marti_visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(swri_image_util REQUIRED)
find_package(swri_math_util REQUIRED)
find_package(swri_route_util REQUIRED)
find_package(swri_transform_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

### QT ###
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5OpenGL REQUIRED)
find_package(Qt5Widgets REQUIRED)
add_definitions(-DWFlags=WindowFlags)

find_package(OpenCV COMPONENTS core imgproc REQUIRED)

### OpenGL ###
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

# Need to include the current dir to include the results of building Qt UI files
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(UI_FILES
  ui/coug_waypoints_config.ui)

set(SRC_FILES
  src/coug_waypoints_plugin.cpp)

set(HEADER_FILES
  include/${PROJECT_NAME}/coug_waypoints_plugin.h)

qt5_wrap_ui(UI_SRC_FILES ${UI_FILES})
qt5_wrap_cpp(MOC_FILES ${HEADER_FILES})

add_library(${PROJECT_NAME} SHARED
    ${MOC_FILES}
    ${SRC_FILES}
    ${UI_SRC_FILES}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PACKAGE_NAME}>)

target_compile_definitions(${PROJECT_NAME}
  PUBLIC
    "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# Iron and later switched some cv_bridge files to .hpp from .h
if ("$ENV{ROS_DISTRO}" STRLESS "iron")
  target_compile_definitions(${PROJECT_NAME} PRIVATE "-DUSE_CVBRIDGE_H_FILES")
endif()

target_link_libraries(${PROJECT_NAME} PUBLIC
    ament_index_cpp::ament_index_cpp
    cv_bridge::cv_bridge
    ${GLUT_LIBRARY}
    ${gps_msgs_TARGETS}
    image_transport::image_transport
    ${map_msgs_TARGETS}
    ${mapviz_TARGETS}
    ${marti_common_msgs_TARGETS}
    ${marti_nav_msgs_TARGETS}
    ${marti_sensor_msgs_TARGETS}
    ${marti_visualization_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    opencv_core
    opencv_imgproc
    opencv_highgui
    pluginlib::pluginlib
    Qt5::Core
    Qt5::Gui
    Qt5::OpenGL
    Qt5::Widgets
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    sensor_msgs::sensor_msgs_library
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${stereo_msgs_TARGETS}
    ${swri_image_util_TARGETS}
    ${swri_transform_util_TARGETS}
    ${swri_route_util_TARGETS}
    tf2::tf2
    tf2_ros::tf2_ros
    ${visualization_msgs_TARGETS})
target_include_directories(${PROJECT_NAME} INTERFACE
  ${swri_roscpp_TARGETS}
)

### Install the plugins ###
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h")

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# install(FILES ${CMAKE_CURRENT_BINARY_DIR}/ui_topicselect.h
#   DESTINATION include/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

pluginlib_export_plugin_description_file(mapviz mapviz_plugins.xml)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  cv_bridge
  geometry_msgs
  gps_msgs
  image_transport
  map_msgs
  mapviz
  marti_common_msgs
  marti_sensor_msgs
  marti_visualization_msgs
  nav_msgs
  OpenCV
  Qt5Core
  Qt5Gui
  Qt5OpenGL
  Qt5Widgets
  rclcpp
  sensor_msgs
  std_msgs
  stereo_msgs
  swri_image_util
  swri_math_util
  swri_route_util
  swri_transform_util
  visualization_msgs
  tf2
  tf2_ros)
ament_package()
