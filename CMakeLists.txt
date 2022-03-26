cmake_minimum_required(VERSION 3.5)
project(decklink_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Options
SET(DECKLINK_SDK_DIR CACHE PATH "Path to Blackmagic SDK.")

# Find third party libraries
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(DeckLinkAPI REQUIRED)

# Decklink ROS library
add_library(
  ${PROJECT_NAME}
  src/decklink_capture.cpp src/decklink_ros_pub_thread.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${DeckLinkAPI_LIBRARIES}
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${DeckLinkAPI_INCLUDE_DIRS}
    src
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  image_transport
  std_msgs
  sensor_msgs
  camera_info_manager
  cv_bridge
)

# Executable spinning Decklink ROS node
add_executable(
  ${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)

target_link_libraries(
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}
)

ament_target_dependencies(
  ${PROJECT_NAME}_node
  rclcpp
)

target_include_directories(
  ${PROJECT_NAME}_node
  PRIVATE
    ${DeckLinkAPI_INCLUDE_DIRS}
    src
)

# Export and install
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  image_transport
  std_msgs
  sensor_msgs
  camera_info_manager
  cv_bridge
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
