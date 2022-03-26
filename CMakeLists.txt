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
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(DeckLinkAPI REQUIRED)
# # find_package() opencv

# add_library(
#   ${PROJECT_NAME}
#   src/decklink_capture.cpp src/ros_image_pub_thread.cpp
# )

# target_link_libraries(
#   ${PROJECT_NAME}
#   ${DeckLinkAPI_LIBRARIES}
# )













# # Targets
# ## library: decklink_capture + pub thread, has a decklink_capture and binds memcpy to decklink_capture
# ## executable: that runs pub thread

# # Link etc

# # Install, export targets

ament_package()
