cmake_minimum_required(VERSION 3.8)
project(go2_lowlevel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(yaml-cpp REQUIRED)

pkg_check_modules(FFMPEG REQUIRED IMPORTED_TARGET
  libavcodec
  libavutil
  libswscale
)

include_directories(include)

add_executable(lowlevel src/lowlevel.cpp)
add_executable(camera src/camera.cpp)

ament_target_dependencies(
  lowlevel
  rclcpp
  unitree_go
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(
  camera
  rclcpp
  unitree_go
  sensor_msgs
)

target_link_libraries(camera
  PkgConfig::FFMPEG
  yaml-cpp::yaml-cpp
)

install(TARGETS
  lowlevel
  camera
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
