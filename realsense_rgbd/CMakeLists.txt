cmake_minimum_required(VERSION 3.5)
project(realsense_rgbd)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(librealsense2 REQUIRED)
find_package(realsense_camera_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

add_library(realsense_camera_component SHARED src/realsense_camera_component.cpp)
target_compile_definitions(realsense_camera_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(realsense_camera_component
  rclcpp  
  image_transport
  librealsense2
  cv_bridge
  realsense_camera_msgs
  std_msgs
  sensor_msgs
  tf2
  tf2_ros)
rclcpp_components_register_nodes(realsense_camera_component "realsense_rgbd::RealSenseCameraComponent")

add_executable(realsense_demo_executor
  src/realsense_demo_executor.cpp)
target_link_libraries(realsense_demo_executor
  realsense_camera_component)
ament_target_dependencies(realsense_demo_executor
  rclcpp
)

install(TARGETS
  realsense_camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(TARGETS 
  realsense_demo_executor
  DESTINATION lib/${PROJECT_NAME})

ament_package()