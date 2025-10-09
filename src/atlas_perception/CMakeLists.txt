cmake_minimum_required(VERSION 3.8)
project(atlas_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FindShape.action"
  DEPENDENCIES geometry_msgs
)

# Include directories
include_directories(include)

# Shape detector node
add_executable(shape_detector_node src/shape_detector_node.cpp)
ament_target_dependencies(shape_detector_node
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  visualization_msgs
  image_transport
  cv_bridge
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(shape_detector_node ${OpenCV_LIBS})

# Make sure the action is generated before building the node
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(shape_detector_node "${cpp_typesupport_target}")

# Install targets
install(TARGETS
  shape_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()