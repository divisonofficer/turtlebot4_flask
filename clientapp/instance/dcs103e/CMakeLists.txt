cmake_minimum_required(VERSION 3.8)
project(dcs103e_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include directories
include_directories(include)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ControlChannel.srv"
  "srv/SetChannelMode.srv"
  DEPENDENCIES std_msgs
)

# Add executable
add_executable(dcs103e_node src/dcs103e_node.cpp src/dcs103e_controller.cpp)

# Link libraries
target_link_libraries(dcs103e_node ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdcs_100.a)

ament_target_dependencies(dcs103e_node
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
)

# Link service interfaces
rosidl_target_interfaces(dcs103e_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install executable
install(TARGETS
  dcs103e_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()
