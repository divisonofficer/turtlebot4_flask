cmake_minimum_required(VERSION 3.8)
project(jai_rosbridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include(FindProtobuf)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
# ros action generator
find_package(rosidl_default_generators REQUIRED)


find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Protobuf REQUIRED)
# OpenCV 찾기
find_package(OpenCV REQUIRED)

# OpenMP 찾기
find_package(OpenMP REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)




set(JAI_LIB_DIR /opt/jai/ebus_sdk/Ubuntu-22.04-x86_64/lib)
link_directories(${JAI_LIB_DIR})

add_executable(jai_node src/jai_node.cpp)

add_library(jai_node_lib src/ros/AppConfig.cpp src/ros/JAINode.cpp src/ros/HdrFusion.cpp ../src/camera/Camera.cpp ../src/acquire/Acquire.cpp ../src/device/Device.cpp
../src/device/DualDevice.cpp ../src/device/ParamManager.cpp
../src/logger/Logger.cpp ../src/stream/Stream.cpp
src/ros/JAIHDRNode.cpp
../../../../public/proto/cpp/jai.pb.cc
)


rosidl_generate_interfaces(${PROJECT_NAME}
  "action/HDRTrigger.action"
  DEPENDENCIES builtin_interfaces action_msgs
)


set(JAI_INCLUDE_DIR /opt/jai/ebus_sdk/Ubuntu-22.04-x86_64/include)

set(JAI_LIBS PtConvertersLib PvAppUtils PvBase PvBuffer PvGenICam PvStream PvDevice PvTransmitter PvVirtualDevice PvPersistence PvSerial PvSystem PvCameraBridge)
target_link_libraries(jai_node
  jai_node_lib
  ${JAI_LIBS}
  ${Protobuf_LIBRARIES}
  )
set(dependencies
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  std_srvs
  cv_bridge
  OpenCV
  OpenMP
)

ament_target_dependencies(jai_node_lib ${dependencies})
ament_target_dependencies(jai_node ${dependencies})
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(jai_node ${typesupport_target})
target_link_libraries(jai_node_lib ${typesupport_target})

set(JAI_LIB_INCLUDE_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../src/camera>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../src/acquire>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../src/device>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../src/logger>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../src/stream>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/ros>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../../../../public/proto/cpp>
  
  $<BUILD_INTERFACE:${JAI_INCLUDE_DIR}>
  >
)

target_include_directories(jai_node_lib PUBLIC
$<INSTALL_INTERFACE:include>
${JAI_LIB_INCLUDE_DIRS}
)



target_include_directories(jai_node PUBLIC
$<INSTALL_INTERFACE:include>
${JAI_LIB_INCLUDE_DIRS}

  )

  target_compile_options(jai_node_lib PUBLIC -D_UNIX_ -D_LINUX_ -DQT_GUI_LIB -fPIC -std=c++17)

target_compile_features(jai_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(jai_node PUBLIC -D_UNIX_ -D_LINUX_ -DQT_GUI_LIB -fPIC -std=c++17)

install(TARGETS jai_node
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)


ament_package()
