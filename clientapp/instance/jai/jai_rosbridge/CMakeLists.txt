cmake_minimum_required(VERSION 3.8)
project(jai_rosbridge)

# ─────────────────────────────────────────────────────────────
# 컴파일 설정
# ─────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # 필요 시 경고 소음 줄이기
  # add_compile_options(-Wno-unused-parameter -Wno-sign-compare -Wno-unknown-pragmas)
endif()

# ─────────────────────────────────────────────────────────────
# 의존성
# ─────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# OpenCV에 HDR 함수가 들어있는 photo 모듈까지 포함
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui photo)

find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Protobuf REQUIRED)
find_package(OpenMP REQUIRED)   # 불필요하면 제거 가능

# ─────────────────────────────────────────────────────────────
# eBUS SDK 경로 (JAI/Pleora)
# ─────────────────────────────────────────────────────────────
set(JAI_LIB_DIR /opt/jai/ebus_sdk/Ubuntu-22.04-x86_64/lib)
set(JAI_INCLUDE_DIR /opt/jai/ebus_sdk/Ubuntu-22.04-x86_64/include)
link_directories(${JAI_LIB_DIR})

# eBUS 라이브러리 묶음
set(JAI_LIBS
  PtConvertersLib
  PvAppUtils
  PvBase
  PvBuffer
  PvGenICam
  PvStream
  PvDevice
  PvTransmitter
  PvVirtualDevice
  PvPersistence
  PvSerial
  PvSystem
  PvCameraBridge
)

# OpenMP 링크 변수 (plain 서명과 호환되게 준비)
set(OPENMP_LIBS "")
if(OpenMP_CXX_FOUND)
  list(APPEND OPENMP_LIBS OpenMP::OpenMP_CXX)
endif()

# ─────────────────────────────────────────────────────────────
# include 경로
# ─────────────────────────────────────────────────────────────
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
)

# ─────────────────────────────────────────────────────────────
# 라이브러리 (.so)
# ─────────────────────────────────────────────────────────────
add_library(jai_node_lib SHARED
  src/ros/AppConfig.cpp
  src/ros/JAINode.cpp
  src/ros/HdrFusion.cpp
  ../src/camera/Camera.cpp
  ../src/acquire/Acquire.cpp
  ../src/device/Device.cpp
  ../src/device/DualDevice.cpp
  ../src/device/ParamManager.cpp
  ../src/logger/Logger.cpp
  ../src/stream/Stream.cpp
  src/ros/JAIHDRNode.cpp
  ../../../../public/proto/cpp/jai.pb.cc
)

target_include_directories(jai_node_lib PUBLIC
  $<INSTALL_INTERFACE:include>
  ${JAI_LIB_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  std_srvs
  cv_bridge
)

# ament 매크로는 내부에서 target_link_libraries의 plain 서명을 사용합니다.
ament_target_dependencies(jai_node_lib ${dependencies})

# 일부 SDK에서 요구하는 매크로/옵션
target_compile_options(jai_node_lib PUBLIC -D_UNIX_ -D_LINUX_ -DQT_GUI_LIB -fPIC)

# ─────────────────────────────────────────────────────────────
# 실행 파일
# ─────────────────────────────────────────────────────────────
add_executable(jai_node src/jai_node.cpp)

target_include_directories(jai_node PUBLIC
  $<INSTALL_INTERFACE:include>
  ${JAI_LIB_INCLUDE_DIRS}
)

ament_target_dependencies(jai_node ${dependencies})
target_compile_features(jai_node PUBLIC c_std_99 cxx_std_17)
target_compile_options(jai_node PUBLIC -D_UNIX_ -D_LINUX_ -DQT_GUI_LIB -fPIC)

# ─────────────────────────────────────────────────────────────
# Action 인터페이스 생성
# ─────────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/HDRTrigger.action"
  DEPENDENCIES builtin_interfaces action_msgs
)

# 생성된 C++ 타입서포트 타깃을 얻어와 링크 (plain 서명 유지)
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ─────────────────────────────────────────────────────────────
# 링크 (★ 모두 plain 서명으로 통일 ★)
# ─────────────────────────────────────────────────────────────
# OpenCV_LIBS에 photo가 포함되지만, 환경에 따라 필요 시 opencv_photo를 명시적으로 추가
target_link_libraries(jai_node_lib
  ${JAI_LIBS}
  ${Protobuf_LIBRARIES}
  ${OpenCV_LIBS}
  ${OPENMP_LIBS}
  ${typesupport_target}
  opencv_photo
)

target_link_libraries(jai_node
  jai_node_lib
  ${JAI_LIBS}
  ${Protobuf_LIBRARIES}
  ${OpenCV_LIBS}
  ${OPENMP_LIBS}
  ${typesupport_target}
  opencv_photo
)

# ─────────────────────────────────────────────────────────────
# 설치 & export
# ─────────────────────────────────────────────────────────────
install(TARGETS jai_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS jai_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)
ament_export_libraries(jai_node_lib)

ament_package()
