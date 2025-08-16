cmake_minimum_required(VERSION 3.8)
project(arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
# 头文件
include_directories(include)

# 可执行节点（把两个实现都放进去）
add_executable(armcontrol
  src/main.cpp
  src/control_node.cpp
  src/inverse_resolse.cpp
)

# 链接 & 安装
ament_target_dependencies(armcontrol
  rclcpp
  sensor_msgs
  Eigen3
  rclcpp_components
  geometry_msgs
 )

install(TARGETS armcontrol
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

ament_package()