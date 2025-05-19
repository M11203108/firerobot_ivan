cmake_minimum_required(VERSION 3.8)
project(motor_base LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找尋相依套件
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_control REQUIRED)
find_package(serial REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 建立硬體介面 shared library
add_library(${PROJECT_NAME} SHARED
    src/motor_set.cpp
    src/zlac8015d.cpp
)


target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "ZLAC8015D_CONTROL_BUILDING_DLL")

ament_target_dependencies(${PROJECT_NAME} 
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    ros2_control
    serial
)

# 註冊 hardware_interface plugin
pluginlib_export_plugin_description_file(hardware_interface motor_base.xml)



# **這裡修正 plugin.xml 安裝路徑**
install(
  FILES motor_base.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/teleop_twist_keyboard_stamped.py
  src/cmd_vel_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)



install(DIRECTORY include/
DESTINATION include/${PROJECT_NAME}
)

# 安裝 launch、config、description 檔案
install(DIRECTORY
launch
config
description
DESTINATION share/${PROJECT_NAME}/
)

add_executable(imu_to_base_frame src/imu_to_base_frame.cpp)
ament_target_dependencies(imu_to_base_frame
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  geometry_msgs
)

install(TARGETS
  imu_to_base_frame
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    ros2_control
    serial
)

ament_package()
