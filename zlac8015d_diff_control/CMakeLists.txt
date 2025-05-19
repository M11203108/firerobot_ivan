cmake_minimum_required(VERSION 3.5)
project(zlac8015d_diff_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找尋所需的套件
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_manager REQUIRED)
find_package(serial REQUIRED PATHS ${CMAKE_INSTALL_PREFIX})


# ======= 建立 motor_set_interface 共享函式庫 (插件) =======
add_library(motor_set_interface SHARED
  src/motor_set.cpp
  src/zlac8015d.cpp
)

target_include_directories(motor_set_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


target_link_libraries(motor_set_interface
  ${serial_LIBRARIES}
)


ament_target_dependencies(motor_set_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial
)

pluginlib_export_plugin_description_file(hardware_interface motor_set_plugin.xml)

install(
  TARGETS motor_set_interface
  EXPORT export_motor_set_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# ======= 安裝標頭檔 (包含 motor_set.h 和 zlac8015d.h) =======
install(
  DIRECTORY include/
  DESTINATION include
)

# ======= 安裝其他檔案 (yaml、launch、urdf、plugin.xml等) =======
install(
  FILES motor_set_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/fire_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

# 匯出插件資訊
ament_export_targets(export_motor_set_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
