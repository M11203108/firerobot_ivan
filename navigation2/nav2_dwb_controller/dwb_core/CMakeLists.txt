cmake_minimum_required(VERSION 3.5)
project(dwb_core)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)

nav2_package()

include_directories(
  include
)

set(dependencies
  rclcpp
  std_msgs
  geometry_msgs
  nav_2d_msgs
  dwb_msgs
  nav2_costmap_2d
  pluginlib
  sensor_msgs
  visualization_msgs
  nav_2d_utils
  nav_msgs
  tf2_ros
  nav2_util
  nav2_core
)

add_library(dwb_core SHARED
  src/dwb_local_planner.cpp
  src/publisher.cpp
  src/illegal_trajectory_tracker.cpp
  src/trajectory_utils.cpp
)

ament_target_dependencies(dwb_core
  ${dependencies}
)

install(TARGETS dwb_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# 安裝 local_planner_plugin.xml 到對應的資料夾
install(FILES local_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(dwb_core)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core local_planner_plugin.xml)

ament_package()
