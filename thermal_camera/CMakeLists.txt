cmake_minimum_required(VERSION 3.5)
project(thermal_camera)

# 設定 C++ 標準
set(CMAKE_CXX_STANDARD 14)

# 添加 include 目錄
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/include/thermal_camera
)

# 找到 ROS2 依賴
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


# 查找系統庫
find_library(UDEV_LIB udev REQUIRED)
find_library(PTHREAD_LIB pthread REQUIRED)
find_library(M_LIB m REQUIRED)

# 設定執行檔
add_executable(thermal_camera_node src/ir_camera_node.cpp)

# 添加相依性
ament_target_dependencies(thermal_camera_node rclcpp sensor_msgs cv_bridge OpenCV image_transport)

# 設定庫目錄
link_directories(${PROJECT_SOURCE_DIR}/lib)

# 連結 GuideUSB2LiveStream 庫以及其他必要庫
target_link_libraries(thermal_camera_node
  GuideUSB2LiveStream
  ${UDEV_LIB}
  ${PTHREAD_LIB}
  ${M_LIB}
  ${OpenCV_LIBS}
)

# 安裝
install(TARGETS thermal_camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()

# 加入 Python 支援
install(PROGRAMS
  scripts/fire_detection.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  scripts/best.pt
  DESTINATION lib/${PROJECT_NAME}
)