cmake_minimum_required(VERSION 3.8)
project(ugv_controller)

### 1. 종속 라이브러리 찾기 ------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

### 2. 실행 타깃 정의 --------------------------------------------------
add_executable(path_follower_node
  src/path_follower_node.cpp
)

add_executable(manual_path_logger_node
  src/manual_path_logger_node.cpp
)

### 3. 타깃별 의존성 연결 --------------------------------------------
ament_target_dependencies(path_follower_node
  rclcpp geometry_msgs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs
)

ament_target_dependencies(manual_path_logger_node
  rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs
)

### 4. 설치 규칙 -------------------------------------------------------
install(TARGETS
  path_follower_node
  manual_path_logger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES mission1.csv
  DESTINATION share/${PROJECT_NAME})

### 5. 패키지 export ---------------------------------------------------
ament_package()
