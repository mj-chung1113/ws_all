cmake_minimum_required(VERSION 3.8)
project(uav_controller_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- find dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED) # tf2 핵심 라이브러리
find_package(tf2_ros REQUIRED) # tf2::Buffer, tf2::TransformListener 등 ROS 인터페이스
find_package(tf2_geometry_msgs REQUIRED) # geometry_msgs와 tf2 변환

# find_package(gazebo_msgs REQUIRED) 
# --- Build Executables ---
add_executable(offboard_control_node src/offboard_control_node.cpp)
ament_target_dependencies(offboard_control_node
    rclcpp
    geometry_msgs
    px4_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    # gazebo_msgs 
)

add_executable(uav_controller src/uav_controller.cpp)
ament_target_dependencies(uav_controller
    rclcpp
    geometry_msgs
    px4_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    # gazebo_msgs 
)

# --- Install Targets ---
install(TARGETS
    offboard_control_node
    uav_controller
    DESTINATION lib/${PROJECT_NAME}
)

# --- Test Settings ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
