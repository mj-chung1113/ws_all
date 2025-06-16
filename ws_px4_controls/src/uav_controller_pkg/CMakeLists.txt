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
# find_package(gazebo_msgs REQUIRED) 
# --- Build Executables ---
add_executable(offboard_control_node src/offboard_control_node.cpp)
ament_target_dependencies(offboard_control_node
    rclcpp
    geometry_msgs
    px4_msgs
    # gazebo_msgs 
)

add_executable(uav_controller src/uav_controller.cpp)
ament_target_dependencies(uav_controller
    rclcpp
    geometry_msgs
    px4_msgs
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
