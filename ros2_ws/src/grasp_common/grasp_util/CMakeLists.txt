cmake_minimum_required(VERSION 3.5)
project(grasp_util)

find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_core REQUIRED)

set(dependencies
    tf2_ros
    tf2
    tf2_geometry_msgs
    geometry_msgs
    nav_msgs
    rclcpp
    lifecycle_msgs
    rclcpp_action
    rclcpp_lifecycle
    action_msgs
    rcl_interfaces
    moveit_msgs
    moveit_common
    moveit_core
)


include_directories(include)

set(library_name ${PROJECT_NAME}_core)
add_subdirectory(src)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()

endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
