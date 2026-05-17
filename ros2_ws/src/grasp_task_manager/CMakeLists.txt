cmake_minimum_required(VERSION 3.8)
project(grasp_task_manager)

# use cpp 17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)
include_directories(kinematics/include)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(grasp_msgs REQUIRED)
find_package(grasp_util REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

FILE(GLOB KINEMATICS_SOURCES kinematics/src/*.cpp)
FILE(GLOB KINEMATICS_HEARS kinematics/include/*.hpp)
set(THIS_PACKAGE_INCLUDE_DEPENDS
  kdl_parser
  behaviortree_cpp_v3
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp src/${PROJECT_NAME}.cpp
    ${KINEMATICS_SOURCES} ${KINEMATICS_HEARS}
)



ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp moveit_ros_planning_interface tf2 
  moveit_core moveit_ros_planning control_msgs grasp_util grasp_msgs tf2_geometry_msgs
  ${THIS_PACKAGE_INCLUDE_DEPENDS})


install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY 
    launch
    config
    urdf
  DESTINATION 
    share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
