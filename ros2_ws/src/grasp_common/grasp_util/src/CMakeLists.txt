add_library(${library_name} SHARED
  string_utils.cpp
  robot_utils.cpp
)

ament_target_dependencies(${library_name}
  rclcpp
  tf2
  tf2_ros
  nav_msgs
  geometry_msgs
  lifecycle_msgs
  rclcpp_lifecycle
  tf2_geometry_msgs
)


install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
