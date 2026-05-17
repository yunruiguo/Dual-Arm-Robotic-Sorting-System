cmake_minimum_required(VERSION 3.5)
project(jaka_controller)
#####################################################################
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
#####################################################################
if(CMAKE_SYSTEM_PROCESSOR MATCHES aarch64)
    message("The system is aarch64")
    set(LIBHV_LIB_DIR aarch64)
else()
    message("The system is x86")
    set(LIBHV_LIB_DIR x86)
endif()
#########################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(grasp_msgs REQUIRED)
find_package(grasp_util REQUIRED)


# Find pthread library
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
################################################################
link_directories(lib/${LIBHV_LIB_DIR})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/3rdparty)

###################################################################
file(GLOB_RECURSE src_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp )
file(GLOB_RECURSE hrd_files  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/jaka_controller/*.h)
add_executable(jaka_controller_node ${src_files} ${hrd_files})


#####################################################################
ament_target_dependencies(jaka_controller_node
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "sensor_msgs"
  "std_srvs"
  "rclcpp_action"
  "grasp_msgs"
  "grasp_util"
  )
target_link_libraries(jaka_controller_node
  jakaAPI
  Threads::Threads
)

#####################################################################

# install(FILES ${CMAKE_SOURCE_DIR}/lib/${LIBHV_LIB_DIR}/libjakaAPI.so DESTINATION lib)

install(TARGETS jaka_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME})

#####################################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
#####################################################################

ament_export_include_directories(include)
ament_export_libraries(jaka_controller_node jakaAPI)
ament_export_dependencies(rclcpp std_msgs sensor_msgs jaka_msgs std_srvs jakaAPI)
ament_package()
