cmake_minimum_required(VERSION 3.5)
project(ros2_rs_local_path_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ros2_rs_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros_mercury_interfaces REQUIRED)
find_package(ros2_rs_type_potential_viewer REQUIRED)
find_package(ament_index_cpp REQUIRED)
set(NODES rs_local_path_planner)

add_library(${PROJECT_NAME} SHARED
  src/local_path_planner.cpp
)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs std_srvs ros2_rs_interfaces geometry_msgs
                                          sensor_msgs nav_msgs tf2 rclcpp_components ros_mercury_interfaces
                                          ros2_rs_type_potential_viewer ament_index_cpp)
target_link_libraries(${PROJECT_NAME} rs_navi ${Boost_LIBRARIES})
rclcpp_components_register_nodes(${PROJECT_NAME} "project_ryusei::ROS2LocalPathPlanner")

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY cfg DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()