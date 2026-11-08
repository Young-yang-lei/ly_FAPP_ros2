cmake_minimum_required(VERSION 3.8)
project(mot_mapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -O3)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(OpenMP QUIET)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(obj_state_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(mapping_node
  src/mapping_node.cpp
  src/mapping_manager.cpp
)

target_include_directories(mapping_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(mapping_node PRIVATE ${PCL_DEFINITIONS})
if(OpenMP_CXX_FOUND)
  target_link_libraries(mapping_node OpenMP::OpenMP_CXX)
endif()
target_link_libraries(mapping_node ${PCL_LIBRARIES})
ament_target_dependencies(mapping_node
  builtin_interfaces
  Eigen3
  message_filters
  nav_msgs
  obj_state_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  visualization_msgs
)

install(TARGETS mapping_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

ament_package()
