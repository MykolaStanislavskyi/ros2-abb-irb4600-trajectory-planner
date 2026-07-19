cmake_minimum_required(VERSION 3.8)
project(block1_stanislavskyi)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(abb_irb4600_ikfast REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ExecuteMachining.srv"
)

add_executable(model_spawner
  src/model_spawner.cpp
)
ament_target_dependencies(model_spawner
  rclcpp
  visualization_msgs
)

add_executable(pose_teacher
  src/pose_teacher.cpp
)
target_include_directories(pose_teacher PRIVATE
  include
)
target_link_libraries(pose_teacher
  Eigen3::Eigen
)
ament_target_dependencies(pose_teacher
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  interactive_markers
  abb_irb4600_ikfast
)

add_executable(manipulator_node
  src/manipulator.cpp
  src/manipulator_node.cpp
)
target_include_directories(manipulator_node PRIVATE
  include
)
target_link_libraries(manipulator_node
  Eigen3::Eigen
)
ament_target_dependencies(manipulator_node
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
  abb_irb4600_ikfast
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  rosidl_typesupport_cpp
)
target_link_libraries(manipulator_node
  "${cpp_typesupport_target}"
)

install(TARGETS
  model_spawner
  pose_teacher
  manipulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()