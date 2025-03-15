cmake_minimum_required(VERSION 3.8)
project(go2_apriltag_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_api REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(talker src/move_robot.cpp)
ament_target_dependencies(talker rclcpp std_msgs geometry_msgs unitree_api)


install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
