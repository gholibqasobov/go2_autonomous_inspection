cmake_minimum_required(VERSION 3.8)
project(go2_apriltag_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(/home/$ENV{USER}/autonomy_stack_go2/src/utilities/calibrate_imu/include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)


set(unitree_api_DIR "/home/$USER/autonomy_stack_go2/install/unitree_api")
find_package(unitree_api REQUIRED)
set(go2_sport_api_DIR "/home/$USER/autonomy_stack_go2/install/go2_sport_api")
find_package(go2_sport_api REQUIRED)


add_executable(talker src/move_robot.cpp)
ament_target_dependencies(talker rclcpp std_msgs geometry_msgs unitree_api go2_sport_api)

add_executable(apriltag_docking src/apriltag_docking.cpp)
ament_target_dependencies(apriltag_docking rclcpp std_msgs geometry_msgs unitree_api go2_sport_api sensor_msgs)

add_executable(apriltag_pose_sub src/apriltag_tf_subscriber.cpp)
ament_target_dependencies(apriltag_pose_sub rclcpp std_msgs geometry_msgs unitree_api go2_sport_api sensor_msgs tf2_msgs)

add_executable(apriltag_docking_controller src/apriltag_docking_controller.cpp)
ament_target_dependencies(apriltag_docking_controller rclcpp std_msgs geometry_msgs unitree_api go2_sport_api sensor_msgs tf2_msgs tf2)


add_executable(move_robot_with_joy src/move_robot_with_joy.cpp)
ament_target_dependencies(move_robot_with_joy rclcpp std_msgs geometry_msgs unitree_api go2_sport_api sensor_msgs tf2_msgs)


install(TARGETS
  talker
  apriltag_docking
  apriltag_pose_sub
  apriltag_docking_controller
  move_robot_with_joy
  DESTINATION lib/${PROJECT_NAME})

ament_package()
