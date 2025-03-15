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

set(unitree_api_DIR "/home/$USER/autonomy_stack_go2/install/unitree_api")
find_package(unitree_api REQUIRED)
set(go2_sport_api_DIR "/home/$USER/autonomy_stack_go2/install/go2_sport_api")
find_package(go2_sport_api REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(talker src/move_robot.cpp)
ament_target_dependencies(talker rclcpp std_msgs geometry_msgs unitree_api go2_sport_api)


install(TARGETS
  talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
