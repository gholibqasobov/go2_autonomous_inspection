cmake_minimum_required(VERSION 3.8)
project(go2_move)

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

find_package(go2_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)



add_executable(go2_move_server src/go2_move_server.cpp)
ament_target_dependencies(go2_move_server rclcpp std_msgs geometry_msgs unitree_api go2_sport_api sensor_msgs tf2_msgs)


install(TARGETS
  go2_move_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
