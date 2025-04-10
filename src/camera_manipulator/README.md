# Camera's manipulator control

Camera is attached to the 2 DOF manipulator consisting of 2 ST3215 Servos, connected to the [Wavashare General Driver for Robots](https://www.waveshare.com/wiki/General_Driver_for_Robots)  based on ESP32. The manipulator is controlled using ROS2 topic through the USB cable connected to ESP32, that runs micro-ros.

## ESP32 firmware
- Open esp_firmware.ino in Arduino IDE
- In Library manager find micro_ros_arduino library, choose version according to your ROS2 version and install it.
- From the [link](https://files.waveshare.com/upload/7/78/SCServo.rar) download archive with SCServo library. It is the wavashare library for ST3215 servos with General Driver for Robots.
- Unzip the archive into *~/Arduino/libraries*
- From the Board manager choose **ESP32 Dev Module**
- Select your port and upload the code.

## micro_ros_agent installation
- source your ROS2 installation

    `source /opt/ros/$ROS_DISTRO/setup.bash`
- create workspace
    
    `mkdir microros_ws`

    `cd microros_ws`
- clone micro-ros tools

    `git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup`
- Update dependencies using rosdep

    `sudo apt update && rosdep update`

    `rosdep install --from-paths src --ignore-src -y`

- Build micro-ROS tools and source them

    `colcon build`

    `source install/local_setup.bash`

- Download micro-ROS-Agent packages

    `ros2 run micro_ros_setup create_agent_ws.sh`
- Build agent

    `ros2 run micro_ros_setup build_agent.sh`

    `source install/local_setup.bash`

## Send comands to manipulator through the ROS2 topic

- Disconnect manipulator's ESP32 from the PC
- Close Serial monitor and programms that use your port if there are such 
- Move to the workspace 

    `cd microros_ws`

- Source ROS2 setup and micro-ros workspace 

    `source /opt/ros/$ROS_DISTRO/setup.bash`

    `source /opt/ros/$ROS_DISTRO/setup.bash`

- Run micro-ros agent
`    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200`
