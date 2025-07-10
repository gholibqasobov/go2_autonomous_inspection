# Camera's manipulator control

Camera is attached to the 2 DOF manipulator consisting of 2 ST3215 Servos, connected to the [Wavashare General Driver for Robots](https://www.waveshare.com/wiki/General_Driver_for_Robots)  based on ESP32. The manipulator is controlled using ROS2 topic through the USB cable connected to ESP32, that runs micro-ros.

## ESP32 firmware
- Open esp_firmware.ino in Arduino IDE
- In Library manager find micro_ros_arduino library, choose version according to your ROS2 version and install it.
- From the [link](https://files.waveshare.com/upload/7/78/SCServo.rar) download archive with SCServo library. It is the wavashare library for ST3215 servos with General Driver for Robots.
- Unzip the archive into *~/Arduino/libraries*
- From the Board manager choose **ESP32 Dev Module**
- Select your port and upload the code.

## Run the node

```ros2 run camera_servo camera_servo_node```


## Send comands to manipulator through the ROS2 topic

 ```ros2 topic pub /servo_move std_msgs/msg/String '{ "data":"1500 1500"}'```

