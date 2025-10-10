# Unitree Go2 Web Interface

Web-based control interface for the Unitree Go2 quadruped robot with real-time map visualization and goal-based navigation.

![Web Interface Preview](https://img.shields.io/badge/Status-Active-success)
![Python](https://img.shields.io/badge/Python-3.10-blue)
![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Iron-blue)

## Features

- =ú **Real-time Map Visualization** - Display navigation maps (PGM format) in your browser
- > **Live Robot Tracking** - Monitor robot position in real-time via ROS2 odometry
- <¯ **Click-to-Navigate** - Set navigation goals by clicking on the map
- = **Dual Operating Modes** - Dummy mode for development, ROS2 mode for real robot
- =á **WebRTC Support** - Connects to Go2 robot over WiFi using WebRTC
- =¥ **Minimal UI** - Clean, responsive interface accessible from any device

## Quick Start

### Prerequisites

- Python 3.10 or higher
- (Optional) ROS2 Humble or Iron for robot connection
- (Optional) `go2_ros2_sdk` for Unitree Go2 WebRTC connection

### Installation

```bash
cd go2_interface

# Create and activate virtual environment (Python 3.10.11)
python3.10 -m venv venv
source venv/bin/activate

# Install Python dependencies
pip install -r requirements.txt
```

### Run in Dummy Mode (Development)

Perfect for frontend development and testing without a robot:

```bash
source venv/bin/activate
python server.py
```

Open browser to **http://localhost:8080**

You can manually adjust the robot position using the API or by modifying the code.

## Connecting to Real Robot (ROS2 Mode)

### Step 1: Setup the Robot

1. **Power on your Unitree Go2 robot**
2. **Connect Go2 to WiFi** (use the mobile app)
3. **Find the robot's IP address**:
   - Open Unitree mobile app
   - Go to: **Device** ’ **Data** ’ **Automatic Machine Inspection**
   - Look for **STA Network: wlan0** - note the IP address
   - Example: `192.168.1.100`

### Step 2: Launch Go2 ROS2 Driver

In **Terminal 1**, start the robot driver with WebRTC connection:

```bash
# Navigate to SDK directory
cd ../go2_ros2_sdk-master

# Source ROS2 environment
source /opt/ros/$ROS_DISTRO/setup.bash

# Build SDK (first time only)
colcon build
source install/setup.bash

# Set robot IP and connection type
export ROBOT_IP="192.168.1.100"  # Replace with your robot's IP
export CONN_TYPE="webrtc"

# Launch the robot driver
ros2 launch go2_robot_sdk webrtc_web.launch.py robot_ip:=$ROBOT_IP
```

**What this does:**
- Establishes WebRTC connection to Go2 robot
- Publishes robot odometry to `/odom` topic
- Subscribes to `/goal_pose` for navigation goals
- Streams camera and lidar data
- Provides robot state and joint information

**Expected output:**
```
[INFO] [go2_driver_node]: Go2 web interface bridge initialized
[INFO] [go2_driver_node]: Subscribed to: odom
[INFO] [go2_driver_node]: Publishing to: /goal_pose, cmd_vel_out
```

### Step 3: Launch Web Interface with ROS2

In **Terminal 2**, start the web interface with ROS2 enabled:

```bash
# Navigate to web interface directory
cd go2_interface

# Source ROS2 and SDK workspaces
source /opt/ros/$ROS_DISTRO/setup.bash
source ../go2_ros2_sdk-master/install/setup.bash

# Enable ROS2 mode
export ENABLE_ROS2=true

# Activate Python virtual environment
source venv/bin/activate

# Start the web server
python server.py
```

**Expected output:**
```
Starting Unitree Go2 Web Interface...
Map base path: /path/to/test_map_ws-main/src/map_provider/maps
Current map: cabinet_unity
Mode: ROS2
ROS2 enabled: True
Server running at http://localhost:8080
```

### Step 4: Access the Interface

1. Open your browser to **http://localhost:8080**
2. You should see:
   - Connection status: **Connected** (green)
   - Real-time robot position updating
   - Map with robot marker

3. **Set a navigation goal**:
   - Click anywhere on the map
   - The robot will receive the goal and start navigating
   - Watch the robot marker move in real-time

### Running with the Autonomy Stack Simulation

When you launch `system_simulation.sh` from `autonomy_stack_go2-foxy-humble`, the simulator publishes odometry on `/state_estimation` and uses `sensor` as the robot base frame. Configure the web interface with matching environment variables before starting `server.py`:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ../autonomy_stack_go2-foxy-humble/install/setup.bash
export ENABLE_ROS2=true
export ODOM_TOPIC=/state_estimation
export ROBOT_FRAME=sensor
export TF_TOPIC=/tf
export TRAJECTORY_TOPIC=/trajectory
export CAMERA_TOPIC=/camera/image/raw
python server.py
```

The bridge now also recognises common base-frame aliases (`base_link`, `base_footprint`, `sensor`, `body`), but setting `ROBOT_FRAME` explicitly keeps the UI aligned with the simulator.

## Configuration

### Environment Variables

| Variable | Description | Default | Example |
|----------|-------------|---------|---------|
| `ENABLE_ROS2` | Enable ROS2 integration | `false` | `true` |
| `ROBOT_NAMESPACE` | Robot namespace for multi-robot | `""` | `robot0` |
| `MAP_FRAME` | Map frame to expect from TF | `map` | `world` |
| `ROBOT_FRAME` | Robot base frame to track | `base` | `sensor` |
| `ODOM_TOPIC` | Odometry topic to subscribe to | `odom` | `/state_estimation` |
| `TF_TOPIC` | TF stream | `/tf` | `/tf` |
| `TRAJECTORY_TOPIC` | Trajectory `PointCloud2` topic | `/trajectory` | `/trajectory` |
| `CAMERA_TOPIC` | Camera image topic | `/camera/image/raw` | `/camera/image/raw` |
| `ROBOT_IP` | Go2 robot IP address | - | `192.168.1.100` |
| `CONN_TYPE` | Connection type | - | `webrtc` |

### Multi-Robot Support

To control multiple robots:

```bash
# Terminal 1: Robot 0
export ROBOT_IP="192.168.1.100"
ros2 launch go2_robot_sdk webrtc_web.launch.py robot_ip:=$ROBOT_IP

# Terminal 2: Web interface for robot 0
export ENABLE_ROS2=true
export ROBOT_NAMESPACE=robot0
python server.py --port 8080

# Terminal 3: Robot 1
export ROBOT_IP="192.168.1.101"
ros2 launch go2_robot_sdk webrtc_web.launch.py robot_ip:=$ROBOT_IP

# Terminal 4: Web interface for robot 1
export ENABLE_ROS2=true
export ROBOT_NAMESPACE=robot1
python server.py --port 8081
```

## API Endpoints

### Map Management

- **GET** `/api/map` - Get current map image and metadata
- **GET** `/api/maps` - List all available maps
- **POST** `/api/maps/<name>` - Switch to a different map

### Robot Position

- **GET** `/api/position` - Get robot position
  - Returns: `{x, y, theta, mode, connected}`
- **POST** `/api/position` - Update position (dummy mode only)
  - Body: `{x, y, theta}`

### Navigation Goals

- **GET** `/api/goal` - Get current goal
- **POST** `/api/goal` - Set navigation goal
  - Body: `{x, y}`
  - In ROS2 mode: publishes PoseStamped to `/goal_pose`

## Troubleshooting

### Robot Not Connecting

**Problem**: Web interface shows "Connected: false"

**Solutions**:
1. Check if `go2_driver_node` is running:
   ```bash
   ros2 node list | grep go2
   ```

2. Verify odometry topic is publishing:
   ```bash
   ros2 topic echo $ODOM_TOPIC --once
   ```

3. Check robot IP is correct:
   ```bash
   ping $ROBOT_IP
   ```

4. Make sure mobile app is closed (only one WebRTC connection allowed)

### Goals Not Working

**Problem**: Clicking map doesn't send robot to goal

**Solutions**:
1. Verify `/goal_pose` topic exists:
   ```bash
   ros2 topic list | grep goal
   ```

2. Check if Nav2 is running (for autonomous navigation):
   ```bash
   ros2 node list | grep nav2
   ```

3. Monitor goal messages:
   ```bash
   ros2 topic echo /goal_pose
   ```

### Map Not Loading

**Problem**: Map shows "Loading..." indefinitely

**Solutions**:
1. Verify map files exist:
   ```bash
   ls ../test_map_ws-main/src/map_provider/maps/
   ```

2. Check Flask console for errors
3. Try switching to a different map in the dropdown

### ROS2 Import Errors

**Problem**: `ModuleNotFoundError: No module named 'rclpy'`

**Solution**: ROS2 packages must be installed system-wide, not via pip:
```bash
# Install ROS2 if not already installed
sudo apt install ros-$ROS_DISTRO-rclpy \
                 ros-$ROS_DISTRO-geometry-msgs \
                 ros-$ROS_DISTRO-nav-msgs

# Source ROS2 before starting server
source /opt/ros/$ROS_DISTRO/setup.bash
```

## Architecture

### Operating Modes

**Dummy Mode**:
- Robot position stored in Flask server
- No ROS2 required
- Good for UI development
- Position updates via API

**ROS2 Mode**:
- Real-time connection to robot
- Subscribes to `/odom` for position
- Publishes to `/goal_pose` for navigation
- Thread-safe ROS2 integration

### Components

```
                 
   Web Browser   
  (http://8080)  
        ,        
          HTTP/WebSocket
         “
                 
  Flask Server   
   (server.py)   
        ,        
         
         “
                 
  ROS2 Bridge    
(ros2_bridge.py) 
        ,        
          ROS2 Topics
         
        4     ,           
    “          “           “
  /odom    /goal_pose  /cmd_vel
                         
              4           
               
               “
                      
      go2_driver_node 
        (WebRTC)      
            ,         
              WiFi
             “
                      
      Unitree Go2     
         Robot        
                      
```

### Coordinate Systems

- **Map Frame**: Meters, origin from YAML, Y-axis up
- **Canvas**: Pixels, origin top-left, Y-axis down
- **Transformations**: `canvasToMap()` and `mapToCanvas()` in `app.js`

## Development

### Adding New Features

1. **Backend**: Modify `server.py` for new API endpoints
2. **ROS2 Integration**: Extend `ros2_bridge.py` for new topics
3. **Frontend**: Update `app.js` for UI logic
4. **Styling**: Modify `style.css` for appearance

### Testing Without Robot

```bash
# Start in dummy mode
python server.py

# In another terminal, test API
curl http://localhost:8080/api/position
curl -X POST http://localhost:8080/api/goal \
  -H "Content-Type: application/json" \
  -d '{"x": 1.0, "y": 2.0}'
```

## Known Limitations

- Single WebRTC connection only (close mobile app before using web interface)
- Goal poses currently don't include orientation (default: facing forward)
- Map changes require manual page refresh
- Coordinate frame must be 'map' (hardcoded)

## Future Enhancements

- [ ] WebSocket for real-time position streaming
- [ ] Path visualization (planned path from Nav2)
- [ ] Obstacle overlay from lidar data
- [ ] Camera feed integration
- [ ] Multiple simultaneous robot control
- [ ] Goal orientation control
- [ ] Waypoint mission planning

## License

This project follows the same license as the parent repository.

## Support

For issues and questions:
1. Check the troubleshooting section above
2. Review logs in the Flask console
3. Check ROS2 topics with `ros2 topic list` and `ros2 topic echo`
4. Consult the [go2_ros2_sdk documentation](https://github.com/abizovnuralem/go2_ros2_sdk)

## Credits

- Built on [go2_ros2_sdk](https://github.com/abizovnuralem/go2_ros2_sdk) by @abizovnuralem
- WebRTC connection based on [go2-webrtc](https://github.com/tfoldi/go2-webrtc) by @tfoldi
- Unitree Go2 robot by Unitree Robotics
