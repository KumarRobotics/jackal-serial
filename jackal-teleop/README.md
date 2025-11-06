
# Jackal Teleop

The **jackal_teleop** package provides teleoperation for controlling the **Jackal robot**. It includes several components and nodes that interface with the robot’s joystick, RC input, and velocity controller. This package allows for controlling the robot through different input sources (like a joystick or RC controller).

## Installation

1. Clone the repository into your ROS2 workspace

2. Install dependencies:
```bash
cd ~/ws
rosdep install --from-paths src --ignore-src -r -y
```

## Building the Package

1. Build the package:
```bash
cd ~/ws
colcon build --symlink-install
```

2. Source the workspace:
```bash
source ~/ws/install/setup.bash
```

## Running the Package

Launch the `jackal_teleop.launch.py`:
```bash
ros2 launch jackal_teleop jackal_teleop.launch.py
```

## Node Details

### 1. **RcToJoy (RC to Joy Converter)**:
   - Converts RC controller input into **sensor_msgs::Joy** messages.
   - Subscribes to `/mavros/rc/in` and publishes to `/joy`.

### 2. **JetiJoy**:
   - Reads input from a Jeti transmitter and converts it into **sensor_msgs::Joy**.

### 3. **JackalTeleop**:
   - Main control node for the Jackal robot using joystick or commands.# jackal_teleop
