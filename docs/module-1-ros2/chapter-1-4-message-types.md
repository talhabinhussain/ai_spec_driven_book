---
title: Chapter 1.4 - ROS 2 Message Types and Data Structures
description: Understanding standard message types and creating custom messages
---

# Chapter 1.4: ROS 2 Message Types and Data Structures

## Overview

In this chapter, you'll explore ROS 2 message types and data structures, which form the foundation of communication between nodes. You'll learn about standard message types, how to create custom messages, and best practices for data design in robotic systems.

By the end of this chapter, you'll understand:
- ✅ Standard ROS 2 message types and their use cases
- ✅ How to create and use custom message types
- ✅ Best practices for message design and serialization
- ✅ Common data structures used in robotics

## Understanding ROS 2 Messages

### What Are Messages?

Messages in ROS 2 are **structured data containers** that carry information between nodes. They define the format and content of data exchanged through topics, services, and actions. Messages are language-agnostic and use a standardized format that can be serialized and transmitted across the network.

### Message Characteristics

- **Typed**: Each message has a specific structure with defined field types
- **Serializable**: Messages can be converted to bytes for transmission
- **Language-Agnostic**: Same message format works across different programming languages
- **Versioned**: Message definitions include version information for compatibility
- **Extensible**: Custom messages can be created for domain-specific data

### Message Structure

A typical ROS 2 message definition looks like this:

```
# Comment describing the message
string name          # Field with string type
int32 id            # Field with 32-bit integer type
float64 value       # Field with 64-bit float type
bool active         # Field with boolean type
---
# For services, response fields go here
string result
bool success
```

## Standard Message Types

### Basic Data Types

ROS 2 provides a set of basic data types that form the building blocks of more complex messages:

| Type | Description | Range/Format | Use Case |
|------|-------------|--------------|----------|
| `bool` | Boolean value | true/false | Status flags, enable/disable |
| `byte` | 8-bit unsigned integer | 0 to 255 | Raw byte data |
| `char` | 8-bit unsigned integer | 0 to 255 | Character data |
| `float32` | 32-bit floating point | IEEE 754 | Position, sensor values |
| `float64` | 64-bit floating point | IEEE 754 | High-precision values |
| `int8` | 8-bit signed integer | -128 to 127 | Small integers |
| `uint8` | 8-bit unsigned integer | 0 to 255 | Small positive integers |
| `int16` | 16-bit signed integer | -32,768 to 32,767 | Medium integers |
| `uint16` | 16-bit unsigned integer | 0 to 65,535 | Medium positive integers |
| `int32` | 32-bit signed integer | -2³¹ to 2³¹-1 | General integers |
| `uint32` | 32-bit unsigned integer | 0 to 2³²-1 | General positive integers |
| `int64` | 64-bit signed integer | -2⁶³ to 2⁶³-1 | Large integers |
| `uint64` | 64-bit unsigned integer | 0 to 2⁶⁴-1 | Large positive integers |
| `string` | UTF-8 encoded string | Variable length | Text data |
| `wstring` | Wide string | Variable length | Unicode text |

### Standard Message Packages

ROS 2 includes several standard message packages with commonly used message types:

#### 1. std_msgs - Basic Data Types

```python
from std_msgs.msg import String, Int32, Float32, Bool, Header

# String message
string_msg = String()
string_msg.data = "Hello, ROS 2!"

# Integer message
int_msg = Int32()
int_msg.data = 42

# Float message
float_msg = Float32()
float_msg.data = 3.14159

# Boolean message
bool_msg = Bool()
bool_msg.data = True
```

#### 2. geometry_msgs - Geometric Primitives

```python
from geometry_msgs.msg import Point, Vector3, Quaternion, Pose, Twist

# Point in 3D space
point = Point()
point.x = 1.0
point.y = 2.0
point.z = 3.0

# 3D vector
vector = Vector3()
vector.x = 0.5
vector.y = 0.0
vector.z = 1.0

# Quaternion for rotation
quaternion = Quaternion()
quaternion.x = 0.0
quaternion.y = 0.0
quaternion.z = 0.0
quaternion.w = 1.0

# Pose (position + orientation)
pose = Pose()
pose.position = point
pose.orientation = quaternion

# Twist (linear + angular velocity)
twist = Twist()
twist.linear = vector
twist.angular = Vector3(x=0.0, y=0.0, z=0.5)
```

#### 3. sensor_msgs - Sensor Data

```python
from sensor_msgs.msg import Image, LaserScan, JointState, Imu

# Laser scan data
laser_scan = LaserScan()
laser_scan.header.stamp = self.get_clock().now().to_msg()
laser_scan.angle_min = -1.57  # -90 degrees
laser_scan.angle_max = 1.57   # 90 degrees
laser_scan.angle_increment = 0.01
laser_scan.ranges = [1.0, 1.1, 1.2, 1.3]  # Distance readings

# Joint states
joint_state = JointState()
joint_state.name = ['joint1', 'joint2', 'joint3']
joint_state.position = [0.0, 0.5, -0.3]
joint_state.velocity = [0.0, 0.1, -0.1]
joint_state.effort = [0.0, 0.5, 0.3]

# IMU data
imu = Imu()
imu.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
imu.angular_velocity = Vector3(x=0.0, y=0.0, z=0.1)
imu.linear_acceleration = Vector3(x=0.0, y=0.0, z=9.81)
```

#### 4. nav_msgs - Navigation Data

```python
from nav_msgs.msg import Odometry, Path, OccupancyGrid

# Odometry message
odometry = Odometry()
odometry.header.stamp = self.get_clock().now().to_msg()
odometry.header.frame_id = "odom"
odometry.child_frame_id = "base_link"
odometry.pose.pose = Pose()  # Position and orientation
odometry.twist.twist = Twist()  # Velocity

# Path message
path = Path()
path.header.stamp = self.get_clock().now().to_msg()
path.header.frame_id = "map"
path.poses = [PoseStamped(), PoseStamped()]  # List of poses
```

## Creating Custom Messages

### Setting Up Message Directories

To create custom messages, you need to set up the proper directory structure in your package:

```
my_robot_pkg/
├── msg/
│   ├── CustomSensor.msg
│   ├── RobotStatus.msg
│   └── Command.msg
├── srv/
│   ├── ProcessData.srv
│   └── ControlRobot.srv
├── action/
│   └── MoveArm.action
└── CMakeLists.txt  # or setup.py for Python packages
```

### Creating a Custom Message Definition

Let's create a custom message for a robot status report. Create `msg/RobotStatus.msg`:

```
# RobotStatus.msg
# Message to report the status of a robot

# Standard header
std_msgs/Header header

# Robot identification
string robot_id
int32 robot_type

# Power status
float32 battery_level
bool charging
float32 power_consumption

# Operational status
bool operational
bool emergency_stop
string status_message

# Position and orientation
geometry_msgs/Pose current_pose

# Joint states
sensor_msgs/JoseState joint_states

# Timestamps
builtin_interfaces/Time last_update
builtin_interfaces/Time next_maintenance
```

### Building Custom Messages

Update your `package.xml` to include the message generation dependency:

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```

Update your `CMakeLists.txt` to generate the message files:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "msg/CustomSensor.msg"
  "msg/Command.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs
)
```

For Python packages using `setup.py`, you need to use `ament_python` build type:

```python
from setuptools import setup
from glob import glob
import os

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    # ... other setup parameters
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Package with custom messages',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
        ],
    },
)
```

### Using Custom Messages in Code

Once built, you can use custom messages in your Python nodes:

```python
#!/usr/bin/env python3
"""
Node demonstrating custom message usage.
"""

import rclpy
from rclpy.node import Node
from my_robot_pkg.msg import RobotStatus  # Import custom message
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState


class CustomMessageNode(Node):
    def __init__(self):
        super().__init__('custom_message_node')

        # Publisher for custom message
        self.publisher = self.create_publisher(RobotStatus, 'robot_status', 10)

        # Timer to publish status updates
        self.timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('Custom Message Node initialized!')

    def publish_status(self):
        """Publish robot status using custom message."""
        msg = RobotStatus()

        # Fill header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        # Fill robot identification
        msg.robot_id = 'robot_001'
        msg.robot_type = 1  # Type 1: mobile manipulator

        # Fill power status
        msg.battery_level = 85.5
        msg.charging = False
        msg.power_consumption = 45.2

        # Fill operational status
        msg.operational = True
        msg.emergency_stop = False
        msg.status_message = 'Operational'

        # Fill pose
        msg.current_pose.position.x = 1.0
        msg.current_pose.position.y = 2.0
        msg.current_pose.position.z = 0.0
        msg.current_pose.orientation.w = 1.0

        # Fill joint states
        msg.joint_states.name = ['joint1', 'joint2', 'joint3']
        msg.joint_states.position = [0.0, 0.5, -0.3]

        # Fill timestamps
        msg.last_update = self.get_clock().now().to_msg()
        msg.next_maintenance = self.get_clock().now().to_msg()

        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published robot status: {msg.robot_id}')


def main(args=None):
    rclpy.init(args=args)
    node = CustomMessageNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Advanced Custom Message Example

Here's a more complex custom message for a sensor fusion system:

Create `msg/SensorFusion.msg`:

```
# SensorFusion.msg
# Message for fused sensor data

std_msgs/Header header

# Fused position estimate
geometry_msgs/Point position
geometry_msgs/Quaternion orientation

# Covariance matrices
float64[36] position_covariance  # 6x6 covariance matrix for position
float64[36] orientation_covariance  # 6x6 covariance matrix for orientation

# Sensor contributions
sensor_msgs/Imu imu_data
sensor_msgs/LaserScan laser_scan
geometry_msgs/PoseStamped visual_pose

# Confidence scores
float32 position_confidence
float32 orientation_confidence
float32 overall_confidence

# Sensor status
bool imu_active
bool laser_active
bool camera_active
```

## Message Serialization and Performance

### Serialization Overview

Messages in ROS 2 use a serialization system that converts structured data to bytes for transmission and back to structured data for processing. The default serialization format is **ROS Interface Definition Language (ROS IDL)** with **Connext Dynamic Types (CDR)** encoding.

### Performance Considerations

```python
class EfficientMessageNode(Node):
    def __init__(self):
        super().__init__('efficient_message_node')

        # Pre-allocate message objects to reduce allocation overhead
        self.msg = RobotStatus()
        self.publisher = self.create_publisher(RobotStatus, 'robot_status', 10)

        # Use efficient data structures
        self.joint_positions = [0.0] * 10  # Pre-allocated list

        self.timer = self.create_timer(0.1, self.publish_efficiently)

    def publish_efficiently(self):
        """Efficiently update and publish message."""
        # Update only the data that changes
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.battery_level = self.get_battery_level()

        # Update pre-allocated arrays
        for i in range(len(self.joint_positions)):
            self.joint_positions[i] += 0.01  # Small increment
        self.msg.joint_states.position = self.joint_positions

        self.publisher.publish(self.msg)

    def get_battery_level(self):
        """Simulate battery level reading."""
        import random
        return 80.0 + random.uniform(-5.0, 5.0)
```

### Message Size Optimization

```python
# Good: Efficient message design
class EfficientStatus:
    def __init__(self):
        self.timestamp = 0  # Use int64 for timestamps instead of full time message
        self.battery = 0.0  # Use float32 instead of float64 if precision allows
        self.status = 0     # Use int8 for status codes instead of strings
        self.position = [0.0, 0.0, 0.0]  # Use array instead of separate x,y,z

# Bad: Inefficient message design
class InefficientStatus:
    def __init__(self):
        self.timestamp = builtin_interfaces.msg.Time()  # Full time message
        self.battery = 0.0  # float64 by default
        self.status = "operational"  # String instead of enum
        self.pos_x = 0.0    # Separate fields
        self.pos_y = 0.0
        self.pos_z = 0.0
```

### Dynamic Message Creation

For scenarios where you need to create messages dynamically:

```python
import struct

class DynamicMessageNode(Node):
    def __init__(self):
        super().__init__('dynamic_message_node')
        self.publisher = self.create_publisher(String, 'dynamic_data', 10)

    def create_binary_message(self, data_list):
        """Create a binary message from a list of values."""
        # Pack data into binary format
        binary_data = struct.pack(f'{len(data_list)}d', *data_list)

        # Create string message with binary data
        msg = String()
        msg.data = binary_data.hex()  # Convert to hex string for transmission
        return msg

    def publish_binary_data(self):
        """Publish binary sensor data efficiently."""
        sensor_values = [1.0, 2.0, 3.0, 4.0, 5.0]
        msg = self.create_binary_message(sensor_values)
        self.publisher.publish(msg)
```

## Common Message Patterns

### 1. Header Pattern

Most messages should include a header for timestamp and frame information:

```
# Recommended header pattern
std_msgs/Header header
---
# Message content follows
```

```python
# In Python code
from std_msgs.msg import Header

def create_templated_message(node):
    """Create a message with proper header."""
    msg = CustomMessage()
    msg.header = Header()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'base_link'  # Coordinate frame
    return msg
```

### 2. Array Pattern

For variable-length data:

```
# SensorArray.msg
std_msgs/Header header
string sensor_type
float64[] values
string[] sensor_names
bool[] sensor_active
```

### 3. Nested Message Pattern

For complex hierarchical data:

```
# RobotSystem.msg
std_msgs/Header header
PowerSystem power
NavigationSystem navigation
ManipulationSystem manipulation

# Separate message files for each subsystem
```

### 4. Event Pattern

For state change notifications:

```
# Event.msg
std_msgs/Header header
string event_type
string event_description
builtin_interfaces/Time event_time
string source_component
int32 event_code
```

## Message Validation and Error Handling

### Message Validation

```python
class ValidatedMessageNode(Node):
    def __init__(self):
        super().__init__('validated_message_node')
        self.publisher = self.create_publisher(RobotStatus, 'robot_status', 10)

    def validate_and_publish(self, status_msg):
        """Validate message before publishing."""
        errors = []

        # Validate battery level range
        if not (0.0 <= status_msg.battery_level <= 100.0):
            errors.append(f'Battery level out of range: {status_msg.battery_level}')

        # Validate robot ID format
        if not status_msg.robot_id or len(status_msg.robot_id) == 0:
            errors.append('Robot ID cannot be empty')

        # Validate pose
        if (abs(status_msg.current_pose.orientation.x) > 1.0 or
            abs(status_msg.current_pose.orientation.y) > 1.0 or
            abs(status_msg.current_pose.orientation.z) > 1.0 or
            abs(status_msg.current_pose.orientation.w) > 1.0):
            errors.append('Quaternion values out of range')

        if errors:
            self.get_logger().error(f'Message validation failed: {", ".join(errors)}')
            return False

        # Publish if validation passed
        self.publisher.publish(status_msg)
        return True
```

### Error Handling in Message Processing

```python
class RobustMessageProcessor(Node):
    def __init__(self):
        super().__init__('robust_message_processor')
        self.subscription = self.create_subscription(
            RobotStatus, 'robot_status', self.process_status, 10
        )

    def process_status(self, msg):
        """Process status message with error handling."""
        try:
            # Validate message structure
            if not hasattr(msg, 'header'):
                self.get_logger().error('Message missing header')
                return

            # Process battery status
            if hasattr(msg, 'battery_level'):
                if 0 <= msg.battery_level <= 20:
                    self.get_logger().warn(f'Low battery: {msg.battery_level}%')
                elif msg.battery_level < 0 or msg.battery_level > 100:
                    self.get_logger().error(f'Invalid battery level: {msg.battery_level}')
                    return

            # Process pose
            if hasattr(msg, 'current_pose'):
                self.validate_pose(msg.current_pose)

            # Process joint states
            if hasattr(msg, 'joint_states'):
                self.validate_joint_states(msg.joint_states)

            self.get_logger().info(f'Processed status for {msg.robot_id}')

        except AttributeError as e:
            self.get_logger().error(f'Missing attribute in message: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')

    def validate_pose(self, pose):
        """Validate pose message."""
        # Check quaternion normalization
        norm = (pose.orientation.x**2 + pose.orientation.y**2 +
                pose.orientation.z**2 + pose.orientation.w**2)**0.5
        if abs(norm - 1.0) > 0.01:
            self.get_logger().warn('Quaternion not normalized')

    def validate_joint_states(self, joint_states):
        """Validate joint states message."""
        if len(joint_states.name) != len(joint_states.position):
            self.get_logger().error('Joint name and position arrays have different lengths')
```

## Message Design Best Practices

### 1. Naming Conventions

- Use **PascalCase** for message type names: `RobotStatus`, `SensorData`
- Use **snake_case** for field names: `robot_id`, `battery_level`
- Use descriptive names that clearly indicate purpose
- Avoid abbreviations unless they're widely understood

### 2. Data Type Selection

```python
# Good: Appropriate data types
float32 position_x      # For position with reasonable precision
int8 robot_state        # For discrete states
bool emergency_stop     # For binary flags

# Avoid: Overly large types when not needed
float64 position_x      # Unless high precision is required
int32 robot_state       # For 3 possible states
string emergency_stop   # Instead of bool
```

### 3. Message Size Considerations

- Keep messages as small as possible while maintaining clarity
- Use arrays instead of repeated individual fields
- Consider the frequency of message transmission
- Use appropriate precision (float32 vs float64)

### 4. Forward Compatibility

```python
# Good: Design for future extensions
# Add comments for future fields
# Use arrays for expandable data
# Keep optional fields separate

# Example of extensible message
class ExtensibleStatus:
    """
    Status message designed for future expansion:
    - Use arrays for sensor data (can grow)
    - Keep optional components separate
    - Document intended future additions
    """
    def __init__(self):
        self.core_status = {}  # Dictionary for flexible core data
        self.sensor_data = []  # Array for expandable sensor readings
        self.extensions = {}   # Dictionary for optional extensions
```

## Working with Message Dependencies

### Message Dependencies in package.xml

```xml
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
<depend>builtin_interfaces</depend>
```

### Message Generation Dependencies in CMakeLists.txt

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomMessage.msg"
  "srv/CustomService.srv"
  "action/CustomAction.action"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
)
```

## Message Inspection and Debugging

### Using Command-Line Tools

```bash
# Show message information
ros2 interface show std_msgs/msg/String

# Show service information
ros2 interface show std_srvs/srv/SetBool

# Show action information
ros2 interface show nav2_msgs/action/NavigateToPose

# Echo messages with detailed information
ros2 topic echo /robot_status --field data

# Check message types on a topic
ros2 topic info /robot_status
```

### Programmatic Message Inspection

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import rosidl_runtime_py.utilities
import rclpy.serialization


class MessageInspectorNode(Node):
    def __init__(self):
        super().__init__('message_inspector_node')

        # Subscribe to inspect messages
        self.subscription = self.create_subscription(
            String, 'inspect_topic', self.inspect_message, 10
        )

    def inspect_message(self, msg):
        """Inspect message structure and content."""
        # Get message type
        msg_type = type(msg)
        self.get_logger().info(f'Message type: {msg_type}')

        # Get message fields
        fields_and_types = rosidl_runtime_py.utilities.get_message_fields_and_types(msg)
        self.get_logger().info(f'Message fields: {fields_and_types}')

        # Serialize message to bytes
        serialized = rclpy.serialization.serialize_message(msg)
        self.get_logger().info(f'Serialized size: {len(serialized)} bytes')

        # Print message content
        self.get_logger().info(f'Message content: {msg.data}')

    def get_message_info(self, msg_type):
        """Get detailed information about a message type."""
        # Get field names and types
        fields = rosidl_runtime_py.utilities.get_message_fields_and_types(msg_type())
        return fields
```

## Message Performance Optimization

### 1. Memory Pool for Messages

```python
class MessagePoolNode(Node):
    def __init__(self):
        super().__init__('message_pool_node')

        # Create a pool of pre-allocated messages
        self.message_pool = []
        self.active_messages = []

        # Pre-allocate messages
        for _ in range(10):
            msg = RobotStatus()
            self.message_pool.append(msg)

        self.publisher = self.create_publisher(RobotStatus, 'robot_status', 10)

    def get_message_from_pool(self):
        """Get a message from the pool or create a new one."""
        if self.message_pool:
            msg = self.message_pool.pop()
            self.active_messages.append(msg)
            return msg
        else:
            # Fallback to creating new message
            msg = RobotStatus()
            self.active_messages.append(msg)
            return msg

    def return_message_to_pool(self, msg):
        """Return a message to the pool after use."""
        if msg in self.active_messages:
            self.active_messages.remove(msg)
            self.message_pool.append(msg)
```

### 2. Batch Message Processing

```python
class BatchMessageNode(Node):
    def __init__(self):
        super().__init__('batch_message_node')

        self.message_buffer = []
        self.buffer_size = 5
        self.publisher = self.create_publisher(RobotStatus, 'robot_status', 10)

    def add_to_buffer(self, msg):
        """Add message to buffer for batch processing."""
        self.message_buffer.append(msg)

        if len(self.message_buffer) >= self.buffer_size:
            self.publish_batch()

    def publish_batch(self):
        """Publish all buffered messages."""
        for msg in self.message_buffer:
            self.publisher.publish(msg)

        self.message_buffer.clear()
```

## Message Versioning and Compatibility

### Version Considerations

When designing messages, consider future compatibility:

1. **Add new fields at the end** of the message definition
2. **Provide default values** for new fields
3. **Avoid removing fields** in minor version updates
4. **Use optional fields** for non-critical data

### Example of Version-Compatible Message Evolution

**Original Message (v1.0):**
```
# RobotStatus.msg (v1.0)
std_msgs/Header header
string robot_id
float32 battery_level
bool operational
```

**Extended Message (v1.1):**
```
# RobotStatus.msg (v1.1) - backward compatible
std_msgs/Header header
string robot_id
float32 battery_level
bool operational
# New fields added at the end
float32 temperature  # Added in v1.1, defaults to 0.0
string status_detail  # Added in v1.1, defaults to ""
```

## Summary

In this chapter, you learned:

- ✅ Standard ROS 2 message types and their appropriate use cases
- ✅ How to create and use custom message types
- ✅ Best practices for message design and performance optimization
- ✅ Message validation, error handling, and debugging techniques
- ✅ How to work with message dependencies and versioning

## Next Steps

Now that you understand message types and data structures, you're ready to learn about parameter management in ROS 2!

**Continue to:** [Chapter 1.5: ROS 2 Parameters and Configuration →](chapter-1-5-parameters)

## Additional Resources

- [ROS 2 Message Definition Guide](https://docs.ros.org/en/humble/How-To-Guides/Defining-custom-interfaces.html)
- [Standard Message Definitions](https://docs.ros.org/en/humble/Packages/StdMsgs.html)
- [Message Generation Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)
- [ROS 2 Interface Definition Language](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html)