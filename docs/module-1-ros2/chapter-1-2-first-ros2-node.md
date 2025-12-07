---
title: Chapter 1.2 - Building Your First ROS 2 Node
description: Create and run your first ROS 2 node with Python and rclpy
---

# Chapter 1.2: Building Your First ROS 2 Node

## Overview

In this chapter, you'll build your first ROS 2 node using Python and the `rclpy` client library. This hands-on experience will solidify your understanding of ROS 2 architecture by creating a simple publisher node that broadcasts messages to other nodes in the system.

By the end of this chapter, you'll understand:
- ✅ How to create a basic ROS 2 Python package
- ✅ The structure of a ROS 2 node with publishers
- ✅ How to run and test your first node
- ✅ Best practices for node development

## Prerequisites

Before starting, ensure you have:

1. **ROS 2 Humble Hawksbill** installed (or your preferred ROS 2 distribution)
2. **Python 3.8+** environment
3. **Basic Python programming** knowledge
4. **Terminal/command line** familiarity

## Setting Up Your Workspace

### Creating a ROS 2 Workspace

First, create a new workspace for your ROS 2 packages:

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash  # Replace 'humble' with your distribution

# Build the workspace (even though it's empty)
colcon build
source install/setup.bash
```

### Creating Your First Package

Create a new ROS 2 package called `my_robot_pkg`:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_pkg
```

This creates a basic package structure. Navigate to the package directory:

```bash
cd my_robot_pkg
ls -la
```

You should see this structure:

```
my_robot_pkg/
├── my_robot_pkg/           # Python package directory
├── resource/              # Resource marker file
├── test/                  # Test directory
├── package.xml           # Package manifest
├── setup.cfg             # Python setup configuration
├── setup.py              # Python package setup
└── README.md             # Documentation file
```

## Understanding the Package Structure

### Python Package Directory

The `my_robot_pkg/my_robot_pkg/` directory contains your Python modules. This is where you'll add your node code:

```
my_robot_pkg/my_robot_pkg/
├── __init__.py           # Makes it a Python package
└── my_first_node.py      # Your first ROS 2 node
```

### Package Manifest (package.xml)

This XML file describes your package and its dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_pkg</name>
  <version>0.0.0</version>
  <description>First ROS 2 Python package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Setup Configuration

The `setup.py` file defines how your package should be built:

```python
from setuptools import find_packages, setup

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='First ROS 2 Python package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_first_node = my_robot_pkg.my_first_node:main',
        ],
    },
)
```

## Creating Your First Node

### Creating the Node File

Create your first node file in `my_robot_pkg/my_robot_pkg/my_first_node.py`:

```python
#!/usr/bin/env python3
"""
Simple publisher node that broadcasts messages.
This node demonstrates the basic structure of a ROS 2 Python node.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MyFirstNode(Node):
    """
    A simple publisher node that sends messages to a topic.
    """

    def __init__(self):
        # Initialize the node with the name 'my_first_node'
        super().__init__('my_first_node')

        # Create a publisher for String messages on the 'my_topic' topic
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)

        # Create a timer to publish messages every 0.5 seconds (2 Hz)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to track published messages
        self.i = 0

        # Log a startup message
        self.get_logger().info('MyFirstNode initialized and ready to publish!')

    def timer_callback(self):
        """Callback function called by the timer to publish messages."""
        # Create a String message
        msg = String()
        msg.data = f'Hello World: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1


def main(args=None):
    """Main function to run the node."""
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the node
    my_first_node = MyFirstNode()

    try:
        # Start spinning to process callbacks
        rclpy.spin(my_first_node)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        pass
    finally:
        # Clean up
        my_first_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Adding the Executable Permission

Make your node file executable:

```bash
chmod +x my_robot_pkg/my_robot_pkg/my_first_node.py
```

## Building Your Package

Before running your node, you need to build your package:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_pkg
source install/setup.bash
```

## Running Your First Node

### Running the Node Directly

Start your node:

```bash
ros2 run my_robot_pkg my_first_node
```

You should see output like:

```
[INFO] [1699123456.789012]: MyFirstNode initialized and ready to publish!
[INFO] [1699123457.289012]: Publishing: "Hello World: 0"
[INFO] [1699123457.789012]: Publishing: "Hello World: 1"
[INFO] [1699123458.289012]: Publishing: "Hello World: 2"
```

### Verifying the Node is Running

Open a new terminal and check that your node is running:

```bash
# List running nodes
ros2 node list

# You should see:
# /my_first_node

# List topics
ros2 topic list

# You should see:
# /my_topic
# /parameter_events
# /rosout

# Check topic info
ros2 topic info /my_topic

# You should see:
# Type: std_msgs/msg/String
# Publisher count: 1
# Subscription count: 0
```

### Listening to the Topic

In another terminal, listen to the messages being published:

```bash
ros2 topic echo /my_topic

# You should see messages like:
# data: 'Hello World: 0'
# ---
# data: 'Hello World: 1'
# ---
# data: 'Hello World: 2'
# ---
```

## Enhanced Node with Subscriptions

Now let's create a more sophisticated node that both publishes and subscribes to messages. Create `my_robot_pkg/my_robot_pkg/two_way_node.py`:

```python
#!/usr/bin/env python3
"""
Node that demonstrates both publishing and subscribing to topics.
This shows how nodes can communicate bidirectionally.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TwoWayNode(Node):
    """
    A node that both publishes and subscribes to topics.
    """

    def __init__(self):
        super().__init__('two_way_node')

        # Publisher for outgoing messages
        self.publisher_ = self.create_publisher(String, 'outgoing_msg', 10)

        # Subscriber for incoming messages
        self.subscription = self.create_subscription(
            String,
            'incoming_msg',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # Timer for publishing messages
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0
        self.get_logger().info('TwoWayNode initialized with publisher and subscriber!')

    def timer_callback(self):
        """Publish outgoing messages."""
        msg = String()
        msg.data = f'Outgoing message #{self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

    def listener_callback(self, msg):
        """Callback for incoming messages."""
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    """Main function to run the two-way node."""
    rclpy.init(args=args)
    two_way_node = TwoWayNode()

    try:
        rclpy.spin(two_way_node)
    except KeyboardInterrupt:
        pass
    finally:
        two_way_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Update your `setup.py` to add the new entry point:

```python
entry_points={
    'console_scripts': [
        'my_first_node = my_robot_pkg.my_first_node:main',
        'two_way_node = my_robot_pkg.two_way_node:main',
    ],
},
```

## Running Multiple Nodes

### Terminal 1 - Start the Two-Way Node:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_robot_pkg two_way_node
```

### Terminal 2 - Start a Simple Publisher to Send Messages:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 topic pub /incoming_msg std_msgs/msg/String "data: 'Hello from external publisher!'"
```

### Terminal 3 - Listen to Outgoing Messages:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 topic echo /outgoing_msg
```

## Understanding Node Structure

### Key Components of a ROS 2 Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import required message types

class MyNode(Node):
    def __init__(self):
        # 1. Initialize the node with a name
        super().__init__('node_name')

        # 2. Create publishers, subscribers, services, etc.
        self.publisher_ = self.create_publisher(String, 'topic_name', 10)

        # 3. Create timers for periodic tasks
        self.timer = self.create_timer(0.5, self.timer_callback)

        # 4. Log startup information
        self.get_logger().info('Node initialized!')

    def timer_callback(self):
        # 5. Callback function for timer
        msg = String()
        msg.data = 'Message content'
        self.publisher_.publish(msg)

def main(args=None):
    # 6. Main function that initializes and runs the node
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### Node Naming and Uniqueness

ROS 2 ensures node names are unique. If you try to run multiple nodes with the same name, the second one will fail:

```bash
# Terminal 1
ros2 run my_robot_pkg my_first_node

# Terminal 2 - This will cause a name collision
ros2 run my_robot_pkg my_first_node  # Will fail with name collision error
```

To run multiple instances, use unique names:

```bash
# Terminal 1
ros2 run my_robot_pkg my_first_node --ros-args --remap __node:=node_instance_1

# Terminal 2 - This works now
ros2 run my_robot_pkg my_first_node --ros-args --remap __node:=node_instance_2
```

## Common Node Patterns

### 1. Publisher-Only Node

```python
class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        self.publisher_ = self.create_publisher(String, 'sensor_data', 10)
        self.timer = self.create_timer(0.1, self.publish_sensor_data)

    def publish_sensor_data(self):
        # Simulate reading sensor data
        msg = String()
        msg.data = f'Sensor reading: {self.get_clock().now().nanoseconds}'
        self.publisher_.publish(msg)
```

### 2. Subscriber-Only Node

```python
class DataProcessorNode(Node):
    def __init__(self):
        super().__init__('data_processor_node')
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.process_data,
            10)

    def process_data(self, msg):
        # Process incoming data
        self.get_logger().info(f'Processing: {msg.data}')
        # Add your processing logic here
```

### 3. Publisher-Subscriber Node

```python
class RelayNode(Node):
    def __init__(self):
        super().__init__('relay_node')
        self.publisher_ = self.create_publisher(String, 'processed_data', 10)
        self.subscription = self.create_subscription(
            String,
            'raw_data',
            self.process_and_relay,
            10)

    def process_and_relay(self, msg):
        # Process and relay the message
        processed_msg = String()
        processed_msg.data = f'Processed: {msg.data}'
        self.publisher_.publish(processed_msg)
```

## Error Handling and Best Practices

### 1. Proper Resource Management

```python
class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.running = True

    def timer_callback(self):
        if self.running:
            try:
                msg = String()
                msg.data = 'Safe message'
                self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Error in timer callback: {e}')

    def stop_node(self):
        self.running = False
        self.destroy_timer(self.timer)
```

### 2. Parameter Configuration

```python
class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')

        # Declare parameters with default values
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('topic_name', 'my_topic')

        # Get parameter values
        publish_rate = self.get_parameter('publish_rate').value
        topic_name = self.get_parameter('topic_name').value

        self.publisher_ = self.create_publisher(String, topic_name, 10)
        self.timer = self.create_timer(1.0/publish_rate, self.timer_callback)
```

### 3. Shutdown Handling

```python
def main(args=None):
    rclpy.init(args=args)
    node = MyNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        # Always clean up resources
        node.destroy_node()
        rclpy.shutdown()
```

## Testing Your Node

### 1. Using Command-Line Tools

```bash
# Check node status
ros2 run my_robot_pkg my_first_node

# In another terminal, check what the node is doing
ros2 node info /my_first_node

# Output shows publishers, subscribers, services, etc.
```

### 2. Unit Testing

Create `test/test_my_first_node.py`:

```python
import unittest
import rclpy
from my_robot_pkg.my_first_node import MyFirstNode


class TestMyFirstNode(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = MyFirstNode()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_creation(self):
        """Test that the node was created successfully."""
        self.assertEqual(self.node.get_name(), 'my_first_node')

    def test_publisher_exists(self):
        """Test that the publisher was created."""
        self.assertIsNotNone(self.node.publisher_)


if __name__ == '__main__':
    unittest.main()
```

## Debugging Techniques

### 1. Logging Levels

```python
# Different logging levels
self.get_logger().debug('Debug information')
self.get_logger().info('General information')
self.get_logger().warn('Warning message')
self.get_logger().error('Error message')
self.get_logger().fatal('Fatal error')
```

### 2. Using rqt Tools

```bash
# Install rqt if not already installed
sudo apt install ros-humble-rqt ros-humble-rqt-graph

# Launch rqt
rqt

# Use rqt_graph to visualize the node graph
ros2 run rqt_graph rqt_graph
```

## Performance Considerations

### 1. QoS Profile Optimization

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# For sensor data (high frequency, best effort)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5
)

# For critical commands (reliable delivery)
command_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

self.publisher_ = self.create_publisher(String, 'topic', sensor_qos)
```

### 2. Memory Management

```python
class EfficientNode(Node):
    def __init__(self):
        super().__init__('efficient_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Pre-allocate message objects to avoid garbage collection
        self.msg = String()

    def timer_callback(self):
        # Reuse the same message object
        self.msg.data = f'Message: {self.get_clock().now().nanoseconds}'
        self.publisher_.publish(self.msg)
```

## Complete Example: Temperature Monitor Node

Let's create a more realistic example that simulates a temperature sensor node:

```python
#!/usr/bin/env python3
"""
Temperature monitoring node that simulates a temperature sensor.
Demonstrates real-world node patterns with error handling.
"""

import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32


class TemperatureMonitorNode(Node):
    def __init__(self):
        super().__init__('temperature_monitor_node')

        # Publisher for temperature readings
        self.temp_publisher_ = self.create_publisher(Float32, 'temperature', 10)

        # Publisher for temperature alerts
        self.alert_publisher_ = self.create_publisher(String, 'temperature_alert', 10)

        # Timer for periodic temperature readings
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.read_temperature)

        # Temperature thresholds
        self.high_temp_threshold = 30.0
        self.low_temp_threshold = 10.0

        # Initialize temperature
        self.current_temp = 20.0

        self.get_logger().info('Temperature Monitor Node initialized!')

    def read_temperature(self):
        """Simulate reading temperature from a sensor."""
        # Simulate temperature reading with some randomness
        self.current_temp += random.uniform(-1.0, 1.0)

        # Create and publish temperature message
        temp_msg = Float32()
        temp_msg.data = round(self.current_temp, 2)
        self.temp_publisher_.publish(temp_msg)

        self.get_logger().info(f'Temperature: {temp_msg.data}°C')

        # Check for temperature alerts
        if temp_msg.data > self.high_temp_threshold:
            alert_msg = String()
            alert_msg.data = f'HIGH TEMP ALERT: {temp_msg.data}°C (threshold: {self.high_temp_threshold}°C)'
            self.alert_publisher_.publish(alert_msg)
            self.get_logger().warn(alert_msg.data)
        elif temp_msg.data < self.low_temp_threshold:
            alert_msg = String()
            alert_msg.data = f'LOW TEMP ALERT: {temp_msg.data}°C (threshold: {self.low_temp_threshold}°C)'
            self.alert_publisher_.publish(alert_msg)
            self.get_logger().warn(alert_msg.data)


def main(args=None):
    rclpy.init(args=args)
    temp_node = TemperatureMonitorNode()

    try:
        rclpy.spin(temp_node)
    except KeyboardInterrupt:
        temp_node.get_logger().info('Temperature monitor stopped by user')
    finally:
        temp_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Add this to your `setup.py` entry points:

```python
entry_points={
    'console_scripts': [
        'my_first_node = my_robot_pkg.my_first_node:main',
        'two_way_node = my_robot_pkg.two_way_node:main',
        'temp_monitor = my_robot_pkg.temperature_monitor_node:main',
    ],
},
```

Build and run the temperature monitor:

```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_pkg
source install/setup.bash

# Terminal 1: Run the temperature monitor
ros2 run my_robot_pkg temp_monitor

# Terminal 2: Listen to temperature readings
ros2 topic echo /temperature

# Terminal 3: Listen to temperature alerts
ros2 topic echo /temperature_alert
```

## Summary

In this chapter, you learned:

- ✅ How to create a ROS 2 workspace and Python package
- ✅ The structure of a basic ROS 2 node with publishers
- ✅ How to create nodes that both publish and subscribe
- ✅ Best practices for node development and error handling
- ✅ How to test and debug your nodes
- ✅ Performance optimization techniques

## Next Steps

Now that you understand how to create basic ROS 2 nodes, you're ready to explore more advanced communication patterns!

**Continue to:** [Chapter 1.3: ROS 2 Communication Patterns →](chapter-1-3-communication-patterns)

## Additional Resources

- [ROS 2 Python Client Library (rclpy) Documentation](https://docs.ros.org/en/humble/p/rclpy/)
- [ROS 2 Package Creation Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [ROS 2 Node and Topic Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)