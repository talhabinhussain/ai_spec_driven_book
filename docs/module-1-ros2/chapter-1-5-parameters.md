---
title: Chapter 1.5 - ROS 2 Parameters and Configuration
description: Managing configuration through parameters, launch files, and parameter files
---

# Chapter 1.5: ROS 2 Parameters and Configuration

## Overview

In this chapter, you'll learn how to manage configuration in ROS 2 using parameters, launch files, and parameter files. Proper configuration management is crucial for creating flexible, maintainable robotic systems that can adapt to different environments and requirements.

By the end of this chapter, you'll understand:
- ✅ How to declare and use parameters in ROS 2 nodes
- ✅ Parameter management through launch files and parameter files
- ✅ Best practices for configuration management
- ✅ Advanced parameter techniques and validation

## Understanding ROS 2 Parameters

### What Are Parameters?

Parameters in ROS 2 are **named, typed values** that control node behavior without requiring code changes. They allow you to configure nodes at runtime, making your robotic systems adaptable to different environments, hardware configurations, and operational requirements.

### Parameter Characteristics

- **Named**: Each parameter has a unique name within a node
- **Typed**: Parameters have specific data types (int, float, string, bool, lists)
- **Runtime-Configurable**: Can be changed while nodes are running
- **Node-Specific**: Parameters belong to specific nodes
- **Hierarchical**: Support for namespaced parameter names

### Parameter Types

ROS 2 supports several parameter types:

| Type | Python Type | Description | Example |
|------|-------------|-------------|---------|
| `bool` | `bool` | Boolean values | `True`, `False` |
| `int` | `int` | Integer values | `42`, `-10` |
| `double` | `float` | Floating point values | `3.14`, `-2.5` |
| `string` | `str` | Text values | `"hello"`, `"config"` |
| `bool_array` | `list[bool]` | Array of booleans | `[True, False, True]` |
| `integer_array` | `list[int]` | Array of integers | `[1, 2, 3, 4]` |
| `double_array` | `list[float]` | Array of floats | `[1.1, 2.2, 3.3]` |
| `string_array` | `list[str]` | Array of strings | `["a", "b", "c"]` |

## Declaring and Using Parameters

### Parameter Declaration in Nodes

To use parameters in your nodes, you must first declare them:

```python
#!/usr/bin/env python3
"""
Node demonstrating parameter declaration and usage.
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import SetParameters
import json


class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values and descriptions
        self.declare_parameter('robot_name', 'default_robot',
                             description='Name of the robot')
        self.declare_parameter('max_velocity', 1.0,
                             description='Maximum linear velocity (m/s)')
        self.declare_parameter('safety_enabled', True,
                             description='Enable safety checks')
        self.declare_parameter('sensor_offsets', [0.0, 0.1, 0.2],
                             description='Sensor offset values')
        self.declare_parameter('debug_topics', ['/debug_info', '/debug_status'],
                             description='Debug topic names')

        # Get parameter values
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.safety_enabled = self.get_parameter('safety_enabled').value
        self.sensor_offsets = self.get_parameter('sensor_offsets').value
        self.debug_topics = self.get_parameter('debug_topics').value

        # Set up parameter callback for runtime changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Timer to demonstrate parameter usage
        self.timer = self.create_timer(2.0, self.timer_callback)

        self.get_logger().info(f'Parameter node initialized with robot: {self.robot_name}')

    def parameter_callback(self, params):
        """
        Callback for parameter changes.
        This is called whenever parameters are set/changed.
        """
        for param in params:
            if param.name == 'robot_name':
                self.robot_name = param.value
                self.get_logger().info(f'Robot name updated to: {self.robot_name}')
            elif param.name == 'max_velocity':
                if param.value > 0:
                    self.max_velocity = param.value
                    self.get_logger().info(f'Max velocity updated to: {self.max_velocity}')
                else:
                    self.get_logger().error('Max velocity must be positive')
                    return rclpy.parameter.ParameterCallbackResult.FAILURE
            elif param.name == 'safety_enabled':
                self.safety_enabled = param.value
                status = 'enabled' if self.safety_enabled else 'disabled'
                self.get_logger().info(f'Safety checks {status}')
            elif param.name == 'sensor_offsets':
                self.sensor_offsets = param.value
                self.get_logger().info(f'Sensor offsets updated: {self.sensor_offsets}')

        return rclpy.parameter.ParameterCallbackResult.SUCCESS

    def timer_callback(self):
        """Timer callback that uses parameters."""
        self.get_logger().info(
            f'Robot: {self.robot_name}, '
            f'Max Vel: {self.max_velocity}, '
            f'Safety: {self.safety_enabled}, '
            f'Sensors: {self.sensor_offsets}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Parameter node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Parameter Validation and Constraints

You can add validation to your parameters:

```python
class ValidatedParameterNode(Node):
    def __init__(self):
        super().__init__('validated_parameter_node')

        # Declare parameters with validation
        self.declare_parameter('robot_radius', 0.5)
        self.declare_parameter('max_acceleration', 2.0)
        self.declare_parameter('control_frequency', 50)

        # Set up validation callback
        self.add_on_set_parameters_callback(self.validated_parameter_callback)

    def validated_parameter_callback(self, params):
        """
        Callback with validation logic.
        """
        for param in params:
            if param.name == 'robot_radius':
                if param.value <= 0:
                    self.get_logger().error('Robot radius must be positive')
                    return rclpy.parameter.ParameterCallbackResult.FAILURE
                elif param.value > 5.0:
                    self.get_logger().warn('Robot radius seems unusually large')
            elif param.name == 'max_acceleration':
                if param.value <= 0:
                    self.get_logger().error('Max acceleration must be positive')
                    return rclpy.parameter.ParameterCallbackResult.FAILURE
                elif param.value > 10.0:
                    self.get_logger().warn('High acceleration values may cause instability')
            elif param.name == 'control_frequency':
                if param.value <= 0:
                    self.get_logger().error('Control frequency must be positive')
                    return rclpy.parameter.ParameterCallbackResult.FAILURE
                elif param.value > 1000:
                    self.get_logger().warn('Very high control frequency may impact performance')

        # Update parameter values after validation
        for param in params:
            setattr(self, param.name.replace('-', '_'), param.value)

        return rclpy.parameter.ParameterCallbackResult.SUCCESS
```

### Dynamic Parameter Reconfiguration

Parameters can be changed at runtime using command-line tools:

```bash
# Set a parameter on a running node
ros2 param set /parameter_node robot_name "new_robot_name"

# Get a parameter value
ros2 param get /parameter_node max_velocity

# List all parameters for a node
ros2 param list /parameter_node

# Load parameters from a file
ros2 param load /parameter_node /path/to/params.yaml

# Dump current parameters to a file
ros2 param dump /parameter_node
```

## Parameter Files (YAML)

### Creating Parameter Files

Parameter files use YAML format and allow you to set parameters before nodes start:

Create `config/robot_params.yaml`:

```yaml
/**:  # Global parameters (applies to all nodes)
  ros__parameters:
    use_sim_time: false
    log_level: "info"

parameter_node:
  ros__parameters:
    robot_name: "my_robot"
    max_velocity: 2.0
    safety_enabled: true
    sensor_offsets: [0.1, 0.2, 0.3]
    debug_topics: ["/debug/info", "/debug/status"]

navigation_node:
  ros__parameters:
    planner_frequency: 5.0
    controller_frequency: 20.0
    recovery_enabled: true
    oscillation_timeout: 10.0
    oscillation_distance: 0.5

controller_node:
  ros__parameters:
    kp: 1.0
    ki: 0.1
    kd: 0.05
    max_vel: 1.0
    min_vel: 0.1
    cmd_timeout: 1.0
```

### Loading Parameter Files

```python
#!/usr/bin/env python3
"""
Node that loads parameters from a file.
"""

import rclpy
from rclpy.node import Node


class ParameterFileNode(Node):
    def __init__(self):
        super().__init__('parameter_file_node')

        # Declare parameters (they will be set from the parameter file)
        self.declare_parameter('robot_name', 'default')
        self.declare_parameter('max_velocity', 1.0)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)

        # Get parameter values after declaration
        self.robot_name = self.get_parameter('robot_name').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        self.get_logger().info(f'Loaded parameters: {self.robot_name}, {self.max_velocity}')


def main(args=None):
    rclpy.init(args=args)
    node = ParameterFileNode()

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

### Loading Parameters at Launch Time

```bash
# Load parameters from file when starting the node
ros2 run my_robot_pkg parameter_file_node --ros-args --params-file config/robot_params.yaml
```

## Launch Files with Parameters

### Basic Launch File with Parameters

Create `launch/parameter_demo.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='default_robot',
        description='Name of the robot'
    )

    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='1.0',
        description='Maximum velocity for the robot'
    )

    # Get launch configurations
    robot_name = LaunchConfiguration('robot_name')
    max_velocity = LaunchConfiguration('max_velocity')

    # Create parameter node
    parameter_node = Node(
        package='my_robot_pkg',
        executable='parameter_node',
        name='parameter_node',
        parameters=[
            {
                'robot_name': robot_name,
                'max_velocity': max_velocity,
                'safety_enabled': True,
                'sensor_offsets': [0.1, 0.2, 0.3]
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_name_arg,
        max_velocity_arg,
        parameter_node
    ])
```

### Advanced Launch File with Multiple Configurations

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='turtlebot4',
        description='Robot model to use'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='default',
        description='Configuration file to use'
    )

    # Get launch configurations
    robot_model = LaunchConfiguration('robot_model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    # Define configuration file path based on robot model
    config_path = PathJoinSubstitution([
        FindPackageShare('my_robot_pkg'),
        'config',
        [config_file, '_', robot_model, '.yaml']
    ])

    # Robot controller node
    robot_controller = Node(
        package='my_robot_pkg',
        executable='robot_controller',
        name='robot_controller',
        parameters=[
            config_path,
            {'use_sim_time': use_sim_time},
            {'robot_model': robot_model}
        ],
        output='screen'
    )

    # Sensor processor node
    sensor_processor = Node(
        package='my_robot_pkg',
        executable='sensor_processor',
        name='sensor_processor',
        parameters=[
            config_path,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_model_arg,
        use_sim_time_arg,
        config_file_arg,
        robot_controller,
        sensor_processor
    ])
```

### Launch File with Conditional Parameters

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Whether to use camera'
    )

    use_lidar_arg = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Whether to use lidar'
    )

    # Get launch configurations
    use_camera = LaunchConfiguration('use_camera')
    use_lidar = LaunchConfiguration('use_lidar')

    # Camera node (only if use_camera is true)
    camera_node = Node(
        package='my_robot_pkg',
        executable='camera_node',
        name='camera_node',
        parameters=[{'use_camera': use_camera}],
        condition=IfCondition(use_camera),
        output='screen'
    )

    # Lidar node (only if use_lidar is true)
    lidar_node = Node(
        package='my_robot_pkg',
        executable='lidar_node',
        name='lidar_node',
        parameters=[{'use_lidar': use_lidar}],
        condition=IfCondition(use_lidar),
        output='screen'
    )

    return LaunchDescription([
        use_camera_arg,
        use_lidar_arg,
        camera_node,
        lidar_node
    ])
```

## Advanced Parameter Techniques

### Parameter Descriptors

You can provide detailed descriptions and constraints for parameters:

```python
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, FloatingPointRange
from rcl_interfaces.srv import SetParameters


class DescriptiveParameterNode(Node):
    def __init__(self):
        super().__init__('descriptive_parameter_node')

        # Declare parameter with descriptor
        velocity_descriptor = ParameterDescriptor(
            name='max_velocity',
            type=ParameterType.PARAMETER_DOUBLE,
            description='Maximum linear velocity of the robot',
            additional_constraints='Must be positive and less than 5.0 m/s',
            read_only=False,
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=5.0, step=0.1)]
        )

        self.declare_parameter('max_velocity', 1.0, descriptor=velocity_descriptor)

        # Integer parameter with range
        count_descriptor = ParameterDescriptor(
            name='loop_count',
            type=ParameterType.PARAMETER_INTEGER,
            description='Number of loops to execute',
            integer_range=[IntegerRange(from_value=1, to_value=100, step=1)]
        )

        self.declare_parameter('loop_count', 10, descriptor=count_descriptor)

        # Get parameter values
        self.max_velocity = self.get_parameter('max_velocity').value
        self.loop_count = self.get_parameter('loop_count').value

        self.get_logger().info(f'Max velocity: {self.max_velocity}, Loop count: {self.loop_count}')
```

### Parameter Groups and Hierarchical Names

Use parameter names with hierarchical structure:

```python
class HierarchicalParameterNode(Node):
    def __init__(self):
        super().__init__('hierarchical_parameter_node')

        # Declare parameters with hierarchical names
        # Navigation parameters
        self.declare_parameter('navigation.planner_frequency', 5.0)
        self.declare_parameter('navigation.controller_frequency', 20.0)
        self.declare_parameter('navigation.timeout', 30.0)

        # Sensor parameters
        self.declare_parameter('sensors.camera.enabled', True)
        self.declare_parameter('sensors.lidar.range_min', 0.1)
        self.declare_parameter('sensors.lidar.range_max', 30.0)
        self.declare_parameter('sensors.imu.enabled', True)

        # Control parameters
        self.declare_parameter('control.pid.kp', 1.0)
        self.declare_parameter('control.pid.ki', 0.1)
        self.declare_parameter('control.pid.kd', 0.05)

        # Get parameters
        self.planner_freq = self.get_parameter('navigation.planner_frequency').value
        self.camera_enabled = self.get_parameter('sensors.camera.enabled').value
        self.kp = self.get_parameter('control.pid.kp').value

        self.get_logger().info(f'Planner freq: {self.planner_freq}, PID Kp: {self.kp}')
```

### Parameter Callbacks for Complex Logic

```python
class ComplexParameterNode(Node):
    def __init__(self):
        super().__init__('complex_parameter_node')

        # Declare related parameters
        self.declare_parameter('velocity.linear', 1.0)
        self.declare_parameter('velocity.angular', 0.5)
        self.declare_parameter('acceleration.linear', 2.0)
        self.declare_parameter('acceleration.angular', 1.0)

        # Set up parameter callback
        self.add_on_set_parameters_callback(self.complex_parameter_callback)

        # Initialize with parameter values
        self.update_from_parameters()

    def complex_parameter_callback(self, params):
        """
        Complex callback that handles interdependent parameters.
        """
        param_updates = {}

        for param in params:
            param_updates[param.name] = param.value

        # Validate interdependent parameters
        if 'velocity.linear' in param_updates and 'acceleration.linear' in param_updates:
            vel = param_updates['velocity.linear']
            acc = param_updates['acceleration.linear']

            # Check if acceleration is sufficient for the desired velocity
            if acc < vel / 2.0:  # Arbitrary safety check
                self.get_logger().warn(
                    f'Acceleration {acc} may be insufficient for velocity {vel}'
                )

        # Update all parameters atomically
        for param in params:
            setattr(self, param.name.replace('.', '_'), param.value)

        # Trigger any dependent logic
        self.update_control_system()

        return rclpy.parameter.ParameterCallbackResult.SUCCESS

    def update_from_parameters(self):
        """Update internal state from current parameters."""
        self.linear_vel = self.get_parameter('velocity.linear').value
        self.angular_vel = self.get_parameter('velocity.angular').value
        self.linear_acc = self.get_parameter('acceleration.linear').value
        self.angular_acc = self.get_parameter('acceleration.angular').value

    def update_control_system(self):
        """Update control system based on parameter changes."""
        self.get_logger().info(
            f'Control system updated: '
            f'Linear {self.linear_vel}/{self.linear_acc}, '
            f'Angular {self.angular_vel}/{self.angular_acc}'
        )
```

## Parameter Management Strategies

### 1. Configuration Profiles

Create different parameter files for different scenarios:

**config/simulation.yaml:**
```yaml
/**:
  ros__parameters:
    use_sim_time: true

robot_controller:
  ros__parameters:
    max_velocity: 2.0
    max_acceleration: 3.0
    safety_distance: 0.5
```

**config/real_robot.yaml:**
```yaml
/**:
  ros__parameters:
    use_sim_time: false

robot_controller:
  ros__parameters:
    max_velocity: 0.5
    max_acceleration: 1.0
    safety_distance: 1.0
```

### 2. Parameter Validation Node

Create a dedicated node for parameter validation:

```python
#!/usr/bin/env python3
"""
Parameter validation node that monitors and validates parameters across the system.
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import String


class ParameterValidatorNode(Node):
    def __init__(self):
        super().__init__('parameter_validator_node')

        # Subscribe to parameter events
        self.param_event_sub = self.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.param_event_callback,
            10
        )

        # Publisher for validation alerts
        self.alert_pub = self.create_publisher(String, 'parameter_alerts', 10)

        self.get_logger().info('Parameter validator node initialized')

    def param_event_callback(self, msg):
        """Handle parameter change events."""
        for new_param in msg.new_parameters:
            self.validate_parameter(msg.node, new_param)

        for changed_param in msg.changed_parameters:
            self.validate_parameter(msg.node, changed_param)

    def validate_parameter(self, node_name, param):
        """Validate individual parameter."""
        param_name = param.name
        param_value = param.value

        # Define validation rules
        validation_rules = {
            'max_velocity': {
                'type': 'double',
                'min': 0.0,
                'max': 5.0,
                'description': 'Maximum velocity must be between 0 and 5 m/s'
            },
            'robot_radius': {
                'type': 'double',
                'min': 0.01,
                'max': 2.0,
                'description': 'Robot radius must be between 0.01 and 2.0 meters'
            }
        }

        if param_name in validation_rules:
            rule = validation_rules[param_name]

            # Check type
            if rule['type'] == 'double' and param_value.type != 3:  # 3 is double
                self.alert(f'Parameter {param_name} in {node_name} has wrong type')
                return False

            # Check range
            value = param_value.double_value
            if value < rule['min'] or value > rule['max']:
                self.alert(f'Parameter {param_name} in {node_name} out of range: {value}. {rule["description"]}')
                return False

        return True

    def alert(self, message):
        """Publish validation alert."""
        alert_msg = String()
        alert_msg.data = message
        self.alert_pub.publish(alert_msg)
        self.get_logger().warn(message)


def main(args=None):
    rclpy.init(args=args)
    node = ParameterValidatorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Parameter validator stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3. Parameter Presets

Create presets for common configurations:

```python
class ParameterPresetNode(Node):
    def __init__(self):
        super().__init__('parameter_preset_node')

        # Declare preset selection parameter
        self.declare_parameter('preset', 'default')

        # Define presets
        self.presets = {
            'default': {
                'max_velocity': 1.0,
                'acceleration': 2.0,
                'safety_distance': 1.0
            },
            'aggressive': {
                'max_velocity': 3.0,
                'acceleration': 5.0,
                'safety_distance': 2.0
            },
            'conservative': {
                'max_velocity': 0.5,
                'acceleration': 1.0,
                'safety_distance': 0.5
            },
            'demo': {
                'max_velocity': 1.5,
                'acceleration': 2.5,
                'safety_distance': 1.5
            }
        }

        # Set up parameter callback
        self.add_on_set_parameters_callback(self.preset_callback)

    def preset_callback(self, params):
        """Handle preset changes."""
        for param in params:
            if param.name == 'preset' and param.value in self.presets:
                preset_name = param.value
                preset_values = self.presets[preset_name]

                self.get_logger().info(f'Loading preset: {preset_name}')

                # Apply preset values
                for key, value in preset_values.items():
                    try:
                        # This would require a more complex setup to actually change parameters
                        # In practice, you'd need to use a different approach
                        self.get_logger().info(f'Setting {key} to {value}')
                    except Exception as e:
                        self.get_logger().error(f'Error applying preset {key}: {e}')

                return rclpy.parameter.ParameterCallbackResult.SUCCESS

        return rclpy.parameter.ParameterCallbackResult.SUCCESS
```

## Command-Line Parameter Management

### Useful Parameter Commands

```bash
# List all parameters for a node
ros2 param list /parameter_node

# Get a specific parameter
ros2 param get /parameter_node robot_name

# Set a parameter
ros2 param set /parameter_node max_velocity 2.5

# Load parameters from YAML file
ros2 param load /parameter_node config/robot_params.yaml

# Dump current parameters to file
ros2 param dump /parameter_node --output config/current_params.yaml

# Watch parameter changes
watch -n 1 'ros2 param list /parameter_node'

# Get parameter descriptions
ros2 param describe /parameter_node robot_name
```

### Parameter Management Scripts

Create a script to manage parameters across multiple nodes:

```python
#!/usr/bin/env python3
"""
Parameter management script for multiple nodes.
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters
import sys
import time


class ParameterManager(Node):
    def __init__(self):
        super().__init__('parameter_manager')

    def set_parameter_for_nodes(self, node_names, param_name, param_value):
        """Set a parameter across multiple nodes."""
        for node_name in node_names:
            try:
                # Create client for the node
                client = self.create_client(
                    SetParameters,
                    f'{node_name}/set_parameters'
                )

                if not client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().error(f'Service not available for {node_name}')
                    continue

                # Create parameter request
                from rcl_interfaces.msg import Parameter, ParameterValue
                param = Parameter()
                param.name = param_name
                param.value = ParameterValue()
                param.value.type = 3  # double
                param.value.double_value = param_value

                request = SetParameters.Request()
                request.parameters = [param]

                # Send request
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self, future)

                if future.result() is not None:
                    response = future.result()
                    if response.results[0].successful:
                        self.get_logger().info(f'Set {param_name}={param_value} for {node_name}')
                    else:
                        self.get_logger().error(f'Failed to set {param_name} for {node_name}')
                else:
                    self.get_logger().error(f'Failed to call service for {node_name}')

            except Exception as e:
                self.get_logger().error(f'Error setting parameter for {node_name}: {e}')

    def batch_configure_nodes(self):
        """Configure multiple nodes with different parameters."""
        # Navigation nodes
        nav_nodes = ['/nav_planner', '/nav_controller']
        for node in nav_nodes:
            self.set_parameter_for_nodes([node], 'planner_frequency', 5.0)
            self.set_parameter_for_nodes([node], 'controller_frequency', 20.0)

        # Sensor nodes
        sensor_nodes = ['/camera_driver', '/lidar_driver']
        for node in sensor_nodes:
            self.set_parameter_for_nodes([node], 'publish_frequency', 10.0)


def main(args=None):
    rclpy.init(args=args)
    manager = ParameterManager()

    try:
        manager.batch_configure_nodes()
        # Keep node alive briefly to allow parameter setting
        time.sleep(1.0)
    except KeyboardInterrupt:
        manager.get_logger().info('Parameter manager stopped')
    finally:
        manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Best Practices for Parameter Management

### 1. Parameter Organization

```python
# Good: Organized parameter structure
class WellOrganizedNode(Node):
    def __init__(self):
        super().__init__('well_organized_node')

        # Group related parameters
        self.declare_parameter('navigation.max_velocity', 1.0)
        self.declare_parameter('navigation.min_velocity', 0.1)
        self.declare_parameter('navigation.acceleration', 2.0)

        self.declare_parameter('sensors.lidar.enabled', True)
        self.declare_parameter('sensors.camera.enabled', True)
        self.declare_parameter('sensors.imu.enabled', True)

        self.declare_parameter('control.pid.kp', 1.0)
        self.declare_parameter('control.pid.ki', 0.1)
        self.declare_parameter('control.pid.kd', 0.05)

# Avoid: Unorganized parameter names
class PoorlyOrganizedNode(Node):
    def __init__(self):
        super().__init__('poorly_organized_node')

        # Unclear, unorganized parameter names
        self.declare_parameter('max_vel', 1.0)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('lidar_on', True)
        self.declare_parameter('cam_en', True)
```

### 2. Parameter Validation

```python
def validate_and_set_parameters(self, params_dict):
    """Validate and set multiple parameters safely."""
    validated_params = {}

    for name, value in params_dict.items():
        if name == 'max_velocity':
            if not (0 < value <= 10.0):
                self.get_logger().error(f'Max velocity {value} out of range')
                continue
        elif name == 'robot_radius':
            if not (0.01 <= value <= 5.0):
                self.get_logger().error(f'Robot radius {value} out of range')
                continue

        validated_params[name] = value

    # Set validated parameters
    for name, value in validated_params.items():
        self.set_parameters([Parameter(name, Parameter.Type.DOUBLE, value)])
```

### 3. Default Values and Documentation

```python
class DocumentedParameterNode(Node):
    def __init__(self):
        super().__init__('documented_parameter_node')

        # Always provide meaningful default values and descriptions
        self.declare_parameter(
            'control_frequency',
            50.0,
            description='Control loop frequency in Hz (default: 50.0)'
        )

        self.declare_parameter(
            'safety_timeout',
            5.0,
            description='Safety timeout in seconds (default: 5.0, 0 to disable)'
        )

        self.declare_parameter(
            'debug_mode',
            False,
            description='Enable debug output (default: false)'
        )
```

### 4. Runtime Parameter Changes

```python
class SafeParameterNode(Node):
    def __init__(self):
        super().__init__('safe_parameter_node')

        self.declare_parameter('max_velocity', 1.0)
        self.add_on_set_parameters_callback(self.safe_parameter_callback)

        # Store critical parameters separately for safety
        self._critical_params = set(['max_velocity', 'safety_timeout'])

    def safe_parameter_callback(self, params):
        """Safe parameter callback with critical parameter handling."""
        # Validate all parameters first
        for param in params:
            if not self.validate_parameter(param):
                return rclpy.parameter.ParameterCallbackResult.FAILURE

        # Apply changes safely
        for param in params:
            # Store old value for potential rollback
            old_value = self.get_parameter(param.name).value if self.has_parameter(param.name) else None

            # Apply new value
            success = self.set_parameters([param])[0].successful

            if not success:
                self.get_logger().error(f'Failed to set parameter {param.name}')
                return rclpy.parameter.ParameterCallbackResult.FAILURE

        return rclpy.parameter.ParameterCallbackResult.SUCCESS

    def validate_parameter(self, param):
        """Validate a single parameter."""
        if param.name == 'max_velocity':
            if param.value <= 0 or param.value > 10.0:
                self.get_logger().error(f'Invalid max_velocity: {param.value}')
                return False
        return True
```

## Parameter Performance Considerations

### Efficient Parameter Access

```python
class EfficientParameterNode(Node):
    def __init__(self):
        super().__init__('efficient_parameter_node')

        # Declare parameters
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('threshold', 0.5)

        # Cache parameter values to avoid repeated calls
        self._update_rate = self.get_parameter('update_rate').value
        self._threshold = self.get_parameter('threshold').value

        # Update cached values periodically or on parameter change
        self.timer = self.create_timer(1.0, self.update_cached_params)

    def update_cached_params(self):
        """Update cached parameter values."""
        self._update_rate = self.get_parameter('update_rate').value
        self._threshold = self.get_parameter('threshold').value

    def process_data(self, data):
        """Process data using cached parameters (faster)."""
        if data > self._threshold:
            # Process data with cached threshold
            return data * self._update_rate
        return 0.0
```

## Summary

In this chapter, you learned:

- ✅ How to declare and use parameters in ROS 2 nodes
- ✅ Parameter management through launch files and parameter files
- ✅ Advanced parameter techniques including validation and callbacks
- ✅ Best practices for configuration management and parameter organization
- ✅ Command-line tools for parameter management

## Next Steps

Now that you understand parameters and configuration management, you're ready to learn about launch files and system orchestration!

**Continue to:** [Chapter 1.6: ROS 2 Launch Files and System Orchestration →](chapter-1-6-launch-files)

## Additional Resources

- [ROS 2 Parameters Documentation](https://docs.ros.org/en/humble/How-To-Guides/Using-Parameters-in-a-class.html)
- [Launch System Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html)
- [Parameter Files Guide](https://docs.ros.org/en/humble/How-To-Guides/Node-arguments.html#parameter-files)
- [Launch Files Best Practices](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)