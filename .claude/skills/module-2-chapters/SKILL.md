```markdown
# Physical AI & Humanoid Robotics - Docusaurus Documentation Structure (Module 2)

## Directory Structure Reference

For reference, the overall directory structure includes Module 2 as follows:
```

docs/
...
├── module-2-digital-twin/
│ ├── index.md
│ ├── chapter-2-1-physics-simulation.md
│ ├── chapter-2-2-gazebo-basics.md
│ ├── chapter-2-3-physics-tuning.md
│ ├── chapter-2-4-sensor-simulation.md
│ ├── chapter-2-5-unity-visualization.md
│ └── chapter-2-6-complex-environments.md
...

````

---

## File: docs/module-2-digital-twin/index.md

```markdown
---
sidebar_position: 3
---

# Module 2: The Digital Twin (Gazebo & Unity)

## Introduction

In the realm of Physical AI and humanoid robotics, a **digital twin** is a virtual replica of a physical robot that mirrors its real-world behavior with high fidelity. This module focuses on creating and utilizing digital twins using **Gazebo** (a physics-based simulator) and **Unity** (a game engine for visualization), allowing safe experimentation, testing, and iteration without risking hardware damage.

Building on the ROS 2 foundation from Module 1, you'll learn to simulate robot physics, sensors, and environments, bridging the gap between software models and physical embodiments.

## The Digital Twin Paradigm

A digital twin shifts development from costly physical prototypes to virtual environments where you can:

- **Iterate Rapidly:** Test control algorithms thousands of times faster than real-time.
- **Simulate Edge Cases:** Create scenarios impossible or dangerous in reality.
- **Transfer to Reality:** Validate sim-to-real transfer techniques.

### Why Gazebo & Unity?

- **Gazebo:** Open-source, ROS-integrated physics simulator with accurate dynamics.
- **Unity:** High-fidelity visualization, real-time rendering, and integration with AI tools.
- **Combined Power:** Gazebo for core simulation, Unity for enhanced visuals and complex interactions.

## Module Focus

This module emphasizes creating virtual environments for humanoid robots:

### Core Topics

1. **Physics Simulation Fundamentals**
   - Rigid body dynamics and collision handling.
   - Integrating URDF models from ROS 2.

2. **Sensor and Environment Modeling**
   - Simulating cameras, LIDAR, and IMUs.
   - Building realistic worlds for training.

3. **Visualization and Tuning**
   - Using Unity for photorealistic rendering.
   - Optimizing simulations for accuracy and performance.

## Learning Path

This module builds progressively from basic simulation to advanced environments:

```mermaid
graph LR
    A[Physics Simulation] --> B[Gazebo Basics]
    B --> C[Physics Tuning]
    C --> D[Sensor Simulation]
    D --> E[Unity Visualization]
    E --> F[Complex Environments]
````

## Chapters

### [Chapter 2.1: Fundamentals of Physics Simulation](chapter-2-1-physics-simulation)

Explore the principles of simulating physical interactions, including dynamics engines and integration with ROS 2.

### [Chapter 2.2: Gazebo Basics](chapter-2-2-gazebo-basics)

Set up Gazebo, spawn robots, and create simple worlds using SDF and URDF.

### [Chapter 2.3: Physics Tuning](chapter-2-3-physics-tuning)

Optimize simulation parameters for realistic humanoid behavior, focusing on joints, friction, and gravity.

### [Chapter 2.4: Sensor Simulation](chapter-2-4-sensor-simulation)

Model and integrate sensors like cameras and LIDAR for perception in digital twins.

### [Chapter 2.5: Unity Visualization](chapter-2-5-unity-visualization)

Leverage Unity for enhanced rendering and visualization of Gazebo simulations.

### [Chapter 2.6: Building Complex Environments](chapter-2-6-complex-environments)

Create dynamic, interactive worlds for training humanoid robots in varied scenarios.

## Prerequisites for This Module

- Completion of Module 1 (ROS 2 basics)
- ROS 2 and Gazebo installed
- Basic knowledge of 3D modeling (e.g., meshes)
- Unity Hub and Editor (version 2022+ recommended)

## Tools You'll Use

- **Gazebo Fortress** (or Ignition Gazebo)
- **Unity Engine** with ROS-TCP-Connector
- **ROS 2 Gazebo Plugins**
- **rviz2** for visualization
- **SDF/URDF Editors**

## Module Assessment

By the end of this module, you'll complete a **Digital Twin Development Project** where you:

- Build a humanoid robot model in URDF/SDF
- Simulate physics and sensors in Gazebo
- Tune parameters for realistic movement
- Visualize in Unity and test interactions
- Export data for sim-to-real analysis

## Estimated Time

⏱️ **4 weeks** (Weeks 6-9 of the course)

- 20-25 hours of content
- 15-20 hours of hands-on practice
- 8-10 hours for the assessment project

---

## Get Started

Ready to create your digital twin? Begin with [Chapter 2.1: Fundamentals of Physics Simulation →](chapter-2-1-physics-simulation)

````

---

## File: docs/module-2-digital-twin/chapter-2-1-physics-simulation.md

```markdown
---
sidebar_position: 1
---

# Chapter 2.1: Fundamentals of Physics Simulation

## Overview

Physics simulation is the cornerstone of digital twins, enabling virtual testing of robot dynamics without physical hardware. This chapter covers the basics of physics engines, how they integrate with ROS 2, and key concepts for simulating humanoid robots.

You'll learn to model forces, collisions, and motions accurately, setting the foundation for realistic digital replicas.

## Why Physics Simulation Matters

In Physical AI, simulations allow:

| Benefit | Description |
|---------|-------------|
| **Safety** | Test dangerous scenarios virtually |
| **Efficiency** | Run experiments 100x faster than real-time |
| **Scalability** | Simulate fleets of robots in parallel |
| **Data Generation** | Create synthetic datasets for AI training |

### Key Challenges in Humanoid Simulation

- Bipedal balance and gait
- Multi-joint coordination
- Contact forces (e.g., feet on ground)
- High degrees of freedom (DoF)

## Core Concepts

### 1. Physics Engines

Physics engines compute object interactions based on Newton's laws.

#### Popular Engines in Robotics

| Engine | Characteristics | Integration |
|--------|-----------------|-------------|
| **ODE** | Fast, stable for rigid bodies | Gazebo default |
| **Bullet** | Real-time capable, collision-focused | Unity & Gazebo |
| **Dart** | Accurate for articulated bodies | Advanced robotics |
| **PhysX** | GPU-accelerated | NVIDIA Isaac |

Gazebo uses plugins to switch engines:

```xml
<plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
  <physics>bullet</physics>
</plugin>
````

### 2. Rigid Body Dynamics

**Rigid bodies** are objects that don't deform. Humanoids are modeled as linked rigid bodies.

#### Equations of Motion

Newton's Second Law: \( \vec{F} = m \vec{a} \)

Torque: \( \vec{\tau} = I \vec{\alpha} \)

In simulation:

- Integrate over time steps (e.g., Euler or Runge-Kutta methods)
- Handle constraints (joints, contacts)

#### Simulation Loop

```mermaid
graph TD
    A[Start Time Step] --> B[Apply Forces/Torques]
    B --> C[Detect Collisions]
    C --> D[Resolve Constraints]
    D --> E[Integrate Positions/Velocities]
    E --> F[Update State]
    F --> A
```

### 3. Joints and Constraints

Joints connect links with specific freedoms.

#### Common Joint Types

- **Revolute:** Rotation around axis (e.g., elbow)
- **Prismatic:** Linear motion (e.g., extendable arm)
- **Ball:** 3D rotation (e.g., shoulder)
- **Fixed:** No motion (e.g., welded parts)

Define in URDF:

```xml
<joint name="shoulder_joint" type="ball">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0 0.5 0" rpy="0 0 0"/>
</joint>
```

### 4. Integration with ROS 2

Gazebo publishes sensor data and subscribes to control commands via ROS topics.

#### Key Plugins

- **gazebo_ros_control:** For joint controllers
- **gazebo_ros_state:** Publishes model states
- **gazebo_ros_diff_drive:** For wheeled bases (adaptable for legs)

Launch example:

```bash
ros2 launch gazebo_ros gazebo.launch.py world:=my_world.sdf
```

## Practical Example: Simple Humanoid Arm Simulation

### Step 1: URDF Model

```xml
<robot name="arm">
  <link name="base_link">
    <visual>
      <geometry><box size="0.1 0.1 0.1"/></geometry>
    </visual>
  </link>
  <link name="arm_link">
    <visual>
      <geometry><cylinder radius="0.05" length="0.5"/></geometry>
    </visual>
  </link>
  <joint name="elbow" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57"/>
  </joint>
</robot>
```

### Step 2: Spawn in Gazebo

```bash
ros2 run gazebo_ros spawn_entity.py -entity arm -file arm.urdf
```

### Step 3: Control via ROS

Publish to `/joint_commands`:

```python
from std_msgs.msg import Float64

publisher = node.create_publisher(Float64, '/elbow_controller/command', 10)
msg = Float64()
msg.data = 0.785  # 45 degrees
publisher.publish(msg)
```

## Best Practices

### 1. Model Design

✅ **DO:**

- Use low-poly meshes for efficiency
- Set accurate masses and inertias
- Test in isolation before full assembly

❌ **DON'T:**

- Overconstrain joints
- Use excessive time steps (affects performance)
- Ignore unit consistency (meters, kg, etc.)

### 2. Simulation Parameters

✅ **DO:**

- Start with default physics settings
- Validate against real data
- Use GPU acceleration when available

❌ **DON'T:**

- Simulate at unrealistically high frequencies
- Neglect gravity compensation
- Forget to reset simulations

## Summary

In this chapter, you learned:

- ✅ Fundamentals of physics engines and rigid body dynamics
- ✅ Joint types and constraints for humanoids
- ✅ Integration of simulations with ROS 2
- ✅ Practical setup for basic arm simulation
- ✅ Best practices for efficient modeling

## Next Steps

With physics basics in place, you're ready to dive into Gazebo specifics.

**Continue to:** [Chapter 2.2: Gazebo Basics →](chapter-2-2-gazebo-basics)

## Additional Resources

- [Gazebo Tutorials](https://gazebosim.org/docs)
- [URDF Documentation](https://wiki.ros.org/urdf)
- [Physics Engine Comparison](https://pybullet.org/)

````

---

## File: docs/module-2-digital-twin/chapter-2-2-gazebo-basics.md

```markdown
---
sidebar_position: 2
---

# Chapter 2.2: Gazebo Basics

## Overview

Gazebo is a powerful, open-source robot simulator that integrates seamlessly with ROS 2. This chapter covers installation, basic setup, world creation, and spawning robots, focusing on humanoid applications.

You'll build your first simulated environment and interact with it using ROS 2 tools.

## Getting Started with Gazebo

### Installation

On Ubuntu:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs
````

For Ignition Gazebo (newer version):

```bash
sudo apt install ros-humble-ign-gazebo6
```

### Launching Gazebo

Basic launch:

```bash
gz sim -v4 empty.sdf  # Ignition Gazebo
```

ROS 2 integration:

```bash
ros2 launch gazebo_ros gazebo.launch.py
```

## Core Concepts

### 1. SDF vs. URDF

**SDF (Simulation Description Format)** is Gazebo's native format, while URDF is ROS-centric.

#### Comparison

| Format   | Strengths                 | Use Case            |
| -------- | ------------------------- | ------------------- |
| **URDF** | ROS integration, simple   | Basic robots        |
| **SDF**  | Advanced features, worlds | Complex simulations |

Convert URDF to SDF:

```bash
gz sdf -p model.urdf > model.sdf
```

### 2. World Files

Worlds define the environment.

Example `world.sdf`:

```xml
<sdf version="1.7">
  <world name="default">
    <light name="sun" type="directional">
      <direction>0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal></geometry>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```

### 3. Models and Plugins

Models are robots or objects; plugins add functionality.

#### Spawning Models

Via ROS 2:

```bash
ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity my_robot
```

## Practical Example: Humanoid Base Simulation

### Step 1: Create Simple Humanoid SDF

```xml
<sdf version="1.7">
  <model name="humanoid">
    <link name="torso">
      <visual name="visual">
        <geometry><box><size>0.3 0.2 0.5</size></box></geometry>
      </visual>
    </link>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"/>
  </model>
</sdf>
```

### Step 2: Launch World and Spawn

```bash
gz sim my_world.sdf
ros2 run gazebo_ros spawn_entity.py -file humanoid.sdf -entity humanoid
```

### Step 3: Interact via GUI/ROS

Use Gazebo GUI to add forces, or ROS topics like `/model/humanoid/pose`.

## Best Practices

### 1. Environment Setup

✅ **DO:**

- Use modular SDF includes
- Optimize model complexity
- Version control worlds/models

❌ **DON'T:**

- Overload with unnecessary plugins
- Ignore lighting/shadows for visuals
- Forget to source ROS setup

### 2. Debugging

Use `gz topic -l` to list topics, `gz model -m humanoid -i` for info.

## Summary

In this chapter, you learned:

- ✅ Gazebo installation and launch
- ✅ SDF/URDF formats and world creation
- ✅ Spawning and interacting with models
- ✅ Basic humanoid setup
- ✅ Debugging tools

## Next Steps

Now tune the physics for realism.

**Continue to:** [Chapter 2.3: Physics Tuning →](chapter-2-3-physics-tuning)

## Additional Resources

- [Gazebo API](https://gazebosim.org/api)
- [ROS-Gazebo Integration](https://ros.org/reps/rep-0147.html)

````

---

## File: docs/module-2-digital-twin/chapter-2-3-physics-tuning.md

```markdown
---
sidebar_position: 3
---

# Chapter 2.3: Physics Tuning

## Overview

Physics tuning optimizes simulation parameters to match real-world behavior. This chapter focuses on adjusting engines, joints, and contacts for accurate humanoid dynamics.

You'll learn to calibrate simulations for stability and realism.

## Core Concepts

### 1. Physics Parameters

Global settings in `world.sdf`:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
</physics>
````

### 2. Joint Tuning

Adjust damping, friction, limits.

Example:

```xml
<joint name="knee" type="revolute">
  <dynamics>
    <damping>0.5</damping>
    <friction>0.1</friction>
  </dynamics>
</joint>
```

### 3. Contact and Collision

Tune surfaces:

```xml
<surface>
  <friction>
    <ode>
      <mu>0.8</mu>  <!-- Static friction -->
      <mu2>0.7</mu2> <!-- Dynamic friction -->
    </ode>
  </friction>
</surface>
```

## Practical Example: Tuning Bipedal Walk

### Step 1: Identify Issues

Run simulation, observe jitter or slippage.

### Step 2: Adjust Parameters

Increase solver iterations:

```xml
<physics>
  <ode>
    <solver>
      <iters>100</iters>
    </solver>
  </ode>
</physics>
```

### Step 3: Validate

Use ROS `/joint_states` to monitor.

## Best Practices

✅ **DO:**

- Iterate incrementally
- Compare with real data
- Use profiling tools

❌ **DON'T:**

- Over-tune (causes instability)
- Ignore mass distributions
- Skip unit tests

## Summary

In this chapter, you learned:

- ✅ Global and local physics parameters
- ✅ Joint and contact tuning
- ✅ Calibration for humanoids
- ✅ Validation methods
- ✅ Optimization tips

## Next Steps

Add sensors to your tuned simulation.

**Continue to:** [Chapter 2.4: Sensor Simulation →](chapter-2-4-sensor-simulation)

## Additional Resources

- [Gazebo Physics Tuning Guide](https://gazebosim.org/tutorials?cat=physics)
- [ODE Documentation](http://ode.org/)

````

---

## File: docs/module-2-digital-twin/chapter-2-4-sensor-simulation.md

```markdown
---
sidebar_position: 4
---

# Chapter 2.4: Sensor Simulation

## Overview

Sensor simulation replicates real hardware outputs in virtual environments. This chapter covers modeling cameras, LIDAR, IMUs, and more for humanoid perception.

You'll integrate sensors into your digital twin for AI training.

## Core Concepts

### 1. Sensor Plugins

Gazebo uses plugins for sensors.

Example camera plugin:

```xml
<sensor name="camera" type="camera">
  <plugin name="camera_plugin" filename="libgazebo_ros_camera.so"/>
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image><width>640</width><height>480</height></image>
  </camera>
</sensor>
````

### 2. Noise Modeling

Add realism with noise:

```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.01</stddev>
</noise>
```

### 3. ROS Integration

Sensors publish to topics like `/camera/image_raw`.

## Practical Example: Humanoid Head Sensors

### Step 1: Add LIDAR

```xml
<sensor name="lidar" type="ray">
  <plugin name="lidar_plugin" filename="libgazebo_ros_laser.so"/>
  <ray>
    <scan><horizontal><samples>360</samples><resolution>1</resolution></horizontal></scan>
  </ray>
</sensor>
```

### Step 2: Visualize

Use `rviz2` to subscribe to `/scan`.

### Step 3: Test Perception

Run object detection on simulated data.

## Best Practices

✅ **DO:**

- Match real sensor specs
- Calibrate noise levels
- Use GPU for ray-tracing

❌ **DON'T:**

- Overlook frame rates
- Ignore distortion models
- Forget extrinsic calibrations

## Summary

In this chapter, you learned:

- ✅ Sensor types and plugins
- ✅ Noise and realism modeling
- ✅ Integration with ROS
- ✅ Humanoid sensor setup
- ✅ Testing methods

## Next Steps

Enhance visuals with Unity.

**Continue to:** [Chapter 2.5: Unity Visualization →](chapter-2-5-unity-visualization)

## Additional Resources

- [Gazebo Sensor Tutorials](https://gazebosim.org/tutorials?cat=sensors)
- [ROS Sensor Messages](https://wiki.ros.org/sensor_msgs)

````

---

## File: docs/module-2-digital-twin/chapter-2-5-unity-visualization.md

```markdown
---
sidebar_position: 5
---

# Chapter 2.5: Unity Visualization

## Overview

Unity provides high-fidelity rendering for Gazebo simulations. This chapter covers integrating Unity with ROS 2 for photorealistic visualization of digital twins.

You'll create immersive views for humanoid robots.

## Core Concepts

### 1. Unity Setup

Install Unity Hub, create project, add ROS-TCP-Connector package.

### 2. Bridging Gazebo to Unity

Use ROS Bridge:

```bash
ros2 launch ros_tcp_endpoint default_server_endpoint.launch.py
````

In Unity, connect via TCP.

### 3. Rendering Pipelines

Use HDRP for realism.

Add assets:

- Import URDF via URDF-Importer
- Sync poses from ROS `/tf`

## Practical Example: Visualizing Humanoid

### Step 1: Import Model

Use Unity's URDF tool to load humanoid.urdf.

### Step 2: Sync Data

Script to subscribe to ROS topics:

```csharp
using RosMessageTypes.Sensor;

public class ImageSubscriber : UnityEngine.MonoBehaviour
{
    void OnMessageReceived(ImageMsg message)
    {
        // Render image to texture
    }
}
```

### Step 3: Add Effects

Apply shaders, lighting, post-processing.

## Best Practices

✅ **DO:**

- Optimize for performance
- Use asset store for environments
- Sync physics if needed

❌ **DON'T:**

- Mix rendering pipelines
- Ignore latency
- Overload with effects

## Summary

In this chapter, you learned:

- ✅ Unity-ROS integration
- ✅ Model importing and syncing
- ✅ Rendering enhancements
- ✅ Visualization scripting
- ✅ Optimization tips

## Next Steps

Build complex worlds.

**Continue to:** [Chapter 2.6: Building Complex Environments →](chapter-2-6-complex-environments)

## Additional Resources

- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS-Unity Bridge](https://github.com/Unity-Technologies/ROS-TCP-Connector)

````

---

## File: docs/module-2-digital-twin/chapter-2-6-complex-environments.md

```markdown
---
sidebar_position: 6
---

# Chapter 2.6: Building Complex Environments

## Overview

Complex environments simulate real-world variability for robust training. This chapter covers dynamic objects, weather, and multi-robot scenarios in Gazebo/Unity.

You'll create rich worlds for humanoid AI development.

## Core Concepts

### 1. Dynamic Elements

Add actors, moving objects:

```xml
<actor name="person">
  <skin><filename>walk.dae</filename></skin>
  <animation name="walk"><filename>walk.dae</filename></animation>
</actor>
````

### 2. Environmental Effects

Plugins for weather:

```xml
<plugin name="weather" filename="libWeatherPlugin.so">
  <rain_intensity>0.5</rain_intensity>
</plugin>
```

### 3. Multi-Robot Simulations

Namespaces for multiple humanoids.

Launch:

```bash
ros2 launch my_pkg multi_humanoid.launch.py
```

## Practical Example: Urban Scenario

### Step 1: Build World

Include buildings, roads, actors.

### Step 2: Add Interactions

Script collisions, grasping.

### Step 3: Test in Unity

Visualize crowd navigation.

## Best Practices

✅ **DO:**

- Modularize environments
- Use procedural generation
- Benchmark performance

❌ **DON'T:**

- Create monolithic worlds
- Ignore scalability
- Skip randomization

## Summary

In this chapter, you learned:

- ✅ Dynamic and environmental modeling
- ✅ Multi-robot setups
- ✅ Complex world building
- ✅ Integration with previous tools
- ✅ Scaling tips

## Next Steps

Proceed to Module 3 for NVIDIA Isaac.

**Return to:** [Module 2 Index](../index)

## Additional Resources

- [Gazebo World Building](https://gazebosim.org/tutorials?cat=build_world)
- [Unity Environment Assets](https://assetstore.unity.com/)

```

---

This completes the detailed structure for Module 2. Each chapter follows the template with overviews, concepts, examples, best practices, and summaries. If you need details for other modules, let me know!
```
