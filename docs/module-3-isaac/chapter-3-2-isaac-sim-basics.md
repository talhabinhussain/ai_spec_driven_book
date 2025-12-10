---
sidebar_position: 2
---

# Chapter 3.2: Getting Started with Isaac Sim

## Overview

Isaac Sim is NVIDIA's flagship robotics simulator, built on the Omniverse platform. This chapter walks you through installation, understanding the interface, importing robots, and running your first simulation with photorealistic physics and sensors.

By the end of this chapter, you'll have a working Isaac Sim environment and understand how to create scenes, import robots, and configure simulations.

## Installation and Setup

### System Requirements Check

Before installing, verify your system meets requirements:

```bash
# Check NVIDIA driver version (need 525+)
nvidia-smi

# Check CUDA version (need 12.0+)
nvcc --version

# Check disk space (need 500GB+ free)
df -h

# Check RAM
free -h
```

### Step 1: Install NVIDIA Omniverse Launcher

#### Download and Install

```bash
# Download Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make executable
chmod +x omniverse-launcher-linux.AppImage

# Run installer
./omniverse-launcher-linux.AppImage
```

#### First Launch Setup

1. **Create NVIDIA Account** (if you don't have one)
2. **Sign in** to the launcher
3. **Accept** the license agreements

### Step 2: Install Isaac Sim

#### Through Omniverse Launcher

```
1. Open Omniverse Launcher
2. Go to "Exchange" tab
3. Search for "Isaac Sim"
4. Click "Install" on Isaac Sim 4.0+
5. Wait for download (~30GB)
```

#### Installation Location

Default installation path:

```bash
~/.local/share/ov/pkg/isaac-sim-4.0.0/
```

#### Verify Installation

```bash
# Navigate to Isaac Sim directory
cd ~/.local/share/ov/pkg/isaac-sim-4.0.0/

# Run Isaac Sim
./isaac-sim.sh
```

First launch will take 5-10 minutes (shader compilation).

### Step 3: Install Isaac Sim Python Environment

Isaac Sim includes a Python environment for scripting:

```bash
# Activate Isaac Sim Python
cd ~/.local/share/ov/pkg/isaac-sim-4.0.0/
source setup_python_env.sh

# Verify installation
python -c "from omni.isaac.core import World; print('Isaac Sim Python OK')"
```

### Step 4: Install Isaac Examples (Optional)

```bash
# Clone Isaac Sim examples repository
cd ~/Documents
git clone https://github.com/NVIDIA-Omniverse/IsaacSim-Samples.git

# These provide excellent starting templates
```

## Understanding the Isaac Sim Interface

### Main Window Layout

```
┌─────────────────────────────────────────────────────────┐
│  Menu Bar                                                │
├───────────┬─────────────────────────┬────────────────────┤
│           │                         │                    │
│  Stage    │                         │   Property         │
│  Tree     │     Viewport            │   Panel            │
│           │     (3D View)           │                    │
│           │                         │                    │
├───────────┴─────────────────────────┴────────────────────┤
│  Timeline                                                │
└─────────────────────────────────────────────────────────┘
```

#### Panel Descriptions

- **Menu Bar**: Contains all major Isaac Sim functions (File, Edit, View, Window, etc.)
- **Stage Tree**: Hierarchical view of all objects in the scene
- **Viewport**: 3D visualization window where the simulation runs
- **Property Panel**: Edit properties of selected objects
- **Timeline**: Control simulation time, playback, and animation

### Essential Interface Elements

#### Stage Tree Operations

- **Right-click**: Context menu for object operations
- **Drag & Drop**: Reorganize objects in the hierarchy
- **Search**: Find specific objects using the search bar
- **Visibility**: Toggle object visibility using eye icons

#### Viewport Controls

- **Orbit**: Alt + Left mouse button (or middle mouse button)
- **Pan**: Alt + Right mouse button (or Shift + middle mouse button)
- **Zoom**: Mouse wheel or Alt + Middle mouse button
- **Frame Selected**: F key to focus on selected object

#### Property Panel Features

- **Transform**: Position, rotation, scale properties
- **Materials**: Surface appearance and physics properties
- **Physics**: Collision, mass, and dynamics parameters
- **Scripts**: Custom behavior attachments

## Importing Robots into Isaac Sim

### Supported Robot Formats

Isaac Sim supports multiple robot description formats:

- **URDF**: Universal Robot Description Format (ROS standard)
- **SDF**: Simulation Description Format (Gazebo standard)
- **USD**: Universal Scene Description (Omniverse native)
- **FBX/OBJ**: 3D model formats for visual-only assets

### Importing a URDF Robot

#### Method 1: Through Menu

1. **File → Import → Robot Description**
2. **Select URDF file** from your file system
3. **Configure import settings**:
   - Robot name and namespace
   - Joint position limits
   - Collision mesh generation
   - Visual scaling options

#### Method 2: Using Python API

```python
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage

# Import robot from URDF
add_reference_to_stage(
    usd_path="path/to/your/robot.urdf",
    prim_path="/World/Robot"
)
```

### Robot Configuration After Import

#### Joint Setup

```python
from omni.isaac.core.articulations import Articulation

# Get reference to imported robot
robot = world.scene.get_object("Robot")

# Configure joint properties
for joint_name in robot.dof_names:
    robot.set_joint_position_soft_limits(
        joint_names=[joint_name],
        lower_positions=[-2.0],
        upper_positions=[2.0]
    )
```

#### Sensor Integration

After importing, you can add sensors to your robot:

```python
from omni.isaac.sensor import Camera

# Add RGB camera to robot
camera = Camera(
    prim_path="/World/Robot/Camera",
    position=np.array([0.0, 0.0, 0.5]),
    orientation=np.array([0, 0, 0, 1])
)
```

## Creating Your First Simulation

### Basic Scene Setup

#### 1. Create a Simple Environment

```python
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_primitive

# Create ground plane
create_primitive(
    prim_path="/World/Ground",
    primitive_type="Plane",
    scale=np.array([10.0, 10.0, 1.0]),
    color=np.array([0.8, 0.8, 0.8])
)

# Create simple obstacles
create_primitive(
    prim_path="/World/Box",
    primitive_type="Cube",
    position=np.array([2.0, 0.0, 0.5]),
    scale=np.array([1.0, 1.0, 1.0]),
    color=np.array([0.5, 0.2, 0.2])
)
```

#### 2. Add Physics Properties

```python
from omni.isaac.core.utils.physics import add_ground_plane

# Add ground plane with physics
add_ground_plane(
    prim_path="/World/GroundPlane",
    size=15.0,
    color=np.array([0.1, 0.1, 0.1])
)
```

### Running the Simulation

#### Using the GUI

1. **Press Play** button in the timeline
2. **Adjust simulation speed** using real-time factor
3. **Pause** or **Stop** as needed
4. **Reset** to initial conditions

#### Using Python Script

```python
# Initialize the world
world = World(stage_units_in_meters=1.0)

# Add your robot and objects
# ... (add robot and scene objects)

# Simulation loop
for i in range(1000):
    # Perform actions
    robot.apply_action(robot_action)

    # Step the world
    world.step(render=True)

    # Get observations
    observations = robot.get_observations()
```

## Basic Physics and Sensors

### Physics Configuration

#### Global Physics Settings

```python
from omni.isaac.core import World

# Configure physics parameters
world = World(stage_units_in_meters=1.0)
world.scene.enable_collisions()

# Set gravity
world.physics_sim_view.set_gravity(
    np.array([0.0, 0.0, -9.81])
)
```

#### Material Properties

```python
from omni.isaac.core.utils.materials import add_material

# Create custom material
add_material(
    prim_path="/World/Looks/CustomMaterial",
    color=(0.8, 0.2, 0.2),
    roughness=0.1,
    metallic=0.9
)
```

### Common Sensors

#### RGB Camera

```python
from omni.isaac.sensor import Camera

camera = Camera(
    prim_path="/World/Camera",
    position=np.array([1.0, 0.0, 1.0]),
    frequency=30  # Hz
)

# Get camera data
rgb_data = camera.get_rgb()
```

#### LiDAR Sensor

```python
from omni.isaac.range_sensor import LidarRtx

lidar = LidarRtx(
    prim_path="/World/Lidar",
    translation=np.array([0, 0, 1.0]),
    orientation=np.array([0, 0, 0, 1]),
    config="Example_Rotary_Mechanical_Lidar",
    rotation_frequency=20,
    channels=16,
    points_per_channel=512
)

# Get point cloud data
point_cloud = lidar.get_point_cloud()
```

## Troubleshooting Common Issues

### Installation Problems

#### Shader Compilation Issues
- **Symptom**: Isaac Sim hangs during startup
- **Solution**: Ensure proper NVIDIA drivers and patience (first run takes time)

#### GPU Memory Issues
- **Symptom**: "Out of memory" errors
- **Solution**: Close other GPU-intensive applications

#### Python Environment Issues
- **Symptom**: ImportError when running Python scripts
- **Solution**: Source the Isaac Sim Python environment properly

### Runtime Issues

#### Slow Performance
- **Cause**: High scene complexity or insufficient GPU power
- **Solution**: Reduce scene complexity or use lower quality settings

#### Physics Instability
- **Cause**: Large time steps or incorrect mass properties
- **Solution**: Use smaller time steps and verify robot masses

## Best Practices

### Performance Optimization

- **Use simplified collision meshes** for complex robots
- **Limit the number of active sensors** during development
- **Use level-of-detail (LOD)** for distant objects
- **Optimize USD file sizes** by removing unnecessary data

### Development Workflow

1. **Start simple**: Begin with basic shapes before importing complex robots
2. **Test incrementally**: Add features one at a time
3. **Use version control**: Track USD scene changes
4. **Document configurations**: Keep notes on working parameters

## Summary

In this chapter, you learned:

- ✅ How to install and set up Isaac Sim
- ✅ Navigate the Isaac Sim interface and key panels
- ✅ Import robots using URDF and other formats
- ✅ Create basic simulation environments
- ✅ Add physics and sensors to your simulation
- ✅ Troubleshoot common issues
- ✅ Best practices for efficient development

## Next Steps

Now that you have Isaac Sim running and understand the basics, you're ready to dive deeper into photorealistic rendering and synthetic data generation!

**Continue to:** [Chapter 3.3: Photorealistic Rendering & Synthetic Data →](chapter-3-3-photorealistic-rendering)

## Additional Resources

- [Isaac Sim Quick Start Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_intro.html)
- [Isaac Sim Python API Documentation](https://docs.omniverse.nvidia.com/py/isaacsim/)
- [USD Primer](https://graphics.pixar.com/usd/docs/USD-Glossary-and-Background.html)
- [Omniverse Troubleshooting Guide](https://docs.omniverse.nvidia.com/isaacsim/latest/troubleshooting.html)