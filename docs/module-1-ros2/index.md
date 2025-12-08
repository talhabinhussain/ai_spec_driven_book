---
title: Module 1 - The Robotic Nervous System (ROS 2)
description: Building robotic nervous systems with nodes, topics, and services
sidebar_position: 1
sidebar_label: "Module Overview"
---

# The Robotic Nervous System (ROS 2)

## Introduction

Just as the human nervous system coordinates communication between the brain and body, ROS 2 (Robot Operating System 2) serves as the middleware that enables seamless communication between different components of a robotic system. This module introduces you to the architectural foundation that powers modern robotics, teaching you how to build distributed systems where perception, planning, and control work in harmony.

ROS 2 represents a paradigm shift from traditional monolithic robot control systems to a modular, message-passing architecture. By the end of this module, you'll understand how to decompose complex robotic behaviors into manageable nodes that communicate through topics and services.

## Module Focus

Middleware for robot control and inter-process communication.

## Key Topics

- **ROS 2 Nodes, Topics, and Services** - Understanding the publish-subscribe pattern and service-based communication
- **Bridging Python Agents to ROS Controllers** - Using rclpy to integrate AI models with robotic control systems
- **Understanding URDF** - Unified Robot Description Format for defining humanoid robot structure

## Learning Objectives

- Master ROS 2 architecture and core concepts
- Create nodes, topics, services, and actions
- Build ROS 2 packages with Python
- Use launch files and parameter management
- Understand URDF and SDF robot description formats

## Chapters

### Chapter 1.1: Introduction to ROS 2 Architecture
- [Chapter 1.1: Introduction to ROS 2 Architecture](./chapter-1-1-ros2-architecture.md)
- The evolution from ROS 1 to ROS 2
- Core concepts: Nodes, topics, services, and actions
- The DDS (Data Distribution Service) middleware
- Understanding the computation graph

### Chapter 1.2: Building Your First ROS 2 Node
- [Chapter 1.2: Building Your First ROS 2 Node](./chapter-1-2-first-ros2-node.md)
- Setting up the ROS 2 development environment
- Creating publisher and subscriber nodes
- Understanding message types and custom messages
- Debugging with `ros2 topic` and `rqt_graph`

### Chapter 1.3: Services and Actions in ROS 2
- [Chapter 1.3: ROS 2 Communication Patterns](./chapter-1-3-communication-patterns.md)
- Request-response patterns with services
- Long-running tasks with actions
- Action servers and clients
- Feedback mechanisms and goal cancellation

### Chapter 1.4: Python Integration with rclpy
- [Chapter 1.4: ROS 2 Message Types and Data Structures](./chapter-1-4-message-types.md)
- The rclpy API and node lifecycle
- Timers and callbacks
- Parameter management and dynamic reconfiguration
- Best practices for Python-based robot control

### Chapter 1.5: URDF and Robot Description
- [Chapter 1.5: ROS 2 Parameters and Configuration](./chapter-1-5-parameters.md)
- XML structure of URDF files
- Links, joints, and kinematic chains
- Visual and collision geometries
- Xacro: Macros for modular robot descriptions
- Defining humanoid robot structure

### Chapter 1.6: Launch Files and System Composition
- [Chapter 1.6: ROS 2 Launch Files and System Orchestration](./chapter-1-6-launch-files.md)
- Writing launch files in Python
- Parameter files and configurations
- Multi-node orchestration
- Namespace management for multiple robots

## Course Timeline - Weeks 3-5: ROS 2 Fundamentals

- ROS 2 architecture and core concepts
- Nodes, topics, services, and actions
- Building ROS 2 packages with Python
- Launch files and parameter management

## Assessment

Complete the ROS 2 Package Development Project - Build a multi-node system with publishers, subscribers, services, and actions.