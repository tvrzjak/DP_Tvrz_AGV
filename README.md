# Autonomous AGV Navigation Repository

This repository serves as a companion to a diploma thesis focused on the design and implementation of autonomous navigation systems for Automated Guided Vehicles (AGVs). Developed under the auspices of Leuze Engineering Czech s.r.o., the work addresses the need for efficient, flexible, and cost-effective navigation solutions in industrial environments, moving beyond traditional line-tracking methods. The repository provides practical examples of controllers, drivers, and simulations that demonstrate the implementation of mathematical methods described in the thesis.

## Overview

The diploma thesis, included as an attachment, explores the integration of theoretical algorithms, software tools (ROS 2 and Nav2), and hardware solutions (Leuze sensors) to create a modular navigation system for AGVs. It covers:
- **Algorithms**: Model predictive control, Pure Pursuit, LIDAR scan fusion, examples of parametrization of navigation stack and data clustering for trajectory control and object detection.
- **Hardware**: Four AGV prototypes with differential and mecanum wheel chassis, equipped with LIDARs for safety and navigation.
- **Software**: ROS 2 framework with Nav2 for localization and trajectory planning.
- **Validation**: Simulations in Gazebo and real-world testing in warehouse-like scenarios.

The repository complements the thesis by providing code samples for:
- Kinematics and controllers for differential and omnidirectional chassis.
- LIDAR scan merging for navigation.
- Simulation setups and driver templates.

## Purpose

This repository is not a complete software solution but a collection of implementation examples. It showcases how the mathematical methods and algorithms described in the thesis can be applied in practice, serving as a foundation or inspiration for developers building custom AGV navigation systems. The code includes partial implementations of regulators, drivers, and simulations, designed to be adapted for specific hardware and industrial requirements.

## Context

Developed as part of a diploma thesis at Leuze Engineering Czech s.r.o., this work contributes to the advancement of autonomous robotics by offering:
- A methodological guide for AGV navigation system design.
- Practical insights into ROS 2 and Nav2 integration.
- Examples of hardware and software components for industrial AGVs.

The thesis abstract and introduction provide a detailed theoretical and practical context, while this repository focuses on actionable code snippets and configurations.

## Usage

- **Thesis Reference**: Consult the attached diploma thesis for a comprehensive understanding of the algorithms, hardware, and system architecture.
- **Code Exploration**: Browse the repository for examples of ROS 2 packages, including chassis drivers, scan mergers, and simulation setups.
- **Customization**: Use the provided code as a template to develop tailored solutions for your AGV hardware and navigation needs.

## Notes

- The repository assumes familiarity with ROS 2, Nav2, and AGV hardware.
- Users must provide their own PLC drivers and adapt kinematics for specific chassis.
- The code is intended for educational and developmental purposes, not production deployment.

For further details, refer to the diploma thesis or contact [Leuze Engineering Czech s.r.o.](https://www.leuze-engineering.com/en/)