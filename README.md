# Dual Optical Flow Sensor Fusion for GPS-Denied UAV Navigation

## Overview

This repository contains the source code and firmware modifications developed for a Master's thesis investigating dual optical flow sensor fusion for GPS-denied autonomous flight using:

- Pixhawk 4
- PX4 v1.14.3
- Raspberry Pi 4
- ROS2 Humble
- Two Holybro H-Flow optical flow sensors

The system performs quality-weighted fusion of two optical flow sensors on the companion computer and publishes the fused result back into PX4 through a custom uORB topic.

## Repository Structure

text
ros2_nodes/
    fusion_node.py
    dmux_node.py
    offboard_manager.py
    position_controller.py
    logger_node.py
    dashboard_node.py
    pre_flight_check.py

px4_modifications/
    FusedOpticalFlow.msg
    VehicleOpticalFlow_changes.md
    dds_topics.yaml

diagrams/
    system_architecture.png
    fusion_pipeline.png
    controller_flowchart.png

docs/
    thesis.pdf

## Features

- Dual optical flow fusion
- Quality-weighted averaging
- Custom PX4 uORB topic integration
- ROS2 offboard control
- GPS-denied navigation
- Three-layer safety fallback architecture

## Thesis

The complete thesis describing the design, implementation and evaluation of the system is available in the `docs` directory.

## Author

Divyanshi Mishra

Master Thesis
University of Pisa/ Zurich University of Applied Sciences (ZHAW)
