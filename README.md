# Real-Time People Detection and Tracking System Using LIDAR and ROS2

## Project Overview

This project develops a real-time system for detecting and tracking people using LIDAR data within dynamic environments. Built on the ROS2 framework, it leverages nodes, topics, and custom messages to process raw sensor inputs into meaningful insights about the environment. The system consists of two primary nodes: one for detecting moving objects and another for tracking and counting these individuals.

### System Components

- **Node 1: Moving Object Detection** - Subscribes to `/scan` topic for LIDAR data, identifies moving objects using a threshold-based comparison, and employs Euclidean clustering to locate individuals, publishing centroids to `/person_location`.
- **Node 2: Tracking and Counting** - Subscribes to `/person_location` for detected individuals' centroids, tracks these individuals across frames, and counts unique occurrences, publishing the cumulative count to `/person_count`.

A launch file coordinates the initialization and termination of system components for synchronized operation.

## Getting Started

### Prerequisites

- ROS2 Humble Hawksbill
- PCL (Point Cloud Library) for Euclidean clustering
- OpenCV (optional) for additional visualization

### Design Choices and Rationale 
Real-Time Processing: Essential for dynamic environments to provide timely information.
Threshold-Based Movement Detection: Efficient at differentiating static and moving objects with minimal computational overhead.
Euclidean Clustering for Object Segmentation: Effectively separates individuals in crowded scenes.
Simple Tracking Algorithm with Constant Velocity Model: Balances accuracy and computational efficiency for real-time applications.
Cumulative Counting of Unique Individuals: Simplifies tracking over time without complex identity management.


### Parameters and Tuning
Movement Detection Threshold, Euclidean Clustering Parameters (distance tolerance, min/max cluster size), and Tracking Update Interval were empirically determined for optimal performance.
Results and Expectations
The system demonstrated high accuracy in detecting and tracking people in various test scenarios. Fine-tuning parameters allowed for significant improvements, especially in challenging conditions like crowded environments or at the edges of the LIDAR range.



### Acknowledgments
ROS2 Community for the comprehensive documentation and forums.
Contributors to the PCL and OpenCV libraries.
