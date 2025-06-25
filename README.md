# vision_tracker

A ROS 2 node that simulates a 2D control system where a drone follows a target using object tracking.

## Features

- Subscribes to YOLO 2D bounding box detections
- Estimates the position and velocity of a follower drone (point mass)
- Uses a PD controller to follow the target

## Dependencies

- ROS 2 Humble
- [`yolo_ros`](https://github.com/mgonzs13/yolo_ros)
