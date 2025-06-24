# Scan Matcher Package

This package provides ICP-based laser scan matching functionality for ROS2, with optional odometry integration for improved pose estimation.

## Overview

The scan matcher performs Iterative Closest Point (ICP) algorithm on consecutive laser scans to estimate robot motion and pose. It can optionally use odometry data to provide better initial guesses for the ICP algorithm.

## Features

- ICP-based scan matching using PCL library
- Optional odometry integration for initial pose guess
- Configurable ICP parameters (correspondence distance, convergence criteria, etc.)
- Point cloud filtering (voxel grid, outlier removal)
- TF2 integration for pose broadcasting
- ROS2 component architecture for flexible deployment

## Nodes

### scan_matcher_node

The main scan matching node that processes laser scans and publishes pose estimates.

#### Subscribed Topics

- `scan` (sensor_msgs/LaserScan): Input laser scan data
- `odom` (nav_msgs/Odometry): Optional odometry data for initial guess

#### Published Topics

- `scan_match_pose` (geometry_msgs/PoseWithCovarianceStamped): Estimated pose from scan matching
- `scan_match_velocity` (geometry_msgs/TwistWithCovarianceStamped): Estimated velocity from scan matching

#### Parameters

- `laser_frame` (string, default: "laser"): Frame ID of the laser scanner
- `base_frame` (string, default: "base_link"): Frame ID of the robot base
- `odom_frame` (string, default: "odom"): Frame ID of the odometry
- `map_frame` (string, default: "map"): Frame ID of the map
- `use_odometry` (bool, default: true): Whether to use odometry for initial guess
- `publish_tf` (bool, default: true): Whether to publish transform to TF
- `max_correspondence_distance` (double, default: 0.5): Maximum distance for point correspondences
- `transformation_epsilon` (double, default: 1e-6): Transformation convergence criteria
- `euclidean_fitness_epsilon` (double, default: 1e-6): Fitness score convergence criteria
- `max_iterations` (int, default: 50): Maximum ICP iterations
- `min_scan_range` (double, default: 0.1): Minimum valid scan range
- `max_scan_range` (double, default: 30.0): Maximum valid scan range
- `voxel_size` (double, default: 0.05): Voxel grid filter size

## Usage

### Basic Usage

```bash
ros2 launch scan_matcher scan_matcher.launch.py
```

### With Custom Configuration

```bash
ros2 launch scan_matcher scan_matcher.launch.py config_file:=/path/to/your/config.yaml
```

### With RViz Visualization

```bash
ros2 launch scan_matcher scan_matcher_with_rviz.launch.py
```

### Custom Topics

```bash
ros2 launch scan_matcher scan_matcher.launch.py laser_topic:=/your_scan_topic odom_topic:=/your_odom_topic
```

## Configuration

The package includes a default configuration file at `config/scan_matcher.yaml`. You can modify this file or create your own configuration file with custom parameters.

Example configuration:
```yaml
scan_matcher:
  ros__parameters:
    laser_frame: "laser"
    base_frame: "base_link"
    max_correspondence_distance: 0.5
    max_iterations: 50
    voxel_size: 0.05
```

## Architecture

### Classes

- **ScanMatcherNode**: Main ROS2 node handling subscriptions, publications, and coordination
- **ICPProcessor**: Core ICP algorithm implementation using PCL
- **LaserScanUtils**: Utility functions for laser scan processing and validation

### Dependencies

- ROS2 (rclcpp, sensor_msgs, geometry_msgs, nav_msgs, tf2)
- PCL (Point Cloud Library) for ICP algorithms
- Eigen3 for mathematical operations

## Building

```bash
cd /path/to/your/ros2_workspace
colcon build --packages-select scan_matcher
source install/setup.bash
```

## Testing

Run the unit tests:
```bash
colcon test --packages-select scan_matcher
colcon test-result --verbose
```

## Contributing

When implementing the actual ICP processing logic:

1. Complete the `process_scans` method in `ICPProcessor`
2. Implement proper error handling and convergence checking
3. Add covariance estimation based on ICP results
4. Implement motion prediction using odometry data
5. Add loop closure detection capabilities

## License

This package is licensed under the MIT License.
