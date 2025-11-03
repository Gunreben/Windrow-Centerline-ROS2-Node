# Windrow Centerline Node

ROS2 node for detecting windrow centerlines using a weighted centroid approach. This node implements a 2D grid-based algorithm that identifies the centerline by computing weighted centroids of the highest points in agricultural windrows.

## Installation
Clone this repository:
```bash
git clone https://github.com/Gunreben/Windrow-Centerline-ROS2-Node.git
```

Make sure ROS Pointcloud is installed:
```bash
sudo apt install ros-<distro>-pcl-ros
```
Build:
```bash
cd ~/ros2_ws/ && colcon build --packages-select windrow_centerline_node
```

## Algorithm

The windrow detection uses a **Weighted Centroid** approach:

1. **Build 2D Grid**: Creates a grid over the specified area with configurable per-axis resolution (x and y)
2. **Height Analysis**: For each y-row (near to far), computes the height distribution across all x-cells
3. **Threshold Calculation**: Determines a height threshold per row using a percentile-based approach
4. **Weighted Centroid**: Computes the centerline x-position by taking a weighted average of all cells above the threshold, where weights are proportional to height above threshold
5. **Smoothing**: Applies moving average smoothing to reduce noise in the centerline
6. **Ridge Extraction**: Around each centerline point, keeps the top K% of points by height

This method provides more robust and accurate centerline detection compared to simple maximum-finding, especially when windrows have irregular shapes or varying heights.

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_topic` | string | `/ouster/points/filtered` | Input point cloud topic |
| `output_centerline_topic` | string | `windrow_centerline` | Output centerline poses topic |
| `output_ridge_points_topic` | string | `ridge_points` | Output ridge points cloud topic |
| `output_markers_topic` | string | `windrow_markers` | Output visualization markers topic |
| `target_frame` | string | `base_link` | Target coordinate frame |
| `grid_resolution` | double | `0.10` | Legacy uniform grid resolution (use x/y below) |
| `x_grid_resolution` | double | `0.40` | Grid cell size in x (meters) |
| `y_grid_resolution` | double | `1.0` | Grid cell size in y (meters) |
| `y_min` | double | `4.0` | Minimum y-coordinate (meters) |
| `y_max` | double | `20.0` | Maximum y-coordinate (meters) |
| `x_min` | double | `-2.0` | Minimum x-coordinate (meters) |
| `x_max` | double | `2.0` | Maximum x-coordinate (meters) |
| `ridge_keep_percent` | double | `30.0` | Percentage of top points to keep as ridge |
| `use_median` | bool | `false` | Use median (true) or mean (false) for height |
| `min_points_per_cell` | int | `3` | Minimum points required per grid cell |
| `smoothing_window` | int | `5` | Window size for centerline smoothing |
| `height_threshold_percentile` | double | `0.5` | Percentile for height threshold (0.0-1.0) |

## Topics

### Subscribed
- `input_topic` (`sensor_msgs/PointCloud2`): Filtered point cloud input (configurable via parameter)

### Published
- `output_centerline_topic` (`geometry_msgs/PoseArray`): Detected centerline poses (default: `windrow_centerline`)
- `output_ridge_points_topic` (`sensor_msgs/PointCloud2`): Ridge points used for detection (default: `ridge_points`)
- `output_markers_topic` (`visualization_msgs/MarkerArray`): Visualization markers for RViz (default: `windrow_markers`)

**Note:** All topic names are configurable via parameters.

## Usage

### Basic Usage
```bash
ros2 run windrow_centerline_node windrow_centerline_node
```

### With Custom Parameters
```bash
ros2 run windrow_centerline_node windrow_centerline_node --ros-args \
  -p input_topic:=/filtered_points \
  -p output_centerline_topic:=/custom/centerline \
  -p output_ridge_points_topic:=/custom/ridge_points \
  -p output_markers_topic:=/custom/markers \
  -p target_frame:=base_link \
  -p x_grid_resolution:=0.40 \
  -p y_grid_resolution:=1.0 \
  -p y_min:=4.0 \
  -p y_max:=20.0 \
  -p ridge_keep_percent:=25.0
```

### Integration with [Lidar Filter](https://github.com/Gunreben/lidar_filter)
The integration with LiDAR Filter is optional, since this node runs within the predefined boundaries.
Since we use the filtering node as preprocessing for different applications you can use it for marginal performance improvements, if you plan to use this node in a bigger software stack.
```bash
# Terminal 1: Run lidar filter
ros2 run lidar_filter combined_lidar_filter_node --ros-args \
  -p target_frame:=base_link \
  -p input_topic:=/points \
  -p output_topic:=/filtered_points

# Terminal 2: Run windrow detection
ros2 run windrow_centerline_node windrow_centerline_node --ros-args \
  -p input_topic:=/filtered_points
```

## Visualization in RViz

1. **Centerline Path**: Add "PoseArray" display, topic `/windrow_centerline`
2. **Ridge Points**: Add "PointCloud2" display, topic `/ridge_points` 
3. **Markers**: Add "MarkerArray" display, topic `/windrow_markers`
   - Red line: Centerline path
   - Yellow spheres: Centerline points

## Grid Configuration

The detection area is defined by:
- **X-range**: Typically `-2.0` to `2.0` meters (lateral extent)
- **Y-range**: Typically `4` to `20` meters (forward distance)
- **Resolution**: independent `x_grid_resolution` and `y_grid_resolution` (e.g., x=0.40m, y=1.0m)

Adjust these parameters based on:
- Vehicle width and sensor positioning
- Expected windrow dimensions
- Required detection precision
- Computational performance needs

## Algorithm Details

### Weighted Centroid Method
The centerline detection uses a sophisticated weighted centroid approach:
- **Per-row threshold**: Calculates a height threshold for each y-slice using the `height_threshold_percentile` parameter (default 50th percentile)
- **Weighted averaging**: Only cells above the threshold contribute to the centerline position
- **Linear weighting**: Each cell's contribution is weighted by its height above the threshold
- **Robustness**: This approach naturally handles irregular windrow shapes and varying heights

### Height Metric Selection
- **Median**: More robust to outliers, better for noisy data
- **Mean**: Faster computation, sensitive to point distribution

### Ridge Extraction
- Uses top `ridge_keep_percent`% of points by height in each selected cell
- Searches within `min(x_grid_resolution, y_grid_resolution) * 0.7` radius around centerline points
- Filters cells with fewer than `min_points_per_cell` points

## Performance

- Grid size: `(x_max - x_min) / grid_resolution × (y_max - y_min) / grid_resolution`
- Default: `40 × 110 = 4400` cells
- Processing time scales with point cloud density and grid resolution

## Dependencies

- ROS2 Humble
- PCL (Point Cloud Library)
- tf2_ros and tf2_eigen for coordinate transforms
- Standard ROS2 message types

## License

Apache-2.0

