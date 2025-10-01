# Windrow Centerline Node

ROS2 node for detecting windrow centerlines using height-grid ridge analysis. This node implements a 2D grid-based approach to find the highest points (ridges) in agricultural windrows and extracts the centerline path.

## Algorithm

The windrow detection uses a **Height-Grid Ridge** approach:

1. **Build 2D Grid**: Creates a grid over the specified area with configurable per-axis resolution
2. **Height Analysis (near-to-far)**: For each y-row (near to far), scans x from left→right and picks the x-cell with maximum height (median or mean z-value)
3. **Ridge Extraction**: Around each centerline point, keeps the top K% of points by height
4. **Centerline Generation**: Produces a continuous path representing the windrow center

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_topic` | string | `/filtered_points` | Input point cloud topic |
| `target_frame` | string | `base_link` | Target coordinate frame |
| `grid_resolution` | double | `0.10` | Legacy uniform grid resolution (use x/y below) |
| `x_grid_resolution` | double | `0.10` | Grid cell size in x (meters) |
| `y_grid_resolution` | double | `0.10` | Grid cell size in y (meters) |
| `y_min` | double | `1.0` | Minimum y-coordinate (meters) |
| `y_max` | double | `12.0` | Maximum y-coordinate (meters) |
| `x_min` | double | `-2.0` | Minimum x-coordinate (meters) |
| `x_max` | double | `2.0` | Maximum x-coordinate (meters) |
| `ridge_keep_percent` | double | `30.0` | Percentage of top points to keep as ridge |
| `use_median` | bool | `true` | Use median (true) or mean (false) for height |
| `min_points_per_cell` | int | `5` | Minimum points required per grid cell |

## Topics

### Subscribed
- `input_topic` (`sensor_msgs/PointCloud2`): Filtered point cloud input

### Published
- `windrow_centerline` (`geometry_msgs/PoseArray`): Detected centerline poses
- `ridge_points` (`sensor_msgs/PointCloud2`): Ridge points used for detection
- `windrow_markers` (`visualization_msgs/MarkerArray`): Visualization markers for RViz

## Usage

### Basic Usage
```bash
ros2 run windrow_centerline_node windrow_centerline_node
```

### With Custom Parameters
```bash
ros2 run windrow_centerline_node windrow_centerline_node --ros-args \
  -p input_topic:=/filtered_points \
  -p target_frame:=base_link \
  -p x_grid_resolution:=0.40 \
  -p y_grid_resolution:=1.0 \
  -p y_min:=2.0 \
  -p y_max:=10.0 \
  -p ridge_keep_percent:=25.0
```

### Integration with Lidar Filter
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
- **Y-range**: Typically `1.0` to `12.0` meters (forward distance)
- **Resolution**: independent `x_grid_resolution` and `y_grid_resolution` (e.g., x=0.40m, y=1.0m)

Adjust these parameters based on:
- Vehicle width and sensor positioning
- Expected windrow dimensions
- Required detection precision
- Computational performance needs

## Algorithm Details

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

