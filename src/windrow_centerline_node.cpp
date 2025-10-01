#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <algorithm>
#include <vector>
#include <map>
#include <cmath>

struct GridCell {
  std::vector<float> z_values;
  double sum_z = 0.0;
  int count = 0;
  
  void addPoint(float z) {
    z_values.push_back(z);
    sum_z += z;
    count++;
  }
  
  double getMedianZ() const {
    if (z_values.empty()) return 0.0;
    
    std::vector<float> sorted_z = z_values;
    std::sort(sorted_z.begin(), sorted_z.end());
    
    size_t n = sorted_z.size();
    if (n % 2 == 0) {
      return (sorted_z[n/2 - 1] + sorted_z[n/2]) / 2.0;
    } else {
      return sorted_z[n/2];
    }
  }
  
  double getMeanZ() const {
    return count > 0 ? sum_z / count : 0.0;
  }
  
  // Get top K% points by height
  std::vector<float> getTopKPercent(double k_percent) const {
    if (z_values.empty()) return {};
    
    std::vector<float> sorted_z = z_values;
    std::sort(sorted_z.rbegin(), sorted_z.rend()); // Sort descending
    
    size_t k_count = std::max(1, static_cast<int>(z_values.size() * k_percent / 100.0));
    k_count = std::min(k_count, sorted_z.size());
    
    return std::vector<float>(sorted_z.begin(), sorted_z.begin() + k_count);
  }
};

class WindrowCenterlineNode : public rclcpp::Node
{
public:
  WindrowCenterlineNode()
  : Node("windrow_centerline_node")
  {
    // Parameters
    this->declare_parameter<std::string>("input_topic", "/ouster/points/filtered");
    this->declare_parameter<std::string>("target_frame", "base_link");
    this->declare_parameter<double>("grid_resolution", 0.10);
    this->declare_parameter<double>("x_grid_resolution", 0.40);
    this->declare_parameter<double>("y_grid_resolution", 1.0);
    this->declare_parameter<double>("y_min", 4.0);
    this->declare_parameter<double>("y_max", 20.0);
    this->declare_parameter<double>("x_min", -2.0);
    this->declare_parameter<double>("x_max", 2.0);
    this->declare_parameter<double>("ridge_keep_percent", 30.0);
    this->declare_parameter<bool>("use_median", false); // true for median, false for mean
    this->declare_parameter<int>("min_points_per_cell", 3);
    
    loadParameters();
    
    // TF setup
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Publishers
    auto sensor_qos = rclcpp::SensorDataQoS();
    centerline_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("windrow_centerline", 10);
    ridge_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ridge_points", sensor_qos);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("windrow_markers", 10);
    
    // Subscriber
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, sensor_qos,
      std::bind(&WindrowCenterlineNode::pointCloudCallback, this, std::placeholders::_1));
    
    // Visualization timer
    viz_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&WindrowCenterlineNode::publishVisualization, this));
    
    // Parameter callback
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&WindrowCenterlineNode::parameterCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Windrow centerline node initialized");
    RCLCPP_INFO(this->get_logger(), "Grid: y[%.1f, %.1f], x[%.1f, %.1f], res=(x=%.2fm, y=%.2fm)", 
                y_min_, y_max_, x_min_, x_max_, x_grid_resolution_, y_grid_resolution_);
    RCLCPP_INFO(this->get_logger(), "Ridge keep: %.1f%%, %s height metric", 
                ridge_keep_percent_, use_median_ ? "median" : "mean");
  }

private:
  void loadParameters() {
    input_topic_ = this->get_parameter("input_topic").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    grid_resolution_ = this->get_parameter("grid_resolution").as_double();
    x_grid_resolution_ = this->get_parameter("x_grid_resolution").as_double();
    y_grid_resolution_ = this->get_parameter("y_grid_resolution").as_double();
    y_min_ = this->get_parameter("y_min").as_double();
    y_max_ = this->get_parameter("y_max").as_double();
    x_min_ = this->get_parameter("x_min").as_double();
    x_max_ = this->get_parameter("x_max").as_double();
    ridge_keep_percent_ = this->get_parameter("ridge_keep_percent").as_double();
    use_median_ = this->get_parameter("use_median").as_bool();
    min_points_per_cell_ = this->get_parameter("min_points_per_cell").as_int();
    
    // Calculate grid dimensions
    x_cells_ = static_cast<int>((x_max_ - x_min_) / x_grid_resolution_) + 1;
    y_cells_ = static_cast<int>((y_max_ - y_min_) / y_grid_resolution_) + 1;
  }
  
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
    // Transform to target frame if needed
    std::string source_frame = cloud_msg->header.frame_id;
    if (!target_frame_.empty() && source_frame != target_frame_) {
      try {
        geometry_msgs::msg::TransformStamped tf_stamped =
          tf_buffer_->lookupTransform(target_frame_, source_frame, tf2::TimePointZero);
        Eigen::Isometry3d T_eig = tf2::transformToEigen(tf_stamped.transform);
        pcl::transformPointCloud(*cloud, *cloud, T_eig.cast<float>().matrix());
        source_frame = target_frame_;
      } catch (const std::exception &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "TF lookup failed (%s -> %s): %s", source_frame.c_str(), target_frame_.c_str(), ex.what());
        return;
      }
    }
    
    // Process windrow detection
    processWindrowDetection(cloud, cloud_msg->header, source_frame);
  }
  
  void processWindrowDetection(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std_msgs::msg::Header& header,
    const std::string& frame_id)
  {
    // Clear previous data
    grid_.clear();
    centerline_points_.clear();
    ridge_points_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    
    // Build 2D height grid
    buildHeightGrid(cloud);
    
    // Find ridge centerline
    findRidgeCenterline();
    
    // Extract ridge points
    extractRidgePoints(cloud);
    
    // Publish results
    publishCenterline(header, frame_id);
    publishRidgePoints(header, frame_id);
    
    current_header_ = header;
    current_frame_id_ = frame_id;
  }
  
  void buildHeightGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    // Initialize grid
    grid_.resize(x_cells_);
    for (auto& row : grid_) {
      row.resize(y_cells_);
    }
    
    // Populate grid with points
    for (const auto& point : cloud->points) {
      // Check if point is within grid bounds
      if (point.x < x_min_ || point.x > x_max_ || 
          point.y < y_min_ || point.y > y_max_) {
        continue;
      }
      
      // Calculate grid indices
      int x_idx = static_cast<int>((point.x - x_min_) / x_grid_resolution_);
      int y_idx = static_cast<int>((point.y - y_min_) / y_grid_resolution_);
      
      // Clamp indices
      x_idx = std::max(0, std::min(x_idx, x_cells_ - 1));
      y_idx = std::max(0, std::min(y_idx, y_cells_ - 1));
      
      // Add point to grid cell
      grid_[x_idx][y_idx].addPoint(point.z);
    }
  }
  
  void findRidgeCenterline()
  {
    centerline_points_.clear();
    
    // For each y-row (near to far), find x with maximum height metric
    for (int y_idx = 0; y_idx < y_cells_; ++y_idx) {
      double max_height = -std::numeric_limits<double>::max();
      int best_x_idx = -1;
      
      // Scan from left (-x) to right (+x) in this y-row
      for (int x_idx = 0; x_idx < x_cells_; ++x_idx) {
        const auto& cell = grid_[x_idx][y_idx];
        if (cell.count < min_points_per_cell_) continue;
        
        double height = use_median_ ? cell.getMedianZ() : cell.getMeanZ();
        if (height > max_height) {
          max_height = height;
          best_x_idx = x_idx;
        }
      }
      
      if (best_x_idx >= 0) {
        double x = x_min_ + best_x_idx * x_grid_resolution_;
        double y = y_min_ + y_idx * y_grid_resolution_;
        
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = max_height;
        pose.orientation.w = 1.0;
        centerline_points_.push_back(pose);
      }
    }
  }
  
  void extractRidgePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    ridge_points_cloud_->clear();
    
    for (const auto& centerline_point : centerline_points_) {
      // Find grid cell for this centerline point
      int x_idx = static_cast<int>((centerline_point.position.x - x_min_) / x_grid_resolution_);
      int y_idx = static_cast<int>((centerline_point.position.y - y_min_) / y_grid_resolution_);
      
      if (x_idx < 0 || x_idx >= x_cells_ || y_idx < 0 || y_idx >= y_cells_) continue;
      
      const auto& cell = grid_[x_idx][y_idx];
      auto top_z_values = cell.getTopKPercent(ridge_keep_percent_);
      
      if (top_z_values.empty()) continue;
      
      // Find threshold for top K% points
      float z_threshold = top_z_values.back(); // Minimum z in top K%
      
      // Add all points in nearby area that meet height threshold
      double search_radius = std::min(x_grid_resolution_, y_grid_resolution_) * 0.7; // Slightly smaller than cell size
      
      for (const auto& point : cloud->points) {
        double dx = point.x - centerline_point.position.x;
        double dy = point.y - centerline_point.position.y;
        
        if (std::sqrt(dx*dx + dy*dy) <= search_radius && point.z >= z_threshold) {
          ridge_points_cloud_->push_back(point);
        }
      }
    }
  }
  
  void publishCenterline(const std_msgs::msg::Header& header, const std::string& frame_id)
  {
    geometry_msgs::msg::PoseArray centerline_msg;
    centerline_msg.header = header;
    centerline_msg.header.frame_id = frame_id;
    centerline_msg.poses = centerline_points_;
    
    centerline_pub_->publish(centerline_msg);
  }
  
  void publishRidgePoints(const std_msgs::msg::Header& header, const std::string& frame_id)
  {
    if (ridge_points_cloud_->empty()) return;
    
    ridge_points_cloud_->width = ridge_points_cloud_->size();
    ridge_points_cloud_->height = 1;
    ridge_points_cloud_->is_dense = true;
    
    sensor_msgs::msg::PointCloud2 ridge_msg;
    pcl::toROSMsg(*ridge_points_cloud_, ridge_msg);
    ridge_msg.header = header;
    ridge_msg.header.frame_id = frame_id;
    
    ridge_points_pub_->publish(ridge_msg);
  }
  
  void publishVisualization()
  {
    if (centerline_points_.empty()) return;
    
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Centerline path marker
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = current_frame_id_;
    line_marker.header.stamp = this->now();
    line_marker.ns = "windrow_centerline";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.1; // Line width
    line_marker.color.r = 1.0;
    line_marker.color.g = 0.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 0.8;
    line_marker.lifetime = rclcpp::Duration(1, 0);
    
    for (const auto& pose : centerline_points_) {
      line_marker.points.push_back(pose.position);
    }
    marker_array.markers.push_back(line_marker);
    
    // Centerline points marker
    visualization_msgs::msg::Marker points_marker;
    points_marker.header.frame_id = current_frame_id_;
    points_marker.header.stamp = this->now();
    points_marker.ns = "windrow_points";
    points_marker.id = 1;
    points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    points_marker.action = visualization_msgs::msg::Marker::ADD;
    points_marker.scale.x = 0.2;
    points_marker.scale.y = 0.2;
    points_marker.scale.z = 0.2;
    points_marker.color.r = 1.0;
    points_marker.color.g = 1.0;
    points_marker.color.b = 0.0;
    points_marker.color.a = 0.9;
    points_marker.lifetime = rclcpp::Duration(1, 0);
    
    for (const auto& pose : centerline_points_) {
      points_marker.points.push_back(pose.position);
    }
    marker_array.markers.push_back(points_marker);
    
    marker_pub_->publish(marker_array);
  }
  
  rcl_interfaces::msg::SetParametersResult parameterCallback(
    const std::vector<rclcpp::Parameter>& parameters)
  {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    
    bool needs_reload = false;
    for (const auto& param : parameters) {
      if (param.get_name().find("grid_") != std::string::npos ||
          param.get_name().find("_min") != std::string::npos ||
          param.get_name().find("_max") != std::string::npos ||
          param.get_name() == "ridge_keep_percent" ||
          param.get_name() == "use_median" ||
          param.get_name() == "min_points_per_cell") {
        needs_reload = true;
      }
    }
    
    if (needs_reload) {
      loadParameters();
      RCLCPP_INFO(this->get_logger(), "Parameters updated - Grid: y[%.1f, %.1f], x[%.1f, %.1f], res=%.2fm", 
                  y_min_, y_max_, x_min_, x_max_, grid_resolution_);
    }
    
    return result;
  }

  // ROS components
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr centerline_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ridge_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr viz_timer_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  
  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Parameters
  std::string input_topic_;
  std::string target_frame_;
  double grid_resolution_;
  double x_grid_resolution_;
  double y_grid_resolution_;
  double y_min_, y_max_, x_min_, x_max_;
  double ridge_keep_percent_;
  bool use_median_;
  int min_points_per_cell_;
  int x_cells_, y_cells_;
  
  // Processing data
  std::vector<std::vector<GridCell>> grid_;
  std::vector<geometry_msgs::msg::Pose> centerline_points_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr ridge_points_cloud_;
  
  // Visualization
  std_msgs::msg::Header current_header_;
  std::string current_frame_id_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WindrowCenterlineNode>());
  rclcpp::shutdown();
  return 0;
}
