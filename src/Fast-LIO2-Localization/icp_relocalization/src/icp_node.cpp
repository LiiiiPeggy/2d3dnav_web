#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#ifdef USE_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

class ICPNode : public rclcpp::Node
{
public:
  ICPNode() : Node("icp_node")
  {
    const double initial_x = declare_parameter<double>("initial_x", 0.0);
    const double initial_y = declare_parameter<double>("initial_y", 0.0);
    const double initial_z = declare_parameter<double>("initial_z", 0.0);
    const double initial_yaw = declare_parameter<double>("initial_yaw", 0.0);
    solver_max_iter_ = declare_parameter<int>("solver_max_iter", 75);
    max_correspondence_distance_ =
        declare_parameter<double>("max_correspondence_distance", 1.0);
    ransac_threshold_ =
        declare_parameter<double>("ransac_outlier_rejection_threshold", 0.5);
    map_path_ = declare_parameter<std::string>("map_path", "");
    map_frame_ = declare_parameter<std::string>("map_frame_id", "map");
    fitness_threshold_ = declare_parameter<double>("fitness_score_threshold", 0.30);
    map_voxel_size_ = declare_parameter<double>("map_voxel_leaf_size", 0.30);
    cloud_voxel_size_ = declare_parameter<double>("cloud_voxel_leaf_size", 0.20);
    required_convergences_ = declare_parameter<int>("required_convergences", 5);
    min_source_points_ = declare_parameter<int>("min_source_points", 100);
    initialpose_ground_to_robot_z_ =
        declare_parameter<double>("initialpose_ground_to_robot_z", 0.0);
    const std::string pcl_type = declare_parameter<std::string>("pcl_type", "livox");
    initial_frame_to_sensor_ = transformParameter(
        "sensor_translation_in_initial_frame", "sensor_rpy_in_initial_frame");

    if (map_path_.empty()) throw std::invalid_argument("map_path must not be empty");
    if (required_convergences_ < 1)
      throw std::invalid_argument("required_convergences must be at least 1");
    if (fitness_threshold_ <= 0.0 || max_correspondence_distance_ <= 0.0)
      throw std::invalid_argument("ICP distance thresholds must be positive");

    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    result_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "icp_sensor_result", rclcpp::QoS(1).reliable());
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("prior_map", map_qos);
    transformed_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "transformed_cloud", rclcpp::QoS(1).reliable());

#ifdef USE_LIVOX
    if (pcl_type == "livox")
    {
      lvx_cloud_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
          "livox_cloud", rclcpp::SensorDataQoS(),
          std::bind(&ICPNode::livoxCallback, this, std::placeholders::_1));
    }
    else
#endif
    {
      cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          "pointcloud", rclcpp::SensorDataQoS(),
          std::bind(&ICPNode::cloudCallback, this, std::placeholders::_1));
    }
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", rclcpp::QoS(1).reliable(),
        std::bind(&ICPNode::poseCallback, this, std::placeholders::_1));

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path_, *target_cloud_) != 0 ||
        target_cloud_->empty())
      throw std::runtime_error("failed to load non-empty prior PCD: " + map_path_);

    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(target_cloud_);
    voxel.setLeafSize(map_voxel_size_, map_voxel_size_, map_voxel_size_);
    voxel.filter(*target_cloud_);
    if (target_cloud_->empty())
      throw std::runtime_error("prior PCD is empty after voxel filtering");

    setInitialGuess(initial_x, initial_y, initial_z, initial_yaw);
    publishMap();
    RCLCPP_INFO(
        get_logger(), "ICP ready: %zu map points from %s; initial robot pose [%.2f %.2f %.2f %.2f]",
        target_cloud_->size(), map_path_.c_str(), initial_x, initial_y, initial_z, initial_yaw);
  }

private:
  static Eigen::Isometry3f poseMatrix(
      double x, double y, double z, const geometry_msgs::msg::Quaternion &orientation)
  {
    tf2::Quaternion quaternion;
    tf2::fromMsg(orientation, quaternion);
    if (quaternion.length2() < 1e-12) quaternion.setRPY(0.0, 0.0, 0.0);
    quaternion.normalize();
    const Eigen::Quaternionf rotation(
        quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z());
    Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
    transform.linear() = rotation.normalized().toRotationMatrix();
    transform.translation() = Eigen::Vector3f(x, y, z);
    return transform;
  }

  Eigen::Isometry3f transformParameter(
      const std::string &translation_name, const std::string &rpy_name)
  {
    const auto translation = declare_parameter<std::vector<double>>(
        translation_name, {0.0, 0.0, 0.0});
    const auto rpy = declare_parameter<std::vector<double>>(rpy_name, {0.0, 0.0, 0.0});
    if (translation.size() != 3 || rpy.size() != 3)
      throw std::invalid_argument("ICP extrinsic parameters must contain three values");
    const Eigen::AngleAxisf roll(static_cast<float>(rpy[0]), Eigen::Vector3f::UnitX());
    const Eigen::AngleAxisf pitch(static_cast<float>(rpy[1]), Eigen::Vector3f::UnitY());
    const Eigen::AngleAxisf yaw(static_cast<float>(rpy[2]), Eigen::Vector3f::UnitZ());
    Eigen::Isometry3f result = Eigen::Isometry3f::Identity();
    result.linear() = (yaw * pitch * roll).toRotationMatrix();
    result.translation() = Eigen::Vector3f(
        static_cast<float>(translation[0]), static_cast<float>(translation[1]),
        static_cast<float>(translation[2]));
    return result;
  }

  void setInitialGuess(double x, double y, double z, double yaw)
  {
    geometry_msgs::msg::Quaternion orientation;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);
    orientation = tf2::toMsg(quaternion);
    // /initialpose describes the robot frame. ICP aligns sensor points, so the
    // initial guess is T_map_robot * T_robot_sensor.
    initial_guess_ =
        (poseMatrix(x, y, z, orientation) * initial_frame_to_sensor_).matrix();
    converged_count_ = 0;
    localization_complete_ = false;
  }

  void poseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_)
    {
      RCLCPP_ERROR(
          get_logger(), "Ignoring initial pose in '%s'; expected '%s'",
          message->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    initial_guess_ =
        (poseMatrix(
             message->pose.pose.position.x, message->pose.pose.position.y,
             message->pose.pose.position.z + initialpose_ground_to_robot_z_,
             message->pose.pose.orientation) *
         initial_frame_to_sensor_)
            .matrix();
    converged_count_ = 0;
    localization_complete_ = false;
    RCLCPP_INFO(
        get_logger(),
        "Initial robot pose updated (clicked z + %.3f m body offset); ICP will restart",
        initialpose_ground_to_robot_z_);
  }

  void publishMap()
  {
    pcl::toROSMsg(*target_cloud_, target_cloud_message_);
    target_cloud_message_.header.stamp = now();
    target_cloud_message_.header.frame_id = map_frame_;
    map_pub_->publish(target_cloud_message_);
  }

  void align(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source)
  {
    if (localization_complete_) return;
    if (!source || static_cast<int>(source->size()) < min_source_points_)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "ICP source cloud has too few effective points");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(source);
    voxel.setLeafSize(cloud_voxel_size_, cloud_voxel_size_, cloud_voxel_size_);
    voxel.filter(*filtered);
    if (static_cast<int>(filtered->size()) < min_source_points_) return;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(filtered);
    icp.setInputTarget(target_cloud_);
    icp.setMaximumIterations(solver_max_iter_);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
    icp.setRANSACOutlierRejectionThreshold(ransac_threshold_);
    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned, initial_guess_);
    const double score = icp.getFitnessScore();

    if (icp.hasConverged() && std::isfinite(score) && score < fitness_threshold_)
    {
      initial_guess_ = icp.getFinalTransformation();
      ++converged_count_;
      RCLCPP_INFO(
          get_logger(), "ICP accepted: score %.4f (%d/%d)", score,
          converged_count_, required_convergences_);
      publishAlignedCloud(*filtered, initial_guess_);
      if (converged_count_ >= required_convergences_)
      {
        publishResult(initial_guess_, score);
        localization_complete_ = true;
        RCLCPP_INFO(get_logger(), "ICP relocalization complete");
      }
    }
    else
    {
      // A bad alignment must not walk the next initial guess away from the
      // operator-supplied pose.
      converged_count_ = 0;
      publishAlignedCloud(*filtered, initial_guess_);
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "ICP rejected: score %.4f", score);
    }
    publishMap();
  }

  void publishResult(const Eigen::Matrix4f &transform, double score)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.stamp = now();
    message.header.frame_id = map_frame_;
    message.pose.pose.position.x = transform(0, 3);
    message.pose.pose.position.y = transform(1, 3);
    message.pose.pose.position.z = transform(2, 3);
    const Eigen::Quaternionf rotation(transform.block<3, 3>(0, 0));
    message.pose.pose.orientation.x = rotation.x();
    message.pose.pose.orientation.y = rotation.y();
    message.pose.pose.orientation.z = rotation.z();
    message.pose.pose.orientation.w = rotation.w();
    message.pose.covariance[0] = score;
    message.pose.covariance[7] = score;
    message.pose.covariance[14] = score;
    result_pub_->publish(message);
  }

  void publishAlignedCloud(
      const pcl::PointCloud<pcl::PointXYZ> &source, const Eigen::Matrix4f &transform)
  {
    pcl::PointCloud<pcl::PointXYZ> aligned;
    pcl::transformPointCloud(source, aligned, transform);
    sensor_msgs::msg::PointCloud2 message;
    pcl::toROSMsg(aligned, message);
    message.header.stamp = now();
    message.header.frame_id = map_frame_;
    transformed_cloud_pub_->publish(message);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*message, *cloud);
    align(cloud);
  }

#ifdef USE_LIVOX
  void livoxCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr message)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->reserve(message->point_num);
    for (std::size_t i = 0; i < message->point_num; ++i)
    {
      const auto &point = message->points[i];
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
        cloud->push_back(pcl::PointXYZ(point.x, point.y, point.z));
    }
    align(cloud);
  }
#endif

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr result_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
#ifdef USE_LIVOX
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lvx_cloud_sub_;
#endif
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_{new pcl::PointCloud<pcl::PointXYZ>()};
  sensor_msgs::msg::PointCloud2 target_cloud_message_;
  Eigen::Matrix4f initial_guess_{Eigen::Matrix4f::Identity()};
  Eigen::Isometry3f initial_frame_to_sensor_{Eigen::Isometry3f::Identity()};
  std::string map_path_;
  std::string map_frame_;
  int solver_max_iter_{75};
  int required_convergences_{5};
  int min_source_points_{100};
  int converged_count_{0};
  double max_correspondence_distance_{1.0};
  double ransac_threshold_{0.5};
  double fitness_threshold_{0.30};
  double initialpose_ground_to_robot_z_{0.0};
  float map_voxel_size_{0.30F};
  float cloud_voxel_size_{0.20F};
  bool localization_complete_{false};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<ICPNode>());
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("icp_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
