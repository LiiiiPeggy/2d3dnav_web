#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace scan_planner
{
class GlobalFrameAdapter : public rclcpp::Node
{
public:
  GlobalFrameAdapter() : Node("global_frame_adapter"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    if (global_frame_.empty()) throw std::invalid_argument("global_frame must not be empty");

    const auto qos = rclcpp::SensorDataQoS();
    body_pub_ = create_publisher<nav_msgs::msg::Odometry>("body_pose_global", qos);
    sensor_pub_ = create_publisher<nav_msgs::msg::Odometry>("sensor_pose_global", qos);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_global", qos);
    body_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "body_pose_local", qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          publishOdometry(*message, body_pub_);
        });
    sensor_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "sensor_pose_local", qos,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
          publishOdometry(*message, sensor_pub_);
        });
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "cloud_local", qos,
        std::bind(&GlobalFrameAdapter::cloudCallback, this, std::placeholders::_1));
    RCLCPP_INFO(
        get_logger(), "Global frame adapter waiting for TF %s <- odometry frame",
        global_frame_.c_str());
  }

private:
  bool lookup(
      const std::string &source_frame, const builtin_interfaces::msg::Time &stamp,
      geometry_msgs::msg::TransformStamped &transform)
  {
    if (source_frame.empty())
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Input frame_id is empty");
      return false;
    }
    if (source_frame == global_frame_)
    {
      transform.header.frame_id = global_frame_;
      transform.child_frame_id = source_frame;
      transform.transform.rotation.w = 1.0;
      return true;
    }
    try
    {
      transform = tf_buffer_.lookupTransform(global_frame_, source_frame, rclcpp::Time(stamp));
      return true;
    }
    catch (const tf2::TransformException &)
    {
      try
      {
        // map->odom is static and may arrive after the first sensor message.
        transform = tf_buffer_.lookupTransform(global_frame_, source_frame, tf2::TimePointZero);
        return true;
      }
      catch (const tf2::TransformException &error)
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Waiting for TF %s <- %s: %s",
            global_frame_.c_str(), source_frame.c_str(), error.what());
        return false;
      }
    }
  }

  static tf2::Transform odometryTransform(const nav_msgs::msg::Odometry &message)
  {
    tf2::Transform transform;
    tf2::fromMsg(message.pose.pose, transform);
    return transform;
  }

  void publishOdometry(
      const nav_msgs::msg::Odometry &input,
      const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &publisher)
  {
    geometry_msgs::msg::TransformStamped global_from_local_message;
    if (!lookup(input.header.frame_id, input.header.stamp, global_from_local_message)) return;

    tf2::Transform global_from_local;
    tf2::fromMsg(global_from_local_message.transform, global_from_local);
    const tf2::Transform global_from_child = global_from_local * odometryTransform(input);

    nav_msgs::msg::Odometry output = input;
    output.header.frame_id = global_frame_;
    output.pose.pose.position.x = global_from_child.getOrigin().x();
    output.pose.pose.position.y = global_from_child.getOrigin().y();
    output.pose.pose.position.z = global_from_child.getOrigin().z();
    output.pose.pose.orientation = tf2::toMsg(global_from_child.getRotation());

    const auto &rotation = global_from_local.getBasis();
    const tf2::Vector3 linear_local(
        input.twist.twist.linear.x, input.twist.twist.linear.y,
        input.twist.twist.linear.z);
    const tf2::Vector3 angular_local(
        input.twist.twist.angular.x, input.twist.twist.angular.y,
        input.twist.twist.angular.z);
    const tf2::Vector3 linear_global = rotation * linear_local;
    const tf2::Vector3 angular_global = rotation * angular_local;
    output.twist.twist.linear.x = linear_global.x();
    output.twist.twist.linear.y = linear_global.y();
    output.twist.twist.linear.z = linear_global.z();
    output.twist.twist.angular.x = angular_global.x();
    output.twist.twist.angular.y = angular_global.y();
    output.twist.twist.angular.z = angular_global.z();
    publisher->publish(output);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input)
  {
    geometry_msgs::msg::TransformStamped transform_message;
    if (!lookup(input->header.frame_id, input->header.stamp, transform_message)) return;

    if (input->is_bigendian)
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "Big-endian PointCloud2 is not supported");
      return;
    }
    sensor_msgs::msg::PointCloud2 output = *input;
    output.header.frame_id = global_frame_;
    try
    {
      tf2::Transform transform;
      tf2::fromMsg(transform_message.transform, transform);
      sensor_msgs::PointCloud2Iterator<float> x(output, "x");
      sensor_msgs::PointCloud2Iterator<float> y(output, "y");
      sensor_msgs::PointCloud2Iterator<float> z(output, "z");
      for (; x != x.end(); ++x, ++y, ++z)
      {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
        const tf2::Vector3 point = transform * tf2::Vector3(*x, *y, *z);
        *x = static_cast<float>(point.x());
        *y = static_cast<float>(point.y());
        *z = static_cast<float>(point.z());
      }
    }
    catch (const std::runtime_error &error)
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000, "Cannot transform PointCloud2: %s",
          error.what());
      return;
    }
    cloud_pub_->publish(output);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr body_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr sensor_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr body_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sensor_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string global_frame_;
};
}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<scan_planner::GlobalFrameAdapter>());
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("global_frame_adapter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
