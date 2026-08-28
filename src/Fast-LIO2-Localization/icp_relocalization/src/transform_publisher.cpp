#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

class TransformPublisherNode : public rclcpp::Node
{
public:
  TransformPublisherNode() : Node("relocalization_transform_publisher")
  {
    odom_frame_ = declare_parameter<std::string>("odom_frame_id", "odom");
    map_frame_ = declare_parameter<std::string>("map_frame_id", "map");
    const auto translation = declare_parameter<std::vector<double>>(
        "sensor_translation_in_odom", {0.0, 0.0, 0.0});
    const auto rpy = declare_parameter<std::vector<double>>(
        "sensor_rpy_in_odom", {0.0, 0.0, 0.0});
    if (translation.size() != 3 || rpy.size() != 3)
      throw std::invalid_argument("sensor extrinsic parameters must contain three values");

    tf2::Quaternion sensor_rotation;
    sensor_rotation.setRPY(rpy[0], rpy[1], rpy[2]);
    odom_to_sensor_.setOrigin(
        tf2::Vector3(translation[0], translation[1], translation[2]));
    odom_to_sensor_.setRotation(sensor_rotation);

    corrected_pose_pub_ =
        create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "icp_result", rclcpp::QoS(1).reliable().transient_local());
    subscription_ =
        create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "icp_sensor_result", rclcpp::QoS(1).reliable(),
            std::bind(&TransformPublisherNode::callback, this, std::placeholders::_1));
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  }

private:
  void callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    tf2::Transform map_to_sensor;
    tf2::fromMsg(message->pose.pose, map_to_sensor);
    // ICP estimates T_map_sensor. FAST-LIO's odom origin is the initial IMU
    // frame, therefore T_map_odom = T_map_sensor * inverse(T_odom_sensor).
    const tf2::Transform map_to_odom = map_to_sensor * odom_to_sensor_.inverse();

    geometry_msgs::msg::PoseWithCovarianceStamped corrected = *message;
    corrected.header.frame_id = map_frame_;
    corrected.pose.pose.position.x = map_to_odom.getOrigin().x();
    corrected.pose.pose.position.y = map_to_odom.getOrigin().y();
    corrected.pose.pose.position.z = map_to_odom.getOrigin().z();
    corrected.pose.pose.orientation = tf2::toMsg(map_to_odom.getRotation());
    corrected_pose_pub_->publish(corrected);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = message->header.stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform = tf2::toMsg(map_to_odom);
    broadcaster_->sendTransform(transform);
    RCLCPP_INFO(
        get_logger(), "Relocalized: publishing fixed TF %s -> %s",
        map_frame_.c_str(), odom_frame_.c_str());
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr corrected_pose_pub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  tf2::Transform odom_to_sensor_;
  std::string odom_frame_;
  std::string map_frame_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<TransformPublisherNode>());
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("transform_publisher"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
