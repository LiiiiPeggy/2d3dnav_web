#!/usr/bin/env python3
"""Publish a fixed body pose for the PCT/App offline map demonstration."""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped


class PctDemoPosePublisher(Node):
    def __init__(self):
        super().__init__('pct_demo_pose_publisher')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.body_frame = self.declare_parameter('body_frame', 'trunk').value
        self.x = float(self.declare_parameter('x', 5.0).value)
        self.y = float(self.declare_parameter('y', 5.0).value)
        self.ground_z = float(self.declare_parameter('ground_z', 0.0).value)
        self.body_height = float(
            self.declare_parameter('body_height', 0.4).value)
        self.yaw = float(self.declare_parameter('yaw', 0.0).value)
        values = (
            self.x, self.y, self.ground_z, self.body_height, self.yaw)
        if not all(math.isfinite(value) for value in values):
            raise ValueError('demo pose parameters must be finite')
        if self.body_height <= 0.0:
            raise ValueError('body_height must be positive')

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.publisher = self.create_publisher(Odometry, 'body_pose', qos)
        self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self.initial_pose_callback,
            qos,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.2, self.publish_pose)
        self.get_logger().info(
            f'Offline PCT start pose: [{self.x:.2f}, {self.y:.2f}, '
            f'{self.ground_z:.2f}] in {self.map_frame}; no control output')

    def initial_pose_callback(self, message):
        if message.header.frame_id and message.header.frame_id != self.map_frame:
            self.get_logger().warning(
                f'Ignoring demo initial pose in {message.header.frame_id!r}; '
                f'expected {self.map_frame!r}')
            return
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        values = (position.x, position.y, position.z)
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('Ignoring non-finite demo initial pose')
            return
        self.x = float(position.x)
        self.y = float(position.y)
        self.ground_z = float(position.z)
        self.yaw = math.atan2(
            2.0 * (orientation.w * orientation.z
                   + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y
                         + orientation.z * orientation.z),
        )
        self.get_logger().info(
            f'Offline PCT start moved from App: [{self.x:.2f}, '
            f'{self.y:.2f}, {self.ground_z:.2f}], yaw={self.yaw:.2f}')

    def publish_pose(self):
        stamp = self.get_clock().now().to_msg()
        body_z = self.ground_z + self.body_height
        half_yaw = 0.5 * self.yaw

        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.map_frame
        message.child_frame_id = self.body_frame
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.position.z = body_z
        message.pose.pose.orientation.z = math.sin(half_yaw)
        message.pose.pose.orientation.w = math.cos(half_yaw)
        self.publisher.publish(message)

        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = self.body_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = body_z
        transform.transform.rotation = message.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PctDemoPosePublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError) as error:
        if not isinstance(error, KeyboardInterrupt):
            rclpy.logging.get_logger('pct_demo_pose_publisher').fatal(str(error))
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
