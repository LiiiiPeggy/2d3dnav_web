#!/usr/bin/env python3

"""Publish nav_msgs/Odometry from Cartographer's odom->base TF.

Cartographer remains the only publisher of localization TF.  This node only
creates the Odometry message that Nav2 controllers use to estimate the current
robot velocity.  When requested, the angular velocity is taken from the IMU
after rotating it into the robot base frame.
"""

import math
from typing import Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformException, TransformListener


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _rotate_vector(
    quaternion: Tuple[float, float, float, float],
    vector: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    """Rotate a vector using quaternion (x, y, z, w)."""
    qx, qy, qz, qw = quaternion
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return vector
    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm
    vx, vy, vz = vector

    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


class CartographerTfToOdom(Node):
    """Convert Cartographer local TF into an Odometry topic without TF output."""

    def __init__(self) -> None:
        super().__init__('cartographer_tf_to_odom')

        self.parent_frame = str(self.declare_parameter(
            'parent_frame', 'odom').value)
        self.child_frame = str(self.declare_parameter(
            'child_frame', 'base_footprint').value)
        output_topic = str(self.declare_parameter(
            'output_topic', '/odom').value)
        publish_rate = float(self.declare_parameter(
            'publish_rate', 50.0).value)
        self.use_imu = bool(self.declare_parameter(
            'use_imu_angular_velocity', False).value)
        imu_topic = str(self.declare_parameter(
            'imu_topic', '/Devices/Imu/Data').value)
        self.imu_timeout = float(self.declare_parameter(
            'imu_timeout', 0.15).value)
        self.velocity_filter_alpha = float(self.declare_parameter(
            'velocity_filter_alpha', 0.30).value)
        self.imu_filter_alpha = float(self.declare_parameter(
            'imu_filter_alpha', 0.25).value)
        self.linear_deadband = float(self.declare_parameter(
            'linear_deadband', 0.015).value)
        self.angular_deadband = float(self.declare_parameter(
            'angular_deadband', 0.015).value)
        self.max_linear_speed = float(self.declare_parameter(
            'max_linear_speed', 2.0).value)
        self.max_angular_speed = float(self.declare_parameter(
            'max_angular_speed', 3.0).value)
        self.max_sample_dt = float(self.declare_parameter(
            'max_sample_dt', 0.25).value)

        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')
        self.velocity_filter_alpha = max(
            0.0, min(1.0, self.velocity_filter_alpha))
        self.imu_filter_alpha = max(
            0.0, min(1.0, self.imu_filter_alpha))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.odom_publisher = self.create_publisher(
            Odometry, output_topic, 10)

        self.latest_imu: Optional[Imu] = None
        self.latest_imu_receipt_ns: Optional[int] = None
        if self.use_imu:
            self.create_subscription(
                Imu, imu_topic, self._imu_callback, qos_profile_sensor_data)

        self.previous_stamp_ns: Optional[int] = None
        self.previous_pose: Optional[Tuple[float, float, float]] = None
        self.last_new_sample_ns: Optional[int] = None
        self.filtered_vx = 0.0
        self.filtered_vy = 0.0
        self.filtered_wz = 0.0
        self.filtered_imu_wz = 0.0
        self.last_tf_warning_ns = 0
        self.last_imu_warning_ns = 0

        self.timer = self.create_timer(1.0 / publish_rate, self._publish)
        angular_source = 'IMU gyro (with TF fallback)' if self.use_imu \
            else 'Cartographer TF derivative'
        self.get_logger().info(
            f'Publishing {output_topic} at {publish_rate:.1f} Hz from '
            f'{self.parent_frame}->{self.child_frame}; angular source: '
            f'{angular_source}. This node does not publish TF.')

    def _imu_callback(self, message: Imu) -> None:
        self.latest_imu = message
        self.latest_imu_receipt_ns = self.get_clock().now().nanoseconds

    def _warn_throttled(self, kind: str, message: str) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        attribute = 'last_tf_warning_ns' if kind == 'tf' \
            else 'last_imu_warning_ns'
        if now_ns - getattr(self, attribute) >= 5_000_000_000:
            self.get_logger().warning(message)
            setattr(self, attribute, now_ns)

    def _imu_angular_z(self, now_ns: int) -> Optional[float]:
        message = self.latest_imu
        receipt_ns = self.latest_imu_receipt_ns
        if message is None or receipt_ns is None:
            return None
        if (now_ns - receipt_ns) * 1e-9 > self.imu_timeout:
            return None

        source_frame = message.header.frame_id
        vector = (
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        )
        if source_frame and source_frame != self.child_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.child_frame, source_frame, Time())
                rotation = transform.transform.rotation
                vector = _rotate_vector(
                    (rotation.x, rotation.y, rotation.z, rotation.w),
                    vector,
                )
            except TransformException as error:
                self._warn_throttled(
                    'imu',
                    f'Cannot transform IMU angular velocity from '
                    f'{source_frame} to {self.child_frame}; using TF '
                    f'derivative: {error}',
                )
                return None

        raw_wz = _clamp(vector[2], self.max_angular_speed)
        alpha = self.imu_filter_alpha
        self.filtered_imu_wz = (
            alpha * raw_wz + (1.0 - alpha) * self.filtered_imu_wz)
        return self.filtered_imu_wz

    def _publish(self) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        try:
            transform = self.tf_buffer.lookup_transform(
                self.parent_frame, self.child_frame, Time())
        except TransformException as error:
            self._warn_throttled(
                'tf',
                f'Waiting for {self.parent_frame}->{self.child_frame} TF: '
                f'{error}',
            )
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = _yaw_from_quaternion(
            rotation.x, rotation.y, rotation.z, rotation.w)
        stamp_ns = (
            int(transform.header.stamp.sec) * 1_000_000_000
            + int(transform.header.stamp.nanosec)
        )
        if stamp_ns == 0:
            stamp_ns = now_ns

        new_sample = (
            self.previous_stamp_ns is None
            or stamp_ns > self.previous_stamp_ns
        )
        if new_sample and self.previous_pose is not None:
            dt = (stamp_ns - self.previous_stamp_ns) * 1e-9
            if 1e-4 <= dt <= self.max_sample_dt:
                previous_x, previous_y, previous_yaw = self.previous_pose
                dx = translation.x - previous_x
                dy = translation.y - previous_y
                raw_vx = (
                    math.cos(yaw) * dx + math.sin(yaw) * dy) / dt
                raw_vy = (
                    -math.sin(yaw) * dx + math.cos(yaw) * dy) / dt
                raw_wz = _wrap_angle(yaw - previous_yaw) / dt
                raw_vx = _clamp(raw_vx, self.max_linear_speed)
                raw_vy = _clamp(raw_vy, self.max_linear_speed)
                raw_wz = _clamp(raw_wz, self.max_angular_speed)
                alpha = self.velocity_filter_alpha
                self.filtered_vx = (
                    alpha * raw_vx + (1.0 - alpha) * self.filtered_vx)
                self.filtered_vy = (
                    alpha * raw_vy + (1.0 - alpha) * self.filtered_vy)
                self.filtered_wz = (
                    alpha * raw_wz + (1.0 - alpha) * self.filtered_wz)
            else:
                self.filtered_vx = 0.0
                self.filtered_vy = 0.0
                self.filtered_wz = 0.0

        if new_sample:
            self.previous_stamp_ns = stamp_ns
            self.previous_pose = (translation.x, translation.y, yaw)
            self.last_new_sample_ns = now_ns
        elif (
            self.last_new_sample_ns is not None
            and (now_ns - self.last_new_sample_ns) * 1e-9
            > self.max_sample_dt
        ):
            self.filtered_vx = 0.0
            self.filtered_vy = 0.0
            self.filtered_wz = 0.0

        wz = self.filtered_wz
        if self.use_imu:
            imu_wz = self._imu_angular_z(now_ns)
            if imu_wz is not None:
                wz = imu_wz

        vx = 0.0 if abs(self.filtered_vx) < self.linear_deadband \
            else self.filtered_vx
        vy = 0.0 if abs(self.filtered_vy) < self.linear_deadband \
            else self.filtered_vy
        wz = 0.0 if abs(wz) < self.angular_deadband else wz

        message = Odometry()
        message.header = transform.header
        message.header.frame_id = self.parent_frame
        if (
            message.header.stamp.sec == 0
            and message.header.stamp.nanosec == 0
        ):
            message.header.stamp = now.to_msg()
        message.child_frame_id = self.child_frame
        message.pose.pose.position.x = translation.x
        message.pose.pose.position.y = translation.y
        message.pose.pose.position.z = translation.z
        message.pose.pose.orientation = rotation
        message.twist.twist.linear.x = vx
        message.twist.twist.linear.y = vy
        message.twist.twist.angular.z = wz
        message.pose.covariance = [
            0.02, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.02, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.04,
        ]
        message.twist.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.08, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06,
        ]
        self.odom_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CartographerTfToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
