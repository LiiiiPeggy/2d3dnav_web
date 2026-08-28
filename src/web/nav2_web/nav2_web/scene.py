"""Bandwidth-bounded ROS 2 scene relay for the mobile 3D viewer."""

from __future__ import annotations

from array import array
import base64
import math
import struct
import sys
import threading
import time

from nav_msgs.msg import Odometry, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker


_POINT_FIELD_FORMATS = {
    PointField.INT8: 'b',
    PointField.UINT8: 'B',
    PointField.INT16: 'h',
    PointField.UINT16: 'H',
    PointField.INT32: 'i',
    PointField.UINT32: 'I',
    PointField.FLOAT32: 'f',
    PointField.FLOAT64: 'd',
}


def _vector3(value) -> list[float]:
    return [float(value.x), float(value.y), float(value.z)]


def _quaternion(value) -> list[float]:
    return [float(value.x), float(value.y), float(value.z), float(value.w)]


def _pose(value) -> dict:
    return {
        'position': _vector3(value.position),
        'orientation': _quaternion(value.orientation),
    }


def _float32_base64(values: array) -> str:
    if sys.byteorder != 'little':
        values.byteswap()
    return base64.b64encode(values.tobytes()).decode('ascii')


def pointcloud_xyz(
    message: PointCloud2,
    point_limit: int,
) -> tuple[array, float, float]:
    """Extract a uniformly sampled xyz Float32 array without numpy."""
    fields = {field.name: field for field in message.fields}
    xyz_fields = [fields.get(axis) for axis in ('x', 'y', 'z')]
    if any(field is None for field in xyz_fields):
        return array('f'), 0.0, 0.0

    readers = []
    endian = '>' if message.is_bigendian else '<'
    for field in xyz_fields:
        format_code = _POINT_FIELD_FORMATS.get(field.datatype)
        if format_code is None:
            return array('f'), 0.0, 0.0
        readers.append(
            (struct.Struct(endian + format_code), int(field.offset)))

    width = int(message.width)
    height = max(1, int(message.height))
    total_points = width * height
    if width <= 0 or total_points <= 0 or int(message.point_step) <= 0:
        return array('f'), 0.0, 0.0
    sample_step = max(1, math.ceil(total_points / max(1, point_limit)))
    row_step = int(message.row_step) or width * int(message.point_step)
    point_step = int(message.point_step)
    raw = memoryview(message.data)
    xyz = array('f')
    min_z = math.inf
    max_z = -math.inf
    linear_index = 0

    for row in range(height):
        row_offset = row * row_step
        for column in range(width):
            if linear_index % sample_step:
                linear_index += 1
                continue
            offset = row_offset + column * point_step
            try:
                x = float(readers[0][0].unpack_from(
                    raw, offset + readers[0][1])[0])
                y = float(readers[1][0].unpack_from(
                    raw, offset + readers[1][1])[0])
                z = float(readers[2][0].unpack_from(
                    raw, offset + readers[2][1])[0])
            except (struct.error, ValueError):
                linear_index += 1
                continue
            linear_index += 1
            if not (
                math.isfinite(x)
                and math.isfinite(y)
                and math.isfinite(z)
            ):
                continue
            xyz.extend((x, y, z))
            min_z = min(min_z, z)
            max_z = max(max_z, z)

    if not xyz:
        return xyz, 0.0, 0.0
    return xyz, min_z, max_z


class SceneRelay:
    """Relay RViz-like scene primitives while keeping phone traffic bounded."""

    def __init__(self, node, websocket, fixed_frame: str):
        self._node = node
        self._websocket = websocket
        self.fixed_frame = fixed_frame
        self.point_limit = max(
            1000, int(node.get_parameter('scene_point_limit').value))
        self.cloud_rate = max(
            0.2, float(node.get_parameter('scene_cloud_rate').value))
        self._lock = threading.RLock()
        self._latest: dict[str, dict] = {}
        self._transforms: dict[str, dict] = {}
        self._last_cloud_time: dict[str, float] = {}
        self._last_pose_time: dict[str, float] = {}
        self._heartbeats: dict[str, float] = {}

        sensor_qos = QoSProfile(depth=1)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        reliable_qos = QoSProfile(depth=10)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        livox_imu_topic = node.get_parameter('scene_livox_imu_topic').value
        if livox_imu_topic:
            node.create_subscription(
                Imu,
                livox_imu_topic,
                lambda _message: self._heartbeat('lidar'),
                sensor_qos,
            )

        cloud_specs = (
            (
                'registered',
                node.get_parameter('scene_registered_cloud_topic').value,
                [0.38, 0.86, 1.0, 0.95],
                'height',
                2.2,
                sensor_qos,
            ),
            (
                'global_map',
                node.get_parameter('scene_global_cloud_topic').value,
                [0.72, 0.77, 0.82, 0.72],
                'height',
                1.4,
                transient_qos,
            ),
            (
                'traversable',
                node.get_parameter('scene_traversable_topic').value,
                [0.18, 1.0, 0.47, 0.94],
                'flat',
                4.6,
                transient_qos,
            ),
            (
                'occupancy',
                node.get_parameter('scene_occupancy_topic').value,
                [0.99, 0.47, 0.29, 0.92],
                'height',
                3.0,
                sensor_qos,
            ),
            (
                'inflated',
                node.get_parameter('scene_inflated_topic').value,
                [0.70, 0.34, 1.0, 0.45],
                'flat',
                3.0,
                sensor_qos,
            ),
        )
        for layer, topic, color, color_mode, point_size, qos in cloud_specs:
            if topic:
                node.create_subscription(
                    PointCloud2,
                    topic,
                    lambda message, selected_layer=layer, selected_topic=topic,
                    selected_color=color, selected_mode=color_mode,
                    selected_size=point_size: self._pointcloud_callback(
                        selected_layer,
                        selected_topic,
                        selected_color,
                        selected_mode,
                        selected_size,
                        message,
                    ),
                    qos,
                )

        pose_specs = (
            (
                'body_pose',
                node.get_parameter('scene_body_pose_topic').value,
                [0.15, 0.92, 0.72, 1.0],
            ),
            (
                'lidar_pose',
                node.get_parameter('scene_lidar_pose_topic').value,
                [1.0, 0.72, 0.20, 1.0],
            ),
            (
                'fastlio_odom',
                node.get_parameter('scene_fastlio_odom_topic').value,
                [0.28, 0.70, 1.0, 0.82],
            ),
        )
        for layer, topic, color in pose_specs:
            if topic:
                node.create_subscription(
                    Odometry,
                    topic,
                    lambda message, selected_layer=layer, selected_topic=topic,
                    selected_color=color: self._pose_callback(
                        selected_layer,
                        selected_topic,
                        selected_color,
                        message,
                    ),
                    sensor_qos,
                )

        scene_path_topic = node.get_parameter('scene_path_topic').value
        if scene_path_topic:
            node.create_subscription(
                Path,
                scene_path_topic,
                lambda message: self._path_callback(scene_path_topic, message),
                reliable_qos,
            )

        for topic in node.get_parameter('scene_marker_topics').value:
            if topic:
                node.create_subscription(
                    Marker,
                    topic,
                    lambda message, selected_topic=topic:
                    self._marker_callback(selected_topic, message),
                    reliable_qos,
                )

        node.create_subscription(
            TFMessage, '/tf',
            lambda message: self._tf_callback(False, message), reliable_qos)
        node.create_subscription(
            TFMessage, '/tf_static',
            lambda message: self._tf_callback(True, message), transient_qos)
        node.create_timer(1.0, self._broadcast_status)

    def config_payload(self) -> dict:
        return {
            'type': 'scene_config',
            'fixed_frame': self.fixed_frame,
            'point_limit': self.point_limit,
            'cloud_rate': self.cloud_rate,
            'layers': [
                {'id': 'registered', 'label': 'FAST-LIO 注册点云'},
                {'id': 'global_map', 'label': '全局点云'},
                {'id': 'traversable', 'label': 'PCT 可通行区域'},
                {'id': 'occupancy', 'label': '占据栅格'},
                {'id': 'inflated', 'label': '膨胀占据'},
                {'id': 'planning', 'label': '规划轨迹/目标'},
                {'id': 'robot', 'label': '机器狗与位姿'},
                {'id': 'tf', 'label': 'TF 坐标系'},
            ],
        }

    def snapshot(self, client=None):
        self._websocket.send_json(self.config_payload(), client)
        with self._lock:
            latest = list(self._latest.values())
            transforms = list(self._transforms.values())
        if transforms:
            self._websocket.send_json({
                'type': 'scene_tf',
                'static': False,
                'transforms': transforms,
            }, client)
        for payload in latest:
            self._websocket.send_json(payload, client)
        self._send_status(client)

    def _pointcloud_callback(
        self,
        layer: str,
        topic: str,
        color: list[float],
        color_mode: str,
        point_size: float,
        message: PointCloud2,
    ):
        now = time.monotonic()
        elapsed = now - self._last_cloud_time.get(layer, 0.0)
        if elapsed < 1.0 / self.cloud_rate:
            return
        # Do not decode high-rate registered clouds until a UI connects.
        if layer == 'registered' and not self._websocket.clients():
            return
        self._last_cloud_time[layer] = now
        xyz, min_z, max_z = pointcloud_xyz(message, self.point_limit)
        payload = {
            'type': 'scene_pointcloud',
            'layer': layer,
            'topic': topic,
            'frame_id': message.header.frame_id or self.fixed_frame,
            'stamp': {
                'sec': int(message.header.stamp.sec),
                'nanosec': int(message.header.stamp.nanosec),
            },
            'count': len(xyz) // 3,
            'encoding': 'float32-le-base64',
            'xyz': _float32_base64(xyz),
            'min_z': min_z,
            'max_z': max_z,
            'color': color,
            'color_mode': color_mode,
            'point_size': point_size,
            'received_at': now,
        }
        with self._lock:
            self._latest[f'cloud:{layer}'] = payload
        self._websocket.send_json(payload)

    def _heartbeat(self, layer: str):
        with self._lock:
            self._heartbeats[layer] = time.monotonic()

    def _pose_callback(
        self,
        layer: str,
        topic: str,
        color: list[float],
        message: Odometry,
    ):
        now = time.monotonic()
        if now - self._last_pose_time.get(layer, 0.0) < 0.05:
            return
        self._last_pose_time[layer] = now
        payload = {
            'type': 'scene_pose',
            'layer': layer,
            'topic': topic,
            'frame_id': message.header.frame_id or self.fixed_frame,
            'child_frame_id': message.child_frame_id,
            'pose': _pose(message.pose.pose),
            'linear_velocity': _vector3(message.twist.twist.linear),
            'angular_velocity': _vector3(message.twist.twist.angular),
            'color': color,
            'received_at': now,
        }
        with self._lock:
            self._latest[f'pose:{layer}'] = payload
            if message.child_frame_id:
                self._transforms[message.child_frame_id] = {
                    'parent': payload['frame_id'],
                    'child': message.child_frame_id,
                    'translation': payload['pose']['position'],
                    'rotation': payload['pose']['orientation'],
                }
        self._websocket.send_json(payload)

    def _path_callback(self, topic: str, message: Path):
        points = [
            _vector3(pose.pose.position)
            for pose in message.poses[:12000]
        ]
        payload = {
            'type': 'scene_path',
            'layer': 'planning',
            'topic': topic,
            'frame_id': message.header.frame_id or self.fixed_frame,
            'points': points,
            'color': [0.18, 1.0, 0.47, 1.0],
            'line_width': 3.0,
            'received_at': time.monotonic(),
        }
        with self._lock:
            self._latest[f'path:{topic}'] = payload
        self._websocket.send_json(payload)

    def _marker_callback(self, topic: str, message: Marker):
        key = f'{topic}:{message.ns}:{message.id}'
        payload = {
            'type': 'scene_marker',
            'layer': 'planning',
            'topic': topic,
            'key': key,
            'frame_id': message.header.frame_id or self.fixed_frame,
            'action': int(message.action),
            'marker_type': int(message.type),
            'pose': _pose(message.pose),
            'scale': _vector3(message.scale),
            'color': [
                float(message.color.r),
                float(message.color.g),
                float(message.color.b),
                float(message.color.a),
            ],
            'points': [_vector3(point) for point in message.points[:20000]],
            'received_at': time.monotonic(),
        }
        with self._lock:
            if message.action == Marker.DELETEALL:
                for cache_key in list(self._latest):
                    if cache_key.startswith(f'marker:{topic}:'):
                        del self._latest[cache_key]
            elif message.action == Marker.DELETE:
                self._latest.pop(f'marker:{key}', None)
            else:
                self._latest[f'marker:{key}'] = payload
        self._websocket.send_json(payload)

    def _tf_callback(self, static: bool, message: TFMessage):
        transforms = []
        for transform in message.transforms:
            parent = transform.header.frame_id.lstrip('/')
            child = transform.child_frame_id.lstrip('/')
            if not parent or not child or parent == child:
                continue
            item = {
                'parent': parent,
                'child': child,
                'translation': _vector3(transform.transform.translation),
                'rotation': _quaternion(transform.transform.rotation),
            }
            transforms.append(item)
            with self._lock:
                self._transforms[child] = item
        if transforms and self._websocket.clients():
            self._websocket.send_json({
                'type': 'scene_tf',
                'static': static,
                'transforms': transforms,
            })

    def _status_payload(self) -> dict:
        now = time.monotonic()
        with self._lock:
            entries = list(self._latest.values())
            transform_count = len(self._transforms)
            heartbeats = dict(self._heartbeats)
        layers = {}
        for payload in entries:
            layer = payload.get('layer')
            if not layer:
                continue
            age = max(0.0, now - float(payload.get('received_at', now)))
            current = layers.setdefault(layer, {'age': age, 'count': 0})
            current['age'] = min(current['age'], age)
            current['count'] += int(payload.get('count', 1))
        for layer, received_at in heartbeats.items():
            age = max(0.0, now - received_at)
            current = layers.setdefault(layer, {'age': age, 'count': 1})
            current['age'] = min(current['age'], age)
        return {
            'type': 'scene_status',
            'fixed_frame': self.fixed_frame,
            'clients': len(self._websocket.clients()),
            'transform_count': transform_count,
            'layers': layers,
        }

    def _send_status(self, client=None):
        self._websocket.send_json(self._status_payload(), client)

    def _broadcast_status(self):
        if self._websocket.clients():
            self._send_status()
