"""ROS 2 to Web bridge for headless Nav2 operation."""

from __future__ import annotations

import base64
import json
import math
import os
import queue
import re
import threading
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import ParticleCloud
from nav2_msgs.srv import SaveMap, SetInitialPose
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from nav2_web.launch_manager import LaunchManager, LaunchManagerError
from nav2_web.launch_manager import launch_profiles_for
from nav2_web.scene import SceneRelay
from nav2_web.server import WebServers


_REMOTE_LAUNCH_NODE_NAMES = {
    'amcl',
    'behavior_server',
    'bt_navigator',
    'cartographer_initial_pose_bridge',
    'cartographer_node',
    'cartographer_occupancy_grid_node',
    'controller_server',
    'lifecycle_manager_localization',
    'lifecycle_manager_map_saver',
    'lifecycle_manager_map_server',
    'lifecycle_manager_navigation',
    'map_saver',
    'map_server',
    'planner_server',
    'rf2o_laser_odometry',
    'rf2o_laser_odometry_prior',
    'smoother_server',
    'velocity_smoother',
    'waypoint_follower',
    'ydlidar_ros2_driver_node',
    'fastlio_mapping',
    'laser_mapping',
    'fastlio_pose_adapter',
    'fastlio_input_monitor',
    'livox_lidar_publisher',
    'scan_planner_node',
}


def _yaw_from_quaternion(quaternion) -> float:
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def _set_yaw(quaternion, yaw: float):
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = math.sin(yaw * 0.5)
    quaternion.w = math.cos(yaw * 0.5)


class Nav2WebBridge(Node):
    """Expose map, pose, velocity, path, and Nav2 actions to a mobile UI."""

    def __init__(self):
        super().__init__('nav2_web_bridge')

        self.declare_parameter('http_port', 8081)
        self.declare_parameter('ws_port', 8891)
        self.declare_parameter('web_root', '')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('mppi_trajectories_topic', '/trajectories')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('particle_topic', '/particle_cloud')
        self.declare_parameter('local_costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('global_costmap_topic', '/global_costmap/costmap')
        self.declare_parameter(
            'local_costmap_node', '/local_costmap/local_costmap')
        self.declare_parameter(
            'global_costmap_node', '/global_costmap/global_costmap')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('set_initial_pose_service', '/set_initial_pose')
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter(
            'scanplanner_goal_topic', '/move_base_simple/goal')
        self.declare_parameter('pct_waypoints_topic', '/pct_waypoints')
        self.declare_parameter('save_map_service', '/map_saver/save_map')
        self.declare_parameter('fastlio_map_save_service', '/map_save')
        self.declare_parameter(
            'reset_localization_service', '/reinitialize_global_localization')
        self.declare_parameter(
            'nomotion_update_service', '/request_nomotion_update')
        self.declare_parameter('map_save_directory', '/home/u/dog_ws/maps')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('localization_backend', 'auto')
        self.declare_parameter('telemetry_rate', 10.0)
        self.declare_parameter('launch_control_enabled', False)
        self.declare_parameter('launch_profile_set', 'nav2')
        self.declare_parameter('scanplanner_keypoints_file', '')
        self.declare_parameter('scanplanner_reference_path_file', '')
        self.declare_parameter('scanplanner_map_file', '')
        self.declare_parameter('scanplanner_tomogram_file', '')
        self.declare_parameter('scene_enabled', True)
        self.declare_parameter('scene_point_limit', 40000)
        self.declare_parameter('scene_cloud_rate', 5.0)
        self.declare_parameter(
            'scene_registered_cloud_topic', '/cloud_registered')
        self.declare_parameter('scene_livox_imu_topic', '/livox/imu')
        self.declare_parameter(
            'scene_global_cloud_topic', '/map_generator/global_cloud')
        self.declare_parameter(
            'scene_traversable_topic', '/pct/traversable')
        self.declare_parameter(
            'scene_occupancy_topic', '/grid_map/occupancy')
        self.declare_parameter(
            'scene_inflated_topic', '/grid_map/occupancy_inflate')
        self.declare_parameter(
            'scene_body_pose_topic', '/scan_planner/body_pose')
        self.declare_parameter(
            'scene_lidar_pose_topic', '/scan_planner/lidar_pose')
        self.declare_parameter('scene_fastlio_odom_topic', '/Odometry')
        self.declare_parameter('scene_path_topic', '/quad_0/path')
        self.declare_parameter('scene_marker_topics', [
            '/grid_map/sliding_map_bbox',
            '/goal_point',
            '/global_list',
            '/init_list',
            '/optimal_list',
            '/a_star_list',
            '/planning/self_inflation',
        ])

        self.http_port = self.get_parameter('http_port').value
        self.ws_port = self.get_parameter('ws_port').value
        self.map_topic = self.get_parameter('map_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.mppi_trajectories_topic = self.get_parameter(
            'mppi_trajectories_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.particle_topic = self.get_parameter('particle_topic').value
        self.local_costmap_topic = self.get_parameter('local_costmap_topic').value
        self.global_costmap_topic = self.get_parameter('global_costmap_topic').value
        self.local_costmap_node = self.get_parameter('local_costmap_node').value
        self.global_costmap_node = self.get_parameter('global_costmap_node').value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').value
        self.set_initial_pose_service = self.get_parameter(
            'set_initial_pose_service').value
        self.navigate_action_name = self.get_parameter('navigate_action').value
        self.scanplanner_goal_topic = self.get_parameter(
            'scanplanner_goal_topic').value
        self.pct_waypoints_topic = self.get_parameter(
            'pct_waypoints_topic').value
        self.save_map_service = self.get_parameter('save_map_service').value
        self.fastlio_map_save_service = self.get_parameter(
            'fastlio_map_save_service').value
        self.reset_localization_service = self.get_parameter(
            'reset_localization_service').value
        self.nomotion_update_service = self.get_parameter(
            'nomotion_update_service').value
        self.map_save_directory = os.path.abspath(os.path.expanduser(
            self.get_parameter('map_save_directory').value
        ))
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.localization_backend = str(
            self.get_parameter('localization_backend').value).lower()
        launch_control_enabled = bool(
            self.get_parameter('launch_control_enabled').value)
        launch_profile_set = str(
            self.get_parameter('launch_profile_set').value)
        scanplanner_keypoints_file = str(
            self.get_parameter('scanplanner_keypoints_file').value)
        scanplanner_reference_path_file = str(
            self.get_parameter('scanplanner_reference_path_file').value)
        scanplanner_map_file = str(
            self.get_parameter('scanplanner_map_file').value)
        scanplanner_tomogram_file = str(
            self.get_parameter('scanplanner_tomogram_file').value)
        telemetry_rate = max(
            1.0,
            float(self.get_parameter('telemetry_rate').value),
        )

        web_root = self.get_parameter('web_root').value
        if not web_root:
            web_root = os.path.join(
                get_package_share_directory('nav2_web'),
                'web',
            )

        self._state_lock = threading.RLock()
        self._command_queue: queue.Queue[dict] = queue.Queue(maxsize=64)
        self._latest_map = None
        self._latest_map_time = 0.0
        self._latest_map_metrics = None
        self._previous_map_grid = None
        self._map_update_count = 0
        self._latest_path = None
        self._latest_mppi_trajectories = None
        self._last_mppi_trajectory_web_time = 0.0
        self._latest_cmd = {'vx': 0.0, 'vy': 0.0, 'wz': 0.0, 'age': None}
        self._latest_cmd_time = 0.0
        self._latest_scan = {
            'age': None,
            'valid_points': 0,
            'total_points': 0,
            'valid_ratio': None,
            'range_max': None,
        }
        self._latest_scan_time = 0.0
        self._latest_scan_points = None
        self._last_scan_web_time = 0.0
        self._latest_particles = None
        self._nomotion_update_pending = False
        self._last_nomotion_update_time = 0.0
        self._localization_reset_pending = False
        self._initial_pose_sent_time = 0.0
        self._localization_reset_state = {
            'state': 'idle',
            'message': '定位器未重置',
        }
        self._local_costmap_info = {
            'frame_id': self.odom_frame,
            'width': None,
            'height': None,
            'resolution': None,
            'age': None,
        }
        self._local_costmap_time = 0.0
        self._latest_costmaps = {'global': None, 'local': None}
        self._inflation_state = {
            scope: {
                'ready': False,
                'state': 'waiting',
                'message': '等待 Costmap 参数服务',
                'enabled': None,
                'inflation_radius': None,
                'cost_scaling_factor': None,
                'inflate_unknown': None,
                'inflate_around_unknown': None,
            }
            for scope in ('global', 'local')
        }
        self._inflation_query_pending = {'global': False, 'local': False}
        self._latest_pose = None
        self._latest_pct_waypoints = []
        self._goal = None
        self._goal_handle = None
        self._node_presence = {
            'slam': False,
            'amcl': False,
            'cartographer_localization': False,
        }
        self._node_presence_time = 0.0
        self._save_map_state = {
            'state': 'idle',
            'message': '尚未保存地图',
            'path': None,
        }
        self._nav_state = {
            'state': 'idle',
            'message': '等待导航目标',
            'distance_remaining': None,
            'navigation_time': None,
        }

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        cmd_qos = QoSProfile(depth=10)
        cmd_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        scan_qos = QoSProfile(depth=5)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        path_qos = QoSProfile(depth=1)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        waypoint_qos = QoSProfile(depth=1)
        waypoint_qos.reliability = ReliabilityPolicy.RELIABLE
        waypoint_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE

        self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._map_callback,
            map_qos,
        )
        self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self._cmd_vel_callback,
            cmd_qos,
        )
        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_callback,
            scan_qos,
        )
        self.create_subscription(
            ParticleCloud,
            self.particle_topic,
            self._particle_callback,
            scan_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            self.local_costmap_topic,
            self._local_costmap_callback,
            map_qos,
        )
        self.create_subscription(
            OccupancyGrid,
            self.global_costmap_topic,
            self._global_costmap_callback,
            map_qos,
        )
        self.create_subscription(
            Path,
            self.path_topic,
            self._path_callback,
            path_qos,
        )
        self.create_subscription(
            MarkerArray,
            self.mppi_trajectories_topic,
            self._mppi_trajectories_callback,
            marker_qos,
        )

        self._initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            10,
        )
        self._scanplanner_goal_publisher = self.create_publisher(
            PoseStamped,
            self.scanplanner_goal_topic,
            10,
        )
        self._pct_waypoints_publisher = self.create_publisher(
            Path,
            self.pct_waypoints_topic,
            waypoint_qos,
        )
        self._set_initial_pose_client = self.create_client(
            SetInitialPose,
            self.set_initial_pose_service,
        )
        self._navigate_client = ActionClient(
            self,
            NavigateToPose,
            self.navigate_action_name,
        )
        self._save_map_client = self.create_client(
            SaveMap,
            self.save_map_service,
        )
        self._fastlio_map_save_client = self.create_client(
            Trigger,
            self.fastlio_map_save_service,
        )
        self._reset_localization_client = self.create_client(
            Empty,
            self.reset_localization_service,
        )
        self._nomotion_update_client = self.create_client(
            Empty,
            self.nomotion_update_service,
        )
        self._inflation_clients = {}
        for scope, node_name in (
            ('global', self.global_costmap_node),
            ('local', self.local_costmap_node),
        ):
            service_root = node_name.rstrip('/')
            self._inflation_clients[scope] = {
                'get': self.create_client(
                    GetParameters, f'{service_root}/get_parameters'),
                'set': self.create_client(
                    SetParameters, f'{service_root}/set_parameters'),
            }

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._launch_manager = None
        self._servers = WebServers(
            web_root,
            int(self.http_port),
            int(self.ws_port),
            self._on_web_message,
            self._on_web_connect,
        )
        self._scene_relay = None
        if bool(self.get_parameter('scene_enabled').value):
            self._scene_relay = SceneRelay(
                self, self._servers.websocket, self.map_frame)
        self._servers.start()
        self._launch_manager = LaunchManager(
            enabled=launch_control_enabled,
            map_directory=self.map_save_directory,
            event_callback=self._on_launch_event,
            profiles=launch_profiles_for(
                launch_profile_set,
                keypoints_file=scanplanner_keypoints_file,
                reference_path_file=scanplanner_reference_path_file,
                map_file=scanplanner_map_file,
                tomogram_file=scanplanner_tomogram_file,
            ),
        )

        self.create_timer(0.05, self._process_web_commands)
        self.create_timer(1.0 / telemetry_rate, self._broadcast_telemetry)
        self.create_timer(2.0, self._refresh_inflation_parameters)

        self.get_logger().info(
            f'Nav2 Web 已启动: http://0.0.0.0:{self.http_port} '
            f'(WebSocket {self.ws_port})'
        )
        self.get_logger().info(
            'Web Launch 控制: '
            f'{"已启用" if launch_control_enabled else "已关闭"}; '
            f'模式={launch_profile_set}; 仅允许预设白名单入口'
        )
        frame_chain = (
            f'{self.map_frame} -> {self.base_frame}'
            if self.map_frame == self.odom_frame
            else f'{self.map_frame} -> {self.odom_frame} -> {self.base_frame}'
        )
        self.get_logger().info(
            f'订阅 map={self.map_topic}, global_path={self.path_topic}, '
            f'mppi={self.mppi_trajectories_topic}, '
            f'cmd_vel={self.cmd_vel_topic}, scan={self.scan_topic}; '
            f'particles={self.particle_topic}; '
            f'TF {frame_chain}'
        )
        if self._scene_relay is not None:
            self.get_logger().info(
                f'手机 3D 场景已启用: fixed_frame={self.map_frame}, '
                f'点云上限={self._scene_relay.point_limit}, '
                f'最高 {self._scene_relay.cloud_rate:.1f} Hz'
            )

    def _map_callback(self, message: OccupancyGrid):
        cells = [int(value) for value in message.data]
        encoded_cells = bytes((value + 1) & 0xFF for value in cells)
        origin = message.info.origin
        now = time.monotonic()
        self._map_update_count += 1
        map_metrics, map_grid = self._measure_map(message, cells, now)
        payload = {
            'type': 'map',
            'frame_id': message.header.frame_id or self.map_frame,
            'width': int(message.info.width),
            'height': int(message.info.height),
            'resolution': float(message.info.resolution),
            'origin': {
                'x': float(origin.position.x),
                'y': float(origin.position.y),
                'yaw': _yaw_from_quaternion(origin.orientation),
            },
            'encoding': 'base64-offset-1',
            'data': base64.b64encode(encoded_cells).decode('ascii'),
        }
        with self._state_lock:
            self._latest_map = payload
            self._latest_map_time = now
            self._latest_map_metrics = map_metrics
            self._previous_map_grid = map_grid
        self._servers.websocket.send_json(payload)
        if self._map_update_count == 1 or self._map_update_count % 20 == 0:
            self.get_logger().info(
                f'地图更新 #{self._map_update_count}: '
                f'{message.info.width}x{message.info.height}, '
                f'已知面积 {map_metrics["known_area"]:.1f} m²'
            )

    def _measure_map(self, message, cells: list[int], now: float):
        """Calculate conservative, explainable mapping-health indicators."""
        resolution = float(message.info.resolution)
        known_cells = sum(value >= 0 for value in cells)
        free_cells = sum(0 <= value <= 25 for value in cells)
        occupied_cells = sum(value >= 65 for value in cells)
        total_cells = len(cells)
        cell_area = resolution * resolution

        # Class changes are more useful than raw probability changes.  Compare the
        # world-aligned overlap so a growing SLAM grid does not look unstable.
        classes = bytes(
            0 if value < 0 else 1 if value <= 25 else 2 if value >= 65 else 3
            for value in cells
        )
        current_grid = {
            'width': int(message.info.width),
            'height': int(message.info.height),
            'resolution': resolution,
            'origin_x': float(message.info.origin.position.x),
            'origin_y': float(message.info.origin.position.y),
            'origin_yaw': _yaw_from_quaternion(message.info.origin.orientation),
            'classes': classes,
            'known_cells': known_cells,
        }
        stability = None
        changed_cells = None
        comparable_cells = 0
        previous = self._previous_map_grid
        if previous is not None:
            same_resolution = math.isclose(
                previous['resolution'], resolution, rel_tol=0.0, abs_tol=1e-6
            )
            aligned_yaw = math.isclose(
                previous['origin_yaw'], current_grid['origin_yaw'],
                rel_tol=0.0, abs_tol=1e-4,
            )
            if same_resolution and aligned_yaw and resolution > 0.0:
                x_offset = round(
                    (current_grid['origin_x'] - previous['origin_x']) / resolution
                )
                y_offset = round(
                    (current_grid['origin_y'] - previous['origin_y']) / resolution
                )
                changed_cells = 0
                previous_classes = previous['classes']
                # Bound the stability calculation on very large maps.  The
                # resulting percentage is a uniform grid sample, not a claim of
                # geometric ground-truth accuracy.
                sample_stride = max(
                    1,
                    math.ceil(math.sqrt(len(classes) / 100000.0)),
                )
                for current_y in range(0, current_grid['height'], sample_stride):
                    previous_y = current_y + y_offset
                    if not 0 <= previous_y < previous['height']:
                        continue
                    current_row = current_y * current_grid['width']
                    previous_row = previous_y * previous['width']
                    for current_x in range(
                        0, current_grid['width'], sample_stride
                    ):
                        previous_x = current_x + x_offset
                        if not 0 <= previous_x < previous['width']:
                            continue
                        current_class = classes[current_row + current_x]
                        previous_class = previous_classes[previous_row + previous_x]
                        if current_class == 0 or previous_class == 0:
                            continue
                        comparable_cells += 1
                        if current_class != previous_class:
                            changed_cells += 1
                if comparable_cells:
                    stability = 100.0 * (
                        1.0 - changed_cells / comparable_cells
                    )

        previous_known = previous['known_cells'] if previous else 0
        metrics = {
            'known_cells': known_cells,
            'free_cells': free_cells,
            'occupied_cells': occupied_cells,
            'unknown_cells': max(0, total_cells - known_cells),
            'known_ratio': 100.0 * known_cells / total_cells if total_cells else 0.0,
            'known_area': known_cells * cell_area,
            'free_area': free_cells * cell_area,
            'occupied_area': occupied_cells * cell_area,
            'known_delta_area': (known_cells - previous_known) * cell_area,
            'stability': stability,
            'changed_cells': changed_cells,
            'comparable_cells': comparable_cells,
            'update_count': self._map_update_count,
            'measured_at': now,
        }
        return metrics, current_grid

    def _cmd_vel_callback(self, message: Twist):
        with self._state_lock:
            self._latest_cmd = {
                'vx': float(message.linear.x),
                'vy': float(message.linear.y),
                'wz': float(message.angular.z),
                'age': 0.0,
            }
            self._latest_cmd_time = time.monotonic()

    def _scan_callback(self, message: LaserScan):
        valid_points = sum(
            math.isfinite(value) and message.range_min <= value <= message.range_max
            for value in message.ranges
        )
        total_points = len(message.ranges)
        with self._state_lock:
            self._latest_scan = {
                'age': 0.0,
                'valid_points': valid_points,
                'total_points': total_points,
                'valid_ratio': (
                    100.0 * valid_points / total_points if total_points else 0.0
                ),
                'range_max': float(message.range_max),
            }
            self._latest_scan_time = time.monotonic()
        self._publish_scan_points(message)

    def _publish_scan_points(self, message: LaserScan):
        now = time.monotonic()
        if now - self._last_scan_web_time < 0.2:
            return
        self._last_scan_web_time = now

        total_points = len(message.ranges)
        # Keep enough points to match RViz clearly while bounding browser work
        # for high-resolution real lidars.
        step = max(1, math.ceil(total_points / 900))
        local_points = []
        for index in range(0, total_points, step):
            distance = float(message.ranges[index])
            if not (
                math.isfinite(distance) and
                message.range_min <= distance <= message.range_max
            ):
                continue
            angle = float(message.angle_min) + index * float(message.angle_increment)
            local_points.append([
                distance * math.cos(angle),
                distance * math.sin(angle),
            ])

        source_frame = message.header.frame_id or 'base_scan'
        map_points = []
        transform_ready = False
        reset_pending = self._localization_reset_pending
        if (
            reset_pending and self._initial_pose_sent_time > 0.0 and
            now - self._initial_pose_sent_time > 1.0
        ):
            self._localization_reset_pending = False
            reset_pending = False
            with self._state_lock:
                self._localization_reset_state = {
                    'state': 'completed',
                    'message': '已收到初始位置，定位器正在重新匹配',
                }
        if not reset_pending:
            try:
                stamp = message.header.stamp
                transform_time = (
                    Time.from_msg(stamp)
                    if stamp.sec != 0 or stamp.nanosec != 0 else Time()
                )
                transform = self._tf_buffer.lookup_transform(
                    self.map_frame,
                    source_frame,
                    transform_time,
                    timeout=Duration(seconds=0.05),
                )
                translation = transform.transform.translation
                yaw = _yaw_from_quaternion(transform.transform.rotation)
                cosine = math.cos(yaw)
                sine = math.sin(yaw)
                map_points = [
                    [
                        float(translation.x) + cosine * x - sine * y,
                        float(translation.y) + sine * x + cosine * y,
                    ]
                    for x, y in local_points
                ]
                transform_ready = True
            except TransformException:
                pass

        payload = {
            'type': 'scan_points',
            'source_frame': source_frame,
            'target_frame': self.map_frame,
            'local_points': local_points,
            'map_points': map_points,
            'transform_ready': transform_ready,
            'reset_pending': reset_pending,
        }
        with self._state_lock:
            self._latest_scan_points = payload
        self._servers.websocket.send_json(payload)

    def _particle_callback(self, message: ParticleCloud):
        if self._localization_reset_pending:
            # Global localization immediately publishes a new, widely spread
            # particle cloud.  Keep it off the map until the operator supplies
            # an explicit initial pose, otherwise it looks like stale matching.
            if (
                self._initial_pose_sent_time <= 0.0 or
                time.monotonic() - self._initial_pose_sent_time < 0.5
            ):
                return
            self._localization_reset_pending = False
            with self._state_lock:
                self._localization_reset_state = {
                    'state': 'completed',
                    'message': '已收到初始位置，定位器正在重新匹配',
                }
        particles = message.particles
        # AMCL defaults to at most 2000 particles; keep all of them so the Web
        # convergence view is as complete as RViz, with a guard for custom
        # configurations using much larger clouds.
        step = max(1, math.ceil(len(particles) / 2500))
        sampled = particles[::step]
        points = [
            [
                float(particle.pose.position.x),
                float(particle.pose.position.y),
                _yaw_from_quaternion(particle.pose.orientation),
                float(particle.weight),
            ]
            for particle in sampled
        ]

        weights = [max(0.0, float(particle.weight)) for particle in particles]
        weight_sum = sum(weights)
        if particles and weight_sum <= 0.0:
            weights = [1.0] * len(particles)
            weight_sum = float(len(particles))
        if particles:
            mean_x = sum(
                weight * float(particle.pose.position.x)
                for particle, weight in zip(particles, weights)
            ) / weight_sum
            mean_y = sum(
                weight * float(particle.pose.position.y)
                for particle, weight in zip(particles, weights)
            ) / weight_sum
            spread = math.sqrt(sum(
                weight * (
                    (float(particle.pose.position.x) - mean_x) ** 2 +
                    (float(particle.pose.position.y) - mean_y) ** 2
                )
                for particle, weight in zip(particles, weights)
            ) / weight_sum)
        else:
            mean_x = None
            mean_y = None
            spread = None

        payload = {
            'type': 'particles',
            'frame_id': message.header.frame_id or self.map_frame,
            'count': len(particles),
            'points': points,
            'mean': [mean_x, mean_y] if mean_x is not None else None,
            'spread': spread,
        }
        with self._state_lock:
            self._latest_particles = payload
        self._servers.websocket.send_json(payload)

    def _local_costmap_callback(self, message: OccupancyGrid):
        with self._state_lock:
            self._local_costmap_info = {
                'frame_id': message.header.frame_id or self.odom_frame,
                'width': int(message.info.width),
                'height': int(message.info.height),
                'resolution': float(message.info.resolution),
                'age': 0.0,
            }
            self._local_costmap_time = time.monotonic()
        self._publish_costmap('local', message)

    def _global_costmap_callback(self, message: OccupancyGrid):
        self._publish_costmap('global', message)

    def _publish_costmap(self, scope: str, message: OccupancyGrid):
        frame_id = message.header.frame_id or (
            self.map_frame if scope == 'global' else self.odom_frame
        )
        frame_x = 0.0
        frame_y = 0.0
        frame_yaw = 0.0
        transform_ready = frame_id == self.map_frame
        if not transform_ready:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self.map_frame,
                    frame_id,
                    Time(),
                )
                frame_x = float(transform.transform.translation.x)
                frame_y = float(transform.transform.translation.y)
                frame_yaw = _yaw_from_quaternion(transform.transform.rotation)
                transform_ready = True
            except TransformException:
                pass

        origin = message.info.origin
        origin_yaw = _yaw_from_quaternion(origin.orientation)
        cosine = math.cos(frame_yaw)
        sine = math.sin(frame_yaw)
        pose_in_map = {
            'x': frame_x + cosine * float(origin.position.x) -
            sine * float(origin.position.y),
            'y': frame_y + sine * float(origin.position.x) +
            cosine * float(origin.position.y),
            'yaw': frame_yaw + origin_yaw,
        }
        encoded_cells = bytes(
            (int(value) + 1) & 0xFF for value in message.data
        )
        payload = {
            'type': 'costmap',
            'scope': scope,
            'frame_id': frame_id,
            'target_frame': self.map_frame,
            'transform_ready': transform_ready,
            'pose': pose_in_map,
            'width': int(message.info.width),
            'height': int(message.info.height),
            'resolution': float(message.info.resolution),
            'encoding': 'base64-offset-1',
            'data': base64.b64encode(encoded_cells).decode('ascii'),
        }
        with self._state_lock:
            self._latest_costmaps[scope] = payload
        self._servers.websocket.send_json(payload)

    def _path_callback(self, message: Path):
        poses = message.poses
        step = max(1, math.ceil(len(poses) / 400))
        points = [
            [float(pose.pose.position.x), float(pose.pose.position.y)]
            for pose in poses[::step]
        ]
        payload = {
            'type': 'path',
            'frame_id': message.header.frame_id or self.map_frame,
            'points': points,
        }
        with self._state_lock:
            self._latest_path = payload
        self._servers.websocket.send_json(payload)

    def _mppi_trajectories_callback(self, message: MarkerArray):
        """Publish a throttled, map-frame view of MPPI rollouts to the browser."""
        now = time.monotonic()
        if now - self._last_mppi_trajectory_web_time < 0.2:
            return
        self._last_mppi_trajectory_web_time = now

        markers = [
            marker for marker in message.markers
            if marker.action == Marker.ADD
        ]
        markers.sort(key=lambda marker: (marker.ns, marker.id))
        source_frame = next(
            (
                marker.header.frame_id
                for marker in markers if marker.header.frame_id
            ),
            self.odom_frame,
        )

        transform_ready = source_frame == self.map_frame
        translation_x = 0.0
        translation_y = 0.0
        transform_yaw = 0.0
        if not transform_ready:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self.map_frame,
                    source_frame,
                    Time(),
                )
                translation_x = float(transform.transform.translation.x)
                translation_y = float(transform.transform.translation.y)
                transform_yaw = _yaw_from_quaternion(
                    transform.transform.rotation)
                transform_ready = True
            except TransformException:
                pass

        cosine = math.cos(transform_yaw)
        sine = math.sin(transform_yaw)

        def point_in_map(marker):
            x = float(marker.pose.position.x)
            y = float(marker.pose.position.y)
            return [
                translation_x + cosine * x - sine * y,
                translation_y + sine * x + cosine * y,
            ]

        candidate_markers = [
            marker for marker in markers
            if 'candidate' in marker.ns.lower()
        ]
        optimal_markers = [
            marker for marker in markers
            if 'optimal' in marker.ns.lower()
        ]

        # Keep WebSocket traffic bounded even when batch_size is increased.
        candidate_step = max(1, math.ceil(len(candidate_markers) / 1200))
        candidate_points = [
            point_in_map(marker)
            for marker in candidate_markers[::candidate_step]
        ] if transform_ready else []
        optimal_points = [
            point_in_map(marker)
            for marker in optimal_markers
        ] if transform_ready else []

        payload = {
            'type': 'mppi_trajectories',
            'frame_id': source_frame,
            'target_frame': self.map_frame,
            'transform_ready': transform_ready,
            'candidate_count': len(candidate_markers),
            'optimal_count': len(optimal_markers),
            'candidate_points': candidate_points,
            'optimal_points': optimal_points,
        }
        with self._state_lock:
            self._latest_mppi_trajectories = payload
        self._servers.websocket.send_json(payload)

    def _on_web_message(self, raw_message: str):
        try:
            message = json.loads(raw_message)
            if not isinstance(message, dict) or 'type' not in message:
                raise ValueError('消息必须是包含 type 的对象')
            self._command_queue.put_nowait(message)
        except (json.JSONDecodeError, ValueError, queue.Full) as error:
            self._servers.websocket.send_json({
                'type': 'error',
                'message': f'无效网页指令: {error}',
            })

    def _on_web_connect(self, client):
        with self._state_lock:
            map_payload = self._latest_map
            path_payload = self._latest_path
            mppi_payload = self._latest_mppi_trajectories
            scan_payload = self._latest_scan_points
            particle_payload = self._latest_particles
            costmap_payloads = list(self._latest_costmaps.values())
        if map_payload is not None:
            self._servers.websocket.send_json(map_payload, client)
        if path_payload is not None:
            self._servers.websocket.send_json(path_payload, client)
        if mppi_payload is not None:
            self._servers.websocket.send_json(mppi_payload, client)
        if scan_payload is not None:
            self._servers.websocket.send_json(scan_payload, client)
        if particle_payload is not None:
            self._servers.websocket.send_json(particle_payload, client)
        for costmap_payload in costmap_payloads:
            if costmap_payload is not None:
                self._servers.websocket.send_json(costmap_payload, client)
        self._servers.websocket.send_json(self._telemetry_payload(), client)
        if self._scene_relay is not None:
            self._scene_relay.snapshot(client)
        if self._launch_manager is not None:
            self._servers.websocket.send_json(
                self._launch_manager.snapshot(), client)
        with self._state_lock:
            waypoints = list(self._latest_pct_waypoints)
        if waypoints:
            self._servers.websocket.send_json({
                'type': 'pct_waypoints_status',
                'state': 'published',
                'message': f'已恢复 {len(waypoints)} 个 PCT 途经点',
                'waypoints': waypoints,
            }, client)

    def _on_launch_event(self, message: dict):
        self._servers.websocket.send_json(message)

    def _external_launch_conflicts(self) -> list[str]:
        active = self._launch_manager.snapshot().get('active')
        if active and active.get('running'):
            return []
        node_names = {
            name for name, _namespace
            in self.get_node_names_and_namespaces()
        }
        return sorted(node_names & _REMOTE_LAUNCH_NODE_NAMES)

    def _process_web_commands(self):
        for _ in range(8):
            try:
                message = self._command_queue.get_nowait()
            except queue.Empty:
                return

            message_type = message.get('type')
            try:
                if message_type == 'nav_goal':
                    self._send_navigation_goal(message)
                elif message_type == 'scanplanner_goal':
                    self._publish_scanplanner_goal(message)
                elif message_type == 'pct_waypoints':
                    self._publish_pct_waypoints(message)
                elif message_type == 'initial_pose':
                    self._publish_initial_pose(message)
                elif message_type == 'cancel_navigation':
                    self._cancel_navigation()
                elif message_type == 'save_map':
                    self._save_map(message)
                elif message_type == 'set_inflation':
                    self._set_inflation(message)
                elif message_type == 'clear_visualization':
                    self._clear_visualization()
                elif message_type == 'reset_localization':
                    self._reset_localization()
                elif message_type == 'request_snapshot':
                    self._servers.websocket.send_json(self._telemetry_payload())
                    self._servers.websocket.send_json(
                        self._launch_manager.snapshot())
                elif message_type == 'request_scene_snapshot':
                    if self._scene_relay is not None:
                        self._scene_relay.snapshot()
                elif message_type == 'launch_start':
                    conflicts = self._external_launch_conflicts()
                    if conflicts:
                        visible_names = ', '.join(conflicts[:5])
                        if len(conflicts) > 5:
                            visible_names += f' 等 {len(conflicts)} 个节点'
                        raise LaunchManagerError(
                            '检测到命令行启动的 ROS 流程: '
                            f'{visible_names}。请先停止旧 launch，'
                            '再从网页启动。'
                        )
                    self._launch_manager.start(
                        str(message.get('profile_id', '')),
                        message.get('map_name'),
                        message.get('parameters'),
                    )
                elif message_type == 'launch_stop':
                    self._stop_managed_launch()
                elif message_type == 'launch_clear_logs':
                    self._launch_manager.clear_logs()
                elif message_type == 'request_launch_status':
                    self._servers.websocket.send_json(
                        self._launch_manager.snapshot())
                else:
                    raise ValueError(f'未知消息类型: {message_type}')
            except (TypeError, ValueError, LaunchManagerError) as error:
                error_type = (
                    'launch_error'
                    if isinstance(message_type, str)
                    and message_type.startswith('launch_')
                    else 'error'
                )
                self._servers.websocket.send_json({
                    'type': error_type,
                    'message': str(error),
                })

    @staticmethod
    def _coordinates(message: dict) -> tuple[float, float, float]:
        x = float(message['x'])
        y = float(message['y'])
        yaw = float(message.get('yaw', 0.0))
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError('坐标必须是有限数值')
        return x, y, yaw

    def _send_navigation_goal(self, message: dict):
        x, y, yaw = self._coordinates(message)
        if not self._navigate_client.server_is_ready():
            self._set_nav_state('unavailable', 'NavigateToPose 服务未就绪')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        _set_yaw(goal.pose.pose.orientation, yaw)

        with self._state_lock:
            self._goal = {'x': x, 'y': y, 'yaw': yaw}
        self._set_nav_state('sending', '正在提交导航目标')
        future = self._navigate_client.send_goal_async(
            goal,
            feedback_callback=self._navigation_feedback,
        )
        future.add_done_callback(self._goal_response)

    def _publish_scanplanner_goal(self, message: dict):
        """Publish a confirmed mobile 3D goal for SCAN-Planner mode 1."""
        x, y, yaw = self._coordinates(message)
        goal = PoseStamped()
        goal.header.frame_id = self.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = float(message.get('z', 0.0))
        if not math.isfinite(goal.pose.position.z):
            raise ValueError('目标高度必须是有限数值')
        _set_yaw(goal.pose.orientation, yaw)
        self._scanplanner_goal_publisher.publish(goal)
        self._servers.websocket.send_json({
            'type': 'scanplanner_goal_status',
            'state': 'published',
            'message': (
                f'目标已发布到 {self.scanplanner_goal_topic}: '
                f'x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}'
            ),
            'goal': {'x': x, 'y': y, 'z': goal.pose.position.z, 'yaw': yaw},
        })

    def _publish_pct_waypoints(self, message: dict):
        """Publish a bounded ordered route for the PCT global planner."""
        values = message.get('waypoints')
        if not isinstance(values, list) or not values:
            raise ValueError('PCT 多点路线至少需要一个途经点')
        if len(values) > 50:
            raise ValueError('PCT 多点路线最多允许 50 个途经点')

        route = Path()
        route.header.frame_id = self.map_frame
        route.header.stamp = self.get_clock().now().to_msg()
        public_waypoints = []
        for index, item in enumerate(values):
            if not isinstance(item, dict):
                raise ValueError(f'途经点 {index + 1} 格式错误')
            x, y, yaw = self._coordinates(item)
            z = float(item.get('z', 0.0))
            if not math.isfinite(z):
                raise ValueError(f'途经点 {index + 1} 高度必须是有限数值')
            pose = PoseStamped()
            pose.header = route.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            _set_yaw(pose.pose.orientation, yaw)
            route.poses.append(pose)
            public_waypoints.append({'x': x, 'y': y, 'z': z, 'yaw': yaw})

        self._pct_waypoints_publisher.publish(route)
        with self._state_lock:
            self._latest_pct_waypoints = public_waypoints
        self._servers.websocket.send_json({
            'type': 'pct_waypoints_status',
            'state': 'published',
            'message': (
                f'已向 {self.pct_waypoints_topic} 发布 '
                f'{len(route.poses)} 个有序途经点'),
            'waypoints': public_waypoints,
        })

    def _goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:  # action transport errors are implementation-specific
            self._set_nav_state('error', f'目标提交失败: {error}')
            return
        if not goal_handle.accepted:
            self._set_nav_state('rejected', 'Nav2 拒绝了目标')
            return
        with self._state_lock:
            self._goal_handle = goal_handle
        self._set_nav_state('navigating', '正在导航')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._navigation_result)

    def _navigation_feedback(self, feedback_message):
        feedback = feedback_message.feedback
        duration = feedback.navigation_time
        navigation_time = float(duration.sec) + float(duration.nanosec) * 1e-9
        with self._state_lock:
            self._nav_state = {
                'state': 'navigating',
                'message': '正在导航',
                'distance_remaining': float(feedback.distance_remaining),
                'navigation_time': navigation_time,
                'recoveries': int(feedback.number_of_recoveries),
            }

    def _navigation_result(self, future):
        try:
            wrapped_result = future.result()
            status = wrapped_result.status
        except Exception as error:  # action transport errors are implementation-specific
            self._set_nav_state('error', f'导航结果异常: {error}')
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._set_nav_state('succeeded', '已到达目标')
        elif status == GoalStatus.STATUS_CANCELED:
            self._set_nav_state('canceled', '导航已取消')
        elif status == GoalStatus.STATUS_ABORTED:
            self._set_nav_state('aborted', '导航失败')
        else:
            self._set_nav_state('finished', f'导航结束，状态码 {status}')
        with self._state_lock:
            self._goal_handle = None

    def _cancel_navigation(self):
        with self._state_lock:
            goal_handle = self._goal_handle
        if goal_handle is None:
            self._set_nav_state('idle', '当前没有活动导航目标')
            return
        goal_handle.cancel_goal_async()
        self._set_nav_state('canceling', '正在取消导航')

    def _publish_initial_pose(self, message: dict):
        x, y, yaw = self._coordinates(message)
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        pose.pose.pose.position.z = float(message.get('z', 0.0))
        if not math.isfinite(pose.pose.pose.position.z):
            raise ValueError('初始位置地面高度必须是有限数值')
        _set_yaw(pose.pose.pose.orientation, yaw)
        pose.pose.covariance[0] = 0.25
        pose.pose.covariance[7] = 0.25
        pose.pose.covariance[35] = math.radians(15.0) ** 2
        self._initial_pose_sent_time = time.monotonic()
        if self._localization_reset_pending:
            with self._state_lock:
                self._localization_reset_state = {
                    'state': 'initializing',
                    'message': '初始位置已发送，等待定位器重新匹配',
                }
        if (
            self.localization_backend != 'cartographer'
            and self._set_initial_pose_client.service_is_ready()
        ):
            request = SetInitialPose.Request()
            request.pose = pose
            future = self._set_initial_pose_client.call_async(request)
            future.add_done_callback(
                lambda completed, fallback_pose=pose:
                self._initial_pose_result(completed, fallback_pose)
            )
            delivery = f'已提交给 {self.set_initial_pose_service}'
        else:
            # RViz also publishes this topic. In graph-localization mode the
            # Cartographer bridge consumes it and starts a new trajectory.
            self._initial_pose_publisher.publish(pose)
            delivery = f'已发布到 {self.initial_pose_topic}'
        self._servers.websocket.send_json({
            'type': 'initial_pose_status',
            'state': 'published',
            'message': f'{delivery}: ({x:.2f}, {y:.2f})',
            'pose': {
                'x': x,
                'y': y,
                'z': pose.pose.pose.position.z,
                'yaw': yaw,
            },
        })

    def _initial_pose_result(self, future, fallback_pose):
        try:
            future.result()
            message = '定位器已接收初始位置，等待激光匹配'
        except Exception as error:  # service transport failures vary by RMW
            self._initial_pose_publisher.publish(fallback_pose)
            message = (
                f'初始位置服务失败，已改发 {self.initial_pose_topic}: {error}'
            )
        self._servers.websocket.send_json({
            'type': 'notice',
            'message': message,
        })

    def _request_particle_refresh(self):
        """Ask stationary AMCL for a fresh cloud after a Web restart."""
        if self.localization_backend == 'cartographer':
            return
        now = time.monotonic()
        with self._state_lock:
            needs_particles = (
                self._latest_pose is not None and
                self._latest_particles is None and
                not self._localization_reset_pending
            )
        if (
            not needs_particles or self._nomotion_update_pending or
            now - self._last_nomotion_update_time < 5.0 or
            not self._nomotion_update_client.service_is_ready()
        ):
            return
        self._nomotion_update_pending = True
        self._last_nomotion_update_time = now
        future = self._nomotion_update_client.call_async(Empty.Request())
        future.add_done_callback(self._particle_refresh_result)

    def _particle_refresh_result(self, _future):
        self._nomotion_update_pending = False

    def _clear_visualization(self, broadcast: bool = True):
        with self._state_lock:
            self._latest_path = None
            self._latest_mppi_trajectories = None
            self._latest_particles = None
            self._latest_scan_points = None
            if self._nav_state.get('state') not in {
                'sending', 'navigating', 'canceling'
            }:
                self._goal = None
        if broadcast:
            self._servers.websocket.send_json({
                'type': 'visualization_cleared',
                'message': '已清除旧激光匹配点、定位粒子和规划轨迹',
            })

    def _reset_localization(self):
        with self._state_lock:
            goal_handle = self._goal_handle
            nav_state = self._nav_state.get('state')
        if goal_handle is not None or nav_state in {
            'sending', 'navigating', 'canceling'
        }:
            raise ValueError('请先取消当前导航，再重置定位')
        if not self._reset_localization_client.service_is_ready():
            raise ValueError('重定位服务未就绪')

        self._clear_visualization(broadcast=True)
        self._localization_reset_pending = True
        self._initial_pose_sent_time = 0.0
        with self._state_lock:
            self._latest_pose = None
            self._localization_reset_state = {
                'state': 'resetting',
                'message': '正在重置定位器',
            }
        future = self._reset_localization_client.call_async(Empty.Request())
        future.add_done_callback(self._reset_localization_result)

    def _reset_localization_result(self, future):
        try:
            future.result()
            state = 'succeeded'
            message = '已重置定位器，请重新设置初始位置'
        except Exception as error:  # service transport errors are implementation-specific
            self._localization_reset_pending = False
            state = 'error'
            message = f'定位器重置失败: {error}'
        with self._state_lock:
            self._localization_reset_state = {
                'state': state,
                'message': message,
            }
        self._servers.websocket.send_json({
            'type': 'localization_reset_status',
            'state': state,
            'message': message,
        })

    def _save_map(self, message: dict):
        map_name = str(message.get('name', '')).strip()
        if not map_name:
            map_name = time.strftime('gazebo_map_%Y%m%d_%H%M%S')
        if len(map_name) > 64 or re.fullmatch(r'[\w-]+', map_name) is None:
            raise ValueError('地图名只能包含中英文、数字、下划线和短横线')
        if not self._save_map_client.service_is_ready():
            self._set_save_map_state('unavailable', '地图保存服务未就绪')
            return

        os.makedirs(self.map_save_directory, exist_ok=True)
        map_path = os.path.join(self.map_save_directory, map_name)
        request = SaveMap.Request()
        request.map_topic = self.map_topic
        request.map_url = map_path
        request.image_format = 'pgm'
        request.map_mode = 'trinary'
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65
        self._set_save_map_state('saving', '正在保存地图', map_path)
        future = self._save_map_client.call_async(request)
        future.add_done_callback(self._save_map_result)

    def _stop_managed_launch(self):
        """Save a FAST-LIO mapping PCD before stopping its launch process."""
        if not self._launch_manager.stop():
            return

        output_path = self._launch_manager.active_pcd_output_path()
        if output_path is None:
            self._finish_fastlio_map_save(
                False, '无法确定当前建图的 PCD 输出路径')
            return
        if not self._fastlio_map_save_client.service_is_ready():
            self._finish_fastlio_map_save(
                False,
                f'保存服务 {self.fastlio_map_save_service} 尚未就绪；'
                '请等待 FAST-LIO 正常建图后重试',
            )
            return

        self._servers.websocket.send_json({
            'type': 'launch_map_save_status',
            'state': 'saving',
            'message': f'正在保存 {os.path.basename(output_path)}，请勿关闭流程',
            'path': output_path,
        })
        try:
            future = self._fastlio_map_save_client.call_async(Trigger.Request())
        except Exception as error:  # RMW client errors vary by implementation
            self._finish_fastlio_map_save(False, f'无法调用保存服务：{error}')
            return
        future.add_done_callback(
            lambda completed, path=output_path:
            self._fastlio_map_save_result(completed, path)
        )

    def _fastlio_map_save_result(self, future, output_path: str):
        try:
            response = future.result()
            success = bool(response.success)
            detail = str(response.message or '').strip()
        except Exception as error:  # service transport failures vary by RMW
            self._finish_fastlio_map_save(False, f'保存服务调用失败：{error}')
            return

        if not success:
            self._finish_fastlio_map_save(
                False, detail or 'FAST-LIO 拒绝了地图保存请求')
            return
        try:
            file_size = os.path.getsize(output_path)
        except OSError as error:
            self._finish_fastlio_map_save(
                False, f'服务返回成功，但目标 PCD 不存在：{error}')
            return
        if file_size <= 0:
            self._finish_fastlio_map_save(
                False, '服务返回成功，但目标 PCD 是空文件')
            return

        size_mib = file_size / (1024.0 * 1024.0)
        summary = f'{os.path.basename(output_path)} · {size_mib:.1f} MiB'
        if detail:
            summary += f' · {detail}'
        self._finish_fastlio_map_save(True, summary, output_path)

    def _finish_fastlio_map_save(
        self,
        success: bool,
        message: str,
        output_path: str = '',
    ):
        transitioned = self._launch_manager.complete_save_before_stop(
            success, message)
        if not transitioned:
            return
        self._servers.websocket.send_json({
            'type': 'launch_map_save_status',
            'state': 'succeeded' if success else 'error',
            'message': (
                f'地图已保存，正在停止建图：{message}'
                if success else f'地图没有保存，建图保持运行：{message}'
            ),
            'path': output_path,
        })

    def _save_map_result(self, future):
        with self._state_lock:
            path = self._save_map_state.get('path')
        try:
            response = future.result()
            succeeded = bool(response.result)
        except Exception as error:  # service transport errors are implementation-specific
            self._set_save_map_state('error', f'地图保存异常: {error}', path)
            return
        if succeeded:
            self._set_save_map_state(
                'succeeded',
                f'地图已保存: {path}.yaml',
                path,
            )
        else:
            self._set_save_map_state('error', '地图保存失败', path)

    def _set_save_map_state(self, state: str, message: str, path=None):
        payload = {'state': state, 'message': message, 'path': path}
        with self._state_lock:
            self._save_map_state = payload
        self._servers.websocket.send_json({'type': 'save_map_status', **payload})

    @staticmethod
    def _inflation_parameter_names():
        return [
            'inflation_layer.enabled',
            'inflation_layer.inflation_radius',
            'inflation_layer.cost_scaling_factor',
            'inflation_layer.inflate_unknown',
            'inflation_layer.inflate_around_unknown',
        ]

    def _refresh_inflation_parameters(self):
        for scope, clients in self._inflation_clients.items():
            ready = clients['get'].service_is_ready() and clients['set'].service_is_ready()
            with self._state_lock:
                self._inflation_state[scope]['ready'] = ready
                setting = self._inflation_state[scope]['state'] == 'setting'
            if not ready or setting or self._inflation_query_pending[scope]:
                continue
            request = GetParameters.Request()
            request.names = self._inflation_parameter_names()
            self._inflation_query_pending[scope] = True
            future = clients['get'].call_async(request)
            future.add_done_callback(
                lambda completed, selected_scope=scope:
                self._inflation_get_result(selected_scope, completed)
            )

    def _inflation_get_result(self, scope: str, future):
        self._inflation_query_pending[scope] = False
        try:
            values = future.result().values
            state = {
                'ready': True,
                'state': 'ready',
                'message': '参数已同步',
                'enabled': bool(values[0].bool_value),
                'inflation_radius': float(values[1].double_value),
                'cost_scaling_factor': float(values[2].double_value),
                'inflate_unknown': bool(values[3].bool_value),
                'inflate_around_unknown': bool(values[4].bool_value),
            }
        except Exception as error:  # parameter service errors are implementation-specific
            with self._state_lock:
                state = dict(self._inflation_state[scope])
            state.update(state='error', message=f'读取参数失败: {error}')
        with self._state_lock:
            self._inflation_state[scope] = state

    @staticmethod
    def _double_parameter(name: str, value: float) -> Parameter:
        return Parameter(
            name=name,
            value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(value),
            ),
        )

    @staticmethod
    def _bool_parameter(name: str, value: bool) -> Parameter:
        return Parameter(
            name=name,
            value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL,
                bool_value=bool(value),
            ),
        )

    def _set_inflation(self, message: dict):
        scope = str(message.get('scope', ''))
        if scope not in self._inflation_clients:
            raise ValueError('膨胀层范围必须是 global 或 local')
        radius = float(message['inflation_radius'])
        scaling = float(message['cost_scaling_factor'])
        if not math.isfinite(radius) or not 0.0 <= radius <= 10.0:
            raise ValueError('inflation_radius 必须在 0.0 到 10.0 m 之间')
        if not math.isfinite(scaling) or not 0.01 <= scaling <= 100.0:
            raise ValueError('cost_scaling_factor 必须在 0.01 到 100.0 之间')
        client = self._inflation_clients[scope]['set']
        if not client.service_is_ready():
            raise ValueError(f'{scope} Costmap 参数服务未就绪')

        request = SetParameters.Request()
        request.parameters = [
            self._bool_parameter(
                'inflation_layer.enabled', bool(message.get('enabled', True))),
            self._double_parameter('inflation_layer.inflation_radius', radius),
            self._double_parameter('inflation_layer.cost_scaling_factor', scaling),
            self._bool_parameter(
                'inflation_layer.inflate_unknown',
                bool(message.get('inflate_unknown', False)),
            ),
            self._bool_parameter(
                'inflation_layer.inflate_around_unknown',
                bool(message.get('inflate_around_unknown', False)),
            ),
        ]
        with self._state_lock:
            self._inflation_state[scope].update(
                state='setting',
                message='正在应用膨胀参数',
                enabled=bool(message.get('enabled', True)),
                inflation_radius=radius,
                cost_scaling_factor=scaling,
                inflate_unknown=bool(message.get('inflate_unknown', False)),
                inflate_around_unknown=bool(
                    message.get('inflate_around_unknown', False)),
            )
        future = client.call_async(request)
        future.add_done_callback(
            lambda completed, selected_scope=scope:
            self._inflation_set_result(selected_scope, completed)
        )

    def _inflation_set_result(self, scope: str, future):
        try:
            results = future.result().results
            failures = [result.reason for result in results if not result.successful]
            if failures:
                state = 'error'
                message = '; '.join(reason or '参数被拒绝' for reason in failures)
            else:
                state = 'succeeded'
                message = f'{scope} 膨胀参数已生效'
        except Exception as error:  # parameter service errors are implementation-specific
            state = 'error'
            message = f'设置膨胀参数失败: {error}'
        with self._state_lock:
            self._inflation_state[scope].update(state=state, message=message)
        self._inflation_query_pending[scope] = False
        self._servers.websocket.send_json({
            'type': 'inflation_status',
            'scope': scope,
            'state': state,
            'message': message,
        })

    def _set_nav_state(self, state: str, message: str):
        with self._state_lock:
            self._nav_state = {
                'state': state,
                'message': message,
                'distance_remaining': None,
                'navigation_time': None,
            }
        self._servers.websocket.send_json({
            'type': 'nav_status',
            **self._nav_state,
        })

    def _update_robot_pose(self):
        if self._localization_reset_pending:
            with self._state_lock:
                self._latest_pose = None
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
            )
        except TransformException:
            with self._state_lock:
                self._latest_pose = None
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        with self._state_lock:
            self._latest_pose = {
                'x': float(translation.x),
                'y': float(translation.y),
                'yaw': _yaw_from_quaternion(rotation),
            }

    def _update_node_presence(self):
        now = time.monotonic()
        if now - self._node_presence_time < 1.0:
            return
        names = {
            f'{namespace.rstrip("/")}/{name}'.replace('//', '/')
            for name, namespace in self.get_node_names_and_namespaces()
        }
        cartographer_present = any(
            name.endswith('/cartographer_node') for name in names)
        cartographer_bridge_present = any(
            name.endswith('/cartographer_initial_pose_bridge')
            for name in names)
        graph_localization_mode = (
            self.localization_backend == 'cartographer'
            or cartographer_bridge_present
        )
        self._node_presence = {
            'slam': any(name.endswith('/slam_toolbox') for name in names)
            or (cartographer_present and not graph_localization_mode),
            'amcl': any(name.endswith('/amcl') for name in names),
            'cartographer_localization': (
                cartographer_present and cartographer_bridge_present),
        }
        self._node_presence_time = now

    def _mapping_status(self, now: float, tf_ready: bool) -> dict:
        with self._state_lock:
            metrics = dict(self._latest_map_metrics) if self._latest_map_metrics else None
            scan = dict(self._latest_scan)
            map_age = now - self._latest_map_time if self._latest_map_time else None
            scan_age = now - self._latest_scan_time if self._latest_scan_time else None

        map_fresh = map_age is not None and map_age < 3.0
        scan_fresh = scan_age is not None and scan_age < 1.0
        score = 0
        score += 25 if self._node_presence['slam'] else 0
        score += 20 if scan_fresh else 0
        score += 20 if tf_ready else 0
        score += 20 if map_fresh else 0
        if metrics is not None:
            stability = metrics.get('stability')
            score += 8 if stability is None else round(15 * stability / 100.0)

        if not self._node_presence['slam']:
            state, message = 'inactive', '未检测到 SLAM 建图节点'
        elif not scan_fresh:
            state, message = 'blocked', '激光数据中断，建图无法继续'
        elif metrics is None:
            state, message = 'starting', '传感器已就绪，等待第一帧地图'
        elif not tf_ready:
            state, message = 'warning', '地图在更新，但 map 到机器人 TF 不可用'
        elif not map_fresh:
            state, message = 'stale', '地图已停止更新，请检查 SLAM 状态'
        elif metrics['known_area'] < 2.0:
            state, message = 'starting', '建图刚开始，请缓慢移动并扫描周围'
        elif metrics.get('stability') is not None and metrics['stability'] < 90.0:
            state, message = 'changing', '地图变化较大，请减速并重访已建区域'
        elif metrics.get('known_delta_area', 0.0) > 0.05:
            state, message = 'exploring', '正在扩展已知区域，继续覆盖未知空间'
        else:
            state, message = 'stable', '当前地图更新稳定，请目视检查重影和墙体断裂'

        scan['age'] = scan_age
        if metrics is not None:
            metrics['age'] = map_age
        return {
            'state': state,
            'message': message,
            'health_score': max(0, min(100, score)),
            'map': metrics,
            'scan': scan,
            'checks': {
                'slam': self._node_presence['slam'],
                'scan': scan_fresh,
                'tf': tf_ready,
                'map': map_fresh,
            },
            'heuristic': True,
        }

    def _workflow_status(self, tf_ready: bool, nav2_ready: bool) -> dict:
        slam_ready = self._node_presence['slam']
        amcl_ready = self._node_presence['amcl']
        cartographer_ready = self._node_presence['cartographer_localization']
        localizer_ready = amcl_ready or cartographer_ready
        graph_mode = (
            self.localization_backend == 'cartographer'
            or cartographer_ready
        )
        localizer_name = '图 SLAM' if graph_mode else 'AMCL'
        if slam_ready:
            stage = 'mapping'
            title = '建图中'
            message = '驾驶机器人覆盖环境，完成后在网页保存地图'
        elif localizer_ready and not tf_ready:
            stage = 'localization'
            title = f'{localizer_name}等待初始位置'
            message = '点击“初始位置”，在地图上拖出机器人朝向'
        elif localizer_ready and tf_ready and nav2_ready:
            stage = 'planning'
            title = '重定位完成'
            message = '可以在地图上设置导航目标并查看规划轨迹'
        elif localizer_ready:
            stage = 'localization'
            title = f'{localizer_name}正在就绪'
            message = '等待定位 TF，必要时重新设置初始位置'
        elif nav2_ready:
            stage = 'planning'
            title = '规划服务已就绪'
            message = '未检测到定位器，发送目标前请确认 map TF 正常'
        else:
            stage = 'waiting'
            title = '等待 ROS 流程'
            message = '请先启动 SLAM 建图，或加载地图启动定位/Nav2'
        return {
            'stage': stage,
            'title': title,
            'message': message,
            'slam_ready': slam_ready,
            'amcl_ready': amcl_ready,
            'cartographer_localization_ready': cartographer_ready,
            'localizer_ready': localizer_ready,
            'localizer_type': 'cartographer' if graph_mode else 'amcl',
            'planner_ready': nav2_ready,
        }

    def _telemetry_payload(self) -> dict:
        self._update_node_presence()
        now = time.monotonic()
        with self._state_lock:
            cmd = dict(self._latest_cmd)
            if self._latest_cmd_time > 0.0:
                cmd['age'] = max(0.0, now - self._latest_cmd_time)
            tf_ready = self._latest_pose is not None
            nav2_ready = self._navigate_client.server_is_ready()
            return {
                'type': 'telemetry',
                'pose': dict(self._latest_pose) if self._latest_pose else None,
                'cmd_vel': cmd,
                'goal': dict(self._goal) if self._goal else None,
                'nav': dict(self._nav_state),
                'workflow': self._workflow_status(tf_ready, nav2_ready),
                'mapping': self._mapping_status(now, tf_ready),
                'save_map': dict(self._save_map_state),
                'localization_reset': dict(self._localization_reset_state),
                'inflation': {
                    scope: dict(value)
                    for scope, value in self._inflation_state.items()
                },
                'map_ready': self._latest_map is not None,
                'tf_ready': tf_ready,
                'nav2_ready': nav2_ready,
                'scan_ready': (
                    self._latest_scan_time > 0.0 and
                    now - self._latest_scan_time < 1.0
                ),
                'save_map_ready': self._save_map_client.service_is_ready(),
                'localization_reset_ready': (
                    self._reset_localization_client.service_is_ready()
                ),
                'set_initial_pose_ready': (
                    self._set_initial_pose_client.service_is_ready()
                ),
                'nomotion_update_ready': (
                    self._nomotion_update_client.service_is_ready()
                ),
                'frames': {
                    'map': self.map_frame,
                    'odom': self.odom_frame,
                    'base': self.base_frame,
                    'scan': (
                        self._latest_scan_points.get('source_frame')
                        if self._latest_scan_points else 'base_scan'
                    ),
                    'local_costmap': self._local_costmap_info['frame_id'],
                },
                'local_costmap': {
                    **self._local_costmap_info,
                    'age': (
                        now - self._local_costmap_time
                        if self._local_costmap_time else None
                    ),
                },
                'topics': {
                    'map': self.map_topic,
                    'path': self.path_topic,
                    'mppi_trajectories': self.mppi_trajectories_topic,
                    'cmd_vel': self.cmd_vel_topic,
                    'scan': self.scan_topic,
                    'particles': self.particle_topic,
                    'local_costmap': self.local_costmap_topic,
                },
                'map_save_directory': self.map_save_directory,
            }

    def _broadcast_telemetry(self):
        self._update_robot_pose()
        self._request_particle_refresh()
        self._servers.websocket.send_json(self._telemetry_payload())

    def destroy_node(self):
        if self._launch_manager is not None:
            self._launch_manager.shutdown()
        try:
            self._servers.stop()
        except KeyboardInterrupt:
            # ros2 launch can deliver SIGINT while the HTTP server is waiting
            # for its worker thread.  Shutdown is already in progress.
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = Nav2WebBridge()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
