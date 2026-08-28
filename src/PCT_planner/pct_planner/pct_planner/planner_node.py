import os

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from pct_planner.config import Config
from pct_planner.planner_wrapper import TomogramPlanner
from pct_planner.utils import traj2ros


class PCTPlanner(Node):
    """Plan a global 3-D body path from live localization and a clicked goal."""

    def __init__(self):
        super().__init__('pct_planner')
        tomogram_path = self.declare_parameter('tomogram_path', '').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.body_height = float(self.declare_parameter('body_height', 0.4).value)
        self.goal_z_is_body = bool(
            self.declare_parameter('goal_z_is_body', False).value)
        self.minimum_goal_distance = float(
            self.declare_parameter('minimum_goal_distance', 0.30).value)
        self.maximum_waypoints = int(
            self.declare_parameter('maximum_waypoints', 50).value)

        if not tomogram_path:
            raise ValueError('tomogram_path must not be empty')
        tomogram_path = os.path.realpath(os.path.expanduser(tomogram_path))
        if not os.path.isfile(tomogram_path):
            raise FileNotFoundError(f'tomogram does not exist: {tomogram_path}')
        if self.body_height <= 0.0:
            raise ValueError('body_height must be positive')
        if self.maximum_waypoints < 1 or self.maximum_waypoints > 200:
            raise ValueError('maximum_waypoints must be between 1 and 200')

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_pub = self.create_publisher(Path, 'global_path', qos)
        self.traversable_pub = self.create_publisher(
            PointCloud2, 'traversable_cloud', qos)
        self.body_sub = self.create_subscription(
            Odometry, 'body_pose', self.body_callback,
            rclpy.qos.qos_profile_sensor_data)
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal', self.goal_callback, QoSProfile(depth=1))
        self.waypoints_sub = self.create_subscription(
            Path, 'waypoints', self.waypoints_callback, qos)

        self.body_pose = None
        self.planner = TomogramPlanner(Config(), body_height=self.body_height)
        self.get_logger().info(f'Loading PCT tomogram: {tomogram_path}')
        self.planner.loadTomogram(tomogram_path)
        self._publish_traversable_cloud()
        self.get_logger().info(
            f'PCT ready: goal -> global_path in frame {self.map_frame}; '
            f'body height {self.body_height:.3f} m')

    def _publish_traversable_cloud(self):
        points = self.planner.traversablePoints()
        header = Header()
        header.frame_id = self.map_frame
        header.stamp = self.get_clock().now().to_msg()
        self.traversable_pub.publish(point_cloud2.create_cloud_xyz32(header, points))
        self.get_logger().info(
            f'Published {len(points)} selectable PCT cells on traversable_cloud')

    def body_callback(self, message):
        if message.header.frame_id != self.map_frame:
            self.get_logger().warning(
                f'Ignoring body pose in {message.header.frame_id!r}; '
                f'expected {self.map_frame!r}', throttle_duration_sec=2.0)
            return
        self.body_pose = message.pose.pose

    def _goal_ground_z(self, position):
        value = float(position.z)
        return value - self.body_height if self.goal_z_is_body else value

    def goal_callback(self, message):
        if message.header.frame_id and message.header.frame_id != self.map_frame:
            self.get_logger().error(
                f'Ignoring goal in {message.header.frame_id!r}; '
                f'expected {self.map_frame!r}')
            return
        position = message.pose.position
        self._plan_route([(
            float(position.x), float(position.y),
            self._goal_ground_z(position))], 'single goal')

    def waypoints_callback(self, message):
        """Plan current body -> waypoint 1 -> ... as one global path."""
        if message.header.frame_id and message.header.frame_id != self.map_frame:
            self.get_logger().error(
                f'Ignoring waypoints in {message.header.frame_id!r}; '
                f'expected {self.map_frame!r}')
            return
        if not message.poses:
            self.get_logger().warning('Ignoring an empty PCT waypoint route')
            return
        if len(message.poses) > self.maximum_waypoints:
            self.get_logger().error(
                f'Ignoring {len(message.poses)} waypoints; maximum is '
                f'{self.maximum_waypoints}')
            return
        waypoints = []
        for index, pose_stamped in enumerate(message.poses):
            frame = pose_stamped.header.frame_id
            if frame and frame != self.map_frame:
                self.get_logger().error(
                    f'Waypoint {index + 1} uses frame {frame!r}; '
                    f'expected {self.map_frame!r}')
                return
            position = pose_stamped.pose.position
            values = (
                float(position.x), float(position.y),
                self._goal_ground_z(position))
            if not all(np.isfinite(value) for value in values):
                self.get_logger().error(
                    f'Waypoint {index + 1} contains a non-finite coordinate')
                return
            waypoints.append(values)
        self._plan_route(waypoints, f'{len(waypoints)} App waypoints')

    def _plan_route(self, waypoints, source):
        if self.body_pose is None:
            self.get_logger().warning(
                'Ignoring route until relocalized body_pose is available')
            return

        current = np.array([
            self.body_pose.position.x,
            self.body_pose.position.y,
        ], dtype=np.float32)
        current_ground_z = self.body_pose.position.z - self.body_height
        segments = []
        try:
            for index, (goal_x, goal_y, goal_ground_z) in enumerate(waypoints):
                goal = np.array([goal_x, goal_y], dtype=np.float32)
                planar_distance = float(np.linalg.norm(goal - current))
                if (planar_distance < self.minimum_goal_distance and
                        abs(goal_ground_z - current_ground_z) < 0.15):
                    self.get_logger().warning(
                        f'Skipping waypoint {index + 1}: too close to previous point')
                    continue
                self.get_logger().info(
                    f'PCT segment {index + 1}/{len(waypoints)}: '
                    f'[{current[0]:.2f}, {current[1]:.2f}, '
                    f'{current_ground_z:.2f}] -> '
                    f'[{goal[0]:.2f}, {goal[1]:.2f}, {goal_ground_z:.2f}]')
                trajectory = self.planner.plan(
                    current, goal,
                    start_ground_z=current_ground_z,
                    end_ground_z=goal_ground_z)
                if trajectory is None or len(trajectory) < 2:
                    raise RuntimeError(
                        f'no traversable path for waypoint {index + 1}')
                if segments and np.linalg.norm(
                        segments[-1][-1] - trajectory[0]) < 1e-4:
                    trajectory = trajectory[1:]
                segments.append(trajectory)
                current = np.asarray(trajectory[-1, :2], dtype=np.float32)
                current_ground_z = float(
                    trajectory[-1, 2] - self.body_height)
        except (RuntimeError, ValueError) as error:
            self.get_logger().error(f'PCT rejected {source}: {error}')
            return

        if not segments:
            self.get_logger().error('PCT route has no usable waypoint segment')
            return
        trajectory = np.concatenate(segments, axis=0)
        path = traj2ros(
            trajectory, frame_id=self.map_frame,
            stamp=self.get_clock().now().to_msg())
        self.path_pub.publish(path)
        self.get_logger().info(
            f'Published route through {len(segments)} segment(s): '
            f'{len(path.poses)} body-height path poses')


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = PCTPlanner()
        rclpy.spin(node)
    except (FileNotFoundError, ValueError) as error:
        rclpy.logging.get_logger('pct_planner').fatal(str(error))
        exit_code = 1
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    main()
