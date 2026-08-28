#!/usr/bin/env python3

"""Apply Web/RViz initial poses to Cartographer pure localization.

Cartographer cannot reuse AMCL's SetInitialPose service.  Relocalization is
performed by finishing the current local trajectory and starting a new one
whose initial pose is expressed relative to the frozen trajectory loaded from
the pbstream.
"""

from copy import deepcopy

from cartographer_ros_msgs.msg import TrajectoryStates
from cartographer_ros_msgs.srv import (
    FinishTrajectory,
    GetTrajectoryStates,
    StartTrajectory,
)
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty


class CartographerInitialPoseBridge(Node):
    """Translate /initialpose and reset requests into trajectory services."""

    def __init__(self):
        super().__init__('cartographer_initial_pose_bridge')
        self.declare_parameter('configuration_directory', '')
        self.declare_parameter(
            'configuration_basename',
            'ydlidar_cartographer_2d_localization.lua',
        )
        self._configuration_directory = str(
            self.get_parameter('configuration_directory').value)
        self._configuration_basename = str(
            self.get_parameter('configuration_basename').value)

        self._states_client = self.create_client(
            GetTrajectoryStates, '/get_trajectory_states')
        self._finish_client = self.create_client(
            FinishTrajectory, '/finish_trajectory')
        self._start_client = self.create_client(
            StartTrajectory, '/start_trajectory')

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self._initial_pose_callback,
            10,
        )
        self.create_service(
            Empty,
            '/reinitialize_global_localization',
            self._reset_callback,
        )

        self._pending_pose = None
        self._wait_for_initial_pose = False
        self._operation_requested = False
        self._busy = False
        self._frozen_trajectory_id = None
        self._active_trajectory_ids = []
        self.create_timer(0.25, self._pump)

        self.get_logger().info(
            'Cartographer initial-pose bridge ready: /initialpose and '
            '/reinitialize_global_localization')

    def _initial_pose_callback(self, message):
        if message.header.frame_id not in ('', 'map'):
            self.get_logger().error(
                'Rejected initial pose in frame "%s"; expected "map".'
                % message.header.frame_id)
            return
        self._pending_pose = deepcopy(message.pose.pose)
        self._wait_for_initial_pose = False
        self._operation_requested = True
        self.get_logger().info(
            'Initial pose received; restarting the Cartographer localization '
            'trajectory against the frozen pbstream.')

    def _reset_callback(self, _request, response):
        self._pending_pose = None
        self._wait_for_initial_pose = True
        self._operation_requested = True
        self.get_logger().info(
            'Localization reset requested; the active trajectory will be '
            'finished. Set a new initial pose on the map afterward.')
        return response

    def _services_ready(self):
        return (
            self._states_client.service_is_ready()
            and self._finish_client.service_is_ready()
            and self._start_client.service_is_ready()
        )

    def _pump(self):
        if self._busy or not self._operation_requested:
            return
        if not self._services_ready():
            self.get_logger().warn(
                'Waiting for Cartographer trajectory services...',
                throttle_duration_sec=5.0,
            )
            return
        self._busy = True
        self._operation_requested = False
        future = self._states_client.call_async(GetTrajectoryStates.Request())
        future.add_done_callback(self._states_result)

    def _states_result(self, future):
        try:
            response = future.result()
        except Exception as error:
            self._fail('Could not query trajectory states: %s' % error)
            return
        if response.status.code != 0:
            self._fail(
                'Cartographer rejected trajectory-state query: %s'
                % response.status.message)
            return

        pairs = zip(
            response.trajectory_states.trajectory_id,
            response.trajectory_states.trajectory_state,
        )
        frozen_ids = []
        active_ids = []
        for trajectory_id, trajectory_state in pairs:
            if trajectory_state == TrajectoryStates.FROZEN:
                frozen_ids.append(int(trajectory_id))
            elif trajectory_state == TrajectoryStates.ACTIVE:
                active_ids.append(int(trajectory_id))

        if not frozen_ids:
            self._fail(
                'No frozen trajectory was found. Check that the pbstream '
                'exists and was loaded with -load_frozen_state=true.')
            return

        self._frozen_trajectory_id = min(frozen_ids)
        self._active_trajectory_ids = active_ids
        self._finish_next_trajectory()

    def _finish_next_trajectory(self):
        if not self._active_trajectory_ids:
            self._after_all_trajectories_finished()
            return
        trajectory_id = self._active_trajectory_ids.pop(0)
        request = FinishTrajectory.Request()
        request.trajectory_id = trajectory_id
        future = self._finish_client.call_async(request)
        future.add_done_callback(
            lambda completed, current_id=trajectory_id:
            self._finish_result(completed, current_id))

    def _finish_result(self, future, trajectory_id):
        try:
            response = future.result()
        except Exception as error:
            self._fail(
                'Could not finish trajectory %d: %s'
                % (trajectory_id, error))
            return
        if response.status.code != 0:
            self._fail(
                'Cartographer could not finish trajectory %d: %s'
                % (trajectory_id, response.status.message))
            return
        self._finish_next_trajectory()

    def _after_all_trajectories_finished(self):
        if self._wait_for_initial_pose and self._pending_pose is None:
            self._operation_requested = False
            self._busy = False
            self.get_logger().info(
                'Old localization trajectory cleared; waiting for '
                '/initialpose.')
            return
        if self._pending_pose is None:
            self._operation_requested = False
            self._busy = False
            return

        pose = self._pending_pose
        self._pending_pose = None
        self._operation_requested = False
        request = StartTrajectory.Request()
        request.configuration_directory = self._configuration_directory
        request.configuration_basename = self._configuration_basename
        request.use_initial_pose = True
        request.initial_pose = pose
        request.relative_to_trajectory_id = self._frozen_trajectory_id
        future = self._start_client.call_async(request)
        future.add_done_callback(self._start_result)

    def _start_result(self, future):
        try:
            response = future.result()
        except Exception as error:
            self._fail('Could not start localization trajectory: %s' % error)
            return
        if response.status.code != 0:
            self._fail(
                'Cartographer rejected the new localization trajectory: %s'
                % response.status.message)
            return

        self._wait_for_initial_pose = False
        self._busy = False
        self.get_logger().info(
            'Started Cartographer localization trajectory %d relative to '
            'frozen trajectory %d.'
            % (response.trajectory_id, self._frozen_trajectory_id))
        if self._pending_pose is not None or self._operation_requested:
            self._operation_requested = True

    def _fail(self, message):
        self.get_logger().error(message)
        self._pending_pose = None
        self._operation_requested = False
        self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = CartographerInitialPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
