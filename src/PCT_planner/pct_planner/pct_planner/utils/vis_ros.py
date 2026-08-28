from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.clock import Clock
import math

def traj2ros(traj_3d, frame_id="map", stamp=None):
    path = Path()
    path.header.stamp = stamp if stamp is not None else Clock().now().to_msg()
    path.header.frame_id = frame_id
    for i in range(traj_3d.shape[0]):
        pose = PoseStamped()
        pose.header.stamp = path.header.stamp
        pose.header.frame_id = path.header.frame_id
        pose.pose.position.x = traj_3d[i, 0]
        pose.pose.position.y = traj_3d[i, 1]
        pose.pose.position.z = traj_3d[i, 2]
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        if traj_3d.shape[0] > 1:
            neighbor = min(i + 1, traj_3d.shape[0] - 1)
            if neighbor == i:
                neighbor = i - 1
            yaw = math.atan2(
                traj_3d[neighbor, 1] - traj_3d[i, 1],
                traj_3d[neighbor, 0] - traj_3d[i, 0])
        else:
            yaw = 0.0
        pose.pose.orientation.z = math.sin(0.5 * yaw)
        pose.pose.orientation.w = math.cos(0.5 * yaw)
        path.poses.append(pose)
    return path
