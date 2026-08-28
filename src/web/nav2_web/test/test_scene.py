import math
import struct

import pytest
from sensor_msgs.msg import PointCloud2, PointField

from nav2_web.scene import pointcloud_xyz


def _cloud(points):
    message = PointCloud2()
    message.height = 1
    message.width = len(points)
    message.is_bigendian = False
    message.point_step = 16
    message.row_step = message.width * message.point_step
    message.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(
            name='intensity', offset=12,
            datatype=PointField.FLOAT32, count=1),
    ]
    message.data = b''.join(
        struct.pack('<ffff', x, y, z, intensity)
        for x, y, z, intensity in points
    )
    return message


def test_pointcloud_xyz_extracts_finite_xyz_and_height_range():
    message = _cloud([
        (1.0, 2.0, -0.5, 10.0),
        (math.nan, 8.0, 9.0, 20.0),
        (3.0, 4.0, 1.5, 30.0),
    ])

    xyz, min_z, max_z = pointcloud_xyz(message, point_limit=10)

    assert list(xyz) == pytest.approx([1.0, 2.0, -0.5, 3.0, 4.0, 1.5])
    assert min_z == pytest.approx(-0.5)
    assert max_z == pytest.approx(1.5)


def test_pointcloud_xyz_uniformly_limits_phone_payload():
    message = _cloud([
        (0.0, 0.0, 0.0, 1.0),
        (1.0, 0.0, 0.0, 1.0),
        (2.0, 0.0, 0.0, 1.0),
        (3.0, 0.0, 0.0, 1.0),
    ])

    xyz, _min_z, _max_z = pointcloud_xyz(message, point_limit=2)

    assert list(xyz) == pytest.approx([
        0.0, 0.0, 0.0,
        2.0, 0.0, 0.0,
    ])
