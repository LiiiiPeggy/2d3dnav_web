#!/usr/bin/python3
# import argparse
import os
import sys
import pathlib
import time
import pickle
from typing import Optional
import numpy as np
import importlib

  
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


try:
    from .tomogram import Tomogram as CudaTomogram
    _TOMOGRAM_IMPORT_ERROR = None
except (ImportError, ModuleNotFoundError) as error:
    CudaTomogram = None
    _TOMOGRAM_IMPORT_ERROR = error
from .tomogram_cpu import Tomogram as CpuTomogram
from .pcd_io import load_xyz

# equal sys.path.append("../")
# parent_dir = pathlib.Path(__file__).resolve().parent
# if str(parent_dir) not in sys.path:
#    sys.path.append(str(parent_dir))

from .config import POINT_FIELDS_XYZI, GRID_POINTS_XYZI
from .config import Config
from .config import scene

# rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'


class Tomography(Node):
    def __init__(self, cfg: Config):
        super().__init__('pointcloud_tomography')

        self.declare_parameter("rsg_root", "")
        rsg_root_param = self.get_parameter("rsg_root")
        self.declare_parameter("scene_name", "plaza")
        scene_name_param = self.get_parameter("scene_name")
        self.declare_parameter("pcd_path", "")
        self.declare_parameter("tomogram_path", "")
        self.declare_parameter("resolution", 0.10)
        self.declare_parameter("ground_height", 0.0)
        self.declare_parameter("slice_height", 0.5)
        self.declare_parameter("compute_backend", "auto")
        self.declare_parameter("benchmark_repeats", 1)
        self.declare_parameter("traversability.kernel_size", 7)
        self.declare_parameter("traversability.minimum_clearance", 0.50)
        self.declare_parameter("traversability.free_clearance", 0.65)
        self.declare_parameter("traversability.maximum_slope", 0.36)
        self.declare_parameter("traversability.maximum_step", 0.17)
        self.declare_parameter("traversability.standable_ratio", 0.20)
        self.declare_parameter("traversability.safe_margin", 0.40)
        self.declare_parameter("traversability.inflation", 0.20)
        
        self.rsg_root = rsg_root_param.get_parameter_value().string_value
        self.scene_name = scene_name_param.get_parameter_value().string_value.lower()
        
        # scene_module_name = f"config.scene_{self.scene_name}"
        # Use relative import for dynamic loading or absolute package path
        # Assuming 'tomography' is the package name
        scene_module_name = f"tomography.config.scene_{self.scene_name}"
        scene_class_name = f"Scene{self.scene_name.capitalize()}"

        try:
            scene_module = importlib.import_module(scene_module_name)
        except ModuleNotFoundError:
             # Fallback to relative import if running as script or different structure
             scene_module = importlib.import_module(f".config.scene_{self.scene_name}", package="tomography")

        scene_cfg: scene.Scene = getattr(scene_module, scene_class_name)()
        scene_cfg.map.resolution = float(self.get_parameter("resolution").value)
        scene_cfg.map.ground_h = float(self.get_parameter("ground_height").value)
        scene_cfg.map.slice_dh = float(self.get_parameter("slice_height").value)
        scene_cfg.trav.kernel_size = int(
            self.get_parameter("traversability.kernel_size").value)
        scene_cfg.trav.interval_min = float(
            self.get_parameter("traversability.minimum_clearance").value)
        scene_cfg.trav.interval_free = float(
            self.get_parameter("traversability.free_clearance").value)
        scene_cfg.trav.slope_max = float(
            self.get_parameter("traversability.maximum_slope").value)
        scene_cfg.trav.step_max = float(
            self.get_parameter("traversability.maximum_step").value)
        scene_cfg.trav.standable_ratio = float(
            self.get_parameter("traversability.standable_ratio").value)
        scene_cfg.trav.safe_margin = float(
            self.get_parameter("traversability.safe_margin").value)
        scene_cfg.trav.inflation = float(
            self.get_parameter("traversability.inflation").value)
        if scene_cfg.trav.kernel_size < 1 or scene_cfg.trav.kernel_size % 2 == 0:
            raise ValueError('traversability.kernel_size must be a positive odd integer')
        if scene_cfg.trav.interval_min <= 0.0:
            raise ValueError('traversability.minimum_clearance must be positive')
        if scene_cfg.trav.interval_free < scene_cfg.trav.interval_min:
            raise ValueError(
                'traversability.free_clearance must be at least minimum_clearance')
        if not 0.0 < scene_cfg.trav.standable_ratio <= 1.0:
            raise ValueError('traversability.standable_ratio must be in (0, 1]')

        self.cfg = cfg

        self.qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        configured_pcd = str(self.get_parameter("pcd_path").value)
        self.pcd_path = os.path.realpath(os.path.expanduser(configured_pcd)) \
            if configured_pcd else os.path.join(
                self.rsg_root, "pcd", scene_cfg.pcd.file_name)
        configured_tomogram = str(self.get_parameter("tomogram_path").value)
        if configured_tomogram:
            self.tomogram_path = os.path.realpath(os.path.expanduser(configured_tomogram))
        else:
            map_file = os.path.splitext(os.path.basename(self.pcd_path))[0] + '.pickle'
            self.tomogram_path = os.path.join(
                self.rsg_root, cfg.map.export_dir.lstrip('/'), map_file)
        self.benchmark_repeats = int(self.get_parameter("benchmark_repeats").value)
        if not os.path.isfile(self.pcd_path):
            raise FileNotFoundError(f"PCD file does not exist: {self.pcd_path}")
        output_dir = os.path.dirname(self.tomogram_path)
        if not output_dir or not os.path.isdir(output_dir):
            raise ValueError(f"Tomogram output directory does not exist: {output_dir}")
        if self.benchmark_repeats < 1:
            raise ValueError("benchmark_repeats must be at least 1")
        self.resolution = scene_cfg.map.resolution
        self.ground_h = scene_cfg.map.ground_h
        self.slice_dh = scene_cfg.map.slice_dh

        compute_backend = str(
            self.get_parameter("compute_backend").value).strip().lower()
        if compute_backend not in ('auto', 'cuda', 'cpu'):
            raise ValueError('compute_backend must be auto, cuda, or cpu')
        cuda_error = _TOMOGRAM_IMPORT_ERROR
        cuda_available = CudaTomogram is not None
        if cuda_available:
            try:
                import cupy as cp
                if cp.cuda.runtime.getDeviceCount() < 1:
                    raise RuntimeError('no CUDA device is available')
            except Exception as error:  # CUDA runtime errors vary by driver
                cuda_available = False
                cuda_error = error
        if compute_backend == 'cuda' and not cuda_available:
            raise RuntimeError(
                'CUDA tomography requested but unavailable: '
                f'{cuda_error}')
        if compute_backend == 'cpu' or not cuda_available:
            self.compute_backend = 'cpu'
            tomogram_type = CpuTomogram
            if compute_backend == 'auto' and cuda_error is not None:
                self.get_logger().warning(
                    f'CUDA/CuPy unavailable; using offline CPU fallback: '
                    f'{cuda_error}')
        else:
            self.compute_backend = 'cuda'
            tomogram_type = CudaTomogram

        self.center = np.zeros(2, dtype=np.float32)
        self.tomogram = tomogram_type(scene_cfg)
        self.get_logger().info(
            f'Tomography compute backend: {self.compute_backend}')

        self.get_logger().info(f"PCD input: {self.pcd_path}")
        points = self.loadPCD()

        # Process
        self.process(points)

    def initROS(self):
        self.map_frame = self.cfg.ros.map_frame
        pointcloud_topic = self.cfg.ros.pointcloud_topic
        layer_G_topic = self.cfg.ros.layer_G_topic
        layer_C_topic = self.cfg.ros.layer_C_topic
        tomogram_topic = self.cfg.ros.tomogram_topic


        self.pointcloud_pub = self.create_publisher(PointCloud2, pointcloud_topic, self.qos)

        self.layer_G_pub_list = []
        self.layer_C_pub_list = []

        for i in range(self.n_slice):
            layer_G_pub = self.create_publisher(PointCloud2, layer_G_topic + str(i), self.qos)
            self.layer_G_pub_list.append(layer_G_pub)
            layer_C_pub = self.create_publisher(PointCloud2, layer_C_topic + str(i), self.qos)
            self.layer_C_pub_list.append(layer_C_pub)

        self.tomogram_pub = self.create_publisher(PointCloud2, tomogram_topic, self.qos)

    def loadPCD(self):
        points = load_xyz(self.pcd_path)

        self.get_logger().info(f"PCD points: {points.shape[0]}")
        points = points[np.isfinite(points).all(axis=1)]
        if points.shape[0] == 0:
            raise ValueError("PCD contains no finite XYZ points")
        
        self.points_max = np.max(points, axis=0)
        self.points_min = np.min(points, axis=0)           
        self.points_min[-1] = self.ground_h
        self.map_dim_x = int(np.ceil((self.points_max[0] - self.points_min[0]) / self.resolution)) + 4
        self.map_dim_y = int(np.ceil((self.points_max[1] - self.points_min[1]) / self.resolution)) + 4
        n_slice_init = max(
            1, int(np.ceil((self.points_max[2] - self.points_min[2]) / self.slice_dh)))
        self.center = (self.points_max[:2] + self.points_min[:2]) / 2
        self.slice_h0 = self.points_min[-1] + self.slice_dh
        self.tomogram.initMappingEnv(self.center, self.map_dim_x, self.map_dim_y, n_slice_init, self.slice_h0)

        self.get_logger().info(f"Map center: [{self.center[0]:.2f}, {self.center[1]:.2f}]", )
        self.get_logger().info(f"Dim_x: {self.map_dim_x}")
        self.get_logger().info(f"Dim_y: {self.map_dim_y}")
        self.get_logger().info(f"Num slices init: {n_slice_init}")

        self.VISPROTO_I, self.VISPROTO_P = \
            GRID_POINTS_XYZI(self.resolution, self.map_dim_x, self.map_dim_y)

        return points
        
    def process(self, points):        
        t_map = 0.0
        t_trav = 0.0
        t_simp = 0.0
        t_all = 0.0
        n_repeat = self.benchmark_repeats

        """ 
        GPU time benchmark, where CUDA events are synchronized for correct time measurement.
        The function is repeatedly run for n_repeat times to calculate the average processing time of each modules.
        The time of the first warm-up run is excluded to reduce timing fluctuation and exclude the overhead in initial invocations.
        See https://docs.cupy.dev/en/stable/user_guide/performance.html for more details
        """
        for i in range(n_repeat + 1):
            t_start = time.time()
            layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c, t_gpu = self.tomogram.point2map(points)

            if i > 0:
                t_map += t_gpu['t_map']
                t_trav += t_gpu['t_trav']
                t_simp += t_gpu['t_simp']
                t_all += (time.time() - t_start) * 1e3

        self.get_logger().info(f"Num slices simp: {layers_g.shape[0]}")
        self.get_logger().info(f"Num repeats (for benchmarking only): {n_repeat}")
        self.get_logger().info(f" -- avg t_map  (ms): {t_map / n_repeat}")
        self.get_logger().info(f" -- avg t_trav (ms): {t_trav / n_repeat}")
        self.get_logger().info(f" -- avg t_simp (ms): {t_simp / n_repeat}")
        self.get_logger().info(f" -- avg t_all  (ms): {t_all / n_repeat}")

        self.n_slice = layers_g.shape[0]
        
        self.exportTomogram(np.stack((layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c)))

        self.initROS()
        self.publishPoints(points)
        self.publishLayers(self.layer_G_pub_list, layers_g, layers_t)
        self.publishLayers(self.layer_C_pub_list, layers_c, None)
        self.publishTomogram(layers_g, layers_t)

    def exportTomogram(self, tomogram):
        data_dict = {
            'data': tomogram.astype(np.float16),
            'resolution': self.resolution,
            'center': self.center,
            'slice_h0': self.slice_h0,
            'slice_dh': self.slice_dh,
        }
        with open(self.tomogram_path, 'wb') as handle:
            pickle.dump(data_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)

        self.get_logger().info(f"Tomogram exported: {self.tomogram_path}")

    def publishPoints(self, points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()

        header.frame_id = self.map_frame

        point_msg = pc2.create_cloud_xyz32(header, points)
        self.pointcloud_pub.publish(point_msg)

    def publishLayers(self, pub_list, layers, color=None):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()

        header.frame_id = self.map_frame

        layer_points = self.VISPROTO_P.copy()
        layer_points[:, :2] += self.center

        for i in range(layers.shape[0]):
            layer_points[:, 2] = layers[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            if color is not None:
                layer_points[:, 3] = color[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            else:
                layer_points[:, 3] = 1.0
        
            valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
            points_msg = pc2.create_cloud(header, POINT_FIELDS_XYZI, valid_points)
            pub_list[i].publish(points_msg) 

    def publishTomogram(self, layers_g, layers_t):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame

        # Extract grid points for all layers at once
        idx_x = self.VISPROTO_I[:, 0]
        idx_y = self.VISPROTO_I[:, 1]
        
        flat_g = layers_g[:, idx_x, idx_y].copy()
        flat_t = layers_t[:, idx_x, idx_y].copy()
        
        n_slice = flat_g.shape[0]

        # Apply tomogram visibility logic
        for i in range(n_slice - 1):
            diff = flat_g[i + 1] - flat_g[i]
            mask_h = diff < self.slice_dh
            flat_g[i, mask_h] = np.nan
            flat_t[i + 1, mask_h] = np.minimum(flat_t[i, mask_h], flat_t[i + 1, mask_h])

        # Flatten arrays to list of points
        g_all = flat_g.flatten()
        t_all = flat_t.flatten()
        
        # Create corresponding XY coordinates
        base_xy = self.VISPROTO_P[:, :2] + self.center
        xy_all = np.tile(base_xy, (n_slice, 1))

        # Filter valid points
        valid_mask = ~np.isnan(g_all)
        
        if np.any(valid_mask):
            global_points = np.column_stack((
                xy_all[valid_mask],
                g_all[valid_mask],
                t_all[valid_mask]
            )).astype(np.float32)
        else:
            global_points = np.empty((0, 4), dtype=np.float32)

        points_msg = pc2.create_cloud(header, POINT_FIELDS_XYZI, global_points)
        self.tomogram_pub.publish(points_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = Tomography(Config())
        rclpy.spin(node)
    except (FileNotFoundError, ValueError, RuntimeError) as error:
        rclpy.logging.get_logger('pointcloud_tomography').fatal(str(error))
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
