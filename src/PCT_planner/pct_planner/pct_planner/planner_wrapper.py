import os
import sys
import pathlib
import pickle
import numpy as np

from .utils import *
from .config import Config

# sys.path.append('../')
parent_dir = pathlib.Path(__file__).resolve().parent
if str(parent_dir) not in sys.path:
    sys.path.append(str(parent_dir))

from .lib import a_star, ele_planner, traj_opt

# rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'


class TomogramPlanner(object):
    def __init__(self, cfg: Config, rsg_root: str = None, body_height: float = 0.4):
        self.cfg = cfg

        self.use_quintic = self.cfg.planner.use_quintic
        self.max_heading_rate = self.cfg.planner.max_heading_rate
        self.body_height = float(body_height)

        self.tomo_dir = (
            os.path.join(rsg_root, self.cfg.wrapper.tomo_dir.lstrip('/'))
            if rsg_root else None
        )

        self.resolution = None
        self.center = None
        self.n_slice = None
        self.slice_h0 = None
        self.slice_dh = None
        self.map_dim = []
        self.offset = None

        self.start_idx = np.zeros(3, dtype=np.int32)
        self.end_idx = np.zeros(3, dtype=np.int32)

    def loadTomogram(self, tomo_file):
        if os.path.isabs(tomo_file):
            tomo_path = tomo_file
        elif self.tomo_dir:
            tomo_path = os.path.join(self.tomo_dir, tomo_file)
        else:
            raise ValueError("tomogram path must be absolute when rsg_root is unset")
        if not tomo_path.endswith('.pickle'):
            tomo_path += '.pickle'
        if not os.path.isfile(tomo_path):
            raise FileNotFoundError(f"tomogram does not exist: {tomo_path}")

        with open(tomo_path, 'rb') as handle:
            data_dict = pickle.load(handle)

            tomogram = np.asarray(data_dict['data'], dtype=np.float32)

            self.resolution = float(data_dict['resolution'])
            self.center = np.asarray(data_dict['center'], dtype=np.double)
            self.n_slice = tomogram.shape[1]
            self.slice_h0 = float(data_dict['slice_h0'])
            self.slice_dh = float(data_dict['slice_dh'])
            self.map_dim = [tomogram.shape[2], tomogram.shape[3]]
            self.offset = np.array([int(self.map_dim[0] / 2), int(self.map_dim[1] / 2)], dtype=np.int32)

        trav = tomogram[0]
        trav_gx = tomogram[1]
        trav_gy = tomogram[2]
        elev_g = tomogram[3]
        elev_g = np.nan_to_num(elev_g, nan=-100)
        elev_c = tomogram[4]
        elev_c = np.nan_to_num(elev_c, nan=1e6)

        self.trav = trav
        self.elev_g = elev_g
        self.initPlanner(trav, trav_gx, trav_gy, elev_g, elev_c)
        
    def initPlanner(self, trav, trav_gx, trav_gy, elev_g, elev_c):
        diff_t = trav[1:] - trav[:-1]
        diff_g = np.abs(elev_g[1:] - elev_g[:-1])

        gateway_up = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t < -8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[1:]))
        gateway_up[:-1] = np.logical_and(mask_t, mask_g)

        gateway_dn = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t > 8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[:-1]))
        gateway_dn[1:] = np.logical_and(mask_t, mask_g)
        
        gateway = np.zeros_like(trav, dtype=np.int32)
        gateway[gateway_up] = 2
        gateway[gateway_dn] = -2

        self.planner = ele_planner.OfflineElePlanner(
            max_heading_rate=self.max_heading_rate, use_quintic=self.use_quintic
        )
        
        self.planner.init_map(
            20, 15, self.resolution, self.n_slice, 0.2,
            trav.reshape(-1, trav.shape[-1]).astype(np.double),
            elev_g.reshape(-1, elev_g.shape[-1]).astype(np.double),
            elev_c.reshape(-1, elev_c.shape[-1]).astype(np.double),
            gateway.reshape(-1, gateway.shape[-1]),
            trav_gy.reshape(-1, trav_gy.shape[-1]).astype(np.double),
            -trav_gx.reshape(-1, trav_gx.shape[-1]).astype(np.double)
        )
        self.planner.set_reference_height(self.body_height)

    def plan(self, start_pos, end_pos, start_ground_z=None, end_ground_z=None):
        self.start_idx[1:] = self.pos2idx(start_pos)
        self.end_idx[1:] = self.pos2idx(end_pos)
        self.start_idx[0] = self.selectLayer(self.start_idx, start_ground_z)
        self.end_idx[0] = self.selectLayer(self.end_idx, end_ground_z)

        if not self.planner.plan(self.start_idx, self.end_idx, True):
            return None
        path_finder: a_star.Astar = self.planner.get_path_finder()
        path = path_finder.get_result_matrix()
        if len(path) == 0:
            return None

        optimizer: traj_opt.GPMPOptimizer = (
            self.planner.get_trajectory_optimizer()
            if not self.use_quintic
            else self.planner.get_trajectory_optimizer_wnoj()
        )

        traj_raw = optimizer.get_result_matrix()
        layers = optimizer.get_layers()
        heights = optimizer.get_heights()

        traj = np.concatenate([traj_raw, layers.reshape(-1, 1)], axis=-1)
        y_idx = (traj.shape[-1] - 1) // 2
        traj_3d = np.stack([traj[:, 0], traj[:, y_idx], heights / self.resolution], axis=1)
        traj_3d = transTrajGrid2Map(self.map_dim, self.center, self.resolution, traj_3d)

        return traj_3d
    
    def pos2idx(self, pos):
        pos = np.asarray(pos, dtype=np.float64)[:2] - self.center
        idx = np.round(pos / self.resolution).astype(np.int32) + self.offset
        idx = np.array([idx[1], idx[0]], dtype=np.float32)
        return idx

    def selectLayer(self, full_idx, desired_ground_z=None):
        x_idx = int(full_idx[2])
        y_idx = int(full_idx[1])
        if (x_idx < 0 or x_idx >= self.map_dim[0] or
                y_idx < 0 or y_idx >= self.map_dim[1]):
            raise ValueError(
                f"goal is outside tomogram grid: x_index={x_idx}, y_index={y_idx}")

        heights = self.elev_g[:, x_idx, y_idx]
        costs = self.trav[:, x_idx, y_idx]
        valid = np.isfinite(heights) & (heights > -99.0) & (costs <= 20.0)
        candidates = np.flatnonzero(valid)
        if candidates.size == 0:
            raise ValueError("start or goal lies on a non-traversable tomogram cell")
        if desired_ground_z is None or not np.isfinite(desired_ground_z):
            return int(candidates[0])
        return int(candidates[np.argmin(np.abs(heights[candidates] - desired_ground_z))])

    def traversablePoints(self, maximum_cost=20.0):
        """Return map-frame ground points accepted by the PCT layer selector."""
        valid = (
            np.isfinite(self.elev_g)
            & (self.elev_g > -99.0)
            & np.isfinite(self.trav)
            & (self.trav <= float(maximum_cost))
        )
        layer_idx, x_idx, y_idx = np.nonzero(valid)
        if layer_idx.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        points = np.empty((layer_idx.size, 3), dtype=np.float32)
        points[:, 0] = (
            (x_idx - self.offset[0]) * self.resolution + self.center[0]
        )
        points[:, 1] = (
            (y_idx - self.offset[1]) * self.resolution + self.center[1]
        )
        points[:, 2] = self.elev_g[layer_idx, x_idx, y_idx]
        return points
