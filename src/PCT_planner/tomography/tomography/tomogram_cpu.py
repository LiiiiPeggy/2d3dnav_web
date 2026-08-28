"""NumPy/SciPy fallback for offline point-cloud tomography.

The CUDA implementation remains the preferred backend.  This version mirrors
its output format and traversal rules so maps can still be prepared on a
development computer without a working NVIDIA/CuPy runtime.
"""

import time

import numpy as np
from scipy import ndimage


class Tomogram:
    """CPU implementation of the CUDA ``Tomogram`` interface."""

    def __init__(self, cfg):
        self.resolution = cfg.map.resolution
        self.slice_dh = cfg.map.slice_dh
        self.half_trav_k_size = int(cfg.trav.kernel_size / 2)
        self.interval_min = cfg.trav.interval_min
        self.interval_free = cfg.trav.interval_free
        self.step_stand = 1.2 * self.resolution * np.tan(cfg.trav.slope_max)
        self.step_cross = cfg.trav.step_max
        kernel_width = 2 * self.half_trav_k_size + 1
        self.standable_th = int(
            cfg.trav.standable_ratio * kernel_width ** 2) - 1
        self.cost_barrier = float(cfg.trav.cost_barrier)
        self.safe_margin = cfg.trav.safe_margin
        self.inflation = cfg.trav.inflation
        self.half_inf_k_size = int(
            (self.safe_margin + self.inflation) / self.resolution)

    def initMappingEnv(
            self, center, map_dim_x, map_dim_y, n_slice_init, slice_h0):
        self.center = np.asarray(center, dtype=np.float32)
        self.map_dim_x = int(map_dim_x)
        self.map_dim_y = int(map_dim_y)
        self.n_slice_init = int(n_slice_init)
        self.slice_h0 = float(slice_h0)
        width = 2 * self.half_inf_k_size + 1
        self.inf_table = np.zeros((width, width), dtype=np.float32)
        for i in range(width):
            for j in range(width):
                distance = np.hypot(
                    self.resolution * (i - self.half_inf_k_size),
                    self.resolution * (j - self.half_inf_k_size),
                )
                self.inf_table[i, j] = np.clip(
                    1.0 - (distance - self.inflation) /
                    (self.safe_margin + self.resolution),
                    0.0,
                    1.0,
                )

    @staticmethod
    def _round_away_from_zero(values):
        """Match C/CUDA round() instead of NumPy's ties-to-even rint()."""
        return np.copysign(
            np.floor(np.abs(values) + 0.5), values).astype(np.int32)

    def _rasterize(self, points):
        layer_size = self.map_dim_x * self.map_dim_y
        layers_g = np.full(
            (self.n_slice_init, layer_size), -1e6, dtype=np.float32)
        layers_c = np.full_like(layers_g, 1e6)

        index_x = self._round_away_from_zero(
            (points[:, 0] - self.center[0]) / self.resolution)
        index_y = self._round_away_from_zero(
            (points[:, 1] - self.center[1]) / self.resolution)
        index_x += self.map_dim_x // 2
        index_y += self.map_dim_y // 2
        valid = (
            (index_x >= 0) & (index_x < self.map_dim_x) &
            (index_y >= 0) & (index_y < self.map_dim_y)
        )
        flat_index = index_x[valid] * self.map_dim_y + index_y[valid]
        height = points[valid, 2]

        for slice_index in range(self.n_slice_init):
            slice_height = self.slice_h0 + slice_index * self.slice_dh
            is_ground = height <= slice_height
            np.maximum.at(
                layers_g[slice_index], flat_index[is_ground],
                height[is_ground])
            np.minimum.at(
                layers_c[slice_index], flat_index[~is_ground],
                height[~is_ground])
        shape = (self.n_slice_init, self.map_dim_x, self.map_dim_y)
        return layers_g.reshape(shape), layers_c.reshape(shape)

    def _traversability(self, layers_g, layers_c):
        grad_mag_sq = np.zeros_like(layers_g)
        grad_mag_max = np.zeros_like(layers_g)
        diff_x_sq = np.maximum(
            (layers_g[:, 1:-1, :] - layers_g[:, :-2, :]) ** 2,
            (layers_g[:, 1:-1, :] - layers_g[:, 2:, :]) ** 2,
        )
        diff_y_sq = np.maximum(
            (layers_g[:, :, 1:-1] - layers_g[:, :, :-2]) ** 2,
            (layers_g[:, :, 1:-1] - layers_g[:, :, 2:]) ** 2,
        )
        grad_mag_sq[:, 1:-1, 1:-1] = (
            diff_x_sq[:, :, 1:-1] + diff_y_sq[:, 1:-1, :])
        grad_mag_max[:, 1:-1, 1:-1] = np.maximum(
            diff_x_sq[:, :, 1:-1], diff_y_sq[:, 1:-1, :])

        interval = layers_c - layers_g
        cost = np.maximum(
            0.0, 20.0 * (self.interval_free - interval)).astype(np.float32)
        insufficient_clearance = interval < self.interval_min
        cost[insufficient_clearance] = self.cost_barrier

        stand_limit = self.step_stand ** 2
        cross_limit = self.step_cross ** 2
        standable = grad_mag_sq <= stand_limit
        direct = standable & ~insufficient_clearance
        cost[direct] += 15.0 * grad_mag_sq[direct] / stand_limit

        neighbor_kernel = np.ones(
            (1, 2 * self.half_trav_k_size + 1,
             2 * self.half_trav_k_size + 1),
            dtype=np.int16,
        )
        neighbor_count = ndimage.convolve(
            (grad_mag_sq < stand_limit).astype(np.int16),
            neighbor_kernel,
            mode='constant',
            cval=0,
        )
        crossing = (
            ~standable & ~insufficient_clearance &
            (grad_mag_max <= cross_limit) &
            (neighbor_count >= self.standable_th)
        )
        cost[crossing] += (
            20.0 * grad_mag_max[crossing] / cross_limit)
        rejected = ~insufficient_clearance & ~direct & ~crossing
        cost[rejected] = self.cost_barrier
        return cost

    def _inflate(self, cost):
        inflated = np.zeros_like(cost)
        half = self.half_inf_k_size
        for dx in range(-half, half + 1):
            source_x = slice(max(0, -dx), min(self.map_dim_x, self.map_dim_x - dx))
            target_x = slice(max(0, dx), min(self.map_dim_x, self.map_dim_x + dx))
            for dy in range(-half, half + 1):
                weight = self.inf_table[dx + half, dy + half]
                if weight <= 0.0:
                    continue
                source_y = slice(
                    max(0, -dy), min(self.map_dim_y, self.map_dim_y - dy))
                target_y = slice(
                    max(0, dy), min(self.map_dim_y, self.map_dim_y + dy))
                np.maximum(
                    inflated[:, target_x, target_y],
                    cost[:, source_x, source_y] * weight,
                    out=inflated[:, target_x, target_y],
                )
        return inflated

    def _simplify(self, layers_g, inflated_cost):
        indices = [0]
        if layers_g.shape[0] > 1:
            lower_index, middle_index = 0, 1
            height_difference = layers_g[1:] - layers_g[:-1]
            while middle_index < self.n_slice_init - 2:
                unique = (
                    ((layers_g[middle_index] - layers_g[lower_index] > 0) |
                     (inflated_cost[lower_index] >
                      inflated_cost[middle_index])) &
                    (height_difference[middle_index] > 0) &
                    (inflated_cost[middle_index] < self.cost_barrier)
                )
                if np.any(unique):
                    indices.append(middle_index)
                    lower_index = middle_index
                middle_index += 1
            indices.append(middle_index)
        return indices

    def point2map(self, points):
        points = np.asarray(points, dtype=np.float32)
        points = points[np.isfinite(points).all(axis=1)]

        started = time.monotonic()
        layers_g, layers_c = self._rasterize(points)
        map_finished = time.monotonic()
        cost = self._traversability(layers_g, layers_c)
        inflated_cost = self._inflate(cost)
        traversal_finished = time.monotonic()
        indices = self._simplify(layers_g, inflated_cost)

        selected_cost = inflated_cost[indices]
        trav_grad_x = np.zeros_like(selected_cost)
        trav_grad_y = np.zeros_like(selected_cost)
        trav_grad_x[:, 1:-1, :] = (
            selected_cost[:, 2:, :] - selected_cost[:, :-2, :])
        trav_grad_y[:, :, 1:-1] = (
            selected_cost[:, :, 2:] - selected_cost[:, :, :-2])
        selected_ground = layers_g[indices].copy()
        selected_ceiling = layers_c[indices].copy()
        selected_ground[selected_ground <= -1e6] = np.nan
        selected_ceiling[selected_ceiling >= 1e6] = np.nan
        finished = time.monotonic()

        return (
            selected_cost,
            trav_grad_x,
            trav_grad_y,
            selected_ground,
            selected_ceiling,
            {
                't_map': (map_finished - started) * 1000.0,
                't_trav': (traversal_finished - map_finished) * 1000.0,
                't_simp': (finished - traversal_finished) * 1000.0,
            },
        )
