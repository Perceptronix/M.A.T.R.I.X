"""
World-frame 2D occupancy costmap, built incrementally from /lidar/points.

Updated as the vehicle moves and looks around. Fixed bounds sized to cover
worlds/outdoor.world.sdf's obstacle course plus margin -- this is a
prototype-scale, single-course costmap, not a general rolling window.

Cells decay after `max_age` seconds unless re-observed. This was added
after measuring /odom drift smear the same physical obstacle across a
growing, distorted region over a run: /odom is pure dead-reckoning with no
correction (see goal_navigator.py's docstring for the original finding),
so successive scans of one real obstacle, transformed into world frame
through a slowly-drifting pose estimate, land at slightly different world
coordinates each time and never stop accumulating in a purely persistent
map -- confirmed directly (odom vs `gz model -p` differed by ~0.8 m after
a 15 s run, and the mapped "obstacle" region kept growing the entire time,
137+ cells for 4 small objects). Decay keeps the map representing only
recent, low-drift observations, self-correcting the same way a real rolling
local costmap does.

Occupied cells get a small inflation, just enough to cover grid
discretization (a raw hit can land up to half a cell away from the true
obstacle surface) -- NOT the vehicle's own size or safety margin. That
belongs entirely to collision_checker.py's rectangular footprint sampling,
which correctly handles orientation-dependent corner cases a simple
radius inflation can't. Applying both was tried first and measured to
double up the margin (footprint half-size + inflation radius stacked),
making even the actual, reachable goal position report as colliding.

Terrain roughness (for the Rubicon-terrain navigation phase) is tracked the
same sensor-only way: LiDAR returns near ground level (not just the
obstacle-height band) are accumulated per cell as a running mean/variance
of height (Welford's algorithm). High local height variance means uneven,
rocky ground; this is a property of the ground itself, not obstacle
occupancy, and is intentionally kept separate. Crucially, this is built
entirely from the robot's own sensor returns -- it never reads the Fuel
model's heightmap or collision geometry, matching the requirement that
planning use only what the vehicle can perceive.
"""
import math

import numpy as np


class Costmap2D:

    def __init__(self, min_x=-3.0, min_y=-4.0, max_x=12.0, max_y=9.0,
                 resolution=0.2, inflation_radius=0.15, max_age=4.0):
        self.min_x = min_x
        self.min_y = min_y
        self.resolution = resolution
        self.max_age = max_age
        self.width = int(math.ceil((max_x - min_x) / resolution))
        self.height = int(math.ceil((max_y - min_y) / resolution))
        self.occupied = np.zeros((self.width, self.height), dtype=bool)
        self.inflated = np.zeros((self.width, self.height), dtype=bool)
        self.last_seen = np.full((self.width, self.height), -1.0e9, dtype=np.float64)
        self.inflation_cells = max(1, int(math.ceil(inflation_radius / resolution)))
        r = self.inflation_cells
        self._disk = [(dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1)
                      if dx * dx + dy * dy <= r * r]

        # Terrain roughness: Welford running mean/variance of ground-level
        # LiDAR return heights per cell, decayed the same way occupancy is.
        self.terrain_n = np.zeros((self.width, self.height), dtype=np.int32)
        self.terrain_mean = np.zeros((self.width, self.height), dtype=np.float64)
        self.terrain_m2 = np.zeros((self.width, self.height), dtype=np.float64)
        self.terrain_last_seen = np.full((self.width, self.height), -1.0e9, dtype=np.float64)

    def world_to_grid(self, x, y):
        return (int((x - self.min_x) / self.resolution),
                int((y - self.min_y) / self.resolution))

    def grid_to_world(self, gx, gy):
        return (self.min_x + (gx + 0.5) * self.resolution,
                self.min_y + (gy + 0.5) * self.resolution)

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def mark_occupied_world(self, x, y, now=0.0):
        gx, gy = self.world_to_grid(x, y)
        if not self.in_bounds(gx, gy):
            return
        self.last_seen[gx, gy] = now
        if self.occupied[gx, gy]:
            return
        self.occupied[gx, gy] = True
        for dx, dy in self._disk:
            nx, ny = gx + dx, gy + dy
            if self.in_bounds(nx, ny):
                self.inflated[nx, ny] = True

    def decay(self, now):
        """
        Drop cells not re-observed within max_age seconds.

        Rebuilds the inflated grid from what remains. Call periodically
        (not every tick -- rebuilding inflation is
        O(remaining_occupied_cells)).
        """
        stale = self.occupied & (self.last_seen < now - self.max_age)
        if np.any(stale):
            self.occupied &= ~stale
            self.inflated.fill(False)
            for gx, gy in np.argwhere(self.occupied):
                for dx, dy in self._disk:
                    nx, ny = gx + dx, gy + dy
                    if self.in_bounds(nx, ny):
                        self.inflated[nx, ny] = True

        terrain_stale = (self.terrain_n > 0) & (self.terrain_last_seen < now - self.max_age)
        if np.any(terrain_stale):
            self.terrain_n[terrain_stale] = 0
            self.terrain_mean[terrain_stale] = 0.0
            self.terrain_m2[terrain_stale] = 0.0

    def mark_ground_sample_world(self, x, y, z, now=0.0):
        """
        Feed one ground-level LiDAR return into the terrain-roughness
        estimate for its cell (Welford's online mean/variance).
        """
        gx, gy = self.world_to_grid(x, y)
        if not self.in_bounds(gx, gy):
            return
        self.terrain_last_seen[gx, gy] = now
        n = self.terrain_n[gx, gy] + 1
        delta = z - self.terrain_mean[gx, gy]
        mean = self.terrain_mean[gx, gy] + delta / n
        delta2 = z - mean
        self.terrain_m2[gx, gy] += delta * delta2
        self.terrain_n[gx, gy] = n
        self.terrain_mean[gx, gy] = mean

    # Height-variance (m^2) at/above this is treated as maximally rough.
    # Chosen from measured Rubicon rock terrain: a 0.2 m cell straddling a
    # rock edge easily sees 10-20 cm of return-height spread (variance
    # ~0.01-0.04); flat ground sees well under 1 cm (variance ~0.0001).
    TERRAIN_VARIANCE_ROUGH = 0.02

    def terrain_cost_world(self, x, y):
        """
        Normalized [0, 1] terrain roughness at a point, from the robot's
        own accumulated ground-level LiDAR returns only. 0 if unobserved
        (unexplored terrain is not penalized -- only measured roughness
        is).
        """
        gx, gy = self.world_to_grid(x, y)
        if not self.in_bounds(gx, gy) or self.terrain_n[gx, gy] < 3:
            return 0.0
        variance = self.terrain_m2[gx, gy] / self.terrain_n[gx, gy]
        return min(1.0, variance / self.TERRAIN_VARIANCE_ROUGH)

    def clear_radius_world(self, x, y, radius):
        """
        Evict occupied cells within `radius` of a world point.

        Rebuilds inflation. Used defensively before planning: if the
        vehicle's own current position reads as colliding, that is a map
        error, not a real collision -- Gazebo's physics would already
        have prevented it -- so clear the immediate surroundings (most
        likely a self-hit or drift-smeared stale cell) rather than let
        the vehicle deadlock against its own map forever.
        """
        gx, gy = self.world_to_grid(x, y)
        cells = int(math.ceil(radius / self.resolution))
        cleared = False
        for dx in range(-cells, cells + 1):
            for dy in range(-cells, cells + 1):
                if dx * dx + dy * dy > cells * cells:
                    continue
                nx, ny = gx + dx, gy + dy
                if self.in_bounds(nx, ny) and self.occupied[nx, ny]:
                    self.occupied[nx, ny] = False
                    cleared = True
        if cleared:
            self.inflated.fill(False)
            for ox, oy in np.argwhere(self.occupied):
                for dx, dy in self._disk:
                    nx, ny = ox + dx, oy + dy
                    if self.in_bounds(nx, ny):
                        self.inflated[nx, ny] = True
        return cleared

    def is_free_world(self, x, y):
        gx, gy = self.world_to_grid(x, y)
        if not self.in_bounds(gx, gy):
            return False
        return not self.inflated[gx, gy]

    def is_free_grid(self, gx, gy):
        if not self.in_bounds(gx, gy):
            return False
        return not self.inflated[gx, gy]

    def occupied_count(self):
        return int(np.count_nonzero(self.occupied))

    def nearest_obstacle_distance(self, x, y, search_radius=3.0):
        """
        Brute-force nearest occupied-cell distance from a world point.

        Only used for scoring (DWA clearance term), not hot-path collision
        checking, so an O(disk) scan at limited radius is fine.
        """
        gx, gy = self.world_to_grid(x, y)
        cells = int(search_radius / self.resolution)
        best = None
        for dx in range(-cells, cells + 1):
            for dy in range(-cells, cells + 1):
                nx, ny = gx + dx, gy + dy
                if not self.in_bounds(nx, ny) or not self.occupied[nx, ny]:
                    continue
                d = math.hypot(dx, dy) * self.resolution
                if best is None or d < best:
                    best = d
        return best if best is not None else search_radius
