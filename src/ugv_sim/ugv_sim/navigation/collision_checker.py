"""
Rectangular-footprint collision checking against a Costmap2D.

The vehicle is never treated as a point: every check samples a grid of
points covering its actual footprint (0.80 x 0.60 m, from
urdf/ugv.urdf.xacro's chassis_length/chassis_width) at the candidate
pose's orientation, plus a safety margin.
"""
import math

# Match the physical chassis (urdf/ugv.urdf.xacro: chassis_length=0.80,
# chassis_width=0.60), plus a safety margin.
VEHICLE_HALF_LENGTH = 0.40 + 0.12
VEHICLE_HALF_WIDTH = 0.30 + 0.12

_SAMPLES = 3  # 3x3 grid over the footprint rectangle


def footprint_points(
        x, y, theta, half_length=VEHICLE_HALF_LENGTH,
        half_width=VEHICLE_HALF_WIDTH, samples=_SAMPLES):
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    pts = []
    for i in range(samples):
        lx = -half_length + (2 * half_length) * i / (samples - 1)
        for j in range(samples):
            ly = -half_width + (2 * half_width) * j / (samples - 1)
            wx = x + lx * cos_t - ly * sin_t
            wy = y + lx * sin_t + ly * cos_t
            pts.append((wx, wy))
    return pts


def is_state_valid(costmap, x, y, theta):
    for wx, wy in footprint_points(x, y, theta):
        if not costmap.is_free_world(wx, wy):
            return False
    return True
