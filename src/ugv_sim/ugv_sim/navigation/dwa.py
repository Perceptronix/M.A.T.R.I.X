"""
Dynamic Window Approach local planner.

Samples feasible (v, omega) pairs within the vehicle's acceleration-limited
dynamic window, forward-simulates each for a short horizon, rejects any
trajectory whose footprint collides with the costmap at any simulated
step, and scores the rest against the Hybrid A* global path (not just the
final goal) plus obstacle clearance, forward-velocity preference, and an
oscillation penalty against recent commands -- this last term specifically
targets the U-turn/wandering failure mode a purely goal-seeking reactive
controller was measured to produce (see goal_navigator.py's docstring for
that history).
"""
import math

from .collision_checker import is_state_valid

MAX_V = 0.40
MIN_V = -0.15          # small reverse allowed, heavily discouraged by scoring
MAX_OMEGA = 0.8
MAX_ACCEL_V = 0.6       # m/s^2
MAX_ACCEL_OMEGA = 2.5   # rad/s^2
V_SAMPLES = 6
OMEGA_SAMPLES = 9
SIM_HORIZON = 1.2       # s
SIM_DT = 0.1

W_PATH_PROGRESS = 1.8
W_PATH_ALIGN = 1.2
W_HEADING = 1.0
W_CLEARANCE = 1.4
W_VELOCITY = 0.6
W_OSCILLATION = 1.0
W_REVERSE = 2.0
# Endpoint terrain roughness (from Costmap2D.terrain_cost_world, sensor-only,
# see costmap.py), same modest weight rationale as Hybrid A*'s
# TERRAIN_COST_WEIGHT: prefer smoother short-horizon trajectories among
# otherwise-similar options, don't refuse to cross real off-road ground.
W_TERRAIN = 1.0


def _wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _simulate(x, y, theta, v, omega, horizon=SIM_HORIZON, dt=SIM_DT):
    traj = [(x, y, theta)]
    for _ in range(int(horizon / dt)):
        theta = _wrap(theta + omega * dt)
        x = x + v * math.cos(theta) * dt
        y = y + v * math.sin(theta) * dt
        traj.append((x, y, theta))
    return traj


def _nearest_path_point(path, x, y, start_idx):
    best_i, best_d = start_idx, None
    # only search forward from start_idx (monotonic progress along path,
    # avoids snapping back to an already-passed point)
    for i in range(start_idx, len(path)):
        px, py, _ = path[i]
        d = math.hypot(px - x, py - y)
        if best_d is None or d < best_d:
            best_d, best_i = d, i
        elif d > best_d + 1.5:
            break  # moving away, stop scanning
    return best_i, best_d if best_d is not None else 0.0


def compute_command(
        pose, v_curr, omega_curr, global_path, path_idx, costmap,
        goal_x, goal_y, recent_omegas, dt_control=0.1):
    """
    Score feasible (v, omega) samples and return the best one.

    pose = (x, y, theta). global_path = list of (x,y,theta). path_idx is
    the last-known nearest-path index (for monotonic tracking). Returns
    (v, omega, new_path_idx, min_clearance, feasible_count) -- v=omega=0
    and feasible_count=0 if no valid trajectory exists (caller should
    treat that as a DWA failure and invoke recovery).
    """
    x, y, theta = pose

    v_lo = max(MIN_V, v_curr - MAX_ACCEL_V * dt_control)
    v_hi = min(MAX_V, v_curr + MAX_ACCEL_V * dt_control)
    om_lo = max(-MAX_OMEGA, omega_curr - MAX_ACCEL_OMEGA * dt_control)
    om_hi = min(MAX_OMEGA, omega_curr + MAX_ACCEL_OMEGA * dt_control)

    lookahead_i, _ = _nearest_path_point(global_path, x, y, path_idx)
    target_i = min(len(global_path) - 1, lookahead_i + 4)
    target_x, target_y, _ = global_path[target_i]
    dist_now = math.hypot(target_x - x, target_y - y)

    best = None
    best_score = None
    feasible_count = 0

    for vi in range(V_SAMPLES):
        v = v_lo + (v_hi - v_lo) * vi / max(1, V_SAMPLES - 1)
        for oi in range(OMEGA_SAMPLES):
            omega = om_lo + (om_hi - om_lo) * oi / max(1, OMEGA_SAMPLES - 1)

            traj = _simulate(x, y, theta, v, omega)
            # Full-resolution collision check every step (safety-critical,
            # but cheap: footprint sampling against precomputed inflated
            # cells). Clearance scoring only needs one representative
            # sample per trajectory, not one per step -- the brute-force
            # disk scan is the expensive part (measured: ~81ms/cycle with
            # it at every step, well over the 100ms control period budget;
            # a single sample at the trajectory endpoint keeps the score
            # meaningful at a fraction of the cost).
            collided = False
            for (tx, ty, tt) in traj[1:]:
                if not is_state_valid(costmap, tx, ty, tt):
                    collided = True
                    break
            if collided:
                continue
            feasible_count += 1

            end_x, end_y, end_theta = traj[-1]
            min_clear = costmap.nearest_obstacle_distance(end_x, end_y, search_radius=1.0)
            _, path_d = _nearest_path_point(global_path, end_x, end_y, lookahead_i)
            progress = dist_now - math.hypot(target_x - end_x, target_y - end_y)
            path_align = -path_d
            heading_to_target = math.atan2(target_y - end_y, target_x - end_x)
            heading_align = -abs(_wrap(heading_to_target - end_theta))
            clearance_score = min(min_clear, 1.2)

            osc_penalty = 0.0
            if recent_omegas:
                sign_flips = sum(
                    1 for a, b in zip(recent_omegas, recent_omegas[1:])
                    if a * b < 0
                )
                if omega * recent_omegas[-1] < -0.05:
                    osc_penalty = 1.0 + sign_flips

            reverse_penalty = W_REVERSE if v < -1e-3 else 0.0
            terrain_cost = costmap.terrain_cost_world(end_x, end_y)

            score = (
                W_PATH_PROGRESS * progress
                + W_PATH_ALIGN * path_align
                + W_HEADING * heading_align
                + W_CLEARANCE * clearance_score
                + W_VELOCITY * v
                - W_OSCILLATION * osc_penalty
                - reverse_penalty
                - W_TERRAIN * terrain_cost
            )

            if best_score is None or score > best_score:
                best_score = score
                best = (v, omega, lookahead_i, min_clear)

    if best is None:
        return 0.0, 0.0, path_idx, 0.0, 0
    v, omega, new_idx, min_clear = best
    return v, omega, new_idx, min_clear, feasible_count
