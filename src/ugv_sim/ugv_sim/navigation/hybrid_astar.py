"""
Hybrid A* global planner.

Search state is (x, y, theta) -- not plain 2D A* -- expanded with fixed-arc
motion primitives (forward straight / forward-left / forward-right, plus
heavily-penalized reverse equivalents) so every generated path is
kinematically committed to a heading at each point, not just a cell
sequence. Collision-checked against the shared Costmap2D using the same
rectangular-footprint checker DWA uses, so a path Hybrid A* accepts is one
the vehicle can actually occupy.
"""
import heapq
import math

from .collision_checker import footprint_points, VEHICLE_HALF_LENGTH, VEHICLE_HALF_WIDTH

# Global planning uses a larger margin than DWA's own (tighter, true)
# collision check. Without this, Hybrid A* -- free to cut arbitrarily
# close to an obstacle as long as the footprint technically clears it --
# routinely proposed paths hugging route_obstacle_A (it sits almost
# exactly on the direct start-to-goal line) tightly enough that DWA
# (rightly) refused to actually execute the squeeze, producing a
# standoff: replan finds the same tight route, DWA balks again. Measured
# directly across many runs: the vehicle got stuck at the same ~1.3-1.5 m
# distance from the obstacle every time. Giving the search extra margin
# makes it prefer routes with real breathing room in the first place.
PLANNING_MARGIN = 0.06


def is_state_valid(costmap, x, y, theta):
    for wx, wy in footprint_points(
            x, y, theta,
            half_length=VEHICLE_HALF_LENGTH + PLANNING_MARGIN,
            half_width=VEHICLE_HALF_WIDTH + PLANNING_MARGIN):
        if not costmap.is_free_world(wx, wy):
            return False
    return True


# Reduced from 0.45: measured directly on Rubicon terrain -- from a
# position squeezed between two large boulders (the vehicle's actual
# start pocket), every one of the 6 fixed-arc primitives at 0.45 m
# immediately collided (10 straight "no path found" in 0.00 s, meaning
# zero expansions succeeded even from a valid start state), even though a
# manual escape maneuver using much smaller ~0.1-0.25 m pulses
# demonstrably worked. The primitive step was simply too coarse to find
# a foothold in a tight gap; finer steps let the search actually explore
# the same space a careful human/rocking maneuver could use.
STEP_LENGTH = 0.25          # m per primitive
TURN_RADIUS = 1.1           # m, for the left/right arc primitives
THETA_BINS = 16              # heading discretization
GRID_RESOLUTION = 0.2        # must match Costmap2D resolution for the state grid
REVERSE_PENALTY = 3.0
TURN_PENALTY = 1.08
MAX_EXPANSIONS = 80000
GOAL_POS_TOL = 0.35
GOAL_YAW_TOL = math.radians(75)  # loose -- final heading is not critical here

# Extra cost per meter of travel through fully-rough (cost=1.0) terrain,
# relative to 1.0 cost-per-meter for flat ground. Kept modest on purpose:
# this is Rubicon Trail, real off-road terrain is *supposed* to be uneven
# (Part 5-7's "must not refuse every route" requirement) -- the term
# should nudge the search toward smoother ground when a comparably-short
# alternative exists, not treat all roughness as near-impassable the way
# obstacle_cost (via is_state_valid) already does for actual obstacles.
TERRAIN_COST_WEIGHT = 1.5


def _wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def _primitives():
    dtheta = STEP_LENGTH / TURN_RADIUS
    prims = [
        (STEP_LENGTH, 0.0, 1.0),            # forward straight
        (STEP_LENGTH, dtheta, TURN_PENALTY),   # forward left
        (STEP_LENGTH, -dtheta, TURN_PENALTY),  # forward right
        (-STEP_LENGTH, 0.0, REVERSE_PENALTY),
        (-STEP_LENGTH, dtheta, REVERSE_PENALTY * TURN_PENALTY),
        (-STEP_LENGTH, -dtheta, REVERSE_PENALTY * TURN_PENALTY),
    ]
    return prims


_PRIMS = _primitives()


def _apply_primitive(x, y, theta, arc_len, dtheta):
    if abs(dtheta) < 1e-6:
        nx = x + arc_len * math.cos(theta)
        ny = y + arc_len * math.sin(theta)
        ntheta = theta
    else:
        radius = arc_len / dtheta
        cx = x - radius * math.sin(theta)
        cy = y + radius * math.cos(theta)
        ntheta = _wrap(theta + dtheta)
        nx = cx + radius * math.sin(ntheta)
        ny = cy - radius * math.cos(ntheta)
    return nx, ny, ntheta


def _state_key(x, y, theta):
    gx = int(round(x / GRID_RESOLUTION))
    gy = int(round(y / GRID_RESOLUTION))
    gt = int(round(_wrap(theta) / (2 * math.pi / THETA_BINS))) % THETA_BINS
    return (gx, gy, gt)


HEADING_HEURISTIC_WEIGHT = 3.5


def _heuristic(x, y, theta, goal_x, goal_y):
    # Pure Euclidean distance alone does not discourage unnecessary
    # heading excursions: a search state reached via a wide, wasteful
    # turn scores exactly as well as one reached directly, as long as
    # both are equally close to the goal position. Measured directly:
    # starting at a 40-degree heading toward a goal at bearing ~31 degrees,
    # the planner accepted a path swinging as far as 87 degrees before
    # settling — no positional need for that, but nothing in the search
    # penalized it. Adding the angular error to the desired bearing
    # (in "meters per radian" via the weight) biases the search toward
    # states that are both closer AND already reasonably aimed at the
    # goal, without hard-forbidding a genuine detour around an obstacle.
    d = math.hypot(goal_x - x, goal_y - y)
    bearing = math.atan2(goal_y - y, goal_x - x)
    heading_err = abs(_wrap(bearing - theta))
    return d + HEADING_HEURISTIC_WEIGHT * heading_err


def plan(start_x, start_y, start_theta, goal_x, goal_y, goal_theta, costmap,
         goal_pos_tol=GOAL_POS_TOL, goal_yaw_tol=GOAL_YAW_TOL,
         max_expansions=MAX_EXPANSIONS):
    """
    Search for a path from start to goal.

    Returns a list of (x, y, theta) waypoints from start to goal, or None
    if no path was found within max_expansions node expansions.
    """
    if not is_state_valid(costmap, start_x, start_y, start_theta):
        return None

    start_key = _state_key(start_x, start_y, start_theta)
    g_score = {start_key: 0.0}
    came_from = {}
    node_state = {start_key: (start_x, start_y, start_theta)}

    h0 = _heuristic(start_x, start_y, start_theta, goal_x, goal_y)
    open_heap = [(h0, 0.0, start_key)]
    closed = set()
    expansions = 0

    goal_key = None
    while open_heap and expansions < max_expansions:
        _, g, key = heapq.heappop(open_heap)
        if key in closed:
            continue
        closed.add(key)
        expansions += 1
        x, y, theta = node_state[key]

        if (math.hypot(goal_x - x, goal_y - y) < goal_pos_tol
                and abs(_wrap(goal_theta - theta)) < goal_yaw_tol):
            goal_key = key
            break

        for arc_len, dtheta, cost_mult in _PRIMS:
            nx, ny, ntheta = _apply_primitive(x, y, theta, arc_len, dtheta)
            nkey = _state_key(nx, ny, ntheta)
            if nkey in closed:
                continue
            if not is_state_valid(costmap, nx, ny, ntheta):
                continue
            terrain_cost = costmap.terrain_cost_world(nx, ny)
            step_cost = abs(arc_len) * cost_mult * (1.0 + TERRAIN_COST_WEIGHT * terrain_cost)
            ng = g + step_cost
            if nkey in g_score and g_score[nkey] <= ng:
                continue
            g_score[nkey] = ng
            came_from[nkey] = key
            node_state[nkey] = (nx, ny, ntheta)
            f = ng + _heuristic(nx, ny, ntheta, goal_x, goal_y)
            heapq.heappush(open_heap, (f, ng, nkey))

    if goal_key is None:
        return None

    path = []
    key = goal_key
    while key in came_from:
        path.append(node_state[key])
        key = came_from[key]
    path.append(node_state[start_key])
    path.reverse()
    return path
