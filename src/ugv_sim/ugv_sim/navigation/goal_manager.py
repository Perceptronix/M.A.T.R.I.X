"""Goal completion check: explicit position + orientation tolerance."""
import math


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class GoalManager:

    def __init__(self, goal_x, goal_y, goal_yaw,
                 pos_tolerance=0.35, yaw_tolerance=math.radians(90)):
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_yaw = goal_yaw
        self.pos_tolerance = pos_tolerance
        self.yaw_tolerance = yaw_tolerance

    def distance(self, x, y):
        return math.hypot(self.goal_x - x, self.goal_y - y)

    def reached(self, x, y, theta):
        if self.distance(x, y) > self.pos_tolerance:
            return False
        return abs(wrap(self.goal_yaw - theta)) <= self.yaw_tolerance
