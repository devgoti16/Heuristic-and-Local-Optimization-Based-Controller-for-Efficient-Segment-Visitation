#!/usr/bin/env python3
"""
Controller node for CSCE 452/752 Project 2.

This node implements a robot controller that:
- Plans an efficient order of segment visits using segment-to-segment distances
  (nearest-neighbor initialization + 2-opt improvement).
- Chooses an execution point on each segment to minimize a weighted cost of
  heading change and travel distance, encouraging smooth straight-line motion.

Author: Dev Pankajbhai Goti
Course: CSCE 452/752 - Fall 2025
"""

import math
import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import PointCloud
from std_srvs.srv import Empty

# ------------------------- Configuration ------------------------

# Control loop frequency (Hz).
CONTROL_RATE_HZ = 20            

# Thresholds for proximity and heading
DIST_THRESHOLD = 0.05           # (m) Distance within which a target/segment is "reached"
ANGLE_THRESHOLD = 0.1           # (rad) Rotate first if heading error exceeds this

# Velocity limits
MAX_LINEAR_SPEED = 25.0          # (m/s)
MAX_ANGULAR_SPEED = 25.0         # (rad/s)

# Weights for selecting execution point on segment
TURN_WEIGHT = 6.0              # Penalty per radian of heading change
DIST_WEIGHT = 1.0               # Penalty per meter of travel

# 2-opt algorithm iteration bound
TWO_OPT_MAX_ITERS = 500


# ---------------------- Geometry Utilities ----------------------

def clamp(v: float, lo: float, hi: float) -> float:
    """Clamp value v into the closed interval [lo, hi]."""
    return max(lo, min(hi, v))


def normalize_angle(a: float) -> float:
    """Wrap angle into [-pi, pi] range for consistent comparisons."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def dist(a: tuple, b: tuple) -> float:
    """Return Euclidean distance between two 2D points a=(x,y), b=(x,y)."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def closest_point_on_segment(pt: tuple, a: tuple, b: tuple) -> tuple:
    """
    Return the closest point on segment (a, b) to point pt.
    Handles degenerate case where a==b.
    """
    ax, ay = a
    bx, by = b
    px, py = pt
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom == 0:
        return a  # degenerate (zero-length) segment
    t = ((px - ax) * dx + (py - ay) * dy) / denom
    t = clamp(t, 0.0, 1.0)  # clamp to segment range
    return (ax + t * dx, ay + t * dy)


def segment_to_segment_distance(s1: tuple, s2: tuple) -> float:
    """
    Return the shortest distance between two segments:
      s1=((x1,y1),(x2,y2)), s2=((x3,y3),(x4,y4))
    Distance candidates: endpoints of one projected onto the other.
    """
    a1, a2 = s1
    b1, b2 = s2
    c1 = dist(closest_point_on_segment(a1, b1, b2), a1)
    c2 = dist(closest_point_on_segment(a2, b1, b2), a2)
    c3 = dist(closest_point_on_segment(b1, a1, a2), b1)
    c4 = dist(closest_point_on_segment(b2, a1, a2), b2)
    return min(c1, c2, c3, c4)



# --------------------- Main Controller Node ---------------------

class Controller(Node):
    """
    Controller node that plans and executes an efficient tour of line segments.

    Workflow:
      1. Wait for /unvisited_targets (PointCloud of segment endpoints).
      2. Build segments as consecutive pairs of endpoints.
      3. Compute tour order (nearest-neighbor + 2-opt) using segment distances.
      4. Execute tour:
         - For each segment, choose execution point that minimizes a weighted
           combination of turn cost and distance.
         - Move toward that point until segment considered "crossed".
      5. Publish cmd_vel at CONTROL_RATE_HZ continuously.
    """

    def __init__(self):
        super().__init__('controller')

        self.start_time = None

        # -------- Publishers & Subscribers --------
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Pose2D, '/pose', self.pose_cb, 10)
        self.create_subscription(PointCloud, '/unvisited_targets', self.targets_cb, 10)

        # -------- Reset sim2 at startup --------
        self.reset_client = self.create_client(Empty, 'reset')
        self._wait_for_service(self.reset_client, 'reset')
        self.reset_client.call_async(Empty.Request())
        self.get_logger().info('Called sim2/reset to start fresh.')

        # -------- State Variables --------
        self.pose = None                # (x,y,theta) tuple from /pose
        self.segments = []              # List of segments: ((x1,y1),(x2,y2))
        self.tour = []                  # Ordered list of indices into self.segments
        self.current_segment_idx = 0    # Progress index within tour
        self.executing = False          # Becomes True after planning

        # -------- Timer for control loop --------
        self.create_timer(1.0 / CONTROL_RATE_HZ, self.control_loop)


    # Callbacks

    def pose_cb(self, msg: Pose2D):
        """Store the latest robot pose (x,y,theta)."""
        self.pose = (msg.x, msg.y, msg.theta)

    def targets_cb(self, msg: PointCloud):
        """
        Receive unvisited segment endpoints (consecutive pairs form a segment).
        Triggered only once at startup to build segments and plan tour.
        """
        if self.segments:  # ignore repeats
            return

        pts = [(p.x, p.y) for p in msg.points]
        if not pts:
            self.get_logger().warn('Received empty /unvisited_targets; nothing to do.')
            return

        if len(pts) % 2 != 0:
            self.get_logger().error('PointCloud has odd number of points; expected pairs.')
            return

        # Build list of segments from consecutive pairs
        self.segments = [(pts[i], pts[i+1]) for i in range(0, len(pts), 2)]
        self.get_logger().info(f'Received {len(self.segments)} segments from sim2.')

        if self.pose is None:
            # Need pose before we can plan intelligently
            self.get_logger().info('Pose not yet available; will plan once pose arrives.')
            return

        self.plan_tour()
        self.executing = True
        self.start_time = time.time() 
        self.get_logger().info('Planning complete; beginning execution of tour.')

    
    # Planning

    def plan_tour(self):
        """
        Compute an efficient tour of all segments:
          - Start with nearest segment to robot.
          - Construct tour using nearest-neighbor heuristic.
          - Refine with 2-opt local optimization.
        """
        n = len(self.segments)
        if n == 0:
            self.get_logger().error('No segments to plan.')
            return

        # Precompute segment-to-segment distances
        dist_mat = [[0.0] * n for _ in range(n)]
        for i in range(n):
            for j in range(i + 1, n):
                d = segment_to_segment_distance(self.segments[i], self.segments[j])
                dist_mat[i][j] = d
                dist_mat[j][i] = d

        # Choose start: closest segment to robot
        robot_xy = (self.pose[0], self.pose[1])
        start_idx = min(
            range(n),
            key=lambda i: dist(closest_point_on_segment(robot_xy, *self.segments[i]), robot_xy)
        )

        # Nearest neighbor tour construction
        unvisited = set(range(n))
        tour = [start_idx]
        unvisited.remove(start_idx)
        current = start_idx
        while unvisited:
            nxt = min(unvisited, key=lambda j: dist_mat[current][j])
            tour.append(nxt)
            unvisited.remove(nxt)
            current = nxt

        # 2-opt local optimization
        self._two_opt_improve(tour, dist_mat)

        self.tour = tour
        self.get_logger().info(f'Planned tour order: {self.tour}')

    def _two_opt_improve(self, tour: list, dist_mat: list):
        """
        Apply 2-opt algorithm to locally improve tour order.
        Tries to shorten path by swapping subsequences.
        """
        n = len(tour)
        if n < 4:
            return

        improved = True
        iters = 0
        while improved and iters < TWO_OPT_MAX_ITERS:
            improved = False
            iters += 1
            for i in range(1, n - 2):
                for j in range(i + 1, n):
                    if j - i == 1:  # skip adjacent nodes
                        continue
                    # Edges before and after swap
                    a, b = tour[i - 1], tour[i]
                    c, d = tour[j - 1], tour[j % n]
                    before = dist_mat[a][b] + dist_mat[c][d]
                    after = dist_mat[a][c] + dist_mat[b][d]
                    if after + 1e-9 < before:  # improve
                        tour[i:j] = reversed(tour[i:j])
                        improved = True
        if iters >= TWO_OPT_MAX_ITERS:
            self.get_logger().info('2-opt reached max iterations; stopping.')


    # Execution / Control

    def control_loop(self):
        """
        Periodic control loop (runs at CONTROL_RATE_HZ).
        Drives the robot along the planned tour by:
          - Picking an execution point on current segment.
          - Moving toward that point with a smooth controller.
          - Skipping segments already crossed.
        """
        if not self.executing or self.pose is None:
            return

        if self.current_segment_idx >= len(self.tour):
            # Tour finished
            total_time = time.time() - self.start_time if self.start_time else 0.0
            self.get_logger().info(f'All segments visited in {total_time:.2f} seconds; stopping robot.')
            self.cmd_pub.publish(Twist())  # stop
            self.executing = False
            return

        seg_idx = self.tour[self.current_segment_idx]
        seg = self.segments[seg_idx]

        # Pick best execution point on segment
        exec_point = self._choose_execution_point(seg)

        # Drive toward it
        self._move_to_point(exec_point)

        # Check if any segments (current or later) have already been crossed
        for idx in list(self.tour[self.current_segment_idx:]):
            if self._segment_reached(self.segments[idx]):
                if idx == seg_idx:
                    self.get_logger().info(f'Segment {seg_idx} reached; advancing.')
                    self.current_segment_idx += 1
                else:
                    self.get_logger().info(f'Segment {idx} already crossed; skipping.')
                    self.tour.remove(idx)

    def _segment_reached(self, seg: tuple) -> bool:
        """Return True if robot is within DIST_THRESHOLD of the given segment."""
        robot_xy = (self.pose[0], self.pose[1])
        closest = closest_point_on_segment(robot_xy, *seg)
        return dist(robot_xy, closest) < DIST_THRESHOLD

    def _choose_execution_point(self, seg: tuple) -> tuple:
        """
        Select an approach point on a segment to minimize combined turn+distance cost.
        Candidates:
          - Closest point to robot (min travel distance).
          - Midpoint (encourages straight-line traversal).
          - Slightly biased point between closest and midpoint (reduces zig-zag).
        """
        a, b = seg
        robot_xy = (self.pose[0], self.pose[1])
        robot_theta = self.pose[2]

        # Candidate points
        cand_closest = closest_point_on_segment(robot_xy, a, b)
        cand_mid = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
        bias_frac = 0.15
        cand_bias = (
            cand_closest[0] + bias_frac * (cand_mid[0] - cand_closest[0]),
            cand_closest[1] + bias_frac * (cand_mid[1] - cand_closest[1])
        )

        candidates = [cand_closest, cand_mid, cand_bias]

        # Score each candidate
        best, best_score = None, float('inf')
        for c in candidates:
            dx, dy = c[0] - robot_xy[0], c[1] - robot_xy[1]
            distance = math.hypot(dx, dy)
            desired_theta = math.atan2(dy, dx)
            turn = abs(normalize_angle(desired_theta - robot_theta))
            score = TURN_WEIGHT * turn + DIST_WEIGHT * distance
            if score < best_score:
                best, best_score = c, score

        return best

    def _move_to_point(self, goal: tuple) -> bool:
        """
        Low-level controller:
          - Rotate in place if heading error is large.
          - Otherwise move forward with proportional heading correction.
        Returns True if goal is reached.
        """
        rx, ry, rtheta = self.pose
        gx, gy = goal
        dx, dy = gx - rx, gy - ry
        distance = math.hypot(dx, dy)
        desired_theta = math.atan2(dy, dx)
        angle_err = normalize_angle(desired_theta - rtheta)

        cmd = Twist()

        if distance < DIST_THRESHOLD:
            self.cmd_pub.publish(Twist())  # stop
            return True

        if abs(angle_err) > ANGLE_THRESHOLD:
            # Prioritize rotation
            cmd.angular.z = clamp(5.0 * angle_err, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
            cmd.linear.x = 0.0
        else:
            # Move forward with heading correction
            cmd.linear.x = clamp(8.0 * distance, 0.0, MAX_LINEAR_SPEED)
            cmd.angular.z = clamp(8.0 * angle_err, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)

        self.cmd_pub.publish(cmd)
        return False


    # Utilities

    def _wait_for_service(self, client, name):
        """Block until given service is available."""
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for service {name}...')


# ------------------------- Entry Point --------------------------

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutdown requested by user.')
    finally:
        node.cmd_pub.publish(Twist())  # stop robot on exit
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
