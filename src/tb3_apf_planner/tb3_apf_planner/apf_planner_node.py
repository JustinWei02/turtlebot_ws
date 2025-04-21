#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os

# APF path computation as before
def compute_apf_path(initial_position, goal_position, obstacles):
    zeta, d, eta, rho_0 = 26, 1.0, 25, 4
    step_size, goal_threshold = 0.1, 0.05
    max_iterations = 1000

    def compute_F_att(q):
        g = goal_position
        dist = np.linalg.norm(q - g)
        if dist <= d:
            return -zeta * (q - g)
        else:
            return -d * zeta * (q - g) / (dist + 1e-9)

    def compute_F_rep(q):
        min_rho, closest = float('inf'), None
        for obs in obstacles:
            rho = np.linalg.norm(q - obs)
            if rho < min_rho:
                min_rho, closest = rho, obs
        if closest is None or min_rho > rho_0:
            return np.zeros(2)
        dir_vec = (q - closest) / (min_rho + 1e-9)
        factor = eta * ((1/min_rho) - (1/rho_0)) / (min_rho**2)
        return factor * dir_vec

    path = []
    pos = np.array(initial_position, dtype=float)
    for _ in range(max_iterations):
        path.append(pos.copy())
        if np.linalg.norm(pos - goal_position) < goal_threshold:
            break
        F = compute_F_att(pos) + compute_F_rep(pos)
        normF = np.linalg.norm(F)
        if normF < 1e-9:
            break
        pos += step_size * (F / normF)
    path.append(pos.copy())
    return path

class Turtlebot3APFPlanner(Node):
    def __init__(self):
        super().__init__('turtlebot3_apf_planner')
        self.get_logger().info("Turtlebot3 APF Planner Node Initialized")

        # State
        self.x = self.y = self.theta = 0.0
        self.goal = (10.0, 10.0)
        self.obstacles = []
        self.robot_positions = []
        self.times = []
        self.distances = []

        # PID
        self.Kp, self.Kd, self.Ki = 2.0, 0.0, 1e-6
        self.prev_err = 0.0
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.prev_time = self.start_time
        self.maxVel, self.minVel = 3.0, -3.0
        self.linear_vel = 0.4

        # Subscribers & Publishers
        self.odom_received = False
        self.create_subscription(Odometry, '/model/my_robot/odometry', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map = None
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Loop
        self.create_timer(0.1, self.run)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        self.theta = math.atan2(siny, cosy)
        self.odom_received = True

    def map_callback(self, msg: OccupancyGrid):
        self.map = msg

    def get_obstacles_from_map(self, th=50):
        obs = []
        if not self.map:
            return obs
        info = self.map.info
        data = np.array(self.map.data).reshape((info.height, info.width))
        xs = np.where(data >= th)
        for r, c in zip(*xs):
            x = info.origin.position.x + (c + 0.5)*info.resolution
            y = info.origin.position.y + (r + 0.5)*info.resolution
            obs.append((x, y))
        return obs

    def pid_controller(self, desired_heading):
        err = desired_heading - self.theta
        if err > math.pi: err -= 2*math.pi
        if err < -math.pi: err += 2*math.pi
        now = self.get_clock().now().seconds_nanoseconds()[0]
        dt = max(now - self.prev_time, 1e-9)
        derivative = (err - self.prev_err) / dt
        ctrl = self.Kp*err + self.Kd*derivative
        ctrl = max(self.minVel, min(self.maxVel, ctrl))
        self.prev_err, self.prev_time = err, now
        return ctrl

    def run(self):
        if not self.odom_received:
            return
        t = (self.get_clock().now().seconds_nanoseconds()[0] - self.start_time)
        dist = math.hypot(self.goal[0]-self.x, self.goal[1]-self.y)
        self.times.append(t)
        self.distances.append(dist)
        self.robot_positions.append((self.x, self.y))

        if dist < 0.25:
            self.get_logger().info("Goal reached!")
            self.cmd_pub.publish(Twist())
            self.generate_plots()
            return

        obs = self.get_obstacles_from_map() or self.obstacles
        path = compute_apf_path((self.x, self.y), self.goal, obs)
        if len(path) <= 1:
            self.cmd_pub.publish(Twist())
            self.generate_plots()
            return

        wp = path[1]
        dh = math.atan2(wp[1]-self.y, wp[0]-self.x)
        ang = self.pid_controller(dh)
        fwd = 0.0 if abs(dh-self.theta)>0.3 else self.linear_vel
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = fwd, ang
        self.cmd_pub.publish(cmd)

    def generate_plots(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))

        # 1. Trajectory vs Planned Path
        rx, ry = zip(*self.robot_positions)
        final_obs = self.get_obstacles_from_map() or self.obstacles
        planned = compute_apf_path(self.robot_positions[0], self.goal, final_obs)
        px, py = zip(*planned)
        plt.close('all')
        plt.figure()
        plt.plot(rx, ry, '-', label='Actual Path')
        plt.plot(px, py, '--', label='Planned Path')
        plt.legend()
        plt.title('Trajectory vs Planned Path')
        plt.xlabel('X'); plt.ylabel('Y')
        plt.savefig(os.path.join(script_dir, 'trajectory_vs_planned.png'))

        # 2. Occupancy-Grid Snapshot
        if self.map:
            info, data = self.map.info, np.array(self.map.data)
            grid = data.reshape((info.height, info.width))
            plt.close('all')
            plt.figure()
            plt.imshow(grid, cmap='gray', origin='lower',
                       extent=[info.origin.position.x,
                               info.origin.position.x + info.width*info.resolution,
                               info.origin.position.y,
                               info.origin.position.y + info.height*info.resolution])
            plt.title('Occupancy-Grid Snapshot')
            plt.xlabel('X'); plt.ylabel('Y')
            plt.savefig(os.path.join(script_dir, 'occupancy_grid_snapshot.png'))

        # 3. Distance-to-Goal
        plt.close('all')
        plt.figure()
        plt.plot(self.times, self.distances)
        plt.title('Distance to Goal Over Time')
        plt.xlabel('Time (s)'); plt.ylabel('Distance (m)')
        plt.savefig(os.path.join(script_dir, 'distance_to_goal.png'))
        self.get_logger().info("Saved all plots.")


def main():
    rclpy.init()
    node = Turtlebot3APFPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
