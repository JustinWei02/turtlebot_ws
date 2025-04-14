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

def compute_apf_path(initial_position, goal_position, obstacles):
    # APF parameters
    zeta = 26
    d = 1.0
    eta = 25
    rho_0 = 4
    step_size = 0.1
    goal_threshold = 0.05
    max_iterations = 1000

    def compute_F_att(q):
        # Attractive force toward the goal.
        g = goal_position
        dist_to_goal = np.linalg.norm(q - g)
        if dist_to_goal <= d:
            return -zeta * (q - g)
        else:
            direction = (q - g) / (dist_to_goal + 1e-9)
            return -d * zeta * direction

    def compute_F_rep(q):
        # Repulsive force from obstacles.
        min_rho = float('inf')
        closest_obs = None
        for obs in obstacles:
            dist_ = np.linalg.norm(q - obs)
            if dist_ < min_rho:
                min_rho = dist_
                closest_obs = obs

        if closest_obs is None or (min_rho > rho_0):
            return np.array([0.0, 0.0])
        else:
            direction = (q - closest_obs) / (min_rho + 1e-9)
            factor = eta * ((1/min_rho) - (1/rho_0)) * (1/(min_rho**2))
            return factor * direction

    path = []
    current_pos = np.array(initial_position, dtype=float)

    for _ in range(max_iterations):
        path.append(current_pos.copy())
        dist_to_goal = np.linalg.norm(current_pos - goal_position)
        if dist_to_goal < goal_threshold:
            break

        F_att = compute_F_att(current_pos)
        F_rep = compute_F_rep(current_pos)
        F_total = F_att + F_rep
        norm_F = np.linalg.norm(F_total)
        if norm_F < 1e-9:
            break

        direction = F_total / norm_F
        current_pos += step_size * direction

    if not np.allclose(path[-1], current_pos, atol=1e-9):
        path.append(current_pos.copy())

    return path

class Turtlebot3APFPlanner(Node):
    def __init__(self):
        super().__init__('turtlebot3_apf_planner')
        self.get_logger().info("Turtlebot3 APF Planner Node Initialized")

        # Robot state variables.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Goal position (in world coordinates).
        self.goal = (10.0, 10.0)
        # Fallback static obstacles (if map data is not available).
        self.obstacles = [(6.0, 5.0), (5.0, 5.0)]
        self.robot_positions = []

        # PID parameters for heading control.
        self.Kp = 2.0
        self.Kd = 0.0
        self.Ki = 0.00001

        self.prev_err = 0.0
        self.prev_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.maxMotorVel = 3.0
        self.minMotorVel = -3.0

        # Desired forward speed.
        self.linear_vel = 0.4

        # Odometry subscription for robot's current pose.
        self.odom_received = False
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/my_robot/odometry',
            self.odom_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Map subscriber to receive occupancy grid updates.
        self.map = None
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Control loop at 10 Hz.
        self.timer = self.create_timer(0.1, self.run)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.odom_received = True

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

    def map_callback(self, msg: OccupancyGrid):
        self.map = msg

    def get_obstacles_from_map(self, threshold=50):
        obstacles = []
        if self.map is None:
            return obstacles
        
        resolution = self.map.info.resolution
        width = self.map.info.width
        height = self.map.info.height
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y

        for idx, value in enumerate(self.map.data):
            if value >= threshold:
                row = idx // width
                col = idx % width
                # Compute world coordinates (center of cell).
                x = origin_x + (col + 0.5) * resolution
                y = origin_y + (row + 0.5) * resolution
                obstacles.append((x, y))
        return obstacles

    def pid_controller(self, desired_heading):
        # Calculate the shortest angular error.
        heading_error = desired_heading - self.theta
        if heading_error > math.pi:
            heading_error -= 2 * math.pi
        elif heading_error < -math.pi:
            heading_error += 2 * math.pi

        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        dt = current_time - self.prev_time
        if dt <= 0.0:
            dt = 1e-9

        derivative = (heading_error - self.prev_err) / dt
        control = (self.Kp * heading_error) + (self.Kd * derivative)

        control = max(self.minMotorVel, min(self.maxMotorVel, control))

        self.prev_err = heading_error
        self.prev_time = current_time
        return control

    def run(self):
        if not self.odom_received:
            self.get_logger().warn("Waiting for valid odometry...")
            return

        # Record the current robot position for later plotting.
        self.robot_positions.append((self.x, self.y))

        dist_to_goal = math.hypot(self.goal[0] - self.x, self.goal[1] - self.y)
        if dist_to_goal < 0.25:
            self.get_logger().info("Goal reached! Stopping.")
            self.cmd_vel_pub.publish(Twist())
            self.draw_final_path()
            return

        # Use dynamic obstacles if map data is available; otherwise, use fallback obstacles.
        dynamic_obstacles = self.obstacles
        if self.map is not None:
            extracted = self.get_obstacles_from_map(threshold=50)
            if extracted:
                dynamic_obstacles = extracted

        current_pos = (self.x, self.y)
        path = compute_apf_path(current_pos, self.goal, dynamic_obstacles)

        if len(path) <= 1:
            self.get_logger().info("APF path short or done. Stopping.")
            self.cmd_vel_pub.publish(Twist())
            self.draw_final_path()
            return

        # Select the next waypoint in the computed path.
        waypoint = path[1]
        dx = waypoint[0] - self.x
        dy = waypoint[1] - self.y
        desired_heading = math.atan2(dy, dx)

        self.get_logger().info(
            f"Dist to goal: {dist_to_goal:.2f}, Next WP=({waypoint[0]:.2f}, {waypoint[1]:.2f}), "
            f"Desired heading={desired_heading:.2f}, Current heading={self.theta:.2f}"
        )

        angular_control = self.pid_controller(desired_heading)
        heading_error = abs(desired_heading - self.theta)
        # Slow down if the heading error is high.
        if heading_error > 0.3:
            fwd_speed = 0.0
        else:
            fwd_speed = self.linear_vel

        twist = Twist()
        twist.linear.x = fwd_speed
        twist.angular.z = angular_control
        self.cmd_vel_pub.publish(twist)

    def draw_final_path(self):
        """
        Generate a PNG showing the robot's traveled path,
        along with obstacles and the final goal.
        """
        if len(self.robot_positions) < 1:
            return

        robot_positions = np.array(self.robot_positions)
        rx = robot_positions[:, 0]
        ry = robot_positions[:, 1]

        plt.close('all')
        plt.figure()

        # Plot the robot's path
        plt.plot(rx, ry, label='Robot Path')
        plt.scatter(rx[-1], ry[-1], c='red', marker='o', label='Final Position')
        plt.scatter(rx[0], ry[0], c='green', marker='o', label='Start Position')
        plt.scatter(self.goal[0], self.goal[1], marker='s', c='orange', label='Goal')

        # Plot obstacles from the occupancy grid if available.
        if self.map is not None:
            obstacles = self.get_obstacles_from_map(threshold=50)
            if obstacles:
                obstacles = np.array(obstacles)
                plt.scatter(obstacles[:, 0], obstacles[:, 1], marker='x', c='black', label='Obstacles')
        else:
            for obs in self.obstacles:
                plt.scatter(obs[0], obs[1], marker='x', c='black', label='Obstacle')

        plt.title("Robot APF Navigation")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend()

        # Save the final path plot to a PNG file.
        script_dir = os.path.dirname(os.path.abspath(__file__))
        filename = os.path.join(script_dir, "final_run_path.png")
        plt.savefig(filename)
        self.get_logger().info(f"Final path saved to {filename}")

def main(args=None):
    rclpy.init(args=args)
    node = Turtlebot3APFPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
