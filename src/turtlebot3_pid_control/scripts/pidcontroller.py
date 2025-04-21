#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class Turtlebot3PIDController(Node):
    def __init__(self):
        super().__init__('turtlebot3_pid_controller')
        print("Turtlebot Subscriber Node Initialized")

        # Robot initial States
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.45 

        # PID gains
        self.Kp = 1.0 
        self.Kd = 0.1  
        self.Ki = 0.01  

        # PID initial parameters
        self.error = 0.0
        self.derivative = 0.0
        self.integral = 0.0
        self.prev_err = 0.0
        self.prev_time = self.get_clock().now().seconds_nanoseconds()[0]  

        self.distThreshold = 0.2  # distance threshold for waypoint transition
        self.maxMotorVel = 2.84
        self.minMotorVel = -2.84
        
        ### (Uncomment to switch between cases)
        # Define waypoints for Case 1 
        #self.waypoints = np.array([[1,1], [5,5], [9,1], [3,-5]])
        

        # Define waypoints for Case 2 (sinusoidal path)
        a = 1.25  # Amplitude of sine wave
        x_values = np.linspace(0, 8, 20)  
        self.waypoints = np.array([[x, a * math.sin(x)] for x in x_values])

        self.waypoint_size = self.waypoints.shape[0]
        self.waypoint_counter = 0

        # Subscribers & Publishers
        self.odom_sub = self.create_subscription(Odometry, '/model/my_robot/odometry', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for control loop (10 Hz)
        self.timer = self.create_timer(0.1, self.run)



    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Get quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

        self.get_logger().info(f"Updated Theta: {self.theta:.3f} radians")


    def calculate_error(self):
        wp_x, wp_y = self.waypoints[self.waypoint_counter]
        desired_theta = math.atan2(wp_y - self.y, wp_x - self.x)
        self.error = desired_theta - self.theta

        # Normalize error between -π and π
        if self.error > math.pi:
            self.error -= 2 * math.pi
        elif self.error < -math.pi:
            self.error += 2 * math.pi

    def pid_controller(self):
        self.calculate_error()
        current_time = self.get_clock().now().seconds_nanoseconds()[0]  
        dt = current_time - self.prev_time

        if dt > 0:
            self.derivative = (self.error - self.prev_err) / dt
            self.integral += self.error * dt
        else:
            self.derivative = 0.0  

        control = self.Kp * self.error + self.Kd * self.derivative + self.Ki * self.integral
        control = max(self.minMotorVel, min(self.maxMotorVel, control))  # Saturates the controller inside the min max range

        self.prev_err = self.error
        self.prev_time = current_time

        return control

    def run(self):
        if self.x == 0.0 and self.y == 0.0:
            self.get_logger().warn("Odometry not received. Ensure the robot is publishing /odom")
            return

        wp_x, wp_y = self.waypoints[self.waypoint_counter]
        self.goal = np.array([wp_x, wp_y])
        current_state = np.array([self.x, self.y])
        self.distError = np.linalg.norm(self.goal - current_state)

        self.get_logger().info(f"Position: ({self.x:.2f}, {self.y:.2f}), Heading: {self.theta:.2f}")
        self.get_logger().info(f"Target: ({wp_x}, {wp_y}), Distance Error: {self.distError:.2f}")

        if self.distError > self.distThreshold:
            angular_control = self.pid_controller()
        else:
            self.waypoint_counter = (self.waypoint_counter + 1) % self.waypoint_size
            angular_control = self.pid_controller()

        twist = Twist()
        twist.linear.x = self.linear_vel
        twist.angular.z = angular_control
        self.cmd_vel_pub.publish(twist)

def main():
    rclpy.init()
    node = Turtlebot3PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
