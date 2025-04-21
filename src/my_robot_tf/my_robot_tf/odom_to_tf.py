#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf_broadcaster')
        self.subscription = self.create_subscription(
            Odometry,
            '/model/my_robot/odometry',
            self.odom_callback,
            10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
    
    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "my_robot/odom"   
        t.child_frame_id = "my_robot/chassis"     
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
