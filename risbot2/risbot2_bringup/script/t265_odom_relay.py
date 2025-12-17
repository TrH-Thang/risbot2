#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.qos import qos_profile_sensor_data

class T265OdomRepublisher(Node):
    def __init__(self):
        super().__init__('t265_odom_republisher')

        # Subscriber từ T265, dùng đúng QoS cho sensor data
        self.sub = self.create_subscription(
            Odometry,
            '/camera/pose/sample',
            self.cb,
            qos_profile_sensor_data
        )

        # Publisher ra topic /odom
        self.pub = self.create_publisher(
            Odometry,
            '/odom',
            10  # default QoS
        )

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def cb(self, msg: Odometry):
        # Tạo Odometry mới
        odom = Odometry()

        # ===== HEADER =====
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        # ===== POSE & TWIST =====
        odom.pose = msg.pose
        odom.twist = msg.twist

        # ===== PUBLISH ODOM =====
        self.pub.publish(odom)

        # ===== TF =====
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf)

def main():
    rclpy.init()
    node = T265OdomRepublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
