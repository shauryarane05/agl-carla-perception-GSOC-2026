import socket
import json
import math
import rclpy

from nav_msgs.msg import Odometry

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('127.0.0.1', 9870))
sock.settimeout(1.0)

rclpy.init()

node = rclpy.create_node('carla_odom_bridge')
pub = node.create_publisher(Odometry, '/carla/odom', 10)

node.get_logger().info('publishing /carla/odom')

while rclpy.ok():
    try:
        data, _ = sock.recvfrom(65535)
    except socket.timeout:
        continue

    d = json.loads(data)

    o = Odometry()

    o.header.stamp = node.get_clock().now().to_msg()
    o.header.frame_id = 'map'
    o.child_frame_id = 'ego'

    o.pose.pose.position.x = float(d['x'])
    o.pose.pose.position.y = float(d['y'])
    o.pose.pose.position.z = float(d['z'])

    y = math.radians(d['yaw'])
    o.pose.pose.orientation.z = math.sin(y / 2)
    o.pose.pose.orientation.w = math.cos(y / 2)

    o.twist.twist.linear.x = float(d['vx'])
    o.twist.twist.linear.y = float(d['vy'])
    o.twist.twist.linear.z = float(d['vz'])

    pub.publish(o)
