import socket, struct, time, numpy as np, rclpy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

HOST, PORT = "127.0.0.1", 9871

def recvall(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk: return None
        buf += chunk
    return buf

rclpy.init()
node = rclpy.create_node("lidar_pc2_bridge")
pub = node.create_publisher(PointCloud2, "/carla/lidar", 10)

fields = [
    PointField(name="x", offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]

s = None
while rclpy.ok() and s is None:
    try:
        s = socket.create_connection((HOST, PORT), timeout=5)
    except OSError:
        node.get_logger().info("lidar source not up, retrying..."); time.sleep(2)
node.get_logger().info("connected to lidar source, publishing /carla/lidar")

count = 0
try:
    while rclpy.ok():
        hdr = recvall(s, 4)
        if hdr is None: break
        (n,) = struct.unpack(">I", hdr)
        data = recvall(s, n)
        if data is None: break
        arr = np.frombuffer(data, dtype=np.float32).reshape(-1, 4).copy()
        arr[:, 1] = -arr[:, 1]   # CARLA left-handed -> ROS right-handed
        npts = arr.shape[0]
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "ego_lidar"
        msg.height = 1
        msg.width = npts
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * npts
        msg.is_dense = True
        msg.data = arr.tobytes()
        pub.publish(msg)
        count += 1
        if count <= 3 or count % 20 == 0:
            node.get_logger().info("published frame %d: %d points" % (count, npts))
except KeyboardInterrupt:
    pass
finally:
    if s: s.close()
    node.destroy_node(); rclpy.shutdown()
