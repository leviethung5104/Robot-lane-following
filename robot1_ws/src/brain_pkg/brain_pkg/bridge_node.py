#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Float32
from cv_bridge import CvBridge

import cv2
import numpy as np
import socket
import struct
import threading
import time


class SocketBridgeNode(Node):
    def __init__(self):
        super().__init__('socket_bridge_node')
        self.bridge = CvBridge()

        # ===== IP PI =====
        self.pi_ip = '10.47.177.54'   # đổi thành IP thật của Pi
        self.pi_port = 9999
        self.client = None

        self.client_lock = threading.Lock()

        # ===== ROS2 PUB/SUB =====
        self.img_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.dist_pub = self.create_publisher(Float32, '/obs_distance', 10)
        self.create_subscription(Int32MultiArray, '/obs_bbox', self.bbox_callback, 10)

        # ===== THREAD NHẬN DỮ LIỆU =====
        self.receive_thread = threading.Thread(target=self.receive_data_from_pi, daemon=True)
        self.receive_thread.start()

    def bbox_callback(self, msg):
        with self.client_lock:
            if self.client is None:
                return

            if len(msg.data) != 4:
                return

            x1, y1, x2, y2 = msg.data

            # Scale từ ảnh gốc 640x480 về 160x120
            if x1 != -1:
                x1 = int(x1 / 4)
                y1 = int(y1 / 4)
                x2 = int(x2 / 4)
                y2 = int(y2 / 4)

            bbox_data = struct.pack("!4i", x1, y1, x2, y2)

            try:
                self.client.sendall(bbox_data)
            except Exception:
                pass

    def recvall(self, n):
        data = bytearray()
        while len(data) < n:
            with self.client_lock:
                if self.client is None:
                    return None
                try:
                    packet = self.client.recv(n - len(data))
                except Exception:
                    return None

            if not packet:
                return None

            data.extend(packet)

        return data

    def receive_data_from_pi(self):
        while rclpy.ok():
            try:
                self.get_logger().info(
                    f"Đang thử kết nối Socket tới Pi ({self.pi_ip}:{self.pi_port})..."
                )

                client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client.settimeout(5.0)
                client.connect((self.pi_ip, self.pi_port))
                client.settimeout(None)

                with self.client_lock:
                    self.client = client

                self.get_logger().info("✅ Đã kết nối Socket!")

                while rclpy.ok():
                    # ===== NHẬN RGB =====
                    length_bytes = self.recvall(4)
                    if not length_bytes:
                        raise ConnectionResetError("Mất kết nối khi nhận size ảnh")

                    img_len = struct.unpack("!I", length_bytes)[0]

                    if img_len <= 0 or img_len > 1000000:
                        raise ValueError(f"Size ảnh không hợp lệ: {img_len}")

                    img_data = self.recvall(img_len)
                    if not img_data:
                        raise ConnectionResetError("Mất kết nối khi nhận ảnh")

                    np_arr = np.frombuffer(img_data, np.uint8)
                    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                    if frame is not None:
                        ros_img = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                        self.img_pub.publish(ros_img)

                        cv2.imshow("PI Camera", frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            rclpy.shutdown()
                            return

                    # ===== NHẬN DISTANCE =====
                    dist_bytes = self.recvall(4)
                    if not dist_bytes:
                        raise ConnectionResetError("Mất kết nối khi nhận distance")

                    distance = struct.unpack("!f", dist_bytes)[0]

                    dist_msg = Float32()
                    dist_msg.data = distance / 1000.0
                    self.dist_pub.publish(dist_msg)

            except Exception as e:
                self.get_logger().error(
                    f"⚠️ Mất kết nối Socket: {e}. Thử lại sau 2 giây..."
                )

                with self.client_lock:
                    if self.client:
                        try:
                            self.client.close()
                        except:
                            pass
                    self.client = None

                time.sleep(2)


def main(args=None):
    rclpy.init(args=args)
    node = SocketBridgeNode()

    try:
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
