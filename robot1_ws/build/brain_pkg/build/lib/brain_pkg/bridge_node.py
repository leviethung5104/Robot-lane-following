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
        
        self.pi_ip = '10.180.114.54' 
        self.pi_port = 9999
        self.client = None
        
        self.img_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.dist_pub = self.create_publisher(Float32, '/obs_distance', 10)
        self.create_subscription(Int32MultiArray, '/obs_bbox', self.bbox_callback, 10)
        
        # Chạy luồng nhận dữ liệu
        self.receive_thread = threading.Thread(target=self.receive_data_from_pi)
        self.receive_thread.start()

    def bbox_callback(self, msg):
        if self.client is None: return # Chưa kết nối thì không gửi
        
        if len(msg.data) == 4:
            x1, y1, x2, y2 = msg.data
            if x1 != -1:
                x1, y1 = int(x1/4), int(y1/4)
                x2, y2 = int(x2/4), int(y2/4)
            
            bbox_data = struct.pack("!4i", x1, y1, x2, y2)
            try:
                self.client.sendall(bbox_data)
            except Exception as e:
                pass # Lỗi mạng sẽ được luồng receive_data xử lý reconnect
     
    def recvall(self, n):
        data = bytearray()
        while len(data) < n:
            packet = self.client.recv(n - len(data))
            if not packet: return None
            data.extend(packet)
        return data

    def receive_data_from_pi(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f"Đang thử kết nối Socket tới Pi ({self.pi_ip})...")
                self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client.settimeout(5.0)
                self.client.connect((self.pi_ip, self.pi_port))
                self.client.settimeout(None)
                self.get_logger().info("✅ Đã kết nối Socket!")        
                
                while rclpy.ok():
                    # 1. Nhận RGB
                    length_bytes = self.recvall(4)
                    if not length_bytes: raise ConnectionResetError("Mất kết nối")
                    img_len = struct.unpack("!I", length_bytes)[0]
                    
                    if img_len > 1000000: # Bảo vệ tràn RAM (ảnh size ảo do rác)
                        continue 
                        
                    img_data = self.recvall(img_len)
                    if img_data:
                        np_arr = np.frombuffer(img_data, np.uint8)
                        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                        if frame is not None:
                            self.img_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

                    # 2. Nhận Distance
                    dist_bytes = self.recvall(4)
                    if not dist_bytes: raise ConnectionResetError("Mất kết nối")
                    distance = struct.unpack("!f", dist_bytes)[0]
                    
                    dist_msg = Float32()
                    dist_msg.data = distance / 1000.0 
                    self.dist_pub.publish(dist_msg)

            except Exception as e:
                self.get_logger().error(f"⚠️ Mất kết nối Socket: {e}. Thử lại sau 2 giây...")
                if self.client: self.client.close()
                self.client = None
                time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    node = SocketBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
