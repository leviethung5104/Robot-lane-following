#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from openni import openni2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.bridge = CvBridge()

        self.pub_rgb = self.create_publisher(Image, '/camera/rgb', 10)
        self.pub_depth = self.create_publisher(Image, '/camera/depth', 10)

        # ================== RGB ==================
        self.cap = cv2.VideoCapture(0)

        # ================== DEPTH (Astra Pro) ==================
        openni2.initialize()
        self.dev = openni2.Device.open_any()

        self.depth_stream = self.dev.create_depth_stream()
        self.depth_stream.start()

        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        # ================== RGB ==================
        ret, frame = self.cap.read()
        if ret:
            rgb_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.pub_rgb.publish(rgb_msg)

        # ================== DEPTH ==================
        frame = self.depth_stream.read_frame()
        frame_data = frame.get_buffer_as_uint16()

        depth_array = np.frombuffer(frame_data, dtype=np.uint16)
        depth_array = depth_array.reshape((480, 640))

        # ===== xử lý nhiễu =====
        depth_array[depth_array == 0] = 9999

        # ===== lấy vùng phía trước =====
        roi = depth_array[200:280, 300:340]
        valid = roi[roi < 5000]

        if len(valid) > 0:
            distance = int(np.mean(valid))
        else:
            distance = 9999

        # DEBUG
        self.get_logger().info(f"Distance: {distance} mm")

        # publish depth (giữ nguyên 16UC1)
        depth_msg = self.bridge.cv2_to_imgmsg(depth_array, encoding='passthrough')
        self.pub_depth.publish(depth_msg)

    def destroy_node(self):
        self.depth_stream.stop()
        openni2.unload()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()