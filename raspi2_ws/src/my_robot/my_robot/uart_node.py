#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class UARTNode(Node):
    def __init__(self):
        super().__init__('uart_node')
        
        # ===== Serial =====
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
            time.sleep(2)  # chờ Arduino reset
            self.get_logger().info("Đã kết nối UART thành công!")
        except Exception as e:
            self.get_logger().error(f"Lỗi kết nối Serial: {e}")
            exit(1)

        # ===== Tham số =====
        self.max_linear = 1.0   # m/s (giới hạn an toàn)
        self.max_angular = 2.0  # rad/s

        # ===== Subscriber =====
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # ===== Timer gửi định kỳ (tránh spam không kiểm soát) =====
        self.last_msg = Twist()
        self.timer = self.create_timer(0.05, self.send_data)  # 20Hz

    def cmd_callback(self, msg):
        # Lưu lại message mới nhất
        self.last_msg = msg

    def send_data(self):
        v = self.last_msg.linear.x
        w = self.last_msg.angular.z

        # ===== Giới hạn =====
        v = max(min(v, self.max_linear), -self.max_linear)
        w = max(min(w, self.max_angular), -self.max_angular)

        # ===== Format dữ liệu =====
        # dạng: v(m/s),w(rad/s)
        data = f"{v:.3f},{w:.3f}\n"

        try:
            if self.ser.is_open:
                self.ser.write(data.encode())
                # Debug nhẹ (bật khi cần)
                # self.get_logger().info(f"Sent: {data.strip()}")
        except Exception as e:
            self.get_logger().error(f"Lỗi khi gửi dữ liệu: {e}")

    def destroy_node(self):
        # ===== Dừng robot an toàn =====
        try:
            if self.ser.is_open:
                self.ser.write("0.0,0.0\n".encode())
                time.sleep(0.1)
                self.ser.close()
        except:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UARTNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()