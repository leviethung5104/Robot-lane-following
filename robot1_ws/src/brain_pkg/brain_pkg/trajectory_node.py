#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import numpy as np

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')
        self.create_subscription(Float32MultiArray, '/trajectory_data', self.data_callback, 10)
        
        # Lưu trữ dữ liệu
        self.history_len = 100 # Hiển thị 100 điểm gần nhất
        self.target_xs = []
        self.actual_xs = []
        self.steerings = []
        
        # Cấu hình biểu đồ
        plt.ion() # Chế độ vẽ interactive (thời gian thực)
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 6))
        self.fig.canvas.manager.set_window_title('Đánh giá thuật toán Pure Pursuit')
        
    def data_callback(self, msg):
        target_x = msg.data[0]
        actual_x = msg.data[1]
        steering = msg.data[2]
        
        self.target_xs.append(target_x)
        self.actual_xs.append(actual_x)
        self.steerings.append(steering)
        
        # Cắt bớt mảng để không tràn RAM
        if len(self.target_xs) > self.history_len:
            self.target_xs.pop(0)
            self.actual_xs.pop(0)
            self.steerings.pop(0)
            
        self.update_plot()

    def update_plot(self):
        # Biểu đồ 1: So sánh đường mục tiêu và đường thực tế
        self.ax1.clear()
        self.ax1.plot(self.target_xs, label='Đường Tính Toán (Target)', color='blue', linestyle='--')
        self.ax1.plot(self.actual_xs, label='Đường Thực Tế Xe Đi', color='red')
        self.ax1.set_ylim([0, 480]) # Theo kích thước ảnh BEV
        self.ax1.set_title('Quỹ Đạo Trục X (Pixel)')
        self.ax1.legend(loc='upper right')
        self.ax1.grid(True)

        # Biểu đồ 2: Xem góc bẻ lái để biết xe có bị giật/lắc không
        self.ax2.clear()
        self.ax2.plot(self.steerings, label='Góc bẻ lái (W)', color='green')
        self.ax2.set_ylim([-2.0, 2.0])
        self.ax2.set_title('Tín Hiệu Điều Khiển Lái')
        self.ax2.legend(loc='upper right')
        self.ax2.grid(True)

        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()
    try:
        # Vòng lặp spin được thay thế bằng pause của matplotlib để không kẹt luồng
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
