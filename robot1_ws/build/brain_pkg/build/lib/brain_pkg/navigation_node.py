#!/usr/bin/env python3
import os
os.environ["QT_QPA_PLATFORM"] = "xcb" 
from std_msgs.msg import Float32
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from std_msgs.msg import Float32MultiArray

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/bev_image', self.bev_callback, 10)
        self.create_subscription(Point, '/obs_center_raw', self.obs_center_callback, 10)
        self.create_subscription(Float32, '/obs_distance', self.distance_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.traj_pub = self.create_publisher(Float32MultiArray, '/trajectory_data', 10)
        
        # --- THAM SỐ CẤU HÌNH ---
        self.PX_PER_CM = 20             
        self.car_x = 240                
        self.car_y = 480                
        self.lookahead_y = self.car_y - int(5 * self.PX_PER_CM)  
        self.offset_lane = int(11 * self.PX_PER_CM)    
        self.filtered_target_x = 240.0         
        
        # Tâm quay của robot cách đáy camera 5cm (5cm * 20px = 100px)
        self.robot_y = self.car_y + int(12 * self.PX_PER_CM)
        width_px = int(20 * self.PX_PER_CM)
        height_px = int(5 * self.PX_PER_CM)
        self.stop_zone = (
            self.car_x - (width_px // 2),  
            self.car_y - height_px,        
            self.car_x + (width_px // 2),  
            self.car_y)
        
        self.normal_speed = 0.25
        self.current_depth_image = None
        self.obs_distance_meters = 999.0
        self.obs_center = None
        self.obs_lane = "CHUA_RO"
        
        # Biến trạng thái máy 
        self.current_state = "bam lan"
        self.car_lane_history = "CHUA_RO"
        self.evade_target_lane = "CHUA_RO"
        self.original_lane = "CHUA_RO"
        
        # Biến đếm thời gian mất làn
        self.last_lane_time = time.time()
        self.get_logger().info("🚀 Navigation Node: Đã cập nhật tính năng giữ tốc độ và chỉ chuyển làn ở nét đứt")

    '''
    def depth_callback(self, msg):
        try:
            self.current_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            pass
    '''
    
    def distance_callback(self, msg):
        self.obs_distance_meters = msg.data

    def obs_center_callback(self, msg):
        if msg.x == -1.0:
            self.obs_distance_meters = 999.0
            self.obs_center = None
            self.obs_lane = "CHUA_RO"
            return

        self.obs_center = (int(msg.x), int(msg.y))
        
        # Đọc làn vật cản từ vision gửi qua (z)
        if msg.z == 1.0:
            self.obs_lane = "TRAI"
        elif msg.z == 2.0:
            self.obs_lane = "PHAI"
        else:
            self.obs_lane = "CHUA_RO"
        
        '''
        # Tính khoảng cách bằng khung 5x5
        if self.current_depth_image is not None:
            h, w = self.current_depth_image.shape[:2]
            cx, cy = self.obs_center
            x_min, x_max = max(0, cx - 2), min(w, cx + 3)
            y_min, y_max = max(0, cy - 2), min(h, cy + 3)
            
            if x_min < x_max and y_min < y_max:
                roi_depth = self.current_depth_image[y_min:y_max, x_min:x_max]
                if self.current_depth_image.dtype == np.float32:
                    valid_mask = ~np.isnan(roi_depth) & (roi_depth > 0.0)
                    valid_depths = roi_depth[valid_mask]
                    self.obs_distance_meters = float(np.mean(valid_depths)) if valid_depths.size > 0 else 999.0
                else:
                    valid_depths = roi_depth[roi_depth > 0]
                    self.obs_distance_meters = float(np.mean(valid_depths)) / 1000.0 if valid_depths.size > 0 else 999.0
	'''

    def bev_callback(self, msg):
        try:
            bev_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return

        cmd_msg = Twist()
        mask_dashed = cv2.inRange(bev_bgr, (250, 0, 0), (255, 5, 5))   
        mask_solid = cv2.inRange(bev_bgr, (0, 250, 0), (5, 255, 5))    
        mask_obs = cv2.inRange(bev_bgr, (0, 0, 250), (5, 5, 255))      

        roi_dashed = mask_dashed[self.lookahead_y, :]
        roi_solid = mask_solid[self.lookahead_y, :]
        
        idx_dashed = np.nonzero(roi_dashed)[0]
        idx_solid = np.nonzero(roi_solid)[0]
        
        # Cập nhật thời gian nhận diện làn đường
        curr_time = time.time()
        if idx_dashed.size > 0 or idx_solid.size > 0:
            self.last_lane_time = curr_time
        
        # Tính thời gian đã trôi qua kể từ lần cuối thấy làn
        time_lost = curr_time - self.last_lane_time

        # 1. Xác định xe đang ở làn nào dựa vào vạch đứt
        current_car_lane = "CHUA_RO"
        if idx_dashed.size > 0:
            avg_dashed = np.mean(idx_dashed)
            current_car_lane = "TRAI" if self.car_x < avg_dashed else "PHAI"
            self.car_lane_history = current_car_lane # Lưu lại để nhớ
        else:
            current_car_lane = self.car_lane_history

        # Quét vùng dừng khẩn cấp
        roi_stop = mask_obs[self.stop_zone[1]:self.stop_zone[3], self.stop_zone[0]:self.stop_zone[2]]
        is_in_stop_zone = np.any(roi_stop) 

        # 2. STATE MACHINE QUYẾT ĐỊNH
        is_critical_danger = is_in_stop_zone or (0.0 < self.obs_distance_meters < 0.4)
        if time_lost > 0.5 or is_critical_danger:
            self.current_state = "dung lai"
        elif self.current_state == "dung lai":
            if not is_critical_danger and (self.obs_distance_meters > 0.5 or self.obs_distance_meters == 999.0):
                self.current_state = "bam lan"

        if self.current_state == "bam lan":
            if (0.6 <= self.obs_distance_meters <= 0.8 and 
                current_car_lane == self.obs_lane and 
                current_car_lane != "CHUA_RO"):
                
                if idx_dashed.size > 0:
                    self.current_state = "chuyen lan"
                    self.original_lane = current_car_lane # LƯU LẠI LÀN BAN ĐẦU
                    self.evade_target_lane = "PHAI" if current_car_lane == "TRAI" else "TRAI"
                    self.get_logger().warn(f"⚠️ Tránh vật cản -> SANG {self.evade_target_lane}")
                else:
                    self.get_logger().warn(f"⛔ Vạch nét liền -> Không chuyển làn!")
                
        elif self.current_state == "chuyen lan":
            is_safe_distance = (self.obs_distance_meters > 1.2 or self.obs_distance_meters == 999.0)
            if current_car_lane == self.evade_target_lane and is_safe_distance:
                self.current_state = "chuyen ve"
                self.get_logger().info(f"✅ Đã vượt. Chuẩn bị CHUYỂN VỀ {self.original_lane}")

        elif self.current_state == "chuyen ve":
            if current_car_lane == self.original_lane:
                self.current_state = "bam lan"
                self.get_logger().info("✅ Đã về làn gốc an toàn!")

        # 3. TÍNH TOÁN ĐIỂM ĐÍCH (Target X) DỰA TRÊN LÀN MỤC TIÊU
        active_lane_goal = self.evade_target_lane if self.current_state == "chuyen lan" else current_car_lane
        raw_target_x = self.car_x

        if active_lane_goal == "TRAI":
            if idx_dashed.size > 0: raw_target_x = np.mean(idx_dashed) - self.offset_lane
            elif idx_solid.size > 0: raw_target_x = np.mean(idx_solid) + self.offset_lane
        elif active_lane_goal == "PHAI":
            if idx_dashed.size > 0: raw_target_x = np.mean(idx_dashed) + self.offset_lane
            elif idx_solid.size > 0: raw_target_x = np.mean(idx_solid) - self.offset_lane
        else:
            if idx_dashed.size > 0: raw_target_x = np.mean(idx_dashed) - self.offset_lane if np.mean(idx_dashed) > 240 else np.mean(idx_dashed) + self.offset_lane
            
        raw_target_x = np.clip(raw_target_x, 0, 480)

        # Bộ lọc Low-Pass Filter (EMA) giúp vô lăng không bị vẩy cá khi nhiễu frame
        self.filtered_target_x = 0.8 * self.filtered_target_x + 0.2 * raw_target_x
        target_x = self.filtered_target_x

        # 4. TRUYỀN LỆNH DỰA THEO TRẠNG THÁI (FIX LƯỢN GẮT)
        if self.current_state == "dung lai":
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
        else:
            dx = target_x - self.car_x
            dy = self.robot_y - self.lookahead_y 
            sq_dist = dx*dx + dy*dy
            
            if sq_dist > 0:
                # Giảm hệ số K từ 1200 xuống 800 để xe lượn mềm mại hơn khi chuyển làn
                steering = (2 * dx / sq_dist) * self.normal_speed * 800.0
                cmd_msg.angular.z = np.clip(steering, -1.5, 1.5)
                
            # Giảm tốc độ tịnh tiến khi đang vào cua gắt để tránh trượt bánh (Drift)
            speed_factor = 1.0 - (0.3 * abs(cmd_msg.angular.z))
            cmd_msg.linear.x = max(0.12, self.normal_speed * speed_factor)

        # 5. điều khiển và quỹ đạo
        self.cmd_pub.publish(cmd_msg)
        
        traj_msg = Float32MultiArray()
        traj_msg.data = [float(target_x), float(self.car_x), float(cmd_msg.angular.z)]
        self.traj_pub.publish(traj_msg)

        # Hiển thị log
        log_warning = "[MAT LAN]" if time_lost > 0.5 else ""
        self.get_logger().info(
            f"[{self.current_state}] {log_warning} Làn: {current_car_lane} | Đích: {target_x:.1f} | "
            f"V: {cmd_msg.linear.x:.2f} | W: {cmd_msg.angular.z:.2f}")

        # --- DRAW BEV DEBUG ---
        cv2.circle(bev_bgr, (int(target_x), self.lookahead_y), 8, (0, 255, 255), -1) 
        cv2.line(bev_bgr, (0, self.lookahead_y), (480, self.lookahead_y), (255, 255, 255), 1)
        
        # Nếu mất làn > 0.5s thì nháy đỏ toàn bộ khung Stop Zone để cảnh báo
        color_zone = (0, 0, 255) if (is_in_stop_zone or time_lost > 0.5) else (0, 255, 0)
        cv2.rectangle(bev_bgr, (self.stop_zone[0], self.stop_zone[1]), 
                      (self.stop_zone[2], self.stop_zone[3]), color_zone, 2)
        
        # Vẽ Text 3 trạng thái lên góc màn hình BEV
        cv2.putText(bev_bgr, f"TRANG THAI: {self.current_state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        if time_lost > 0.5:
            cv2.putText(bev_bgr, "MAT LAN!", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Goc nhin BEV", bev_bgr)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
