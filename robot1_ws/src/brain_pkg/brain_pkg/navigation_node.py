#!/usr/bin/env python3
import os
os.environ["QT_QPA_PLATFORM"] = "xcb" 

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.bridge = CvBridge()
        
        self.create_subscription(Image, '/bev_image', self.bev_callback, 10)
        self.create_subscription(Point, '/obs_center_raw', self.obs_center_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.depth_debug_pub = self.create_publisher(Image, '/depth_debug_image', 10)
        
        # --- THAM SỐ CẤU HÌNH ---
        self.PX_PER_CM = 20             
        self.car_x = 240                
        self.car_y = 480                
        self.lookahead_y = self.car_y - int(5 * self.PX_PER_CM)  
        self.offset_lane = int(11 * self.PX_PER_CM)             
        
        # Tâm quay của robot cách đáy camera 5cm (5cm * 20px = 100px)
        self.robot_y = self.car_y + int(12 * self.PX_PER_CM)
        
        width_px = int(20 * self.PX_PER_CM)
        height_px = int(5 * self.PX_PER_CM)
        self.stop_zone = (
            self.car_x - (width_px // 2),  
            self.car_y - height_px,        
            self.car_x + (width_px // 2),  
            self.car_y                     
        )
        
        self.normal_speed = 0.25
        self.current_depth_image = None
        
        self.obs_distance_meters = 999.0
        self.obs_center = None
        self.obs_lane = "CHUA_RO"
        
        # Biến trạng thái máy (State Machine)
        self.current_state = "bam lan" # Mặc định bám làn
        self.car_lane_history = "CHUA_RO"
        self.evade_target_lane = "CHUA_RO"
        
        # Biến đếm thời gian mất làn
        self.last_lane_time = time.time()
        
        self.get_logger().info("🚀 Navigation Node: Đã cập nhật tính năng giữ tốc độ và chỉ chuyển làn ở nét đứt")

    def depth_callback(self, msg):
        try:
            self.current_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            pass

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
        # Ưu tiên cao nhất là Dừng khẩn cấp do Mất làn hoặc Vật cản
        if time_lost > 0.5:
            self.current_state = "dung lai"
        elif is_in_stop_zone or (0.0 < self.obs_distance_meters < 0.4):
            self.current_state = "dung lai"
        elif self.current_state == "dung lai":
            # Nếu vật cản đi khỏi vùng an toàn và xe ĐÃ NHÌN THẤY LẠI LÀN ĐƯỜNG
            self.current_state = "bam lan"

        if self.current_state == "bam lan":
            # Điều kiện kích hoạt chuyển làn: Cách 0.6m - 0.8m, trùng làn, và phải biết rõ làn hiện tại
            if (0.6 <= self.obs_distance_meters <= 0.8 and 
                current_car_lane == self.obs_lane and 
                current_car_lane != "CHUA_RO"):
                
                # CHỈ CHUYỂN LÀN NẾU PHÁT HIỆN NÉT ĐỨT (idx_dashed.size > 0)
                if idx_dashed.size > 0:
                    self.current_state = "chuyen lan"
                    self.evade_target_lane = "PHAI" if current_car_lane == "TRAI" else "TRAI"
                    self.get_logger().warn(f"⚠️ Vật cản ở làn {self.obs_lane}, cách {self.obs_distance_meters:.2f}m -> BẮT ĐẦU CHUYỂN LÀN SANG {self.evade_target_lane}")
                else:
                    self.get_logger().warn(f"⛔ Vật cản phía trước nhưng vạch ngăn là NÉT LIỀN -> Không chuyển làn, chuẩn bị dừng khẩn cấp!")
                    # Xe tiếp tục trạng thái "bam lan" và sẽ bị bắt buộc "dung lai" khi khoảng cách < 0.4m
                
        elif self.current_state == "chuyen lan":
            # Điều kiện kết thúc chuyển làn: Đã sang hẳn làn bên kia VÀ vật cản đã an toàn
            is_safe_distance = (self.obs_distance_meters > 1.2 or self.obs_distance_meters == 999.0)
            if current_car_lane == self.evade_target_lane and is_safe_distance:
                self.current_state = "bam lan"
                self.get_logger().info("✅ Đã vượt và ổn định ở làn mới. Trở lại bám làn.")

        # 3. TÍNH TOÁN ĐIỂM ĐÍCH (Target X) CHỈ QUA VẠCH ĐỨT
        active_lane_goal = self.evade_target_lane if self.current_state == "chuyen lan" else current_car_lane
        target_x = self.car_x

        if active_lane_goal == "TRAI":
            if idx_dashed.size > 0: target_x = np.mean(idx_dashed) - self.offset_lane
            elif idx_solid.size > 0: target_x = np.mean(idx_solid) + self.offset_lane
        elif active_lane_goal == "PHAI":
            if idx_dashed.size > 0: target_x = np.mean(idx_dashed) + self.offset_lane
            elif idx_solid.size > 0: target_x = np.mean(idx_solid) - self.offset_lane
        else:
            # Fallback nếu mất hoàn toàn dữ liệu
            if idx_dashed.size > 0 and idx_solid.size > 0:
                target_x = (np.mean(idx_dashed) + np.mean(idx_solid)) / 2
            elif idx_dashed.size > 0:
                avg_d = np.mean(idx_dashed)
                target_x = avg_d - self.offset_lane if avg_d > self.car_x else avg_d + self.offset_lane

        target_x = np.clip(target_x, 0, 480)

        # 4. TRUYỀN LỆNH DỰA THEO TRẠNG THÁI
        if self.current_state == "dung lai":
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = 0.0
        else:
            # CẬP NHẬT: Giữ nguyên tốc độ (self.normal_speed) khi chuyển làn thay vì chia 2
            cmd_msg.linear.x = self.normal_speed
            
            # TÍNH PURE PURSUIT TỪ TÂM ROBOT THỰC TẾ
            dx = target_x - self.car_x
            dy = self.robot_y - self.lookahead_y 
            sq_dist = dx*dx + dy*dy
            
            if sq_dist > 0:
                steering = (2 * dx / sq_dist) * cmd_msg.linear.x * 1200.0
                cmd_msg.angular.z = np.clip(steering, -1.5, 1.5)

        # Lựa chọn hiển thị log
        log_warning = "[MAT LAN]" if time_lost > 0.5 else ""
        self.get_logger().info(
            f"[{self.current_state}] {log_warning} Làn xe: {current_car_lane} | Vật cản: {self.obs_lane} ({self.obs_distance_meters:.2f}m) | "
            f"V: {cmd_msg.linear.x:.2f} m/s | W: {cmd_msg.angular.z:.2f} rad/s"
        )
        self.cmd_pub.publish(cmd_msg)

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
        
        # --- DRAW DEPTH JET ---
        if self.current_depth_image is not None:
            if self.current_depth_image.dtype == np.float32:
                depth_mm = self.current_depth_image * 1000.0
                depth_mm = np.nan_to_num(depth_mm, nan=0.0)
            else:
                depth_mm = self.current_depth_image.astype(np.float32)

            depth_8bit = cv2.convertScaleAbs(depth_mm, alpha=255.0/4500.0)
            depth_color = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)
            depth_color[depth_mm == 0] = [0, 0, 0] 
            
            if self.obs_center is not None and self.obs_distance_meters != 999.0:
                cx, cy = self.obs_center
                cv2.rectangle(depth_color, (cx - 2, cy - 2), (cx + 2, cy + 2), (0, 255, 0), 1)
                cv2.circle(depth_color, (cx, cy), 1, (0, 0, 255), -1)
                cv2.putText(depth_color, f"{self.obs_distance_meters:.2f}m", 
                            (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                            
            cv2.imshow("Camera Chieu Sau ROS", depth_color)
            
            try:
                self.depth_debug_pub.publish(self.bridge.cv2_to_imgmsg(depth_color, encoding="bgr8"))
            except Exception:
                pass

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
