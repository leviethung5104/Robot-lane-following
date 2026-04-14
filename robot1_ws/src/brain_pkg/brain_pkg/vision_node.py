#!/usr/bin/env python3
import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        
        # Đăng ký nhận ảnh từ camera màu
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 1)
        
        # Các kênh xuất dữ liệu (Publishers)
        self.bev_pub = self.create_publisher(Image, '/bev_image', 10)
        self.raw_pub = self.create_publisher(Image, '/raw_image', 10)
        self.obs_center_pub = self.create_publisher(Point, '/obs_center_raw', 10) 
        
        self.get_logger().info("⏳ Đang khởi tạo Vision Node với luồng camera ROS...")

        # Nạp mô hình AI nhận diện làn đường và vật cản
        self.model = YOLO("best.pt", task='segment') 
        dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        self.model.predict(dummy_frame, verbose=False)
        self.get_logger().info("✅ AI Nhận diện (Segmentation) đã sẵn sàng!")

        # Ma trận biến đổi không gian sang góc nhìn từ trên xuống (Bird's Eye View)
        self.M = cv2.getPerspectiveTransform(
            np.float32([[151, 227], [465, 226], [637, 410], [7, 419]]), 
            np.float32([[0, 0], [480, 0], [480, 480], [0, 480]])
        )

        self.is_processing = False

    def image_callback(self, msg):
        if self.is_processing: return
        self.is_processing = True

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            results = self.model.predict(frame, conf=0.5, verbose=False)[0]
            
            bev_canvas = np.zeros((480, 640, 3), dtype=np.uint8) 
            # Tạo overlay để tô màu phân vùng (Segmentation)
            seg_overlay = np.zeros_like(frame) 
            
            # Biến z = 0 (Chưa rõ), 1 (Bên TRÁI nét đứt), 2 (Bên PHẢI nét đứt)
            obs_center_msg = Point()
            obs_center_msg.x = -1.0 
            obs_center_msg.y = -1.0
            obs_center_msg.z = 0.0  

            obs_bbox = None

            if results.masks is not None:
                masks = results.masks.data.cpu().numpy()
                classes = results.boxes.cls.cpu().numpy().astype(int)
                boxes = results.boxes.xyxy.cpu().numpy().astype(int)

                for i, mask in enumerate(masks):
                    cls_id = classes[i]
                    mask_resized = cv2.resize(mask, (640, 480))
                    binary_mask = (mask_resized > 0.5).astype(np.uint8)

                    if cls_id == 1: # Nét liền
                        bev_canvas[binary_mask == 1] = (0, 255, 0)
                        seg_overlay[binary_mask == 1] = (0, 255, 0)
                        
                    elif cls_id == 0: # Nét đứt
                        bev_canvas[binary_mask == 1] = (255, 0, 0)
                        seg_overlay[binary_mask == 1] = (255, 0, 0)
                        
                    elif cls_id == 2: # Vật cản
                        x1, y1, x2, y2 = boxes[i]
                        obs_bbox = (x1, y1, x2, y2)
                        
                        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                        obs_center_msg.x = float(cx)
                        obs_center_msg.y = float(cy)
                        
                        bx, by = int((x1 + x2) / 2), int(y2)
                        cv2.circle(bev_canvas, (bx, by), 10, (0, 0, 255), -1) 
                        seg_overlay[binary_mask == 1] = (0, 0, 255)

            # --- TRỘN ẢNH VÀ HIỂN THỊ KẾT QUẢ NHẬN DIỆN ---
            seg_img = cv2.addWeighted(frame, 0.7, seg_overlay, 0.5, 0)
            if obs_bbox is not None:
                x1, y1, x2, y2 = obs_bbox
                cv2.rectangle(seg_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(seg_img, "vat_can", (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # Cửa sổ hiển thị ảnh gốc với lớp màng phân vùng
            cv2.imshow("Goc nhin AI (Phan vung)", seg_img)
            cv2.waitKey(1)

            # --- TÍNH TOÁN VỊ TRÍ LÀN CỦA VẬT CẢN TRÊN ẢNH GỐC ---
            if obs_center_msg.x != -1.0:
                x1, y1, x2, y2 = obs_bbox
                bx, by = int((x1 + x2) / 2), int(y2) # bx, by là tọa độ điểm dưới cùng (đáy) của vật cản
                
                # Quét một dải ngang vùng đáy vật cản (+-15 pixel) trên mask gốc để tìm nét đứt
                dashed_x = None
                for y_offset in range(-15, 16, 3):
                    check_y = by + y_offset
                    if 0 <= check_y < 480:
                        # bev_canvas đang lưu màu xanh dương (Kênh B > 200) cho nét đứt
                        blue_mask = (bev_canvas[check_y, :, 0] > 200) & (bev_canvas[check_y, :, 1] < 50)
                        xs = np.nonzero(blue_mask)[0]
                        if xs.size > 0:
                            dashed_x = np.mean(xs)
                            break # Nếu tìm thấy nét đứt thì dừng quét ngang
                
                # Gán làn cho vật cản dựa vào vị trí của nó so với nét đứt
                if dashed_x is not None:
                    if bx < dashed_x:
                        obs_center_msg.z = 1.0 # Vật cản nằm bên TRÁI nét đứt
                    else:
                        obs_center_msg.z = 2.0 # Vật cản nằm bên PHẢI nét đứt
            
            # --- CHUYỂN ĐỔI SANG BIRD'S EYE VIEW (Góc nhìn từ trên xuống) ---
            bev_colored = cv2.warpPerspective(bev_canvas, self.M, (480, 480))
                        
            # Truyền dữ liệu lên các topic ROS
            self.obs_center_pub.publish(obs_center_msg)
            self.raw_pub.publish(self.bridge.cv2_to_imgmsg(cv2.resize(frame, (320, 240)), "bgr8"))
            self.bev_pub.publish(self.bridge.cv2_to_imgmsg(bev_colored, "bgr8")) 

        except Exception as e:
            self.get_logger().error(f"❌ Lỗi Vision: {e}")
        finally:
            self.is_processing = False

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
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
