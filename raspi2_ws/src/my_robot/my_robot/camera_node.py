import socket, struct, cv2, time
import numpy as np
from openni import openni2

# ===== INIT =====
openni2.initialize("/home/pi-robot/OpenNI-Linux-Arm64-2.3/Redist")
dev = openni2.Device.open_any()

depth_stream = dev.create_depth_stream()
depth_stream.start()

cap = cv2.VideoCapture(0)
cap.set(3, 160)
cap.set(4, 120)

# ===== SOCKET SERVER =====
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Chống lỗi Address in use
server.bind(('0.0.0.0', 9999))
server.listen(1)

print("🚀 Server Camera Pi đang chạy. Đợi kết nối từ Laptop...")

TARGET_FPS = 10
FRAME_TIME = 1.0 / TARGET_FPS

while True: 
    try:
        conn, addr = server.accept()
        print(f"✅ Đã kết nối với Laptop: {addr}")
        conn.setblocking(False) 
        
        current_bbox = None
        recv_buffer = bytearray() # BỘ ĐỆM XỬ LÝ LỖI MẠNG CHẬP CHỜN

        while True: # VÒNG LẶP INNER: TRUYỀN DỮ LIỆU
            start_time = time.time()
            
            ret, frame = cap.read()
            if not ret: continue

            # 1. Nén ảnh RGB
            _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            rgb_bytes = buffer.tobytes()

            try:
                # GỬI RGB
                conn.sendall(struct.pack("!I", len(rgb_bytes)) + rgb_bytes)

                # 2. NHẬN BBOX (Sử dụng Persistent Buffer)
                try:
                    # Chỉ lấy số lượng byte còn thiếu để đủ 16 bytes
                    chunk = conn.recv(16 - len(recv_buffer))
                    if chunk:
                        recv_buffer.extend(chunk)
                    
                    # Khi đã gom đủ 16 bytes -> Giải mã và xóa buffer
                    if len(recv_buffer) == 16:
                        x1, y1, x2, y2 = struct.unpack("!4i", recv_buffer)
                        if x1 == -1:
                            current_bbox = None
                        else:
                            current_bbox = (x1, y1, x2, y2)
                        recv_buffer.clear() # Dọn dẹp để hứng gói tiếp theo
                        
                except BlockingIOError:
                    pass # Chưa có dữ liệu mạng gửi tới, bỏ qua để camera không bị kẹt

                # 3. XỬ LÝ DEPTH
                depth_frame = depth_stream.read_frame()
                depth = np.frombuffer(depth_frame.get_buffer_as_uint16(), dtype=np.uint16).reshape((480, 640))
                depth = cv2.resize(depth, (160, 120), interpolation=cv2.INTER_NEAREST)

                # Đồng bộ 999.0 với logic của navigation_node
                distance = 999.0 
                if current_bbox is not None:
                    x1, y1, x2, y2 = current_bbox
                    x1, y1 = max(0, x1-10), max(0, y1-10)
                    x2, y2 = min(160, x2+10), min(120, y2+10)

                    roi = depth[y1:y2, x1:x2]
                    if roi.size > 0:
                        roi_valid = roi[roi > 0]
                        if roi_valid.size > 0:
                            distance = float(np.median(roi_valid))

                # 4. GỬI DISTANCE
                conn.sendall(struct.pack("!f", distance))

            except (ConnectionResetError, BrokenPipeError):
                print("❌ Laptop đã ngắt kết nối.")
                break # Thoát vòng lặp inner để dọn dẹp
                
            # 5. GIỚI HẠN FPS BẢO VỆ CPU
            elapsed = time.time() - start_time
            if elapsed < FRAME_TIME:
                time.sleep(FRAME_TIME - elapsed)
                
        # Dọn dẹp dứt điểm tài nguyên cũ trước khi mở kết nối mới
        conn.close()
        print("🔄 Đã dọn dẹp Socket. Đang chờ kết nối mới...\n")
                
    except Exception as e:
        print(f"⚠️ Lỗi Server: {e}")
        # Đảm bảo đóng cả socket trong trường hợp văng exception lạ
        if 'conn' in locals() and conn:
            try: conn.close()
            except: pass