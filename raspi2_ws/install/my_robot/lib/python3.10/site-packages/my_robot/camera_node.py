import socket, struct, pickle, cv2
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

# ===== SOCKET =====
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('0.0.0.0', 9999))
server.listen(1)

print("Waiting...")
conn, addr = server.accept()
print("Connected:", addr)

conn.setblocking(False) 
current_bbox = None

while True:
    # ===== RGB =====
    ret, frame = cap.read()
    if not ret:
        continue

    _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
    rgb_bytes = buffer.tobytes()

    # ===== SEND RGB =====
    conn.sendall(struct.pack("!I", len(rgb_bytes)) + rgb_bytes)

    # ===== RECEIVE BBOX (Không bị treo) =====
    try:
        # Nhận đúng 16 bytes (Tương ứng 4 số nguyên từ struct của bridge_node)
        bbox_data = conn.recv(16)
        if len(bbox_data) == 16:
            x1, y1, x2, y2 = struct.unpack("!4i", bbox_data)
            if x1 == -1:
                current_bbox = None # Laptop báo không có vật cản
            else:
                current_bbox = (x1, y1, x2, y2)
    except BlockingIOError:
        pass # Chưa có dữ liệu Bbox mới gửi tới, bỏ qua để chạy tiếp
    except Exception as e:
        pass

    # ===== DEPTH LOCAL =====
    depth_frame = depth_stream.read_frame()
    depth = np.frombuffer(
        depth_frame.get_buffer_as_uint16(),
        dtype=np.uint16
    ).reshape((480, 640))

    depth = cv2.resize(depth, (160, 120))

    if current_bbox is not None:
        x1, y1, x2, y2 = current_bbox
        x1, y1 = max(0, x1-10), max(0, y1-10)
        x2, y2 = min(160, x2+10), min(120, y2+10)

        roi = depth[y1:y2, x1:x2]
        if roi.size > 0:
            roi_valid = roi[roi > 0]
            if roi_valid.size > 0:
                distance = np.min(roi_valid) # Lấy điểm gần nhất của vật cản
                print("Distance:", distance, "mm")
                conn.sendall(struct.pack("!f", float(distance)))
            else:
                conn.sendall(struct.pack("!f", 9999.0))
        else:
            conn.sendall(struct.pack("!f", 9999.0))
    else:
        # Không có vật cản
        conn.sendall(struct.pack("!f", 9999.0))
        
        
