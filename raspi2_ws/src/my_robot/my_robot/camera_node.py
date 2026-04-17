from openni import openni2
import numpy as np
import cv2
import socket
import struct
import pickle
import time

# ===== INIT CAMERA =====
openni2.initialize("/home/pi-robot/OpenNI-Linux-Arm64-2.3/Redist")
dev = openni2.Device.open_any()

depth_stream = dev.create_depth_stream()
depth_stream.start()

cap = cv2.VideoCapture(0)

# ===== SOCKET SERVER =====
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('0.0.0.0', 9999))
server_socket.listen(1)

print("Waiting for laptop...")
conn, addr = server_socket.accept()
print("Connected:", addr)

# ===== MAIN LOOP =====
while True:
    try:
        # ===== DEPTH =====
        depth_frame = depth_stream.read_frame()
        depth_data = depth_frame.get_buffer_as_uint16()
        depth_img = np.frombuffer(depth_data, dtype=np.uint16).reshape((480, 640))

        # ===== RGB =====
        ret, frame = cap.read()
        if not ret:
            continue

        # ===== NÉN RGB =====
        _, rgb_buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        rgb_bytes = rgb_buffer.tobytes()

        # ===== NÉN DEPTH =====
        depth_norm = cv2.convertScaleAbs(depth_img, alpha=0.03)
        _, depth_buffer = cv2.imencode('.jpg', depth_norm, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        depth_bytes = depth_buffer.tobytes()

        # ===== PACK DATA =====
        data = (rgb_bytes, depth_bytes)
        packet = pickle.dumps(data)

        # ===== GỬI =====
        conn.sendall(struct.pack("!I", len(packet)) + packet)

        # ===== LIMIT FPS =====
        time.sleep(0.03)  # ~30 FPS

    except Exception as e:
        print("Error / Disconnected:", e)
        break

conn.close()
server_socket.close()
