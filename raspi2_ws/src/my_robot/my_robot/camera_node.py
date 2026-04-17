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
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# ===== SOCKET SERVER =====
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(('0.0.0.0', 9999))
server_socket.listen(1)

print("Waiting for laptop...")

while True:
    conn, addr = server_socket.accept()
    print("Connected:", addr)

    try:
        while True:
            # ===== DEPTH RAW =====
            depth_frame = depth_stream.read_frame()
            depth_data = depth_frame.get_buffer_as_uint16()

            depth_img = np.frombuffer(depth_data, dtype=np.uint16).reshape((480, 640))

            # resize nhẹ lại cho đỡ nặng
            depth_img = cv2.resize(depth_img, (320, 240))

            # ===== RGB =====
            ret, frame = cap.read()
            if not ret:
                continue

            # ===== NÉN RGB =====
            _, rgb_buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            rgb_bytes = rgb_buffer.tobytes()

            # ===== DEPTH RAW BYTES =====
            depth_bytes = depth_img.tobytes()

            # ===== PACK =====
            data = (rgb_bytes, depth_bytes, depth_img.shape)
            packet = pickle.dumps(data)

            # ===== GỬI =====
            conn.sendall(struct.pack("!I", len(packet)) + packet)

            time.sleep(0.03)  # ~30 FPS

    except Exception as e:
        print("Disconnected:", e)
        conn.close()
