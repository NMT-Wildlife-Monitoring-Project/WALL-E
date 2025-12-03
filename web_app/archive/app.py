#!/usr/bin/python3
from flask import Flask, render_template, Response, jsonify
# import gps
import cv2
import time
import netifaces
import numpy as np
import signal
import sys

app = Flask(__name__)

# === Camera Setup ===
def init_camera(index=0):
    cam = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cam.isOpened():
        print("Warning: Could not open camera.")
        return None

    for _ in range(10):
        success, _ = cam.read()
        if success:
            print("Camera initialized.")
            return cam
        time.sleep(0.2)

    print("Warning: Camera opened but not returning frames.")
    cam.release()
    return None

camera = init_camera()

# === GPS Data ===

def get_gps_data(timeout=2.0):
    # GPS temporarily disabled
    return {
        'latitude': None,
        'longitude': None,
        'fix': False,
        'satellites': None,
        'timestamp': None
    }
    
    # print("Getting GPS data...")
    # data = {
    #     'latitude': None,
    #     'longitude': None,
    #     'fix': False,
    #     'satellites': None,
    #     'timestamp': None
    # }
    # start_time = time.time()
    # try:
    #     session = gps.gps(mode=gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)
    #     for report in session:
    #         # Check for timeout
    #         if time.time() - start_time > timeout:
    #             print("GPS timeout reached.")
    #             break

    #         if report.get('class') == 'TPV':
    #             data['latitude'] = getattr(report, 'lat', None)
    #             data['longitude'] = getattr(report, 'lon', None)
    #             data['timestamp'] = getattr(report, 'time', None)
    #             data['fix'] = True if getattr(report, 'mode', 0) >= 2 else False

    #         elif report.get('class') == 'SKY':
    #             data['satellites'] = len(report.get('satellites', []))

    #         # Stop early if we have a valid fix
    #         if data['fix'] and data['latitude'] is not None and data['longitude'] is not None:
    #             break

    # except Exception as e:
    #     print(f"GPS error: {e}")

    # return data

# === Video Stream Generator ===
def generate_camera_frames(framerate=15):
    if camera is None:
        while True:
            frame = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(frame, "No Camera", (70, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            ret, buffer = cv2.imencode('.jpg', frame)
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
            time.sleep(1.0 / framerate)
    
    while True:
        success, frame = camera.read()
        if not success:
            continue

        frame = cv2.resize(frame, (320, 240))
        _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 30])
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(1.0 / framerate)

# === Map Stream Generator - DISABLED ===
# def generate_map_frames(framerate=5):
#     while True:
#         map_image = cv2.imread('/tmp/shared/map_stream.jpg')
#         if map_image is None:
#             print("Map image not found.")
#             time.sleep(1.0 / framerate)
#             continue

#         _, buffer = cv2.imencode('.jpg', map_image, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
#         yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
#         time.sleep(1.0 / framerate)

# === Flask Routes ===
@app.route('/')
def index():
    gps_data = get_gps_data(timeout=2.0)
    return render_template('index.html', gps=gps_data)

@app.route('/video_feed')
def video_feed():
    return Response(generate_camera_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# @app.route('/map_feed')
# def map_feed():
#     return Response(generate_map_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/gps_status')
def gps_status():
    data = get_gps_data(timeout=2.0)
    return jsonify(data)

# === Cleanup ===
def shutdown_handler(sig, frame):
    print("Shutting down. Releasing camera.")
    if camera is not None:
        camera.release()
    sys.exit(0)

signal.signal(signal.SIGINT, shutdown_handler)
signal.signal(signal.SIGTERM, shutdown_handler)

# === Start App ===
if __name__ == "__main__":
    print("Starting Flask app on all interfaces (0.0.0.0:5000)...")
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
