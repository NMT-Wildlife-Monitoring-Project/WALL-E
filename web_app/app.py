#!/usr/bin/python
from flask import Flask, render_template, Response
import gps
import cv2

app = Flask(__name__)

# Setup the camera (0 = first detected camera)
camera = cv2.VideoCapture(0)

def get_gps_data():
    try:
        # Connect to the gpsd daemon
        session = gps.gps(mode=gps.WATCH_ENABLE | gps.WATCH_NEWSTYLE)

        for report in session:
            if report['class'] == 'TPV':
                latitude = getattr(report, 'lat', None)
                longitude = getattr(report, 'lon', None)
                if latitude is not None and longitude is not None:
                    return {'latitude': latitude, 'longitude': longitude}
    except Exception as e:
        print(f"GPS error: {e}")

    return {'latitude': 0.0, 'longitude': 0.0}

# New function to generate video frames
def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            # OPTIONAL: Resize the frame smaller for faster LTE streaming
            frame = cv2.resize(frame, (320, 240))  # Resize to 320x240

            # Encode the frame as JPEG with more compression
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 30]  # 0-100, lower = more compressed
            ret, buffer = cv2.imencode('.jpg', frame, encode_param)
            frame = buffer.tobytes()

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            
@app.route('/')
def index():
    gps_data = get_gps_data()
    return render_template('index.html', gps=gps_data)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)
