import cv2
from flask import Flask, Response

app = Flask(__name__)

def generate_frames():
    cap = cv2.VideoCapture(0)  # /dev/video0

    if not cap.isOpened():
        raise RuntimeError("Failed to open camera.")

    while True:
        success, frame = cap.read()
        if not success:
            break
        # Encode frame to JPEG
        _, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()

        # Yield as MJPEG
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

    cap.release()

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return '''
    <html>
        <head><title>ROS 2 Camera Stream</title></head>
        <body>
            <h1>Live Camera Feed</h1>
            <img src="/video_feed" width="640" height="480">
        </body>
    </html>
    '''

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
