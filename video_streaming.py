import cv2
import os
import subprocess
import time
import threading
import signal
import logging
from queue import Queue
from flask import Flask, Response
import argparse

# Configure logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.WARNING)
handler = logging.StreamHandler()
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

# Global variables
RTSP_URL = "rtsp://192.168.144.25:8554/video1"  # Default RTSP URL, can be changed
frame_queue = Queue(maxsize=10)
video_thread_running = False
video_thread = None
ffmpeg_process = None
app = Flask(__name__)


class FFmpegVideoCapture:
    """Custom video capture class that uses FFmpeg to process the RTSP stream"""

    def __init__(self, rtsp_url):
        self.rtsp_url = rtsp_url
        self.pipe = None
        self.cap = None
        self.is_opened = False
        self.connect()

    def connect(self):
        global ffmpeg_process

        # Start FFmpeg if not running
        if not ffmpeg_process:
            if not start_ffmpeg_processor():
                logger.error("Failed to start FFmpeg")
                return False

        try:
            # Use OpenCV's VideoCapture to read from the FFmpeg pipe
            self.cap = cv2.VideoCapture('pipe:{}'.format(ffmpeg_process.stdout.fileno()), cv2.CAP_FFMPEG)
            self.is_opened = self.cap.isOpened()
            return self.is_opened
        except Exception as e:
            logger.error(f"Error connecting FFmpegVideoCapture: {str(e)}")
            return False

    def isOpened(self):
        return self.is_opened

    def read(self):
        if not self.is_opened:
            return False, None

        try:
            ret, frame = self.cap.read()
            if not ret:
                self.reconnect()
                return False, None
            return ret, frame
        except Exception as e:
            logger.error(f"Error reading frame: {str(e)}")
            self.reconnect()
            return False, None

    def reconnect(self):
        self.release()
        # Force restart of FFmpeg
        stop_ffmpeg_processor()
        time.sleep(1)
        self.connect()

    def release(self):
        if self.cap:
            self.cap.release()
            self.cap = None
        self.is_opened = False


def start_ffmpeg_processor():
    """Start FFmpeg process to process thermal RTSP stream"""
    global ffmpeg_process, RTSP_URL

    # Kill any existing FFmpeg processes
    stop_ffmpeg_processor()

    try:
        # FFmpeg command using parameters from reference
        ffmpeg_cmd = [
            "ffmpeg",
            "-y",  # Overwrite output without asking
            "-rtsp_transport", "tcp",  # Use TCP for input
            "-i", RTSP_URL,  # Input from RTSP URL
            "-an",  # Disable audio
            "-c:v", "libx264",  # Use H.264 codec
            "-pix_fmt", "yuv420p",  # Set pixel format
            "-preset", "ultrafast",  # Use ultrafast preset
            "-tune", "zerolatency",  # Tune for zero latency
            "-r", "15",  # Force output framerate
            "-f", "mpegts",  # Output format MPEG-TS
            "pipe:1"  # Output to stdout
        ]

        # Start FFmpeg process with piping
        ffmpeg_process = subprocess.Popen(
            ffmpeg_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            bufsize=10 ** 8  # Large buffer
        )

        logger.info(f"Started FFmpeg processor: {' '.join(ffmpeg_cmd)}")

        # Start a thread to log FFmpeg error output
        def log_ffmpeg_errors():
            while ffmpeg_process and ffmpeg_process.poll() is None:
                try:
                    line = ffmpeg_process.stderr.readline().decode('utf-8', errors='ignore').strip()
                    if line and ("error" in line.lower() or "warning" in line.lower()):
                        logger.warning(f"FFmpeg: {line}")
                except Exception as e:
                    pass

        error_thread = threading.Thread(target=log_ffmpeg_errors)
        error_thread.daemon = True
        error_thread.start()

        # Allow time for FFmpeg to start processing
        time.sleep(2)

        return True
    except Exception as e:
        logger.error(f"Failed to start FFmpeg processor: {str(e)}")
        return False


def stop_ffmpeg_processor():
    """Stop FFmpeg process if running"""
    global ffmpeg_process

    if ffmpeg_process:
        try:
            ffmpeg_process.terminate()
            ffmpeg_process.wait(timeout=5)
            logger.info("FFmpeg processor terminated")
        except Exception as e:
            logger.error(f"Error stopping FFmpeg: {str(e)}")
            try:
                ffmpeg_process.kill()
                logger.info("FFmpeg processor forcefully killed")
            except:
                pass
        finally:
            ffmpeg_process = None


def get_rtsp_frame():
    """Generator function to yield frames from the RTSP stream"""
    global RTSP_URL, ffmpeg_process

    while True:  # Infinite loop to keep trying
        cap = None
        try:
            # Create our custom FFmpeg-based capture
            cap = FFmpegVideoCapture(RTSP_URL)

            if not cap.isOpened():
                logger.warning("Failed to open camera, retrying...")
                if cap:
                    cap.release()

                time.sleep(2)
                continue

            frame_count = 0

            # Read frames in a loop
            while True:
                ret, frame = cap.read()

                if not ret or frame is None:
                    logger.warning("Failed to read frame, reconnecting...")
                    break

                frame_count += 1

                # Skip first few frames as they might be corrupt
                if frame_count <= 5:
                    continue

                # Yield the frame for processing
                yield frame

        except Exception as e:
            logger.error(f"Camera error: {str(e)}")

        finally:
            if cap:
                cap.release()

        # Short delay before retry
        time.sleep(1)


def video_capture_thread():
    """Thread function to capture frames using FFmpeg pipe"""
    global video_thread_running, RTSP_URL, ffmpeg_process, frame_queue

    while video_thread_running:
        cap = None
        try:
            # Use our custom capture
            cap = FFmpegVideoCapture(RTSP_URL)

            if not cap.isOpened():
                logger.warning("Failed to open camera in thread, retrying...")
                if cap:
                    cap.release()
                time.sleep(2)
                continue

            frame_count = 0

            while video_thread_running:
                ret, frame = cap.read()

                if not ret or frame is None:
                    logger.warning("Failed to read frame in thread, reconnecting...")
                    break

                frame_count += 1

                # Skip first few frames
                if frame_count <= 5:
                    continue

                # If queue is full, remove oldest frame
                if frame_queue.full():
                    try:
                        frame_queue.get_nowait()
                    except:
                        pass

                # Add new frame
                frame_queue.put(frame)

                # Small delay to prevent CPU overuse
                time.sleep(0.033)

        except Exception as e:
            logger.error(f"Camera error in thread: {str(e)}")
            time.sleep(1)
        finally:
            if cap:
                cap.release()


def start_video_thread():
    """Start the video capture thread if not already running"""
    global video_thread, video_thread_running

    if not video_thread or not video_thread.is_alive():
        video_thread_running = True
        video_thread = threading.Thread(target=video_capture_thread)
        video_thread.daemon = True
        video_thread.start()
        logger.info("Video capture thread started")


def stop_video_thread():
    """Stop the video capture thread"""
    global video_thread_running
    video_thread_running = False
    logger.info("Video capture thread stopping")


def generate_frames():
    """Generate frames from the queue instead of directly from camera"""
    global frame_queue

    # Start video thread if not running
    start_video_thread()

    while True:
        try:
            # Get frame with timeout
            frame = frame_queue.get(timeout=0.5)

            ret, buffer = cv2.imencode('.jpg', frame)
            if not ret:
                continue

            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

        except Exception as e:
            # If queue is empty or error occurs, yield empty frame
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + b'' + b'\r\n')
            time.sleep(0.1)  # Prevent tight loop


def cleanup():
    """Clean up resources"""
    stop_video_thread()
    stop_ffmpeg_processor()
    logger.info("Video streaming cleanup completed")


def set_rtsp_url(url):
    """Set the RTSP URL"""
    global RTSP_URL
    RTSP_URL = url
    logger.info(f"RTSP URL set to {url}")

    # Restart processing
    stop_ffmpeg_processor()
    return True


@app.route('/')
def index():
    """Video streaming home page."""
    return """
    <html>
      <head>
        <title>Thermal Video Stream</title>
        <style>
          body {{ font-family: Arial, sans-serif; margin: 0; padding: 20px; text-align: center; }}
          h1 {{ color: #333; }}
          .video-container {{ margin: 20px auto; max-width: 800px; }}
          img {{ width: 100%; border: 1px solid #ddd; box-shadow: 0 0 10px rgba(0,0,0,0.1); }}
        </style>
      </head>
      <body>
        <h1>Thermal Video Stream</h1>
        <div class="video-container">
          <img src="/video_feed" />
        </div>
        <p>RTSP URL: {}</p>
      </body>
    </html>
    """.format(RTSP_URL)


@app.route('/video_feed')
def video_feed():
    """Video streaming route."""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Thermal Video Streaming Server')
    parser.add_argument('--url', type=str, default=RTSP_URL,
                        help='RTSP URL of the thermal camera')
    parser.add_argument('--port', type=int, default=5001,
                        help='Port to run the streaming server on')
    parser.add_argument('--host', type=str, default='0.0.0.0',
                        help='Host to run the streaming server on')
    parser.add_argument('--debug', action='store_true',
                        help='Run in debug mode')

    args = parser.parse_args()

    # Update RTSP URL if provided
    if args.url != RTSP_URL:
        set_rtsp_url(args.url)

    # Set up logging
    if args.debug:
        logger.setLevel(logging.DEBUG)
        logger.info("Debug logging enabled")

    print(f"Starting video streaming server on http://{args.host}:{args.port}")
    print(f"Streaming from RTSP URL: {RTSP_URL}")
    print("Press Ctrl+C to quit")

    try:
        # Start the streaming server
        app.run(host=args.host, port=args.port, debug=args.debug, threaded=True)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        cleanup()