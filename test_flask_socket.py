from flask import Flask, render_template
from flask_socketio import SocketIO
import logging
import sys

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", logger=True, engineio_logger=True)

logging.basicConfig(level=logging.DEBUG, stream=sys.stdout)
logger = logging.getLogger(__name__)

@app.route('/functionality-added-v2')
def index():
    logger.debug("Index route accessed")
    return render_template('test_flask.html')

@socketio.on_error_default
def default_error_handler(e):
    logger.error(f'An error occurred: {str(e)}')
    logger.exception(e)

@socketio.on('connect')
def test_connect():
    logger.debug("Client connected")

@socketio.on('disconnect')
def test_disconnect():
    logger.debug("Client disconnected")

if __name__ == '__main__':
    logger.info("Starting the application")
    try:
        socketio.run(app, debug=True, host='0.0.0.0', port=5000)
    except Exception as e:
        logger.error(f"Failed to run the application: {str(e)}")
        logger.exception(e)