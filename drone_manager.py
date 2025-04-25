import threading
import time
import json
from pymavlink import mavutil
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class DroneManager:
    def __init__(self):
        self.vehicle = None
        self.connection_string = 'udpin:127.0.0.1:14550'
        self.drone_data = {}
        self.running = False

    def connect(self):
        try:
            self.vehicle = mavutil.mavlink_connection(self.connection_string)
            self.vehicle.wait_heartbeat()
            logger.info("Connected to vehicle")
            return True
        except Exception as e:
            logger.error(f"Connection failed: {str(e)}")
            return False

    def fetch_data(self):
        while self.running:
            try:
                msg = self.vehicle.recv_match(blocking=True, timeout=1.0)
                if msg:
                    msg_type = msg.get_type()
                    msg_dict = msg.to_dict()
                    self.drone_data[msg_type] = msg_dict
            except Exception as e:
                logger.error(f"Error fetching data: {str(e)}")
            time.sleep(0.1)

    def start(self):
        if self.connect():
            self.running = True
            threading.Thread(target=self.fetch_data, daemon=True).start()

    def stop(self):
        self.running = False
        if self.vehicle:
            self.vehicle.close()

    def get_data(self):
        return json.dumps(self.drone_data)

    def send_command(self, command, *args):
        if not self.vehicle:
            return {"error": "Vehicle not connected"}

        try:
            func = getattr(self.vehicle.mav, command)
            func(*args)
            return {"message": f"{command} command sent successfully"}
        except AttributeError:
            return {"error": f"Command {command} not found"}
        except Exception as e:
            return {"error": f"Error executing command: {str(e)}"}


drone_manager = DroneManager()