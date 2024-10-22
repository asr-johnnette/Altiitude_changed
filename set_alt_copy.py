import logging
import time
from pymavlink import mavutil

logging.basicConfig(
    filename='altitude_change.log',  # Log file name
    level=logging.DEBUG,               # Log level
    format='%(asctime)s - %(levelname)s - %(message)s'  # Log message format
)
class ALT():
    def __init__(self):
        ip = '127.0.0.1'
        port = '14550'
        connection_string = f"tcp:{ip}:{port}"
        self.master = mavutil.mavlink_connection(connection_string)
        
        logging.info("Connecting to vehicle...")
        
        self.check_system_status()
        self.set_altitude(100)  # Change to your desired altitude

    
    def check_system_status(self):
        logging.info("Waiting for heartbeat...")
        while True:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True)
            if msg:
                logging.info(f"System mode: {msg.base_mode}, System status: {msg.system_status}")
                if msg.system_status == mavutil.mavlink.MAV_STATE_ACTIVE:
                    logging.info("The vehicle is active and ready to receive commands.")
                    break
                else:
                    logging.info(f"Vehicle status: {msg.system_status}, awaiting readiness.")
                    time.sleep(1)
    
    def set_altitude(self, altitude):
        logging.info(f"Attempting to change altitude to: {altitude} meters.")

        # Use MAV_CMD_DO_REPOSITION to change only altitude, keeping current lat/lon and yaw
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT,
            0,                    # Confirmation
            0,                    # Bitmask (0 for default behavior)
            float('NaN'),        # Loiter radius (ignored)
            float('NaN'),        # Yaw (NaN to maintain current yaw)
            float('NaN'),        # Latitude (NaN to keep current position)
            float('NaN'),        # Longitude (NaN to keep current position)
            altitude            # Altitude in meters
        )

        logging.info("Altitude change command sent, waiting for acknowledgment...")
        
        # Wait for command acknowledgment
        timeout = time.time() + 10  # Set a timeout of 10 seconds
        while time.time() < timeout:
            ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
            if ack_msg:
                logging.info(f"Received command acknowledgment: {ack_msg}")
                if ack_msg.command == mavutil.mavlink.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT:
                    if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        logging.info(f"Altitude changed to {altitude} meters successfully.")
                        break
                    else:
                        logging.error(f"Failed to change altitude: {ack_msg.result} (Error Code: {ack_msg.result})")
                        if ack_msg.result == mavutil.mavlink.MAV_RESULT_DENIED:
                            logging.error("The command was denied. Check altitude limits and flight mode.")
                        break
        else:
            logging.warning("Timeout waiting for altitude change acknowledgment.")

if __name__ == '__main__':
    alt = ALT()