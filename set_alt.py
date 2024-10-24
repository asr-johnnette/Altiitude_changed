from PyQt5.QtWidgets import QApplication, QVBoxLayout, QWidget, QPushButton, QDoubleSpinBox, QMessageBox
from pymavlink import mavutil
import sys
import time

class MAVLinkCommunication:
    def __init__(self):
        ip = '127.0.0.1'
        port = '14550'
        connection_string = f"tcp:{ip}:{port}"
        try:
            # Initialize connection
            self.master = mavutil.mavlink_connection(connection_string)
            self.master.wait_heartbeat()  # Wait for heartbeat to ensure communication is established
            print("Heartbeat received successfully.")
        except Exception as e:
            print(f"Failed to connect: {e}")
            raise  # Raise exception to handle connection issues

    def set_new_wp_alt(self, altitude):
        try:
            # Get the current mission count
            lat, lon = 0, 0  # Placeholder for actual location data
            seq = 0  # Adjust the sequence as needed
            frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT

            # Send the MAVLink command to change altitude
            self.master.mav.mission_item_send(
                self.master.target_system,           # Target system ID
                self.master.target_component,        # Target component ID
                seq,                                 # Sequence (which waypoint)
                frame,                               # Frame type
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, # Command type (waypoint)
                2,                                   # Current (2 = set as current waypoint)
                1,                                   # Autocontinue
                0, 0, 0, 0,                          # Parameters for heading, speed, etc.
                lat,                                 # Latitude
                lon,                                 # Longitude
                altitude                             # Altitude (new altitude)
            )

            print(f"Waypoint altitude command sent: {altitude}")

            # Wait for COMMAND_ACK to confirm the command
            timeout = time.time() +50  # Wait for 10 seconds max
            while time.time() < timeout:
                ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
                if ack_msg:
                    if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print(f"Command acknowledged: {ack_msg}")
                        return True
                    else:
                        print(f"Command rejected: {ack_msg}")
                        return False
            raise TimeoutError("No acknowledgment received within the timeout period.")

        except Exception as e:
            print(f"Failed to set altitude: {e}")
            raise  # Raise exception to signal failure

class AltitudeSetter(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Change Altitude")

        # MAVLink communication setup
        try:
            self.mavlink_comm = MAVLinkCommunication()
            self.connection_status = True
        except Exception:
            QMessageBox.critical(self, "Connection Error", "Failed to establish MAVLink connection.")
            self.connection_status = False

        # UI setup: altitude input and button
        self.altitudeSpinBox = QDoubleSpinBox(self)
        self.altitudeSpinBox.setRange(0, 1000)  # Minimum and maximum altitude
        self.altitudeSpinBox.setDecimals(1)
        self.altitudeSpinBox.setValue(100)  # Default altitude

        self.modifyButton = QPushButton("Change Alt", self)
        self.modifyButton.clicked.connect(self.modify_and_set_alt)

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.altitudeSpinBox)
        layout.addWidget(self.modifyButton)
        self.setLayout(layout)

    def modify_and_set_alt(self):
        if not self.connection_status:
            QMessageBox.critical(self, "Error", "No MAVLink connection. Can't set altitude.")
            return

        new_alt = self.altitudeSpinBox.value()  # Get altitude value from the spinbox
        try:
            # Send new altitude via MAVLink and wait for acknowledgment
            if self.mavlink_comm.set_new_wp_alt(new_alt):
                QMessageBox.information(self, "Success", f"New altitude {new_alt} set successfully.")
            else:
                QMessageBox.warning(self, "Failed", f"Failed to set new altitude {new_alt}.")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to set altitude: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    alt_setter = AltitudeSetter()
    alt_setter.show()
    sys.exit(app.exec_())
