from PyQt5.QtWidgets import QApplication, QVBoxLayout, QWidget, QPushButton, QDoubleSpinBox, QMessageBox
import sys

class AltitudeSetter(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Change Altitude")

        # Create a spinbox for altitude input
        self.altitudeSpinBox = QDoubleSpinBox(self)
        self.altitudeSpinBox.setRange(0, 1000)  # Minimum and maximum altitude
        self.altitudeSpinBox.setDecimals(1)
        self.altitudeSpinBox.setValue(100)  # Default altitude

        # Create a button to apply the change
        self.modifyButton = QPushButton("Change Alt", self)
        self.modifyButton.clicked.connect(self.modify_and_set_alt)

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.altitudeSpinBox)
        layout.addWidget(self.modifyButton)
        self.setLayout(layout)

    def modify_and_set_alt(self):
        # Get the altitude value from the spinbox
        new_alt = self.altitudeSpinBox.value()

        # Send the new altitude value via MAVLink (pseudo-communication code)
        try:
            self.set_new_wp_alt(new_alt)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to communicate: {e}")

    def set_new_wp_alt(self, new_alt):
        # Placeholder for MAVLink communication
        print(f"Setting new waypoint altitude: {new_alt}")

        # You would call the MAVLink communication code here, like:
        # mavlink_command.send_waypoint(altitude=new_alt)


from pymavlink import mavutil

class MAVLinkCommunication:
    def __init__(self, connection_string="udp:localhost:14550"):
        # Initialize connection
        self.master = mavutil.mavlink_connection(connection_string)
        self.master.wait_heartbeat()

    def set_new_wp_alt(self, altitude):
        # Create and send MAVLink waypoint message
        lat, lon = 0, 0  # Placeholder for actual location data
        seq = 1  # Waypoint index, for example
        frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT

        self.master.mav.mission_item_send(
            self.master.target_system,
            self.master.target_component,
            seq,
            frame,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            2,  # Set as current
            1,  # Autocontinue
            0, 0, 0, 0,  # Parameters (like heading, speed)
            lat,
            lon,
            altitude  # Altitude
        )

        print(f"New waypoint altitude set: {altitude}")



class AltitudeSetter(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Change Altitude")

        # MAVLink communication
        self.mavlink_comm = MAVLinkCommunication()

        # UI setup (as before)
        self.altitudeSpinBox = QDoubleSpinBox(self)
        self.altitudeSpinBox.setRange(0, 1000)
        self.altitudeSpinBox.setDecimals(1)
        self.altitudeSpinBox.setValue(100)

        self.modifyButton = QPushButton("Change Alt", self)
        self.modifyButton.clicked.connect(self.modify_and_set_alt)

        layout = QVBoxLayout()
        layout.addWidget(self.altitudeSpinBox)
        layout.addWidget(self.modifyButton)
        self.setLayout(layout)

    def modify_and_set_alt(self):
        new_alt = self.altitudeSpinBox.value()
        try:
            # Send new altitude via MAVLink
            self.mavlink_comm.set_new_wp_alt(new_alt)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to communicate: {e}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    alt_setter = AltitudeSetter()
    alt_setter.show()
    sys.exit(app.exec_())
