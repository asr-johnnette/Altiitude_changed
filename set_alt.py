from pymavlink import mavutil
import time 


class ALT():
    def __init__(self):
        ip = '127.0.0.1'
        port = '14550'
        connection_string = f"tcp:{ip}:{port}"
        self.master = mavutil.mavlink_connection(connection_string)
        self.check_system_status()

        # Fetch and print the current mode before changing to AUTO
        current_mode = self.get_current_mode()
        if current_mode:
            print(f"Previous flight mode: {current_mode}")

        self.set_mode("TAKEOFF")

    
    def check_system_status(self):
        while True:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True)
            if msg:
                print(f"System mode: {msg.base_mode}, System status: {msg.system_status}")
                if msg.system_status == mavutil.mavlink.MAV_STATE_ACTIVE:
                    print("The vehicle is active and ready to receive commands.")
                    break
                else:
                    print(f"Vehicle status: {msg.system_status}, awaiting readiness.")
                    time.sleep(1)
    
    def set_mode(self, mode):
        mode_mapping = self.master.mode_mapping()
        if mode not in mode_mapping:
            print(f"Mode {mode} not found in mode mapping.")
            return

        mode_id = mode_mapping[mode]
        print(f"Changing mode to: {mode}, mode ID: {mode_id}")

        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            base_mode,    # Base mode with custom mode flag enabled
            mode_id,      # The desired mode ID (custom mode)
            0, 0, 0, 0, 0
        )

        # Wait for acknowledgment
        timeout = time.time() + 10  # Set timeout of 10 seconds
        while time.time() < timeout:
            ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
            if ack_msg and ack_msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print(f"Mode {mode} set successfully.")
                    break
                else:
                    print(f"Failed to set mode: {ack_msg.result}")
                    break
        else:
            print("Timeout waiting for mode change acknowledgment.")


    def get_current_mode(self):
        # Fetch the current mode from the HEARTBEAT message
        print("Fetching current flight mode...")
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True)
        if msg:
            base_mode = msg.base_mode
            custom_mode = msg.custom_mode

            # Convert base_mode and custom_mode to a readable mode name
            mode_mapping = self.master.mode_mapping()
            for mode_name, mode_id in mode_mapping.items():
                if custom_mode == mode_id:
                    return mode_name
        
        return None


if __name__ == '__main__':
    alt = ALT()