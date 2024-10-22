from pymavlink import mavutil
import time

# Connection settings
ip = '127.0.0.1'
port = '14550'
connection_string = f"tcp:{ip}:{port}"

# Connect to the vehicle
master = mavutil.mavlink_connection(connection_string)
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received.")
print(f"Target system: {master.target_system}, Target component: {master.target_component}")

# Set the target component to 1 manually (autopilot)
master.target_component = 1
print(f"Setting Target component to: {master.target_component}")

# Check System Status (HEARTBEAT)
def check_system_status():
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg:
            print(f"System mode: {msg.base_mode}, System status: {msg.system_status}")
            if msg.system_status == mavutil.mavlink.MAV_STATE_ACTIVE:
                print("The vehicle is active and ready to receive commands.")
                break
            else:
                print(f"Vehicle status: {msg.system_status}, awaiting readiness.")
                time.sleep(1)

# Set Mode (Switch between available modes like GUIDED, STABILIZE, etc.)
def set_mode(mode='TAKEOFF'):
    mode_mapping = master.mode_mapping()
    if mode not in mode_mapping:
        print(f"Mode {mode} not found in mode mapping.")
        return

    mode_id = mode_mapping[mode]
    print(f"Changing mode to: {mode}, mode ID: {mode_id}")

    # Set the base mode with the custom mode flag enabled
    base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED

    # Send the MAVLink command to change the mode
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        base_mode,    # Base mode with custom mode flag enabled
        mode_id,      # The desired mode ID (custom mode)
        0, 0, 0, 0, 0
    )

    # Wait for mode acknowledgement
    timeout = time.time() + 10  # Set timeout of 10 seconds
    while time.time() < timeout:
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        if ack_msg and ack_msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"Mode {mode} set successfully.")
                break
            else:
                print(f"Failed to set mode: {ack_msg.result}")
                break
    else:
        print("Timeout waiting for mode change acknowledgment.")


# Main process
if __name__ == "__main__":
    check_system_status()

    # Try switching the mode
    set_mode('TAKEOFF')