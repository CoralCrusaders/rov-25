import socket
from inputs import get_gamepad
import time
import threading
import typing

HOST = '192.168.68.12'
PORT = 10110  # standard NMEA port

NMEA_HEADER = "$CTCTL"  # computer topside control
GAMEPAD_NUM_INPUTS = 18
TRANSMIT_RATE = 0.033  # ~30Hz (33ms)

# Socket setup
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("Socket created")
s.bind((HOST, PORT))
print("Socket bound")
s.listen()
print("Listening for connection")
conn, addr = s.accept()
print("Connection address:", addr)

# Global storage for gamepad state
gamepad_inputs = [0] * GAMEPAD_NUM_INPUTS
gamepad_lock = threading.Lock()

# PIDFS tuning parameters
depth_rate_p = 0.3
depth_rate_i = 0.0
depth_rate_d = 0.0
depth_rate_f = 0.0
depth_rate_s = 0.0
depth_pwr_p = 1.0
depth_pwr_i = 0.0
depth_pwr_d = 0.0
depth_pwr_f = 0.0
depth_pwr_s = 0.0
yaw_rate_p = 0.7
yaw_rate_i = 0.0
yaw_rate_d = 0.0
yaw_rate_f = 0.0
yaw_rate_s = 0.0
yaw_pwr_p = 0.003
yaw_pwr_i = 0.0
yaw_pwr_d = 0.0
yaw_pwr_f = 0.0
yaw_pwr_s = 0.0

# Mapping between gamepad events and array indices
gamepad_map = {
    "ABS_X": 0,      # i16, LSX
    "ABS_Y": 1,      # i16, LSY
    "ABS_RX": 2,     # i16, RSX
    "ABS_RY": 3,     # i16, RSY
    "BTN_THUMBL": 4, # 0, 1, LSB
    "BTN_THUMBR": 5, # 0, 1, RSB
    "ABS_HAT0X": 6,  # -1, 0, 1, DPadX
    "ABS_HAT0Y": 7,  # -1, 0, 1, DPadY
    "BTN_SOUTH": 8,  # 0, 1, A
    "BTN_EAST": 9,   # 0, 1, B
    "BTN_NORTH": 10, # 0, 1, X
    "BTN_WEST": 11,  # 0, 1, Y
    "BTN_TL": 12,    # 0, 1, LB
    "BTN_TR": 13,    # 0, 1, RB
    "ABS_Z": 14,     # u8, LT
    "ABS_RZ": 15,    # u8, RT
    "BTN_START": 16, # 0, 1, Start
    "BTN_SELECT": 17 # 0, 1, Back
}

def monitor_gamepad():
    """Thread to continuously monitor gamepad events and update state array"""
    print("Gamepad monitoring started")
    while True:
        try:
            events = get_gamepad()
            with gamepad_lock:
                for event in events:
                    if event.code in gamepad_map and event.code != "SYN_REPORT":
                        gamepad_inputs[gamepad_map[event.code]] = event.state
        except Exception as e:
            print(f"Gamepad error: {e}")
            time.sleep(0.1)

def monitor_socket_input():
    """Thread to receive data from RPi and display it"""
    print("Socket monitoring started")
    while True:
        try:
            # Get message length (first 4 bytes)
            datalen = int(conn.recv(4).decode('ASCII'))
            # Read the actual message
            data = conn.recv(datalen).decode('ASCII')
            print(f"From RPi: {data}")
            
            # Parse sensor data for display if needed
            if data.startswith("$TNCTL,"):
                values = data.split(",")[1:-1]  # Skip header and checksum
                if len(values) >= 20:  # Ensure we have enough values
                    print(f"Depth: {values[9]}m, Yaw: {values[11]}°, Pitch: {values[12]}°, Roll: {values[13]}°")
        except Exception as e:
            # Skip if no data or error
            time.sleep(0.01)

def transmit_gamepad():
    """Thread to periodically send gamepad state to RPi"""
    print("Transmission thread started")
    time.sleep(1)  # Initial delay to ensure connection is ready
    
    while True:
        try:
            # Build NMEA string with PIDFS parameters and current gamepad state
            nmea = NMEA_HEADER
            
            # Add PIDFS parameters
            nmea += f",{depth_rate_p},{depth_rate_i},{depth_rate_d},{depth_rate_f},{depth_rate_s}"
            nmea += f",{depth_pwr_p},{depth_pwr_i},{depth_pwr_d},{depth_pwr_f},{depth_pwr_s}"
            nmea += f",{yaw_rate_p},{yaw_rate_i},{yaw_rate_d},{yaw_rate_f},{yaw_rate_s}"
            nmea += f",{yaw_pwr_p},{yaw_pwr_i},{yaw_pwr_d},{yaw_pwr_f},{yaw_pwr_s}"
            
            # Add gamepad data
            with gamepad_lock:
                for code, index in gamepad_map.items():
                    nmea += f",{code}:{gamepad_inputs[index]}"
            
            # Prepare and send message with length prefix
            len_str = str(len(nmea)).zfill(4)
            transmission = (len_str + nmea).encode('ASCII')
            conn.send(transmission)
            
        except Exception as e:
            print(f"Transmission error: {e}")
        
        # Sleep to maintain transmission rate
        time.sleep(TRANSMIT_RATE)

def handle_keys():
    """Thread to handle keyboard inputs for PIDFS adjustments"""
    print("Press 'd' to display current PIDFS values")
    print("Press 'q' to quit")
    # Add more key commands for adjusting PIDFS values

    while True:
        cmd = input("> ")
        if cmd == 'q':
            print("Exiting...")
            exit(0)
        elif cmd == 'd':
            print(f"Depth Rate: P={depth_rate_p}, I={depth_rate_i}, D={depth_rate_d}, F={depth_rate_f}, S={depth_rate_s}")
            print(f"Depth Power: P={depth_pwr_p}, I={depth_pwr_i}, D={depth_pwr_d}, F={depth_pwr_f}, S={depth_pwr_s}")
            print(f"Yaw Rate: P={yaw_rate_p}, I={yaw_rate_i}, D={yaw_rate_d}, F={yaw_rate_f}, S={yaw_rate_s}")
            print(f"Yaw Power: P={yaw_pwr_p}, I={yaw_pwr_i}, D={yaw_pwr_d}, F={yaw_pwr_f}, S={yaw_pwr_s}")
        # Add more commands for adjusting individual parameters

def main():
    try:
        # Start gamepad monitoring thread
        gamepad_thread = threading.Thread(target=monitor_gamepad, daemon=True)
        gamepad_thread.start()
        
        # Start receiving thread
        receive_thread = threading.Thread(target=monitor_socket_input, daemon=True)
        receive_thread.start()
        
        # Start keyboard input thread for PIDFS tuning
        key_thread = threading.Thread(target=handle_keys, daemon=True)
        key_thread.start()
        
        # Run transmission function in main thread
        transmit_gamepad()
        
    except KeyboardInterrupt:
        print("Program terminated by user")
    except Exception as e:
        print(f"Error in main: {e}")
    finally:
        try:
            conn.close()
            s.close()
        except:
            pass

if __name__ == "__main__":
    main()