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
        except Exception as e:
            # Skip if no data or error
            time.sleep(0.01)

def transmit_gamepad():
    """Thread to periodically send gamepad state to RPi"""
    print("Transmission thread started")
    time.sleep(1)  # Initial delay to ensure connection is ready
    
    while True:
        try:
            # Build NMEA string with current gamepad state
            nmea = NMEA_HEADER
            
            # Lock while copying the current state
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

def main():
    try:
        # Start gamepad monitoring thread
        gamepad_thread = threading.Thread(target=monitor_gamepad, daemon=True)
        gamepad_thread.start()
        
        # Start receiving thread
        receive_thread = threading.Thread(target=monitor_socket_input, daemon=True)
        receive_thread.start()
        
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