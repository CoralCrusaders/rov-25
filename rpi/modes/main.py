import typing
import threading
import serial
import nmea_encode_c_ext.nmea_encode as nmea_encode
import time
import serial.tools.list_ports
import socket
import select

HOST = '192.168.68.12'
PORT = 10110  # standard NMEA port

TEENSY_4_1_PIPE = '/dev/ttyACM0'
GAMEPAD_NUM_INPUTS = 18
SERIAL_BAUD_RATE = 115200

# Global variables for storing state
onboard_data = ""
gamepad_inputs = [0] * GAMEPAD_NUM_INPUTS

# Setup serial connection to Teensy
try:
    ser = serial.Serial(TEENSY_4_1_PIPE, SERIAL_BAUD_RATE, timeout=0.1)
    ser.flush()
    print(f"Connected to Teensy at {TEENSY_4_1_PIPE}")
except Exception as e:
    print(f"Error connecting to Teensy: {e}")
    exit(1)

# List available serial ports for debugging
for p in serial.tools.list_ports.comports():
    print(f"Found port: {p}")

# Setup socket connection to topside computer
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("Socket created")
try:
    s.connect((HOST, PORT))
    # Making the socket blocking for this approach
    s.setblocking(1)
    print("Socket connected to topside")
except Exception as e:
    print(f"Socket connection failed: {e}")
    exit(1)

# Map between gamepad codes and array indices
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

def process_topside_data():
    """Process incoming data from topside and forward to Teensy"""
    try:
        # Get message length (first 4 bytes)
        datalen_bytes = s.recv(4)
        if not datalen_bytes:  # Connection closed
            print("Connection to topside lost")
            return False
            
        datalen = int(datalen_bytes.decode('ASCII'))
        
        # Read the actual message
        topside_data = s.recv(datalen).decode('ASCII')
        chunks = topside_data.split(",")
        
        # Validate NMEA header
        if chunks[0] != "$CTCTL":
            print(f"Invalid NMEA header from topside: {chunks[0]}")
            return True
            
        # Process all gamepad inputs
        for chunk in chunks[1:]:
            if ":" in chunk:
                code, state = chunk.split(":")
                if code in gamepad_map:
                    gamepad_inputs[gamepad_map[code]] = int(state)
        
        # Convert to tuple and encode NMEA for Teensy
        gamepad_input_tuple = tuple(gamepad_inputs)
        nmea_bytes = nmea_encode.nmea_encode(gamepad_input_tuple)
        
        # Send to Teensy immediately
        if nmea_bytes:
            ser.write(nmea_bytes)
            
        return True
        
    except ConnectionResetError:
        print("Connection to topside reset")
        return False
    except Exception as e:
        print(f"Error processing topside data: {e}")
        return True  # Try to continue

def process_teensy_data():
    """Process incoming data from Teensy and forward to topside"""
    try:
        # Check if data is available from Teensy
        if ser.in_waiting > 0:
            try:
                # Read and decode a line from Teensy
                data = ser.readline().decode('ASCII').rstrip()
                if data:
                    print(f"From Teensy: {data}")
                    
                    # Forward to topside immediately
                    nmea = data
                    len_str = str(len(nmea)).zfill(4)
                    transmission = (len_str + nmea).encode('ASCII')
                    s.send(transmission)
                    
            except UnicodeDecodeError:
                print("Unicode decode error from Teensy")
                # Send error message to topside
                nmea = "$RPCTL,unicode_error,*FF"
                len_str = str(len(nmea)).zfill(4)
                transmission = (len_str + nmea).encode('ASCII')
                s.send(transmission)
                
        return True
                
    except Exception as e:
        print(f"Error processing Teensy data: {e}")
        return True  # Try to continue

def main():
    """Main function that uses select to monitor both socket and serial"""
    try:
        print("Monitoring for data from topside and Teensy...")
        running = True
        
        while running:
            # Use select to monitor both socket and serial port
            readable, _, _ = select.select([s], [], [], 0.1)
            
            # Check for socket data (from topside)
            if s in readable:
                running = process_topside_data()
                
            # Always check for serial data (from Teensy)
            process_teensy_data()
                
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    except Exception as e:
        print(f"Unexpected error in main loop: {e}")
    finally:
        print("Closing connections")
        try:
            ser.close()
            s.close()
        except:
            pass

if __name__ == "__main__":
    main()