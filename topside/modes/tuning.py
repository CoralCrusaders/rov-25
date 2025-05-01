import socket
from inputs import get_gamepad
import time
import threading
import typing
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from collections import deque

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

# Data storage for plotting - using deques for efficient appending and fixed max length
HISTORY_LENGTH = 500  # Number of data points to keep in memory
# Time data
timestamps = deque(maxlen=HISTORY_LENGTH)
# Depth control data
depth_actual = deque(maxlen=HISTORY_LENGTH)
depth_target = deque(maxlen=HISTORY_LENGTH)
depth_rate_actual = deque(maxlen=HISTORY_LENGTH)
depth_rate_target = deque(maxlen=HISTORY_LENGTH)
# Yaw control data
yaw_actual = deque(maxlen=HISTORY_LENGTH)
yaw_target = deque(maxlen=HISTORY_LENGTH)
yaw_rate_actual = deque(maxlen=HISTORY_LENGTH)
yaw_rate_target = deque(maxlen=HISTORY_LENGTH)

# Plotting setup - interactive mode
plt.ion()
fig, axs = plt.subplots(2, 2, figsize=(14, 8))
fig.canvas.manager.set_window_title('ROV PID Control Visualization')
fig.tight_layout(pad=3.0)

# Initialize plot lines
depth_lines = [axs[0,0].plot([], [], label=label)[0] for label in ['Actual Depth', 'Target Depth']]
depth_rate_lines = [axs[0,1].plot([], [], label=label)[0] for label in ['Actual Depth Rate', 'Target Depth Rate']]
yaw_lines = [axs[1,0].plot([], [], label=label)[0] for label in ['Actual Yaw', 'Target Yaw']]
yaw_rate_lines = [axs[1,1].plot([], [], label=label)[0] for label in ['Actual Yaw Rate', 'Target Yaw Rate']]

# Configure subplot properties
for i, title in enumerate(['Depth Control', 'Depth Rate Control', 'Yaw Control', 'Yaw Rate Control']):
    row, col = i // 2, i % 2
    axs[row, col].set_title(title)
    axs[row, col].grid(True)
    axs[row, col].legend(loc='upper right')

axs[0,0].set_ylabel('Depth (m)')
axs[0,1].set_ylabel('Depth Rate (m/s)')
axs[1,0].set_ylabel('Yaw (degrees)')
axs[1,1].set_ylabel('Yaw Rate (deg/s)')

for ax in axs.flatten():
    ax.set_xlabel('Time (s)')

# Setup time display window - show last X seconds of data
DISPLAY_WINDOW = 10  # seconds

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
    initial_timestamp = None
    
    while True:
        try:
            # Get message length (first 4 bytes)
            datalen = int(conn.recv(4).decode('ASCII'))
            # Read the actual message
            data = conn.recv(datalen).decode('ASCII')
            
            # Parse sensor data for visualization if needed
            if data.startswith("$TNCTL,"):
                values = data.split(",")  # Header, values, and checksum
                
                # Ignore if we don't have enough values or if the format is wrong
                if len(values) < 24:  # Need at least up through the millis value
                    continue
                
                try:
                    # Extract data from message based on the Teensy's transmit_rov_data format
                    # From the Teensy code, we know where each variable is located
                    curr_depth = float(values[9])           # sensor_data.depth_m
                    curr_depth_rate = float(values[10])     # sensor_data.depth_m_s
                    curr_yaw = float(values[11])            # sensor_data.yaw_deg
                    curr_yaw_rate = float(values[14])       # sensor_data.yaw_deg_s
                    
                    # PID targets (from indices seen in transmit_rov_data)
                    curr_depth_target = float(values[17])   # depth_rate_controller.target
                    curr_depth_rate_target = float(values[18]) # depth_pwr_controller.target 
                    curr_yaw_target = float(values[19])      # yaw_target
                    curr_yaw_rate_target = float(values[20]) # yaw_pwr_controller.target
                    
                    # Get millis and convert to seconds
                    millis = int(values[21])
                    if initial_timestamp is None:
                        initial_timestamp = millis
                    
                    time_sec = (millis - initial_timestamp) / 1000.0
                    
                    # Store data for plotting
                    timestamps.append(time_sec)
                    depth_actual.append(curr_depth)
                    depth_target.append(curr_depth_target)
                    depth_rate_actual.append(curr_depth_rate)
                    depth_rate_target.append(curr_depth_rate_target)
                    yaw_actual.append(curr_yaw)
                    yaw_target.append(curr_yaw_target)
                    yaw_rate_actual.append(curr_yaw_rate)
                    yaw_rate_target.append(curr_yaw_rate_target)
                    
                    # Update the plots
                    update_plots()
                    
                except (ValueError, IndexError) as e:
                    print(f"Error parsing data: {e}")
                    
        except Exception as e:
            # Skip if no data or error
            time.sleep(0.01)

def update_plots():
    """Update all plot lines with current data"""
    if not timestamps:
        return
    
    # Convert deques to lists for plotting
    times = list(timestamps)
    
    # Calculate time window limits
    current_time = times[-1]
    start_time = max(0, current_time - DISPLAY_WINDOW)
    
    # Update data for all plots
    # Depth plot
    for line, data in zip(depth_lines, [depth_actual, depth_target]):
        line.set_data(times, list(data))
    
    # Depth rate plot
    for line, data in zip(depth_rate_lines, [depth_rate_actual, depth_rate_target]):
        line.set_data(times, list(data))
    
    # Yaw plot
    for line, data in zip(yaw_lines, [yaw_actual, yaw_target]):
        line.set_data(times, list(data))
    
    # Yaw rate plot
    for line, data in zip(yaw_rate_lines, [yaw_rate_actual, yaw_rate_target]):
        line.set_data(times, list(data))
    
    # Set x limits to show moving window
    for ax in axs.flatten():
        ax.set_xlim(start_time, current_time)
        
    # Auto-scale y limits for all plots
    if depth_actual and depth_target:
        min_val = min(min(depth_actual), min(depth_target))
        max_val = max(max(depth_actual), max(depth_target))
        padding = (max_val - min_val) * 0.1 or 0.1  # Add 10% padding, with fallback
        axs[0,0].set_ylim(min_val - padding, max_val + padding)
        
    if depth_rate_actual and depth_rate_target:
        min_val = min(min(depth_rate_actual), min(depth_rate_target))
        max_val = max(max(depth_rate_actual), max(depth_rate_target))
        padding = (max_val - min_val) * 0.1 or 0.1
        axs[0,1].set_ylim(min_val - padding, max_val + padding)
        
    if yaw_actual and yaw_target:
        min_val = min(min(yaw_actual), min(yaw_target))
        max_val = max(max(yaw_actual), max(yaw_target))
        padding = (max_val - min_val) * 0.1 or 0.1
        axs[1,0].set_ylim(min_val - padding, max_val + padding)
        
    if yaw_rate_actual and yaw_rate_target:
        min_val = min(min(yaw_rate_actual), min(yaw_rate_target))
        max_val = max(max(yaw_rate_actual), max(yaw_rate_target))
        padding = (max_val - min_val) * 0.1 or 0.1
        axs[1,1].set_ylim(min_val - padding, max_val + padding)
    
    # Redraw the figure
    fig.canvas.draw_idle()
    fig.canvas.flush_events()

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
            plt.close()
            conn.close()
            s.close()
        except:
            pass

if __name__ == "__main__":
    main()