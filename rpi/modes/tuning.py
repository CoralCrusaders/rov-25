import typing
import glob
import threading
import serial
import nmea_encode_c_ext.nmea_encode as nmea_encode
import time
import serial.tools.list_ports
import socket

HOST = '192.168.68.12'
PORT = 10110 # standard NMEA port

TEENSY_4_1_PIPE = '/dev/serial0'

GAMEPAD_NUM_INPUTS: int = 18

onboard_data: str = ""
nmea_bytes = b''

# python floats are 64-bit doubles
pidfs_consts: list[float] = [0.0] * 5 * 4
pidfs_tuple: tuple[float, ...] = tuple(pidfs_consts)

gamepad_inputs: list[int] = [0] * GAMEPAD_NUM_INPUTS
gamepad_input_tuple: tuple[int, ...] = tuple(gamepad_inputs)

ser = serial.Serial(TEENSY_4_1_PIPE, 115200, write_timeout = 2)
ser.flush()

for p in serial.tools.list_ports.comports():
    print(p)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("socket created")
try:
    s.connect((HOST, PORT))
    s.setblocking(0)
    print("socket connected")
except:
    print("socket connection failed")
    exit(1)

gamepad_map: dict[str, int] = {
    "ABS_X": 0, # i16, LSX
    "ABS_Y": 1, # i16, LSY
    "ABS_RX": 2, # i16, RSX
    "ABS_RY": 3, # i16, RSY
    "BTN_THUMBL": 4, # 0, 1, LSB
    "BTN_THUMBR": 5, # 0, 1, RSB
    "ABS_HAT0X": 6, # -1, 0, 1, DPadX
    "ABS_HAT0Y": 7, # -1, 0, 1, DPadY
    "BTN_SOUTH": 8, # 0, 1, A
    "BTN_EAST": 9, # 0, 1, B
    "BTN_NORTH": 10, # 0, 1, X
    "BTN_WEST": 11, # 0, 1, Y
    "BTN_TL": 12, # 0, 1, LB
    "BTN_TR": 13, # 0, 1, RB
    "ABS_Z": 14, # u8, LT
    "ABS_RZ": 15, # u8, RT
    "BTN_START": 16, # 0, 1, Start
    "BTN_SELECT": 17 # 0, 1, Back
}

def monitor_socket_input():
    try:
        datalen: int = int(s.recv(4).decode('ASCII'))
    except:
        return
    topside_data: str = s.recv(datalen).decode('ASCII')
    time.sleep(0.01)
    chunks: list[str] = topside_data.split(",");
    if(chunks[0] != "$CTCTL"):
        print("Invalid NMEA header topside")
        return
    # the first 5 * 4 values are the pidfs constants
    pidfs_strs: list[str] = chunks[1:21]
    try:
        pidfs_consts = [float(x) for x in pidfs_strs]
    except ValueError:
        print(topside_data)
    for event in chunks[21:]:
        code, state = event.split(":")
        gamepad_inputs[gamepad_map[code]] = int(state)
    return

def transmit_topside_socket():
    time.sleep(1)
    while(True):
        nmea: str = "$RPCTL," + str(onboard_data) # dummy checksum included
        len_str: str = str(len(nmea)).zfill(4)
        transmission = (len_str + nmea).encode('ASCII')
        s.send(transmission)
        time.sleep(0.2)

def transmit_serial():
    time.sleep(1)
    while(True):
        ser.write(nmea_bytes)
        #print(nmea_bytes)
        time.sleep(0.1)

def main():
    global nmea_bytes
    global onboard_data
    transmission_thread = threading.Thread(target=transmit_serial)
    transmission_thread.daemon = True
    transmission_thread.start()
    topside_thread = threading.Thread(target=transmit_topside_socket)
    topside_thread.daemon = True
    topside_thread.start()
    while True:
        monitor_socket_input()
        pidfs_tuple = tuple(pidfs_consts)
        gamepad_input_tuple = tuple(gamepad_inputs)
        transmission_tuple = pidfs_tuple + gamepad_input_tuple
        #print(transmission_tuple)
        nmea_bytes = nmea_encode.nmea_encode_tuning(transmission_tuple)
        #nmea_bytes = nmea_encode.nmea_encode(gamepad_input_tuple)
        if(ser.in_waiting > 0):
            print("a")
            old_data = onboard_data
            try:
                onboard_data = ser.readline().decode('ASCII').rstrip()
                print(onboard_data)
                if onboard_data.startswith("$TNCTL,"):
                    onboard_data = onboard_data[7:] # Remove the header
                else:
                    print("invalid header: " + onboard_data)    
                    onboard_data = old_data
            except UnicodeDecodeError:
                onboard_data = "unicode error"
            except Exception as e:
                onboard_data = f"error: {str(e)}"

main()
