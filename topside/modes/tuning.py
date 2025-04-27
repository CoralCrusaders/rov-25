import socket
from inputs import get_gamepad
import time
import threading
import typing
import ast
from tkinter import *

HOST = '192.168.68.12'
PORT = 10110 # standard NMEA port

NMEA_HEADER = "$CTCTL" # computer topside control

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("socket created")
s.bind((HOST, PORT))
print("socket bound")
s.listen()
print("listening for connection")
conn, addr = s.accept()
print("connection address:", addr)
s.setblocking(0)

depth_rate_p: float = 0.3
depth_rate_i: float = 0.0
depth_rate_d: float = 0.0
depth_rate_f: float = 0.0
depth_rate_s: float = 0.0
depth_pwr_p: float = 1.0
depth_pwr_i: float = 0.0
depth_pwr_d: float = 0.0
depth_pwr_f: float = 0.0
depth_pwr_s: float = 0.0
yaw_rate_p: float = 0.7
yaw_rate_i: float = 0.0
yaw_rate_d: float = 0.0
yaw_rate_f: float = 0.0
yaw_rate_s: float = 0.0
yaw_pwr_p: float = 0.003
yaw_pwr_i: float = 0.0
yaw_pwr_d: float = 0.0
yaw_pwr_f: float = 0.0
yaw_pwr_s: float = 0.0


def monitor_socket_input():
    data: str = ""
    time.sleep(1)
    while True:
        try:
            datalen: int = int(conn.recv(4).decode('ASCII'))
            data = conn.recv(datalen).decode('ASCII')
            #print(data)
        except ValueError:
            pass
        time.sleep(0.1)


def gamepad_to_nmea() -> bytes:
    events = get_gamepad()
    transmission: bytes = None
    nmea: str = NMEA_HEADER
    len_str: str = ""
    nmea += "," + str(depth_rate_p) + "," + str(depth_rate_i) + "," + str(depth_rate_d) + "," + str(depth_rate_f) + "," + str(depth_rate_s)
    nmea += "," + str(depth_pwr_p) + "," + str(depth_pwr_i) + "," + str(depth_pwr_d) + "," + str(depth_pwr_f) + "," + str(depth_pwr_s)
    nmea += "," + str(yaw_rate_p) + "," + str(yaw_rate_i) + "," + str(yaw_rate_d) + "," + str(yaw_rate_f) + "," + str(yaw_rate_s)
    nmea += "," + str(yaw_pwr_p) + "," + str(yaw_pwr_i) + "," + str(yaw_pwr_d) + "," + str(yaw_pwr_f) + "," + str(yaw_pwr_s)
    for event in events:
        if(event.code != "SYN_REPORT"):
            nmea += "," + event.code + ":" + str(event.state) 
    
    len_str = str(len(nmea)).zfill(4)
    transmission = (len_str + nmea).encode('ASCII')
    print(transmission)
    return transmission

def transmit_gamepad():
    while True:
        conn.send(gamepad_to_nmea())

def main():
    monitor_socket_thread = threading.Thread(target=monitor_socket_input)
    monitor_socket_thread.daemon = True
    monitor_socket_thread.start()
    transmit_gamepad()

main()
