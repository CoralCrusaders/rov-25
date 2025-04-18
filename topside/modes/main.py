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

nmea_bytes: bytes = b''

def monitor_socket_input():
    data: str = ""
    time.sleep(1)
    while True:
        try:
            datalen: int = int(conn.recv(4).decode('ASCII'))
            data = conn.recv(datalen).decode('ASCII')
        except ValueError:
            pass
        print(data)
        time.sleep(0.15)


def gamepad_to_nmea() -> bytes:
    events = get_gamepad()
    transmission: bytes = None
    nmea: str = NMEA_HEADER
    len_str: str = ""
    for event in events:
        if(event.code != "SYN_REPORT"):
            nmea += "," + event.code + ":" + str(event.state) 
    len_str = str(len(nmea)).zfill(4)
    transmission = (len_str + nmea).encode('ASCII')
    #print("Data to send: ", transmission)
    return transmission

def transmit_gamepad():
    while True:
        conn.send(gamepad_to_nmea())

def main():
    monitor_socket_thread = threading.Thread(target=monitor_socket_input)
    monitor_socket_thread.daemon = True
    monitor_socket_thread.start()
    transmit_gamepad_thread = threading.Thread(target=transmit_gamepad)
    transmit_gamepad_thread.daemon = True
    transmit_gamepad_thread.start()

main()