import socket
import time

HOST = "10.2.30.60"
PORT = 30002

points = [
    [-1.58, -1.8, 1.63, -1.31, -1.6, 1.73],
    [-1.58, -1.8, 1.63, -2.90, -1.6, 1.73],
    [-1.58, -1.29, 2.39, -4.19, -1.62, 1.67],
    [-1.58, -1.21, 2.33, -4.19, -1.62, 1.67],
    [-1.58, -1.15, 2.22, -4.13, -1.62, 1.67],
    [-1.58, -1.25, 2.42, -4.23, -1.62, 1.67],
    [-1.58, -1.81, 2.99, -3.24, -1.62, 1.67]
]

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

for i, point in enumerate(points):
    urscript = f"movej({point}, a=0.5, v=0.2)\n"
    print(f"Envoi du point {i+1}")
    s.send(urscript.encode('utf-8'))
    time.sleep(4)  

s.close()