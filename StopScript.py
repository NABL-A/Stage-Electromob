import numpy as np
import time
import threading
from queue import Queue
from scipy.spatial.transform import Rotation as Ro
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from GripRobot import Pince
import socket

# Initialisation
maPince = Pince()

ROBOT_IP = "10.2.30.60"
rtde_c = RTDEControl(ROBOT_IP)
rtde_r = RTDEReceive(ROBOT_IP)

IP_robot = "10.2.30.60"
port_dashboard = 29999  # Port pour l'IHM
port_robot = 30002      # Port pour la connexion directe au robot


"""Établit la connexion via socket à la pince."""
robot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
dashboard = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
robot.connect((IP_robot, port_robot))
dashboard.connect((IP_robot, port_dashboard))
# Charger le script de la pince (si nécessaire)
dashboard.send(("clear" + "\n").encode('utf-8'))