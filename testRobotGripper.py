#import
import socket
import time

IP_robot = "10.2.30.60"
port_dashboard = 29999  # Pour la connexion via socket à l'IHM
port_robot = 30002  # Pour la connexion via socket au robot lui-même

# Create sockets
robot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
dashboard = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Connect to the robot and dashboard
try:
    robot.connect((IP_robot, port_robot))
    print("Connected to robot")
except Exception as e:
    print(f"Failed to connect to robot: {e}")

try:
    dashboard.connect((IP_robot, port_dashboard))
    print("Connected to dashboard")
except Exception as e:
    print(f"Failed to connect to dashboard: {e}")

# Send commands to control the gripper

robot.send(("set_standard_digital_out(0,True)" + "\n").encode('utf8'))
print("Sent command to close gripper")
time.sleep(4)
robot.send(("set_standard_digital_out(0,False)" + "\n").encode('utf8'))
print("Sent command to open gripper")

# Close the connection
robot.close()
dashboard.close()