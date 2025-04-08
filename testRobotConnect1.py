import numpy as np 
import rtde_receive
import rtde_control
import dashboard_client
import cv2 as cv
import math
import time

def close_gripper(rtde_c):
    script = "def close_gripper():\n"
    script += " socket_open(\"127.0.0.1\", 63352, \"gripper_socket\")\n"
    script += " socket_send_string(\"SET ACT 1\", \"gripper_socket\")\n"
    script += " socket_send_byte(0, \"gripper_socket\")\n"
    script += " socket_close(\"gripper_socket\")\n"
    script += "end\n"
    rtde_c.sendCustomScript(script)

# Function to open the gripper
def open_gripper(rtde_c):
    script = "def open_gripper():\n"
    script += " socket_open(\"127.0.0.1\", 63352, \"gripper_socket\")\n"
    script += " socket_send_string(\"SET ACT 1\", \"gripper_socket\")\n"
    script += " socket_send_byte(1, \"gripper_socket\")\n"
    script += " socket_close(\"gripper_socket\")\n"
    script += "end\n"
    rtde_c.sendCustomScript(script)

robot_r = rtde_receive.RTDEReceiveInterface("10.2.30.60")
robot = rtde_control.RTDEControlInterface("10.2.30.60")
dashboard = dashboard_client.DashboardClient("10.2.30.60")

pose = robot_r.getActualTCPPose()
articulations = robot_r.getActualQ()
articulations_degrees = [articulation/math.pi*180 for articulation in articulations]

print(pose)
print(articulations)
print([articulation/math.pi*180 for articulation in articulations])

#robot.moveJ(articulations)


'''
print("Strarting gripper test...")
while(True):
    close_gripper(robot)
    print("Gripper closed")
    time.sleep(1)
    
    open_gripper(robot)
    print("Gripper opened")
    time.sleep(1)
'''


#robot.setStandardDigitalOut(0,True)
