import rtde_receive
import rtde_control

robot_ip = "10.2.30.60"

# Initialisation des interfaces RTDE pour le contrôle et la réception de données

rtde_c = rtde_control.RTDEControlInterface(robot_ip)
rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)

transfo = [-1.49,-2.35,0.99,-0.17,-2.96,0.56]

#rtde_c.setTcp([-1.49,-2.35,0.99,-0.17,-2.96,0.56])


pose = rtde_r.getActualTCPPose()

for i in range(len(pose)):
    pose[i] += transfo[i]

print(pose)