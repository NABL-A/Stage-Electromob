"il faut lancer le programme gripper_open.urp sur l'UR5 pour que la pince s'ouvre"

from robot import Robot
from pince import Pince
import numpy as np
import time
temps_debut=time.time()

# création des instances de classes
robot = Robot()
pince = Pince()

class Move :
    def __init__(self):  
        self.points=[
            np.array([-0.02, -0.06, 0.08, 1]),
            np.array([-0.02, 0.025, 0.08, 1]),
            np.array([-0.02, -0.12, 0.08, 1]),
            np.array([-0.02, -0.20, 0.30, 1]),
            np.array(robot.point1),
            np.array(robot.point2),
            np.array(robot.point1),
            np.array(robot.point3),
            np.array(robot.point4),
            np.array(robot.point3),
            np.array(robot.point1),
            np.array(robot.point5),
            np.array(robot.point1),
            np.array(robot.point3),
            np.array(robot.point4),
            np.array(robot.point3),
            np.array(robot.point1),
            np.array(robot.point2),
            np.array(robot.point1),
        ]

        self.joints=[
            [-1.6755712668048304, -1.4491103331195276, 0.8367433547973633, -0.9699614683734339, -1.4714487234698694, -1.5762398878680628],
            [-0.4743412176715296, -1.5091918150531214, 1.348893642425537, -1.3945730368243616, -1.4682758490191858, -2.0456507841693323],
            [-0.07224733034242803, -1.9188702742206019, 1.8160276412963867, -1.4937194029437464, -1.4703596274005335, -1.5883601347552698],
            [-1.5707710425006312, -1.9037888685809534, 1.8204197883605957, -1.5371840635882776, -1.4706586042987269, -1.5850275198565882]
        ]

"""
move = Move()


robot.bougerJ(robot.pose_init)
pince.lacher()
robot.bougerL(move.points[0])
robot.bougerL(move.points[1])
pince.prise()
robot.bougerL(move.points[2])
robot.bougerL(move.points[3])
robot.bougerJ(move.joints[0])
robot.bougerJ(move.joints[1])
robot.bougerL(move.points[4])
robot.bougerJ(move.joints[2])
robot.bougerL(move.points[5])
pince.lacher()
robot.bougerL(move.points[6], 0.1, 0.1)
robot.bougerL(move.points[7], 0.1, 0.1)
pince.prise()
robot.bougerL(move.points[8], 0.1, 0.1)
robot.bougerL(move.points[9], 0.1, 0.1)
robot.bougerL(move.points[10], 0.1, 0.1)
robot.bougerL(move.points[11], 0.1, 0.1)
robot.bougerL(move.points[12], 0.1, 0.1)
robot.bougerL(move.points[13], 0.1, 0.1)
pince.lacher()
robot.bougerL(move.points[14], 0.1, 0.1)
robot.bougerL(move.points[15], 0.1, 0.1)
robot.bougerL(move.points[16], 0.1, 0.1)
pince.prise()
robot.bougerL(move.points[17], 0.1, 0.1)
robot.bougerJ(move.joints[3], 0.1, 0.1)


temps_fin=time.time()
delta_temps=temps_fin-temps_debut
print(delta_temps)
"""