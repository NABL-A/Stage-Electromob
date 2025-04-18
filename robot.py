import rtde_receive
import rtde_control
#from Transfo import create_matrice, matrice_to_pose
import numpy as np
from pince import Pince

class Robot :

    ROBOT_IP = "10.2.30.60"

    #rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    #rtde_c = rtde_control.RTDEReceiveInterface(ROBOT_IP)


    P0 = [-0.02595167,  0.8523917 ,  0.12572524]
    P1 = [0.0439653 , 0.85237297, 0.12583492]
    P2 = [-0.0249712 ,  0.88233544,  0.12582908]

    injecteur = [-0.35783121664972006, -0.2558472234496133, 0.18701865215398125]
    box = [-0.06721406482480582, 0.46819916287833735, 0.0013963899393563683]

    P0 = np.array(P0)
    P1 = np.array(P1)
    P2 = np.array(P2)
    injecteur_local = np.array(list(injecteur) + [1])
    box_local= np.array(list(box) + [1])

    # Calcul du repère local (rotation + origine)
    x_axis = P1 - P0
    x_axis /= np.linalg.norm(x_axis)

    temp_y = P2 - P0
    z_axis = np.cross(x_axis, temp_y)
    z_axis /= np.linalg.norm(z_axis)

    y_axis = np.cross(z_axis, x_axis)

    # Matrice de rotation (repère local → global)
    R = np.column_stack((x_axis, y_axis, z_axis))

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = P0
    T_inv = np.linalg.inv(T)
    repere_box = T_inv @ box_local
    repere_box_1 =np.array(list(repere_box[:3])+[1])
    repere_injecteur = T_inv @ injecteur_local
    repere_injecteur_1=np.array(list(repere_injecteur[:3]) + [1])


    def connexion(self):
        """Fonction pour se connecter au robot grâce à son IP"""
        self.robot_r = rtde_receive.RTDEReceiveInterface(self.ROBOT_IP)
        self.robot_c = rtde_control.RTDEControlInterface(self.ROBOT_IP)

    def deconnexion(self): 
        """Déconnexion du robot"""
        self.robot_c.disconnect()

    def bougerJ(self, pos, speed=0.2, acceleration=0.2):
        """
        Déplacement du robot selon une pose donnée avec connexion préalable et déconnexion à la fin de l'action.

        Args:
            pos (list[float]): position à laquelle le robot doit aller.
            speed (float, optional): vitesse du déplacement.
            acceleration (float, optional): acceleration pour le déplacement.
        """
        self.connexion()
        self.robot_c.moveJ(pos, speed, acceleration)
        self.deconnexion()

    def bougerL(self, pos, speed = 0.2, acceleration =0.2) :
        self.connexion()
        global_point = Robot.T @ pos
        pose_target = [float(x) for x in global_point[:3]] + self.robot_r.getActualTCPPose()[3:]
        self.robot_c.moveL(pose_target, speed, acceleration)
        self.deconnexion()

    def deplacement_point(self, point, indice, distance):
        arrivee=point.copy()
        arrivee[indice]+=distance
        return arrivee

    def __init__(self):
        self.pose_init = [-1.6491854826556605, -1.6341984907733362, 1.8493223190307617,
                 -3.355762783681051, -1.4974659124957483, -1.5762279669391077]
        #déplacement / injecteur 1cm sur X, 15cm sur Y, 10cm sur Z
        self.point1 = self.deplacement_point(self.repere_injecteur_1, 0, -0.005)
        self.point1 = self.deplacement_point(self.point1, 1, 0.13)
        self.point1 = self.deplacement_point(self.point1, 2, 0.15)
        #déplacement -15cm sur Z
        self.point2 = self.deplacement_point(self.point1, 2, -0.23)
        #déplacement -9.5cm sur X
        self.point3 = self.deplacement_point(self.point1, 0, -0.102)
        #déplacement -20 cm sur Z
        self.point4 = self.deplacement_point(self.point3, 2, -0.25)
        #déplacement -10 cm sur Z
        self.point5 = self.deplacement_point(self.point1, 2, -0.112)

        # déplacement / boite
        # -14.5cm sur X, 1,85 cm, 20 cm sur Z
        self.pointBoite1 = self.deplacement_point(self.repere_box_1, 0, -0.145)
        self.pointBoite1 = self.deplacement_point(self.pointBoite1, 1, 0.0185)
        self.pointBoite1 = self.deplacement_point(self.pointBoite1, 2, 0.2)
        #déplacement de 4.65 cm sur Y
        self.pointBoite2 = self.deplacement_point(self.pointBoite1, 1, 0.0465)
        #déplacement de 4.65 cm sur Y
        self.pointBoite3 = self.deplacement_point(self.pointBoite2, 1, 0.0465)
        #déplacement de 4.65 cm sur Y
        self.pointBoite4 = self.deplacement_point(self.pointBoite3, 1, 0.0465)
        #déplacement de 4.65 cm sur Y
        self.pointBoite5 = self.deplacement_point(self.pointBoite4, 1, 0.0465)

        self.points=[
            #point0
            np.array([-0.048, -0.06, 0.08, 1]),
            #point1
            np.array([-0.048, 0.025, 0.08, 1]),
            #point2
            np.array([-0.048, -0.12, 0.08, 1]),
            #point3
            np.array([-0.048, -0.20, 0.30, 1]),
            #point4
            np.array([-0.048, -0.60, -0.023, 1]),
            #point5
            np.array([-0.048, -0.40, 0.05, 1]),
            #point6
            np.array([-0.048, -0.40, 0.30, 1]),
            #point7
            np.array(self.point1),
            #point8
            np.array(self.point2),
            #point9
            np.array(self.point1),
            #point10
            np.array(self.point3),
            #point11
            np.array(self.point4),
            #point12
            np.array(self.point3),
            #point13
            np.array(self.point1),
            #point14
            np.array(self.point5),
            #point15
            np.array(self.point1),
            #point16
            np.array(self.point3),
            #point17
            np.array(self.point4),
            #point18
            np.array(self.point3),
            #point19
            np.array(self.point1),
            #np.array20
            np.array(self.point2),
            #np.array21
            np.array(self.point1),
        ]

        self.joints=[
            [-1.6755712668048304, -1.4491103331195276, 0.8367433547973633, -0.9699614683734339, -1.4714487234698694, -1.5762398878680628],
            [-0.034966293965474904, -1.4821026960956019, 0.7819619178771973, -0.8718579451190394, -1.4708979765521448, -1.5708215872394007],
            [-0.02170879045595342, -1.852485481892721, 1.634385585784912, -1.3550799528705042, -1.4711735884295862, -1.5699823538409632],
            #[-0.4743412176715296, -1.5091918150531214, 1.348893642425537, -1.3945730368243616, -1.4682758490191858, -2.0456507841693323],
            #[-0.07224733034242803, -1.9188702742206019, 1.8160276412963867, -1.4937194029437464, -1.4703596274005335, -1.5883601347552698],
            [-1.5707710425006312, -1.9037888685809534, 1.8204197883605957, -1.5371840635882776, -1.4706586042987269, -1.5850275198565882],
        ]

        self.pointsBoite = [
            self.pointBoite1,
            self.pointBoite2,
            self.pointBoite3,
            self.pointBoite4,
            self.pointBoite5,
        ]