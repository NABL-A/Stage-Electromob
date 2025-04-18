import numpy as np
import time
import threading
from queue import Queue
from scipy.spatial.transform import Rotation as Ro
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
from GripRobot import Pince

# Initialisation
maPince = Pince()

ROBOT_IP = "10.2.30.60"
rtde_c = RTDEControl(ROBOT_IP)
rtde_r = RTDEReceive(ROBOT_IP)

# Points de référence
P0 = [-0.02595167, 0.8523917, 0.12572524]
P1 = [0.0439653, 0.85237297, 0.12583492]
P2 = [-0.0249712, 0.88233544, 0.12582908]

injecteur = [-0.35783121664972006, -0.2558472234496133, 0.18701865215398125]
box = [0.000735568204164521, 0.48088659168585735, 0.0010128348377972107]

P0 = np.array(P0)
P1 = np.array(P1)
P2 = np.array(P2)
injecteur_local = np.array(list(injecteur) + [1])
box_local = np.array(list(box) + [1])

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

# Calcul des points dans le repère local
repere_box = T_inv @ box_local
repere_injecteur = T_inv @ injecteur_local

input("Repère local défini. Origine: {} \nAxes:\nX: {}\nY: {}\nZ: {}\nAppuyez sur une touche pour démarrer le déplacement.".format(P0, x_axis, y_axis, z_axis))

# Définition d'une fonction de déplacement d'un point sur une dimension donnée
def deplacement_point(point, indice, distance):
    arrivee = point.copy()
    arrivee[indice] += distance
    return arrivee

# Définition des points de mouvement (exemple)
points = [
    np.array([-0.04, -0.06, 0.08, 1]),
    np.array([-0.04, 0.06, 0.08, 1]),
    np.array([-0.04, -0.12, 0.08, 1]),
    np.array([-0.04, -0.20, 0.30, 1]),
]

# Calcul de nouveaux points à partir du point injecteur (repère local)
point1 = deplacement_point(repere_injecteur, 1, 0.15)
point1 = deplacement_point(point1, 2, 0.10)
point1 = deplacement_point(point1, 0, 0.01)
point2 = deplacement_point(point1, 2, -0.15)
point3 = deplacement_point(point1, 0, -0.095)
point4 = deplacement_point(point3, 2, -0.20)
point5 = deplacement_point(point1, 2, -0.10)

points2 = [
    np.array(point1),
    np.array(point2),
    np.array(point1),
    np.array(point3),
    np.array(point4),
    np.array(point3),
    np.array(point1),
    np.array(point5),
    np.array(point1),
    np.array(point3),
    np.array(point4),
    np.array(point3),
    np.array(point1),
    np.array(point2),
    np.array(point1),
]

joints = [
    [-1.6755712668048304, -1.4491103331195276, 0.8367433547973633, -0.9699614683734339, -1.4714487234698694, -1.5762398878680628],
    [-0.4743412176715296, -1.5091918150531214, 1.348893642425537, -1.3945730368243616, -1.4682758490191858, -2.0456507841693323],
]

joints2 = [
    [-1.5707710425006312, -1.9037888685809534, 1.8204197883605957, -1.5371840635882776, -1.4706586042987269, -1.5850275198565882]
]

# --- Mise en place des threads ---

# File de commandes pour la pince
gripper_queue = Queue()

def gripper_worker():
    """
    Le thread de la pince attend et exécute les commandes envoyées par la file.
    Les commandes possibles : "connexion", "prise", "lacher"
    Un 'None' permet de stopper la boucle.
    """
    while True:
        cmd = gripper_queue.get()
        if cmd is None:
            break  # Fin du thread
        print("[Gripper] Exécution de la commande :", cmd)
        if cmd == "connexion":
            maPince.connexion()
        elif cmd == "prise":
            maPince.prise()
        elif cmd == "lacher":
            maPince.lacher()
        else:
            print("[Gripper] Commande inconnue :", cmd)
        gripper_queue.task_done()

def robot_movement():
    """
    Fonction qui enchaîne les mouvements du robot.
    Au lieu d'appeler directement les méthodes de la pince, 
    les commandes sont ajoutées dans la file pour être traitées par gripper_worker.
    """
    # Pose initiale
    rtde_c.stopScript()
    pose_init = [-1.6491854826556605, -1.6341984907733362, 1.8493223190307617,
                 -3.355762783681051, -1.4974659124957483, -1.5762279669391077]
    rtde_c.moveJ(pose_init, speed=0.2, acceleration=0.2)
    
    # Commandes initiales pour la pince
    gripper_queue.put("connexion")
    gripper_queue.put("lacher")
    
    # Première série de mouvements linéaires
    for i, point in enumerate(points):
        global_point = T @ point
        # On conserve la partie position et on récupère l'orientation actuelle
        current_pose = rtde_r.getActualTCPPose()
        pose_target = [float(x) for x in global_point[:3]] + current_pose[3:]
        rtde_c.moveL(pose_target, speed=0.2, acceleration=0.2)
        if i == 1:
            gripper_queue.put("connexion")
            gripper_queue.put("prise")
        time.sleep(2)
    
    # Mouvements par joints
    for point in joints:
        rtde_c.moveJ(point, speed=0.2, acceleration=0.2)
        time.sleep(2)
    
    # Seconde série de mouvements linéaires
    for i, point in enumerate(points2):
        global_point = T @ point
        current_pose = rtde_r.getActualTCPPose()
        pose_target = [float(x) for x in global_point[:3]] + current_pose[3:]
        print("[Robot] Pose cible :", pose_target)
        rtde_c.moveL(pose_target, speed=0.2, acceleration=0.2)
        if i in [1, 9]:
            gripper_queue.put("connexion")
            gripper_queue.put("lacher")
        if i in [4, 12]:
            gripper_queue.put("connexion")
            gripper_queue.put("prise")
        time.sleep(2)
    
    # Derniers mouvements par joints
    for point in joints2:
        rtde_c.moveJ(point, speed=0.2, acceleration=0.2)
        time.sleep(2)

# --- Exécution des threads ---

if __name__ == "__main__":
    # Démarrage du thread pour la pince
    gripper_thread = threading.Thread(target=gripper_worker, name="GripperThread")
    gripper_thread.start()

    # Démarrage du thread de mouvement du robot
    robot_thread = threading.Thread(target=robot_movement, name="RobotMovementThread")
    robot_thread.start()

    # Attente de la fin des mouvements du robot
    robot_thread.join()
    
    # Une fois terminé, on envoie une commande None pour stopper le thread de la pince
    gripper_queue.put(None)
    gripper_thread.join()
    
    # Vérifier si le programme du robot est encore en cours d'exécution
    while rtde_c.isProgramRunning():
        time.sleep(0.1)
    
    rtde_c.stopScript()
    print("Mouvement terminé.")
