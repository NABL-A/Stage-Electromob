import numpy as np
import time
from scipy.spatial.transform import Rotation as Ro
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive

def calculate_local_frame(P0, P1, P2):
    """
    Calcule la matrice de transformation T qui passe du repère local au repère global,
    à partir de trois points connus (P0, P1 et P2) définissant l’origine et la direction des axes.
    """
    # Axe local X (de P0 vers P1)
    x_axis = P1 - P0
    x_axis = x_axis / np.linalg.norm(x_axis)

    # Calcul temporaire pour définir Z : le produit vectoriel de X et d'un vecteur vers P2
    temp_y = P2 - P0
    z_axis = np.cross(x_axis, temp_y)
    z_axis = z_axis / np.linalg.norm(z_axis)

    # Axe local Y : produit vectoriel de Z et X
    y_axis = np.cross(z_axis, x_axis)

    # Matrice de rotation : colonnes = axes locaux exprimés dans le repère global
    R = np.column_stack((x_axis, y_axis, z_axis))

    # Matrice de transformation homogène
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = P0

    return T

def move_along_points(rtde_c, rtde_r, T, local_points, delay=2):
    """
    Parcourt une liste de points définis dans le repère local,
    convertit chacun en repère global et déplace le robot en ligne droite.
    """
    for i, local_pt in enumerate(local_points):
        # Conversion du point local (homogène) vers le global
        global_pt = T @ local_pt
        # Conserve l'orientation TCP actuelle
        current_pose = rtde_r.getActualTCPPose()
        pose_target = list(global_pt[:3]) + current_pose[3:]
        print(f"Déplacement vers le point {i}: {pose_target}")
        rtde_c.moveL(pose_target, speed=0.2, acceleration=0.2)
        time.sleep(delay)

def move_to_joint_positions(rtde_c, joint_positions, delay=2):
    """
    Déplace le robot à une série de configurations articulaires.
    """
    for i, joints in enumerate(joint_positions):
        print(f"Déplacement en position conjointe {i}: {joints}")
        rtde_c.moveJ(joints, speed=0.2, acceleration=0.2)
        time.sleep(delay)

def rotate_UR5_around_local_Z(rtde_c, rtde_r, degrees):
    """
    Effectue une rotation du robot autour de l'axe Z (global) en gardant sa position TCP.
    Les rotations sont effectuées en multipliant la rotation actuelle par une rotation autour de Z.
    """
    pose = rtde_r.getActualTCPPose()
    pos = np.array(pose[:3])
    rotvec = np.array(pose[3:])
    
    R_current = Ro.from_rotvec(rotvec).as_matrix()
    angle_rad = np.radians(degrees)
    Rz = Ro.from_euler('z', angle_rad).as_matrix()

    R_new = Rz @ R_current  # Appliquer la rotation autour de Z
    new_rotvec = Ro.from_matrix(R_new).as_rotvec()

    pose_target = list(pos) + list(new_rotvec)
    print(f"Rotation autour de Z de {degrees}°")
    print("Pose cible :", pose_target)

    rtde_c.moveL(pose_target, speed=0.2, acceleration=0.5)

def main():
    # Adresse IP du robot
    ROBOT_IP = "10.2.30.60"
    
    # Initialisation des interfaces de contrôle et de réception
    rtde_c = RTDEControl(ROBOT_IP)
    rtde_r = RTDEReceive(ROBOT_IP)
    
    # Définition des points servant à construire le repère local (en global)
    P0 = np.array([-0.02595167,  0.8523917 ,  0.12572524])
    P1 = np.array([ 0.0439653,    0.85237297, 0.12583492])
    P2 = np.array([-0.0249712,   0.88233544, 0.12582908])
    
    # Définition de points d'intérêt et conversion en coordonnées homogènes
    injecteur = np.array([-0.4983755735001421, 0.25895164851486685, 0.1858297414071211])
    box = np.array([0.000735568204164521, 0.48088659168585735, 0.0010128348377972107])
    injecteur_local = np.hstack((injecteur, 1))
    box_local = np.hstack((box, 1))
    
    # Déplacement initial en configuration joint pour stabiliser le robot
    pose_init = [-1.6491854826556605, -1.6341984907733362, 1.8493223190307617,
                 -3.355762783681051, -1.4974659124957483, -1.5762279669391077]
    rtde_c.moveJ(pose_init, speed=0.2, acceleration=0.2)
    
    # Calcul du repère local et de sa matrice de transformation
    T = calculate_local_frame(P0, P1, P2)
    T_inv = np.linalg.inv(T)
    repere_box = T_inv @ box_local
    repere_injecteur = T_inv @ injecteur_local
    
    print("\n=== Repère local défini ===")
    print("Origine :", P0)
    print("Axes :")
    print("  X:", T[:3, 0])
    print("  Y:", T[:3, 1])
    print("  Z:", T[:3, 2])
    
    input("Appuyez sur Entrée pour lancer le déplacement...")
    
    # Définition d'une série de points locaux pour le déplacement (points homogènes)
    local_points = [
        np.array([-0.04, -0.06, 0.08, 1]),
        np.array([-0.04,  0.06, 0.08, 1]),
        np.array([-0.04, -0.12, 0.08, 1]),
        np.array([-0.04, -0.20, 0.30, 1])
        # Vous pouvez ajouter ici : np.array(repere_injecteur)
    ]
    
    # Calcul de points additionnels basés sur le point injecteur
    point1 = repere_injecteur.copy()
    point1[1] -= 0.14
    point1[2] += 0.20

    point2 = point1.copy()
    point2[2] -= 0.30

    point3 = point1.copy()
    point3[0] += 0.115

    # Attention : modification de l'homogénéité (indice 3) n'est généralement pas souhaitée.
    # Ici, nous conservons l'homogénéité à 1 ; ajustez si nécessaire.
    point4 = point3.copy()

    additional_points = [point1, point2, point1, point3, point4]
    
    # Déplacement par interpolation le long des points définis
    print("\n--- Déplacements linéaires sur points définis ---")
    move_along_points(rtde_c, rtde_r, T, local_points, delay=2)
    
    # Déplacement via configurations articulaires prédéfinies
    joint_positions = [
        [-1.6755712668048304, -1.4491103331195276, 0.8367433547973633,
         -0.9699614683734339, -1.4714487234698694, -1.5762398878680628],
        [-0.4743412176715296, -1.5091918150531214, 1.348893642425537,
         -1.3945730368243616, -1.4682758490191858, -2.0456507841693323]
    ]
    print("\n--- Déplacements par positions articulaires ---")
    move_to_joint_positions(rtde_c, joint_positions, delay=2)
    
    # Déplacement sur les points additionnels calculés
    print("\n--- Déplacements sur points additionnels ---")
    move_along_points(rtde_c, rtde_r, T, additional_points, delay=2)
    
    # Rotation autour de l'axe Z (exemple d'utilisation)
    # rotate_UR5_around_local_Z(rtde_c, rtde_r, 90)
    
    # Attente de la fin du programme robot
    while rtde_c.isProgramRunning():
        time.sleep(0.1)
    
    rtde_c.stopScript()
    print("Mouvement terminé.")

if __name__ == "__main__":
    main()
