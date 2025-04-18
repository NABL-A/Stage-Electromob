import json
import paho.mqtt.client as mqtt
import time
import threading
import numpy as np 
import rtde_receive
import rtde_control
import dashboard_client  # Assurez-vous que ce module est correctement configuré
import math
import cv2 as cv
import socket
from queue import Queue

##########################################
# Variables globales et synchronisation
##########################################

# Données des capteurs MQTT
convoyeur_data = [0] * 2  # Pour capteurs_convoyeur
bac_data = [0] * 5        # Pour capteurs_bac

# Variable globale pour partager l'état des capteurs
global_out = None
out_lock = threading.Lock()

# Événement pour indiquer que la pince est en cours d'action
gripper_busy = threading.Event()

##########################################
# Partie 1 : MQTT
##########################################

MQTT_BROKER = "10.2.30.162"
MQTT_PORT = 1883
TOPIC_B = "capteurs_bac/etat"
TOPIC_C = "capteurs_convoyeur/etat"

def on_connect(client, userdata, flags, rc):
    print("Connecté au broker MQTT avec le code de retour", rc)
    client.subscribe(TOPIC_B)
    client.subscribe(TOPIC_C)

def on_message(client, userdata, msg):
    global convoyeur_data, bac_data, global_out
    try:
        data = json.loads(msg.payload.decode('utf-8'))
        if msg.topic == TOPIC_C:  # capteurs_convoyeur/etat (2 pins)
            convoyeur_data = [data["pin1"], data["pin2"]]
        elif msg.topic == TOPIC_B:  # capteurs_bac/etat (5 pins)
            bac_data = [data[f"pin{i+1}"] for i in range(5)]
        with out_lock:
            global_out = [convoyeur_data, bac_data]
        print("Données MQTT reçues :", global_out)
    
    except Exception as e:
        print("Erreur lors du traitement du message :", e)

def mqtt_client_thread():
    client = mqtt.Client("PythonClient")
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_forever()

##########################################
# Partie 2 : Contrôle de la pince via RTDE (Grip)
##########################################

class Pince:
    def __init__(self):
        self.IP_robot = "10.2.30.60"
        self.port_dashboard = 29999  # Port pour l'IHM
        self.port_robot = 30002      # Port pour la connexion directe au robot

    def connexion(self):
        """Établit la connexion via socket à la pince."""
        self.robot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.dashboard = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.robot.connect((self.IP_robot, self.port_robot))
        self.dashboard.connect((self.IP_robot, self.port_dashboard))
        # Charger le script de la pince (si nécessaire)
        self.dashboard.send(("load gripper_open.urp\n").encode('utf-8'))

    def _action_pince(self, action):
        """
        Exécute l'action sur la pince : fermeture (prise) ou ouverture (lacher).
        L'événement gripper_busy est activé pendant l'exécution de l'action.
        """
        gripper_busy.set()
        try:
            self.connexion()
            if action == "prise": 
                self.robot.send(("set_standard_digital_out(0,True)\n").encode('utf8'))
            elif action == "lacher":
                self.robot.send(("set_standard_digital_out(0,False)\n").encode('utf8'))
            # Pour exécuter l'action de la pince, on lance le script
            self.dashboard.send(("stop\n").encode('utf8'))
            time.sleep(1)
            self.dashboard.send(("play\n").encode('utf8'))
            time.sleep(5)
            self.dashboard.send(("stop\n").encode('utf8'))
        except Exception as e:
            print("Erreur lors de l'action de la pince :", e)
        finally:
            gripper_busy.clear()

    def prise(self):
        """Ferme la pince."""
        self._action_pince("prise")

    def lacher(self):
        """Ouvre la pince."""
        self._action_pince("lacher")
        
    def stop_grip_control_script(self):
        """
        Force l'arrêt du script de contrôle du grip afin de ne pas interférer
        avec les mouvements du robot.
        On se connecte et on envoie uniquement 'stop' sans relancer 'play'.
        """
        try:
            self.connexion()
            self.dashboard.send(("stop\n").encode('utf8'))
            # On ferme les sockets afin de libérer la communication.
            self.robot.close()
            self.dashboard.close()
            print("[Gripper] Script de contrôle arrêté.")
        except Exception as e:
            print("Erreur lors de l'arrêt du script grip :", e)

# Instance unique de la pince
maPince = Pince()

# File de commandes pour la pince
gripper_queue = Queue()

def gripper_worker():
    """
    Thread dédié aux commandes de la pince.
    Lit les commandes dans la file et les exécute.
    """
    while True:
        cmd = gripper_queue.get()
        if cmd is None:
            break
        print("[Gripper] Exécution de la commande :", cmd)
        if cmd == "prise":
            maPince.prise()
        elif cmd == "lacher":
            maPince.lacher()
        else:
            print("[Gripper] Commande inconnue :", cmd)
        gripper_queue.task_done()

##########################################
# Partie 3 : Contrôle du robot via RTDE avec cheminTuile
##########################################

def wait_for_rtde(rtde_r, timeout=5):
    """Attend que le script RTDE redémarre en testant getActualTCPPose()"""
    start_time = time.time()
    while True:
        try:
            _ = rtde_r.getActualTCPPose()
            return True
        except Exception as e:
            if time.time() - start_time > timeout:
                print("Timeout: Le script RTDE ne redémarre pas.")
                return False
            print("Attente du redémarrage du script RTDE...")
            time.sleep(0.2)

def robot_control_thread():
    robot_ip = "10.2.30.60"
    rtde_c = rtde_control.RTDEControlInterface(robot_ip)
    rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip)

    # --- Configuration du repère local ---
    P0 = np.array([-0.02595167, 0.8523917, 0.12572524])
    P1 = np.array([0.0439653, 0.85237297, 0.12583492])
    P2 = np.array([-0.0249712, 0.88233544, 0.12582908])
    
    x_axis = P1 - P0
    x_axis /= np.linalg.norm(x_axis)
    temp_y = P2 - P0
    z_axis = np.cross(x_axis, temp_y)
    z_axis /= np.linalg.norm(z_axis)
    y_axis = np.cross(z_axis, x_axis)
    R = np.column_stack((x_axis, y_axis, z_axis))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = P0
    T_inv = np.linalg.inv(T)
    
    # --- Définition des points pour le chemin principal ---
    points = [
        np.array([-0.04, -0.06, 0.08, 1]),
        np.array([-0.04,  0.06, 0.08, 1]),
        np.array([-0.04, -0.12, 0.08, 1]),
        np.array([-0.04, -0.20, 0.30, 1]),
    ]
    
    # Fonction pour ajuster un point sur une dimension donnée
    def deplacement_point(point, indice, distance):
        arr = point.copy()
        arr[indice] += distance
        return arr

    injecteur = [-0.35783121664972006, -0.2558472234496133, 0.18701865215398125]
    box = [0.000735568204164521, 0.48088659168585735, 0.0010128348377972107]
    
    injecteur_local = np.array(injecteur + [1])
    box_local = np.array(box + [1])
    
    repere_injecteur = T_inv @ injecteur_local

    # Calcul de points pour le chemin secondaire
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
        [-1.6755712668048304, -1.4491103331195276, 0.8367433547973633, -0.9699614683734339, -1.4714487234698694, -1.5762279669391077],
        [-0.4743412176715296, -1.5091918150531214, 1.348893642425537, -1.3945730368243616, -1.4682758490191858, -2.0456507841693323],
    ]
    
    joints2 = [
        [-1.5707710425006312, -1.9037888685809534, 1.8204197883605957, -1.5371840635882776, -1.4706586042987269, -1.5850275198565882]
    ]
    
    # --- Fonction définissant le chemin complet (cheminTuile) ---
    def cheminTuile():
        # Avant d'envoyer un mouvement, s'assurer que le script du grip est bien arrêté
        maPince.stop_grip_control_script()
        
        # Pose initiale
        pose_init = [-1.6491854826556605, -1.6341984907733362,
                     1.8493223190307617, -3.355762783681051,
                     -1.4974659124957483, -1.5762279669391077]
        while gripper_busy.is_set():
            time.sleep(0.1)
        if not wait_for_rtde(rtde_r):
            print("RTDE indisponible pour la pose initiale.")
            return
        rtde_c.moveJ(pose_init, speed=0.2, acceleration=0.2)
        time.sleep(1)
        
        # Action d'ouverture de la pince
        gripper_queue.put("lacher")
        time.sleep(2)
        
        # Première série de mouvements linéaires
        for i, point in enumerate(points):
            global_point = T @ point
            current_pose = rtde_r.getActualTCPPose()
            pose_target = [float(x) for x in global_point[:3]] + current_pose[3:]
            if not wait_for_rtde(rtde_r):
                print("RTDE indisponible avant le mouvement linéaire", i+1)
                continue
            print(f"[Robot] Mouvement linéaire {i+1} vers {pose_target}")
            rtde_c.moveL(pose_target, speed=0.2, acceleration=0.2)
            time.sleep(2)
            # Déclencher une action de la pince après le deuxième point
            if i == 1:
                gripper_queue.put("prise")
        
        # Mouvements par joints
        for point in joints:
            while gripper_busy.is_set():
                time.sleep(0.1)
            print(f"[Robot] Mouvement par joints vers {point}")
            rtde_c.moveJ(point, speed=0.2, acceleration=0.2)
            time.sleep(2)
        
        # Seconde série de mouvements linéaires
        for i, point in enumerate(points2):
            global_point = T @ point
            current_pose = rtde_r.getActualTCPPose()
            pose_target = [float(x) for x in global_point[:3]] + current_pose[3:]
            if not wait_for_rtde(rtde_r):
                print("RTDE indisponible avant le mouvement secondaire", i+1)
                continue
            print(f"[Robot] Mouvement linéaire secondaire {i+1} vers {pose_target}")
            rtde_c.moveL(pose_target, speed=0.2, acceleration=0.2)
            if i in [1, 9]:
                gripper_queue.put("lacher")
            if i in [4, 12]:
                gripper_queue.put("prise")
            time.sleep(2)
        
        # Derniers mouvements par joints
        for point in joints2:
            while gripper_busy.is_set():
                time.sleep(0.1)
            print(f"[Robot] Dernier mouvement par joints vers {point}")
            rtde_c.moveJ(point, speed=0.2, acceleration=0.2)
            time.sleep(2)
        
        print("Séquence cheminTuile terminée.")

    # Boucle de contrôle basée sur les données MQTT
    triggered = False
    try:
        while True:
            with out_lock:
                current_data = global_out
            
            if current_data is not None:
                if current_data[0][0] == 1 and not triggered:
                    triggered = True
                    print("Déclenchement de la séquence cheminTuile.")
                    cheminTuile()
                    print("Séquence cheminTuile effectuée.")
                elif current_data[0][0] == 0 and triggered:
                    triggered = False
                    print("Réinitialisation de l'état de déclenchement.")
            
            time.sleep(0.1)
    
    except Exception as e:
        print("Erreur dans le thread robot :", e)
    
    finally:
        print("Contrôle via RTDE terminé.")

##########################################
# Démarrage des threads
##########################################

if __name__ == "__main__":
    # Thread MQTT
    mqtt_thread = threading.Thread(target=mqtt_client_thread, daemon=True)
    mqtt_thread.start()
    time.sleep(1)
    
    # Thread de la pince
    gripper_thread = threading.Thread(target=gripper_worker, daemon=True)
    gripper_thread.start()
    
    # Thread de contrôle du robot avec cheminTuile
    robot_thread = threading.Thread(target=robot_control_thread, daemon=True)
    robot_thread.start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Arrêt du programme.")
        gripper_queue.put(None)
