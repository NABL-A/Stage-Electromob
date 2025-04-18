import json
import paho.mqtt.client as mqtt
import time
import threading
import numpy as np 
import rtde_receive
import rtde_control
import time
from pince import Pince
from robot import Robot

# Variables globales pour partager les données
convoyeur_data = [None] * 2  # Pour capteurs_convoyeur
bac_data = [None] * 5        # Pour capteurs_bac

# Variable globale "global_out" accessible de partout dans le programme,
# ainsi qu'un verrou pour protéger l'accès concurrent
global_out = None
out_lock = threading.Lock()
emplacement_bac_libre = None

##########################################
# Partie 1 : MQTT
##########################################

# Paramètres MQTT
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
        # Décoder le message JSON
        data = json.loads(msg.payload.decode('utf-8'))
        
        # Mettre à jour les données en fonction du topic
        if msg.topic == TOPIC_C:  # capteurs_convoyeur/etat (2 pins)
            convoyeur_data = [data["pin1"], data["pin2"]]
        elif msg.topic == TOPIC_B:  # capteurs_bac/etat (5 pins)
            bac_data = [data[f"pin{i+1}"] for i in range(5)]
        
        # Mettre à jour la variable globale "global_out" de façon sécurisée
        with out_lock:  # Début de la section critique
            global_out = [convoyeur_data, bac_data]
        
        print("Données MQTT reçues :", global_out)
    
    except Exception as e:
        print("Erreur lors du traitement du message :", e)

def mqtt_client_thread():
    client = mqtt.Client("PythonClient")
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    
    # Boucle bloquante dans un thread séparé
    client.loop_forever()

##########################################
# Partie 2 : Contrôle du robot via rtde
##########################################

def robot_control_thread():

    robot = Robot()
    pince = Pince()

    # Définition des points de destination (vecteurs de joints)
    # Vous pouvez définir ici la séquence complète des mouvements à exécuter
    points = robot.points
    joints = robot.joints
    # Variable d'état pour s'assurer qu'on ne lance la séquence qu'une fois par activation.
    pointsBoite = robot.pointsBoite

    triggered = False

    actions = [
        "robot.bougerJ(robot.pose_init)",
        "pince.lacher()",
        "robot.bougerL(robot.points[0])",
        "robot.bougerL(robot.points[1])",
        "pince.prise()",
        "robot.bougerL(robot.points[2])",
        "robot.bougerL(robot.points[3])",
        "robot.bougerJ(robot.joints[0])",
        "robot.bougerL(robot.points[4], 0.05, 0.05)",
        "robot.bougerL(robot.points[5], 0.05, 0.05)",
        "robot.bougerL(robot.points[6], 0.05, 0.05)",
        "robot.bougerJ(robot.joints[1])",
        "robot.bougerJ(robot.joints[2])",
        "robot.bougerL(robot.points[7])",
        "robot.bougerL(robot.points[8], 0.05, 0.05)",
        "pince.lacher()",
        "robot.bougerL(robot.points[9], 0.1, 0.1)",
        "robot.bougerL(robot.points[10], 0.1, 0.1)",
        "robot.bougerL(robot.points[11], 0.1, 0.1)",
        "pince.prise()",
        "robot.bougerL(robot.points[12], 0.1, 0.1)",
        "robot.bougerL(robot.points[13], 0.1, 0.1)",
        "robot.bougerL(robot.points[14], 0.1, 0.1)",
        "robot.bougerL(robot.points[15], 0.1, 0.1)",
        "robot.bougerL(robot.points[16], 0.1, 0.1)",
        "robot.bougerL(robot.points[17], 0.1, 0.1)",
        "pince.lacher()",
        "robot.bougerL(robot.points[18], 0.1, 0.1)",
        "robot.bougerL(robot.points[19], 0.1, 0.1)",
        "robot.bougerL(robot.points[20], 0.1, 0.1)",
        "pince.prise()",
        "robot.bougerL(robot.points[21],0.1, 0.1)",
        "robot.bougerJ(robot.joints[3], 0.1, 0.1)",

        ### On insère ici l'instruction pour faire bouger le robot au bon emplacement du bac

        "pince.lacher()",
        ]
    
    try:
        while True:
            # Lecture sécurisée de la variable globale

            with out_lock:
                current_data = global_out
            

            
            # On vérifie que les données sont disponibles
            if global_out[0][0] is not None and global_out[1][0] is not None :

                

                # Si le capteur (par exemple global_out[0][0]) est à 1 et qu'on n'a pas encore déclenché la séquence
                if current_data[0][0] == 1 and not triggered:

                    emplacement_bac_libre = disponible(global_out[1])

                    if emplacement_bac_libre != -1 :
                        actions.insert(-1,f"robot.bougerL(pointsBoite[{emplacement_bac_libre}])")

                    triggered = True
                    print("Déclenchement de la séquence de mouvements.")
                    
                    # Exécuter la séquence des mouvements
                    for i in range(len(actions)):
                        print(f"Envoi de l'action {i+1} au robot via RTDE.")
                        eval(actions[i])
                        
                        # Attendre un délai pour permettre l'exécution du mouvement (pour la démonstration)
                        #time.sleep(2)
                    
                    if emplacement_bac_libre != -1 :
                        actions.pop()

                    print("Séquence de mouvements terminée.")
                
                # Réinitialiser l'état dès que la valeur revient à 0 afin de pouvoir déclencher de nouveau
                elif current_data[0][0] == 0 and triggered:
                    triggered = False
                    print("Réinitialisation de l'état de déclenchement.")
            
            # Petite pause pour éviter une boucle trop gourmande en CPU
            time.sleep(0.1)
    
    except Exception as e:
        print("Erreur lors de la communication avec le robot via RTDE :", e)
    

def disponible(l):
    out = []
    for i in range(len(l)):
        if not l[i]:
            out.append(i)

    if out != [] :
        return out[0]
    
    else :
        return -1

    


##########################################
# Démarrage des threads
##########################################

if __name__ == "__main__":
    # Lancer le thread du client MQTT
    mqtt_thread = threading.Thread(target=mqtt_client_thread, daemon=True)
    mqtt_thread.start()
    
    # Laisser un peu de temps pour la connexion MQTT
    time.sleep(1)
    
    # Lancer le thread de contrôle du robot via RTDE
    robot_thread = threading.Thread(target=robot_control_thread, daemon=True)
    robot_thread.start()

    robot_thread.join()  # Attendre la fin du thread robot si non-infini
    
    # Garder le programme actif pour le thread MQTT
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Arrêt du programme.")

