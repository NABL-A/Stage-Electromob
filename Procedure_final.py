import json
import paho.mqtt.client as mqtt
import numpy as np 
import rtde_receive
import rtde_control
import dashboard_client
import cv2 as cv
import math
import time
import socket


##########################################
# CONNNEXION AU RESEAU ET AU BROKET MQTT
##########################################

MQTT_BROKER = "10.2.30.162"
MQTT_PORT = 1883

TOPIC_B = "capteurs_bac/etat"
TOPIC_C = "capteurs_convoyeur/etat"

# Variables globales pour stocker les données des deux topics
convoyeur_data = [None] * 2  # Liste pour capteurs_convoyeur (pin1 à pin2)
bac_data = [None] * 5        # Liste pour capteurs_bac (pin1 à pin5)

def on_connect(client, userdata, flags, rc):
    print("Connecté au broker MQTT avec le code de retour", rc)
    client.subscribe(TOPIC_B)
    client.subscribe(TOPIC_C)

def on_message(client, userdata, msg):
    global convoyeur_data, bac_data
    try:
        # Décoder le message JSON
        data = json.loads(msg.payload.decode('utf-8'))
        
        # Mettre à jour les données selon le topic
        if msg.topic == TOPIC_C:  # capteurs_convoyeur/etat (2 pins)
            convoyeur_data = [data["pin1"], data["pin2"]]
        elif msg.topic == TOPIC_B:  # capteurs_bac/etat (5 pins)
            bac_data = [data[f"pin{i+1}"] for i in range(5)]
        
        # Afficher la liste combinée sous la forme demandée
        out = [convoyeur_data, bac_data]
        print(out)
        
    except Exception as e:
        print("Erreur lors du traitement du message :", e)

# Initialisation du client avec la version de l'API de rappel
client = mqtt.Client("PythonClient")
client.on_connect = on_connect
client.on_message = on_message

client.connect(MQTT_BROKER, MQTT_PORT, 60)

client.loop_forever()

