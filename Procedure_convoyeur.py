import json
import paho.mqtt.client as mqtt

MQTT_BROKER = "10.2.30.162"
MQTT_PORT = 1883

TOPIC = "capteurs_convoyeur/etat"

def on_connect(client, userdata, flags, rc):
    print("Connecté au broker MQTT avec le code de retour", rc)
    client.subscribe(TOPIC)

def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode('utf-8'))
        out = []
        for i in range(2):
            out.append(data[f"pin{i+1}"])
        print(out)
    except Exception as e:
        print("Erreur lors du traitement du message :", e)

client = mqtt.Client("PythonClient")
client.on_connect = on_connect
client.on_message = on_message

client.connect(MQTT_BROKER, MQTT_PORT, 60)

client.loop_forever()