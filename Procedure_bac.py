import requests
import time

ESP32_IP = "http://172.20.10.13" 

URL = f"{ESP32_IP}/read"

while True:
    try:
        response = requests.get(URL, timeout=5)
        if response.status_code == 200:
            data = response.json()
            if data['pin1'] == 1 :
                print("Emplacement 1 occupé")
            if data['pin2'] == 1 :
                print("Emplacement 2 occupé")
            if data['pin3'] == 1 :
                print("Emplacement 3 occupé")                
            if data['pin4'] == 1 :
                print("Emplacement 4 occupé")


        else:
            print(f"Erreur HTTP {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Erreur de connexion : {e}")
    
    time.sleep(0.05)  
