import requests
import time

ESP32_CONVOYEUR_IP = "http://172.20.10.12"
ESP32_BAC_IP = "http://172.20.10.13"

URL_CONVOYEUR = f"{ESP32_CONVOYEUR_IP}/read"
URL_BAC = f"{ESP32_BAC_IP}/read"

while True:
    out_convoyeur = []
    out_bac = []
    
    try:
        response_convoyeur = requests.get(URL_CONVOYEUR, timeout=5)
        if response_convoyeur.status_code == 200:
            data_convoyeur = response_convoyeur.json()
            for i in range(2):
                out_convoyeur.append(data_convoyeur.get(f'pin{i+1}', None))
        else:
            print(f"Erreur de réponse pour le convoyeur : {response_convoyeur.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Erreur de connexion pour le convoyeur : {e}")
    
    try:
        response_bac = requests.get(URL_BAC, timeout=5)
        if response_bac.status_code == 200:
            data_bac = response_bac.json()
            for i in range(5):
                out_bac.append(data_bac.get(f'pin{i+1}', None))
        else:
            print(f"Erreur de réponse pour le bac : {response_bac.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Erreur de connexion pour le bac : {e}")
    
    totalout = [out_convoyeur, out_bac]
    print(totalout)
    
    time.sleep(0.5)
