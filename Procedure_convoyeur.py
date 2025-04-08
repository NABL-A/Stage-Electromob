import requests
import time

ESP32_IP = "http://172.20.10.12" 

URL = f"{ESP32_IP}/read"

while True:
    try:
        response = requests.get(URL, timeout=5)
        if response.status_code == 200:
            data = response.json()
            out = []
            for i in range(2):
                out.append(data[f'pin{i+1}'])

            print(out)

        else:
            print(f"Erreur HTTP {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Erreur de connexion : {e}")
    
    time.sleep(0.05)  
