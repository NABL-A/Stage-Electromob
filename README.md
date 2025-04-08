### 1ère étape : Alimenter l'esp32

La carte esp32 a besoin d'être alimenté avec du 5V pour être fonctionnel.

### 2ème étape : Configurer le code embarqué

On utilise l'IDE Arduino pour téléverser le code que l'on veut faire utiliser à la carte. Ici, on souhaite accéder
aux valeurs des sorties (GPIO) afin de savoir si un capteur détecte une présence ou non.

### Code :

Pour accéder à ces infos, on doit connecter l'esp à un réseau pour que l'on puisse ensuite accéder à celle-ci depuis
un appareil connecté au même réseau. Tout se fait par requête http (port 80)

On doit d'abord initialiser le baud rate d'échange d'informations (ici 115200) pour assurer la rapidité de l'envoie
et la fiabilité du processus. 

```cpp
Serial.begin(115200)
```

On définit le ssid et le password du réseau que l'on va utiliser 

```cpp
Wifi.begin(ssid,password)
```

Une fois connecté, une IP est attribué à la carte que l'on accède avec la méthode :

```cpp
Serial.println(WiFi.localIP())
```

Puis, on peut enfin accéder aux informations importantes en les rajoutant sur le serveur
http sous format json pour une lecture standardisée et facile à utiliser.

```cpp
  server.on("/read", HTTP_GET, []() {
    server.send(200, "application/json", readInputs());
  });
```

Pour la partie récupération des données : le script python envoit des requêtes http à l'adresse ip
locale de l'esp32 sur le réseau et traduit les données json.

```python
import requests
import time

ESP32_IP = "http://172.20.10.12" 

URL = f"{ESP32_IP}/read"

while True:
    try:
        response = requests.get(URL, timeout=5)
        if response.status_code == 200:
            data = response.json()
            print(f"Valeur GPIO 21 : {data['pin1']}")
        else:
            print(f"Erreur HTTP {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Erreur de connexion : {e}")
    
    time.sleep(0.05)  
```