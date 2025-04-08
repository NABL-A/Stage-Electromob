1ère étape : Alimenter l'esp32

La carte esp32 a besoin d'être alimenté avec du 5V pour être fonctionnel.

2ème étape : Configurer le code embarqué

On utilise l'IDE Arduino pour téléverser le code que l'on veut faire utiliser à la carte. Ici, on souhaite accéder
aux valeurs des sorties (GPIO) afin de savoir si un capteur détecte une présence ou non.

Code :

Pour accéder à ces infos, on doit connecter l'esp à un réseau pour que l'on puisse ensuite accéder à celle-ci depuis
un appareil connecté au même réseau. Tout se fait par requête http (port 80)

On doit d'abord initialiser le baud rate d'échange d'informations (ici 115200) pour assurer la rapidité de l'envoie
et la fiabilité du processus. 

```
Serial.begin(115200)
```

On définit le ssid et le password du réseau que l'on va utiliser 

```
Wifi.begin(ssid,password)
```

Une fois connecté, une IP est attribué à la carte que l'on accède avec la méthode :

```
Serial.println(WiFi.localIP())
```

Puis, on peut enfin accéder aux informations importantes en les rajoutant sur le serveur
http sous format json pour une lecture standardisée et facile à utiliser.

```
  server.on("/read", HTTP_GET, []() {
    server.send(200, "application/json", readInputs());
  });
```

