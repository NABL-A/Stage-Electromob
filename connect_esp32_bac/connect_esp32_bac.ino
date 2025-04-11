#include <WiFi.h>
#include <PubSubClient.h>

// Paramètres WiFi
const char* ssid = "NETGEAR_11N";  
const char* password = "Robotur5"; 

// Configuration IP statique
IPAddress local_IP(10, 2, 30, 164);   
IPAddress gateway(10, 2, 30, 161);     
IPAddress subnet(255, 255, 0, 0);    
IPAddress primaryDNS(8, 8, 8, 8);   

// Paramètres MQTT
const char* mqtt_server = "10.2.30.162";
const int mqtt_port = 1883;
const char* mqtt_topic = "capteurs_bac/etat";
const char* mqtt_client_id = "ESP32Client";  // ID client fixe pour meilleure traçabilité

// Configuration des broches
const int PIN1 = 2;
const int PIN2 = 4;
const int PIN3 = 21;
const int PIN4 = 22;
const int PIN5 = 23;

// Variables pour suivre l'état des capteurs
int lastPinStates[5] = {-1, -1, -1, -1, -1};  // Initialisation avec valeurs impossibles

// Variables pour gérer la publication
unsigned long lastPublishTime = 0;
const unsigned long PUBLISH_INTERVAL = 200;  // Intervalle minimum entre publications (ms)
const unsigned long FORCE_PUBLISH_INTERVAL = 2000;  // Publication forcée même sans changement (heartbeat)

// Compteurs pour la surveillance des connexions
unsigned long lastWifiReconnectAttempt = 0;
unsigned long lastMqttReconnectAttempt = 0;
int failedMqttPublishes = 0;
unsigned long lastDebugOutput = 0;

WiFiClient espClient;
PubSubClient client(espClient);

// Fonction de connexion WiFi améliorée
bool connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return true;
  }
  
  Serial.print("Connexion au WiFi...");
  
  // Tentative de connexion avec timeout
  unsigned long startAttemptTime = millis();
  
  // Reconnexion WiFi si nécessaire
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect();
    delay(100);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // Désactive le mode économie d'énergie pour plus de stabilité
    
    // Configuration IP statique
    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS)) {
      Serial.println("Configuration IP statique échouée, passage en DHCP");
    }
    
    WiFi.begin(ssid, password);
  }
  
  // Attente de connexion avec timeout
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnecté au WiFi");
    Serial.print("Adresse IP: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println("\nÉchec de connexion WiFi");
    return false;
  }
}

// Fonction de reconnexion MQTT améliorée
bool connectMQTT() {
  if (client.connected()) {
    return true;
  }
  
  Serial.print("Connexion au broker MQTT...");
  
  // Génération d'un identifiant client unique avec suffixe aléatoire pour éviter les conflits
  String clientId = mqtt_client_id;
  clientId += "-";
  clientId += String(random(0xffff), HEX);
  
  if (client.connect(clientId.c_str())) {
    Serial.println("connecté");
    failedMqttPublishes = 0;  // Réinitialisation du compteur d'erreurs
    return true;
  } else {
    Serial.print("échec, code=");
    Serial.print(client.state());
    Serial.println(" nouvelle tentative dans quelques secondes");
    return false;
  }
}

// Fonction pour lire l'état des capteurs
String readSensors(int currentPinValues[]) {
  // Lecture des valeurs actuelles
  currentPinValues[0] = digitalRead(PIN1);
  currentPinValues[1] = digitalRead(PIN2);
  currentPinValues[2] = digitalRead(PIN3);
  currentPinValues[3] = digitalRead(PIN4);
  currentPinValues[4] = digitalRead(PIN5);
  
  // Génération du message JSON
  String message = "{";
  message += "\"pin1\": " + String(currentPinValues[0]);
  message += ", \"pin2\": " + String(currentPinValues[1]);
  message += ", \"pin3\": " + String(currentPinValues[2]);
  message += ", \"pin4\": " + String(currentPinValues[3]);
  message += ", \"pin5\": " + String(currentPinValues[4]);
  message += "}";
  
  return message;
}

// Fonction pour vérifier si l'état a changé
bool hasStateChanged(int currentPinValues[]) {
  for (int i = 0; i < 5; i++) {
    if (currentPinValues[i] != lastPinStates[i]) {
      return true;
    }
  }
  return false;
}

// Fonction pour mettre à jour les états précédents
void updateLastStates(int currentPinValues[]) {
  for (int i = 0; i < 5; i++) {
    lastPinStates[i] = currentPinValues[i];
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n\n--- Démarrage du système de capteurs bac ---");
  
  // Configuration des broches en entrée avec pull-up interne pour plus de stabilité
  pinMode(PIN1, INPUT_PULLUP);
  pinMode(PIN2, INPUT_PULLUP);
  pinMode(PIN3, INPUT_PULLUP);
  pinMode(PIN4, INPUT_PULLUP);
  pinMode(PIN5, INPUT_PULLUP);
  
  // Configuration du client MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setBufferSize(256);  // Augmentation de la taille du buffer si nécessaire
  
  // Première tentative de connexion
  connectWiFi();
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Gestion de la connexion WiFi
  if (WiFi.status() != WL_CONNECTED) {
    if (currentMillis - lastWifiReconnectAttempt > 10000) {  // Tentative toutes les 10 secondes
      lastWifiReconnectAttempt = currentMillis;
      Serial.println("WiFi déconnecté. Tentative de reconnexion...");
      connectWiFi();
    }
    return;  // Ne pas continuer si le WiFi est déconnecté
  }
  
  // Gestion de la connexion MQTT
  if (!client.connected()) {
    if (currentMillis - lastMqttReconnectAttempt > 5000) {  // Tentative toutes les 5 secondes
      lastMqttReconnectAttempt = currentMillis;
      connectMQTT();
    }
    return;  // Ne pas continuer si MQTT est déconnecté
  }
  
  // Maintien de la connexion MQTT
  client.loop();
  
  // Lecture de l'état actuel des capteurs
  int currentPinValues[5];
  String message = readSensors(currentPinValues);
  
  // Vérifier s'il y a eu un changement d'état ou si l'intervalle de publication forcée est atteint
  bool stateChanged = hasStateChanged(currentPinValues);
  bool forcePublish = (currentMillis - lastPublishTime >= FORCE_PUBLISH_INTERVAL);
  
  // Publication MQTT si nécessaire et avec respect de l'intervalle minimum
  if ((stateChanged || forcePublish) && (currentMillis - lastPublishTime >= PUBLISH_INTERVAL)) {
    Serial.print("Publication: ");
    Serial.println(message);
    
    if (client.publish(mqtt_topic, message.c_str(), false)) {  // false = non-retenu
      // Publication réussie
      lastPublishTime = currentMillis;
      updateLastStates(currentPinValues);
      failedMqttPublishes = 0;  // Réinitialisation du compteur d'erreurs
    } else {
      // Échec de publication
      failedMqttPublishes++;
      Serial.print("Échec de publication MQTT (");
      Serial.print(failedMqttPublishes);
      Serial.println(" échecs consécutifs)");
      
      // Si plusieurs échecs consécutifs, forcer une reconnexion
      if (failedMqttPublishes >= 3) {
        Serial.println("Trop d'échecs de publication, reconnexion MQTT");
        client.disconnect();
      }
    }
  }
  
  // Affichage d'un message de débogage périodique
  if (currentMillis - lastDebugOutput > 10000) {  // Toutes les 10 secondes
    lastDebugOutput = currentMillis;
    Serial.print("État du système - WiFi: ");
    Serial.print(WiFi.status() == WL_CONNECTED ? "Connecté" : "Déconnecté");
    Serial.print(" | MQTT: ");
    Serial.print(client.connected() ? "Connecté" : "Déconnecté");
    Serial.print(" | RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.print("dBm | Capteurs: ");
    for (int i = 0; i < 5; i++) {
      Serial.print(currentPinValues[i]);
      if (i < 4) Serial.print(",");
    }
    Serial.println();
  }
  
  // Petit délai pour la stabilité
  delay(10);
}
