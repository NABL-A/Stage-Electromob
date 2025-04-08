#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "iPhone Alban";  
const char* password = "Squeezie3"; 

WebServer server(80);

String readInputs() {
  int pin1Value = digitalRead(21);
  return "{\"pin1\": " + String(pin1Value) + "}";
}

void setup() {
  Serial.begin(115200);
  pinMode(21, INPUT); 

  Serial.println("Réinitialisation du WiFi...");
  WiFi.disconnect(true);
  delay(1000);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) {
    Serial.print("Statut WiFi : ");
    Serial.println(WiFi.status());
    delay(1000);
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connecté au WiFi !");
    Serial.print("Adresse IP : ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Échec de connexion WiFi !");
    Serial.println("Vérifiez SSID, mot de passe et si le réseau est en 2.4 GHz.");
  }

  server.on("/read", HTTP_GET, []() {
    server.send(200, "application/json", readInputs());
  });

  server.begin();
}

void loop() {
  server.handleClient(); 
}
