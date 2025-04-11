#include <WiFi.h>

void setup() {
  // Initialisation du port série
  Serial.begin(115200);
  delay(1000);  // Petit délai pour la stabilisation du port série
  
  Serial.println("=== Scan des réseaux Wi-Fi ===");
  
  // Configuration en mode station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();  // Optionnel : déconnecte toute connexion antérieure
  
  Serial.println("Recherche des réseaux disponibles...");
  int n = WiFi.scanNetworks();   // Lancement du scan
  Serial.println("Scan terminé");
  
  if (n == 0) {
    Serial.println("Aucun réseau détecté.");
  } else {
    Serial.print(n);
    Serial.println(" réseaux détectés :");
    for (int i = 0; i < n; i++) {
      // Affiche le numéro, le SSID, le RSSI et le canal de chaque réseau
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(" dBm) - Canal: ");
      Serial.println(WiFi.channel(i));
      delay(10);
    }
  }
  
  // Libère la mémoire allouée pour les résultats du scan
  WiFi.scanDelete();
}

void loop() {
  // Pour ce simple scan statique, rien n'est requis dans loop().
  // Vous pouvez néanmoins relancer un scan périodiquement si besoin.
  delay(30000);  // Par exemple, attendre 30 secondes avant le prochain scan (si placé dans loop)
}
