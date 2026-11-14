/*
 * =====================================================================================
 * EXPÉRENCE 01 : Scanner BLE pour Prises Voltcraft SEM6000
 * =====================================================================================
 * Objectif :
 *   - Scanner les périphériques Bluetooth Low Energy à portée de l'ESP32-S3.
 *   - Identifier spécifiquement les prises SEM6000 (par leur nom ou leur service UUID FFF0).
 *   - Afficher l'adresse MAC et le RSSI (puissance du signal en dBm) pour chaque prise.
 *
 * Résultat attendu sur le Moniteur Série (115200 bauds) :
 *   >>> [SEM6000 RECONNUE] MAC: xx:xx:xx:xx:xx:xx | RSSI: -65 dBm | Nom: Voltcraft SEM6000
 * =====================================================================================
 */

#include <Arduino.h>
#include <NimBLEDevice.h>

static const NimBLEUUID SERVICE_FFF0_UUID("0000fff0-0000-1000-8000-00805f9b34fb");

void setup() {
    Serial.begin(115200);

    // Pause d'initialisation spécifique à l'USB CDC natif de l'ESP32-S3
    delay(2000);

    Serial.println("\n========================================================");
    Serial.println("   Passerelle SEM6000 — EXP 01 : Scanner BLE           ");
    Serial.println("========================================================");

    NimBLEDevice::init("Gateway_Scanner");

    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setActiveScan(true);  // Active scan pour récupérer le nom complet
    pScan->setInterval(100);     // Intervalle de scan en ms
    pScan->setWindow(99);        // Fenêtre d'écoute en ms
}

void loop() {
    Serial.println("\n[BLE] Démarrage d'un scan de 5 secondes...");

    NimBLEScan* pScan = NimBLEDevice::getScan();
    
    // getResults(ms, continue) effectue un scan bloquant de 5000 ms en NimBLE 2.x
    NimBLEScanResults results = pScan->getResults(5000, false);

    Serial.printf("[BLE] Scan terminé : %d appareil(s) détecté(s).\n", results.getCount());
    Serial.println("--------------------------------------------------------");

    int semCount = 0;

    for (int i = 0; i < results.getCount(); i++) {
        const NimBLEAdvertisedDevice* pDevice = results.getDevice(i);
        if (!pDevice) continue;

        String name = pDevice->getName().c_str();
        String mac = pDevice->getAddress().toString().c_str();
        int rssi = pDevice->getRSSI();

        bool isSEM = (name.indexOf("SEM6000") >= 0);
        if (pDevice->haveServiceUUID() && pDevice->isAdvertisingService(SERVICE_FFF0_UUID)) {
            isSEM = true;
        }

        if (isSEM) {
            semCount++;
            Serial.printf("  >>> [PRISE SEM6000 RECONNUE] MAC: %s | RSSI: %d dBm | Nom: %s\n",
                          mac.c_str(), rssi, name.c_str());
        } else {
            Serial.printf("      Périphérique BLE : MAC: %s | RSSI: %d dBm | Nom: %s\n",
                          mac.c_str(), rssi, (name.length() > 0) ? name.c_str() : "(anonyme)");
        }
    }

    Serial.println("--------------------------------------------------------");
    if (semCount == 0) {
        Serial.println("  [info] Aucune prise SEM6000 repérée lors de ce cycle.");
        Serial.println("         (Vérifiez que la prise est branchée et sous tension)");
    } else {
        Serial.printf("  [succès] %d prise(s) SEM6000 repérée(s) !\n", semCount);
    }

    // Libération de la mémoire tampon des résultats
    pScan->clearResults();

    Serial.println("[BLE] Prochain scan dans 10 secondes...\n");
    delay(10000);
}
