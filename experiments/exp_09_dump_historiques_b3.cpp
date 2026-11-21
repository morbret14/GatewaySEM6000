/*
 * =====================================================================================
 * EXPÉRIENCE 09 : Dump Intégral des 30 Jours et 12 Mois & Inspection Exhaustive de B3
 * =====================================================================================
 * Cible : Prise Voltcraft SEM6000-FR (MAC: 94:a9:a8:1a:37:0c)
 *
 * Objectif :
 *   Lire et afficher les 4 octets bruts [B0, B1, B2, B3] de :
 *     - Tous les 30 jours de l'historique journalier (0x0B00)
 *     - Tous les 12 mois de l'historique mensuel (0x0C00)
 *   afin de vérifier la plage de valeurs réelles de B3 (0..100 vs 0..255).
 * =====================================================================================
 */

#include <Arduino.h>
#include "SEM6000Device.h"

static const char* TARGET_MAC = "94:a9:a8:1a:37:0c";
static const uint32_t TARGET_PIN = 0x00000000;
static const int BUTTON_BOOT_PIN = 0;

static SEM6000Device device(TARGET_MAC, TARGET_PIN);

void waitUserPrompt(const char* title, const char* instructions) {
    Serial.println("\n================================================================================");
    Serial.printf(" >>> ÉTAPE : %s\n", title);
    Serial.println("--------------------------------------------------------------------------------");
    Serial.println(instructions);
    Serial.println("\n  👉 [Appuyez sur le bouton physique BOOT de l'ESP32 ou envoyez 'Entrée' en console]");
    Serial.println("================================================================================\n");

    while (Serial.available()) Serial.read();

    uint32_t lastBeacon = millis();
    while (true) {
        if (digitalRead(BUTTON_BOOT_PIN) == LOW) {
            delay(150);
            while (digitalRead(BUTTON_BOOT_PIN) == LOW) delay(20);
            break;
        }
        if (Serial.available()) {
            while (Serial.available()) Serial.read();
            break;
        }
        if (millis() - lastBeacon >= 3000) {
            Serial.printf("  [EN ATTENTE - %s] Appuyez sur le bouton BOOT (GPIO 0) ou 'Entrée'...\n", title);
            lastBeacon = millis();
        }
        delay(50);
    }
    Serial.println(">>> Action validée par l'utilisateur ! Démarrage de l'étape...\n");
}

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_BOOT_PIN, INPUT_PULLUP);

    uint32_t startWait = millis();
    while (!Serial && (millis() - startWait < 4000)) {
        delay(50);
    }
    delay(500);

    NimBLEDevice::init("Gateway_Exp09");

    Serial.println("\n\n");
    Serial.println("================================================================================");
    Serial.println("  🔍 EXPÉRIENCE 09 : DUMP EXHAUSTIF DES 30 JOURS ET 12 MOIS (ANALYSE B3)        ");
    Serial.println("================================================================================");

    waitUserPrompt("DÉMARRAGE DU DUMP", "Appuyez sur BOOT pour vous connecter et scanner tous les registres.");

    Serial.printf("[BLE] Connexion à %s...\n", TARGET_MAC);
    if (!device.connect(8000)) {
        Serial.println("[BLE] ❌ Échec de connexion.");
        while (true) delay(1000);
    }

    if (device.authenticate32((uint32_t)TARGET_PIN) != SEM_OK) {
        Serial.println("[BLE] ❌ Échec authentification.");
        device.disconnect();
        while (true) delay(1000);
    }
    Serial.println("[BLE] ✅ Connecté et authentifié !\n");

    // =================================================================================
    // 1. DUMP DES 30 JOURS (0x0B00)
    // =================================================================================
    uint32_t dummyDaily[30];
    Serial.println("================================================================================");
    Serial.println("  📅 HISTORIQUE DES 30 DERNIERS JOURS (0x0B00) — LES 4 OCTETS BRUTS");
    Serial.println("================================================================================");
    Serial.println("  Slot | Jour relatif | Énergie (Wh) | Octets [B0 B1 B2] | Octet B3 (Hex) | Octet B3 (Dec)");
    Serial.println("-------+--------------+--------------+-------------------+----------------+---------------");

    if (device.requestMonthHistory(dummyDaily) == SEM_OK) {
        const uint8_t* rawBuf = nullptr;
        size_t len = device.getLastRxBuffer(&rawBuf);

        uint8_t minB3 = 255;
        uint8_t maxB3 = 0;
        int countNonZero = 0;

        for (int i = 0; i < 30; i++) {
            size_t idx = 4 + (i * 4);
            uint8_t b0 = rawBuf[idx];
            uint8_t b1 = rawBuf[idx + 1];
            uint8_t b2 = rawBuf[idx + 2];
            uint8_t b3 = rawBuf[idx + 3];
            uint32_t wh = ((uint32_t)b0 << 16) | ((uint32_t)b1 << 8) | b2;

            int daysAgo = 29 - i;
            const char* tag = (i == 29) ? " (J-0 / Aujourd'hui)" :
                              (i == 28) ? " (J-1 / Hier)" : "";

            Serial.printf("  [%02d] | J-%02d %-11s | %8lu Wh   | [0x%02X 0x%02X 0x%02X]   |     0x%02X     |      %3u\n",
                          i, daysAgo, tag, (unsigned long)wh, b0, b1, b2, b3, b3);

            if (wh > 0) {
                countNonZero++;
                if (b3 < minB3) minB3 = b3;
                if (b3 > maxB3) maxB3 = b3;
            }
        }
        Serial.println("--------------------------------------------------------------------------------");
        Serial.printf("  📊 STATISTIQUES B3 SUR JOURS ACTIFS (%d jours > 0 Wh) :\n", countNonZero);
        Serial.printf("     Valeur Minimale B3 : %u (0x%02X)\n", minB3, minB3);
        Serial.printf("     Valeur Maximale B3 : %u (0x%02X)\n", maxB3, maxB3);
        Serial.println("================================================================================\n");
    }

    // =================================================================================
    // 2. DUMP DES 12 MOIS (0x0C00)
    // =================================================================================
    uint32_t dummyMonthly[12];
    Serial.println("================================================================================");
    Serial.println("  📆 HISTORIQUE DES 12 DERNIERS MOIS (0x0C00) — LES 4 OCTETS BRUTS");
    Serial.println("================================================================================");
    Serial.println("  Slot | Mois relatif | Énergie (Wh) | Octets [B0 B1 B2] | Octet B3 (Hex) | Octet B3 (Dec)");
    Serial.println("-------+--------------+--------------+-------------------+----------------+---------------");

    if (device.requestYearHistory(dummyMonthly) == SEM_OK) {
        const uint8_t* rawBuf = nullptr;
        size_t len = device.getLastRxBuffer(&rawBuf);

        uint8_t minB3 = 255;
        uint8_t maxB3 = 0;
        int countNonZero = 0;

        for (int i = 0; i < 12; i++) {
            size_t idx = 4 + (i * 4);
            uint8_t b0 = rawBuf[idx];
            uint8_t b1 = rawBuf[idx + 1];
            uint8_t b2 = rawBuf[idx + 2];
            uint8_t b3 = rawBuf[idx + 3];
            uint32_t wh = ((uint32_t)b0 << 16) | ((uint32_t)b1 << 8) | b2;

            int monthsAgo = 11 - i;
            const char* tag = (i == 11) ? " (M-0 / Mois en cours)" :
                              (i == 10) ? " (M-1 / Mois dernier)" : "";

            Serial.printf("  [%02d] | M-%02d %-13s | %8lu Wh   | [0x%02X 0x%02X 0x%02X]   |     0x%02X     |      %3u\n",
                          i, monthsAgo, tag, (unsigned long)wh, b0, b1, b2, b3, b3);

            if (wh > 0) {
                countNonZero++;
                if (b3 < minB3) minB3 = b3;
                if (b3 > maxB3) maxB3 = b3;
            }
        }
        Serial.println("--------------------------------------------------------------------------------");
        Serial.printf("  📊 STATISTIQUES B3 SUR MOIS ACTIFS (%d mois > 0 Wh) :\n", countNonZero);
        Serial.printf("     Valeur Minimale B3 : %u (0x%02X)\n", minB3, minB3);
        Serial.printf("     Valeur Maximale B3 : %u (0x%02X)\n", maxB3, maxB3);
        Serial.println("================================================================================\n");
    }

    Serial.println("[BLE] Déconnexion propre...");
    device.disconnect();
    Serial.println(">>> Fin du dump !\n");
}

void loop() {
    delay(1000);
}
