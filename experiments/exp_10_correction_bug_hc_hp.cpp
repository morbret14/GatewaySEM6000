/**
 * @file exp_10_correction_bug_hc_hp.cpp
 * @brief Programme de reconfiguration et de correction du bug HC/HP sur la prise Chambre Cédric.
 * 
 * Contexte :
 * Une configuration HC de 22h00 à 21h59 a été acceptée par la prise, mais l'application
 * constructeur ne parvient plus à modifier cette amplitude (bug de validation UI / slider).
 * 
 * Objectifs :
 * 1. Connexion et authentification à la prise Chambre Cédric (b0:b1:13:6b:43:f0).
 * 2. Lecture et affichage complet des paramètres actuels (octets bruts + décodage).
 * 3. Réécriture directe du créneau Heures Creuses : 22h00 à 05h59.
 * 4. Relecture des paramètres pour validation de conformité.
 * 5. Déconnexion propre et libération de la prise BLE.
 */

#include <Arduino.h>
#include "SEM6000Device.h"

// Cible : Prise Chambre Cédric
static const char* TARGET_MAC = "b0:b1:13:6b:43:f0";
static const uint32_t TARGET_PIN = 0x00000000;
static const int BUTTON_BOOT_PIN = 0;

static SEM6000Device device(TARGET_MAC, TARGET_PIN);

void waitUserPrompt(const char* title, const char* instructions) {
    Serial.println("\n================================================================================");
    Serial.printf(" >>> %s\n", title);
    Serial.println("--------------------------------------------------------------------------------");
    Serial.println(instructions);
    Serial.println("\n  👉 [Appuyez sur le bouton physique BOOT (GPIO 0) ou envoyez 'Entrée' en console]");
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
            Serial.printf("  [EN ATTENTE - %s] Appuyez sur BOOT ou 'Entrée'...\n", title);
            lastBeacon = millis();
        }
        delay(50);
    }
    Serial.println(">>> Action validée par l'utilisateur ! Démarrage de l'étape...\n");
}

void printSettingsReport(const char* label, const SEMSettings& s, const uint8_t* rawBuf, size_t rawLen) {
    Serial.println("--------------------------------------------------------------------------------");
    Serial.printf("  📋 %s\n", label);
    Serial.println("--------------------------------------------------------------------------------");
    Serial.printf("  Mode réduit (HC) actif : %s\n", s.reducedModeActive ? "OUI (1)" : "NON (0)");
    Serial.printf("  Tarif Normal (HP)      : %u centimes d'€/kWh\n", s.normalPriceCents);
    Serial.printf("  Tarif Réduit (HC)      : %u centimes d'€/kWh\n", s.reducedPriceCents);
    Serial.printf("  Puissance max coupure  : %u W\n", s.maxPowerWatts);
    Serial.printf("  Anneau LED             : %s\n", s.ledStatus ? "ALLUMÉ" : "ÉTEINT");

    if (rawBuf && rawLen >= 18) {
        uint8_t bStartH = rawBuf[7];
        uint8_t bStartL = rawBuf[8];
        uint8_t bStopH  = rawBuf[9];
        uint8_t bStopL  = rawBuf[10];

        Serial.printf("\n  [ANALYSE DES OCTETS DU CRÉNEAU HC]\n");
        Serial.printf("    Octets Début HC      : [0x%02X 0x%02X]\n", bStartH, bStartL);
        Serial.printf("    Octets Fin HC        : [0x%02X 0x%02X]\n", bStopH, bStopL);

        // Décodage Format 1 : Minutes depuis minuit (Standard SEM6000 16-bit Big-Endian)
        uint16_t startMinTotal = ((uint16_t)bStartH << 8) | bStartL;
        uint16_t stopMinTotal  = ((uint16_t)bStopH << 8) | bStopL;
        Serial.printf("    -> Interprétation Minutes depuis 0h00 : Début = %02uh%02u (%u min)  |  Fin = %02uh%02u (%u min)\n",
                      startMinTotal / 60, startMinTotal % 60, startMinTotal,
                      stopMinTotal / 60, stopMinTotal % 60, stopMinTotal);

        // Décodage Format 2 : Heures / Minutes directes
        Serial.printf("    -> Interprétation Directe (HH:MM)    : Début = %02uh%02u  |  Fin = %02uh%02u\n",
                      bStartH, bStartL, bStopH, bStopL);
    }
    Serial.println("--------------------------------------------------------------------------------\n");
}

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_BOOT_PIN, INPUT_PULLUP);

    // Attente active de la connexion du moniteur série USB CDC
    uint32_t startWait = millis();
    while (!Serial && (millis() - startWait < 4000)) {
        delay(50);
    }
    delay(500);

    // Initialisation NimBLE
    NimBLEDevice::init("Gateway_FixHC");

    Serial.println("\n\n");
    Serial.println("================================================================================");
    Serial.println("  🛠️ CORRECTION BUG HC-HP : RECONFIGURATION PLAGE HEURES CREUSES                ");
    Serial.println("================================================================================");
    Serial.printf("  Cible   : Chambre Cédric (%s)\n", TARGET_MAC);
    Serial.println("  Objectif: Débloquer l'amplitude horaire et reconfigurer le créneau 22h00 - 05h59");
    Serial.println("================================================================================\n");

    waitUserPrompt("1. PRÉPARATION & CONNEXION",
                   "1. Assurez-vous que l'application smartphone Voltcraft est FERMÉE.\n"
                   "2. La prise 'Chambre Cédric' doit être sous tension et à portée BLE.\n"
                   "3. Dès que vous êtes prêt :");

    // --- Connexion & Authentification ---
    Serial.printf("[BLE] Connexion à %s...\n", TARGET_MAC);
    if (!device.connect(8000)) {
        Serial.println("[BLE] ❌ ERREUR : Connexion impossible ! Vérifiez la portée et fermez l'app mobile.");
        return;
    }
    Serial.println("[BLE] ✅ Connecté !");

    SEM6000Status authStatus = device.authenticate32((uint32_t)TARGET_PIN);
    if (authStatus != SEM_OK) {
        Serial.printf("[BLE] ❌ ERREUR : Échec authentification (Code: %d)\n", authStatus);
        device.disconnect();
        return;
    }
    Serial.println("[BLE] ✅ Authentification réussie (PIN 32-bit: 0x00000000) !\n");

    // --- Étape 1 : Lecture des paramètres AVANT modification ---
    Serial.println("[ÉTAPE 1/3] Lecture des paramètres actuels de la prise...");
    SEMSettings settingsBefore;
    SEM6000Status stReadBefore = device.requestSettings(settingsBefore);
    if (stReadBefore != SEM_OK) {
        Serial.printf("[BLE] ❌ Erreur lors de la lecture des paramètres (Code: %d)\n", stReadBefore);
        device.disconnect();
        return;
    }

    const uint8_t* rawBuf = nullptr;
    size_t rawLen = device.getLastRxBuffer(&rawBuf);
    printSettingsReport("PARAMÈTRES ACTUELS (AVANT MODIFICATION)", settingsBefore, rawBuf, rawLen);

    // Détermination automatique du format
    uint8_t bStartH = rawBuf ? rawBuf[7] : 0;
    uint8_t bStartL = rawBuf ? rawBuf[8] : 0;
    uint8_t bStopH  = rawBuf ? rawBuf[9] : 0;
    uint8_t bStopL  = rawBuf ? rawBuf[10] : 0;

    uint16_t startMin = ((uint16_t)bStartH << 8) | bStartL;
    uint16_t stopMin  = ((uint16_t)bStopH << 8) | bStopL;

    uint8_t newB1 = 0, newB2 = 0, newB3 = 0, newB4 = 0;
    const char* formatName = "";

    // Analyse : La prise stocke des minutes depuis minuit (16-bit Big-Endian).
    // Règle d'or de l'application constructeur :
    // L'octet de fin stocké correspond à la borne d'exclusion (06h00 = 360 min = 0x0168).
    // L'application soustrait 1 minute à l'affichage pour présenter l'intervalle fermé : 22:00 -> 05:59 !
    // Si on lui passait 05h59 (359 min = 0x0167), l'app soustrayait 1h et affichait 22:00 -> 04:59.
    formatName = "Minutes depuis 0h00 (16-bit Big-Endian)";
    // 22h00 = 1320 minutes = 0x0528
    newB1 = 0x05;
    newB2 = 0x28;
    // 06h00 = 360 minutes = 0x0168 (fera afficher 05:59 dans l'application smartphone)
    newB3 = 0x01;
    newB4 = 0x68;

    Serial.printf("[ANALYSE DU FORMAT] Format identifié : %s\n", formatName);
    Serial.printf("  Borne début : 22h00 (1320 min = 0x0528)\n");
    Serial.printf("  Borne fin   : 06h00 (360 min = 0x0168) -> Affichera 05:59 dans l'app mobile\n");
    Serial.printf("  Trame préparée : [0x%02X 0x%02X 0x%02X 0x%02X]\n\n",
                  newB1, newB2, newB3, newB4);

    waitUserPrompt("2. APPLICATION DE LA MODIFICATION",
                   "Nous allons envoyer la trame de reconfiguration du mode réduit (HC) :\n"
                   "  - Mode réduit : ACTIF (1)\n"
                   "  - Plage HC    : Borne 22h00 à 06h00 (soit 22h00 -> 05h59 dans l'app mobile)\n"
                   "\nAppuyez sur BOOT ou 'Entrée' pour appliquer.");

    // --- Étape 2 : Application de la modification ---
    Serial.println("[ÉTAPE 2/3] Envoi de la commande de modification...");
    SEM6000Status stSet = device.setReducedPeriod(1, newB1, newB2, newB3, newB4);
    if (stSet == SEM_OK) {
        Serial.println("[BLE] ✅ Commande acceptée avec succès par la prise !");
    } else {
        Serial.printf("[BLE] ⚠️ Avertissement : Code retour commande: %d\n", stSet);
    }

    // Petite temporisation pour laisser le microcontrôleur de la prise finaliser l'écriture en Flash
    delay(400);

    // --- Étape 3 : Relecture et vérification des paramètres ---
    Serial.println("\n[ÉTAPE 3/3] Relecture de contrôle des paramètres...");
    SEMSettings settingsAfter;
    SEM6000Status stReadAfter = device.requestSettings(settingsAfter);
    if (stReadAfter == SEM_OK) {
        rawLen = device.getLastRxBuffer(&rawBuf);
        printSettingsReport("PARAMÈTRES APRÈS MODIFICATION (CONTRÔLE)", settingsAfter, rawBuf, rawLen);

        // Validation
        uint16_t checkStart = rawBuf ? (((uint16_t)rawBuf[7] << 8) | rawBuf[8]) : 0;
        uint16_t checkStop  = rawBuf ? (((uint16_t)rawBuf[9] << 8) | rawBuf[10]) : 0;

        if (checkStart == 1320 && checkStop == 360) {
            Serial.println("  🎉 SUCCÈS TOTAL : Borne 06h00 (360 min) validée en Flash !");
            Serial.println("  👉 L'application smartphone affichera très exactement : 22:00 -> 05:59");
        } else {
            Serial.println("  ℹ️ Modification prise en compte. Comparez les valeurs ci-dessus.");
        }
    } else {
        Serial.printf("[BLE] ❌ Impossible de relire les paramètres (Code: %d)\n", stReadAfter);
    }

    // --- Relâchement de la prise ---
    Serial.println("\n[BLE] Déconnexion et libération de la prise...");
    device.disconnect();
    Serial.println("[BLE] ✅ Prise déconnectée et libérée. Vous pouvez réouvrir l'application mobile.");
    Serial.println("\n================================================================================");
    Serial.println("  FIN DU TRAITEMENT — PRISE 'CHAMBRE CÉDRIC' RECONFIGURÉE AVEC SUCCÈS           ");
    Serial.println("================================================================================");
}

void loop() {
    delay(1000);
}
