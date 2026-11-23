/**
 * @file exp_12_test_surcharge_zone1.cpp
 * @brief Expérience 12 : Analyse de la Zone 1 (Octets 13-14 de la télémétrie 0x0400)
 *        par réduction dichotomique progressive du seuil de surcharge.
 * 
 * Protocole :
 * 0. Programmation du seuil initial de surcharge à 2000 W et relais mis à OFF.
 * 1. Consigne utilisateur pour brancher une charge (1000 W <= P <= 1800 W) et allumer son commutateur.
 * 2. Attente de confirmation utilisateur via bouton BOOT (GPIO 0) ou Entrée.
 * 3. Allumage du relais (ON). Affichage complet de la télémétrie 0x0400 (notamment octets 13-14)
 *    et de la trame FFF2. Attente de 10 secondes.
 * 4. Baisse du seuil max de la moitié de l'écart (Delta = P_max - P_mesuré). Si Delta <= 10 W, réduction de 10 W.
 * 5. Rebouclage en 3 tant que le relais reste allumé (ON).
 * 6. Lorsque la surcharge disjoncte le relais (OFF) : relevé final, restauration d'un seuil sûr (3680 W),
 *    déconnexion et libération de la prise.
 */

#include <Arduino.h>
#include "SEM6000Device.h"

// Prise de test par défaut (Théoule)
static const char* TARGET_MAC = "94:a9:a8:1a:37:0c";
// static const char* TARGET_MAC = "b0:b1:13:6b:43:f0"; // Chambre Cédric
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
    Serial.println(">>> Action validée par l'utilisateur ! Démarrage...\n");
}

void printRawTelemetryReport(int iteration, uint16_t currentPmax, const SEMTelemetry& telem, const uint8_t* rawBuf, size_t rawLen) {
    Serial.println("--------------------------------------------------------------------------------");
    Serial.printf("  📊 [ITÉRATION #%02d] — SEUIL P_MAX ACTUEL : %u W\n", iteration, currentPmax);
    Serial.println("--------------------------------------------------------------------------------");

    // Données physiques décodées
    float deltaP = (float)currentPmax - telem.powerWatts;
    Serial.printf("  État Relais       : %s\n", telem.relayState ? "🟢 ON (FERMÉ)" : "🔴 OFF (OUVERT / DISJONCTÉ)");
    Serial.printf("  Puissance Active  : %.2f W (Écart avec le seuil : %.2f W)\n", telem.powerWatts, deltaP);
    Serial.printf("  Tension Réseau    : %u V\n", telem.voltageVolts);
    Serial.printf("  Courant Actif     : %.3f A\n", telem.currentAmperes);
    Serial.printf("  Fréquence         : %u Hz\n", telem.frequencyHz);

    // Caractéristique FFF2 (diagnostic léger)
    uint16_t word0 = 0;
    uint8_t curByte = 0;
    float curDirectA = 0.0f;
    if (device.readRawFFF2(word0, curByte, curDirectA)) {
        Serial.printf("  Trame FFF2        : Word0 (Seuil mA) = %u mA  |  Word1 (Courant instantané) = %.3f A (raw byte2: 0x%02X)\n",
                      word0, curDirectA, curByte);
    }

    // Octets bruts complets de 0x0400
    if (rawBuf && rawLen >= 19) {
        Serial.print("  Trame 0x0400 brute: ");
        for (size_t i = 0; i < rawLen; i++) {
            Serial.printf("%02X ", rawBuf[i]);
        }
        Serial.println();

        // Focus ZONE 1 : Octets 13 et 14 (Index 12 et 13 du buffer)
        uint8_t byte13 = rawBuf[12];
        uint8_t byte14 = rawBuf[13];
        uint16_t unknownWord = ((uint16_t)byte13 << 8) | byte14;

        Serial.println("  ----------------------------------------------------------------------------");
        Serial.printf("  🎯 [FOCUS ZONE 1 — OCTETS 13 & 14] : [0x%02X 0x%02X]  (Décimal: %u / 0x%04X)\n",
                      byte13, byte14, unknownWord, unknownWord);
        Serial.println("  ----------------------------------------------------------------------------");
    }
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_BOOT_PIN, INPUT_PULLUP);

    uint32_t startWait = millis();
    while (!Serial && (millis() - startWait < 4000)) {
        delay(50);
    }
    delay(500);

    NimBLEDevice::init("Gateway_Exp12");

    Serial.println("\n\n");
    Serial.println("================================================================================");
    Serial.println("  ⚡ EXPÉRIENCE 12 : ANALYSE ZONE 1 (OCTETS 13-14) & SURCHARGE DICHOTOMIQUE      ");
    Serial.println("================================================================================");
    Serial.printf("  Cible : %s\n", TARGET_MAC);
    Serial.println("  Objectif : Observer les octets 13-14 de la télémétrie 0x0400 à mesure que le  ");
    Serial.println("             seuil max se rapproche de la consommation réelle jusqu'à disjonction");
    Serial.println("================================================================================\n");

    // Connexion
    Serial.printf("[BLE] Connexion à %s...\n", TARGET_MAC);
    if (!device.connect(8000)) {
        Serial.println("[BLE] ❌ ERREUR : Impossible de se connecter. Fermez l'application smartphone.");
        return;
    }
    Serial.println("[BLE] ✅ Connecté !");

    if (device.authenticate32((uint32_t)TARGET_PIN) != SEM_OK) {
        Serial.println("[BLE] ❌ ERREUR : Authentification échouée.");
        device.disconnect();
        return;
    }
    Serial.println("[BLE] ✅ Authentifié avec succès !\n");

    // --- ÉTAPE 0 : Initialisation à 2000 W et Relais OFF ---
    Serial.println("[ÉTAPE 0] Programmation du seuil initial à 2000 W et coupure du relais (OFF)...");
    device.setOverloadPower(2000);
    device.setRelayState(false);
    delay(400);

    // --- ÉTAPES 1 & 2 : Consigne utilisateur ---
    waitUserPrompt("1. PRÉPARATION MATÉRIELLE DE LA CHARGE",
                   "1. Branchez sur la prise SEM6000 un appareil consommateur de puissance :\n"
                   "     -> Comprise entre 1000 W et 1800 W (ex: bouilloire, fer à repasser, radiateur).\n"
                   "2. Placez l'interrupteur mécanique de cet appareil sur ON (en marche).\n"
                   "3. Dès que l'appareil est branché et armé, appuyez sur BOOT ou Entrée.");

    // --- ÉTAPE 3 : Allumage du relais ---
    Serial.println("[ÉTAPE 3] Allumage du relais (ON)...");
    device.setRelayState(true);
    delay(2000); // 2 secondes pour stabiliser l'appel de courant

    uint16_t currentPmax = 2000;
    int iteration = 1;
    bool tripped = false;

    // --- BOUCLE DICHOTOMIQUE (Étapes 3, 4, 5) ---
    while (!tripped) {
        // Lecture télémétrie 0x0400
        SEMTelemetry telem;
        SEM6000Status st = device.captureMeasurement(telem);
        if (st != SEM_OK) {
            Serial.printf("[BLE] ⚠️ Erreur de lecture télémétrie (Code: %d)\n", st);
            delay(1000);
            continue;
        }

        const uint8_t* rawBuf = nullptr;
        size_t rawLen = device.getLastRxBuffer(&rawBuf);

        // Affichage des données et des octets 13-14
        printRawTelemetryReport(iteration, currentPmax, telem, rawBuf, rawLen);

        // Vérification si le relais a déjà disjoncté
        if (!telem.relayState) {
            tripped = true;
            break;
        }

        // Alerte si la puissance mesurée est nulle ou trop faible
        if (telem.powerWatts < 100.0f) {
            Serial.println("  ⚠️ ATTENTION : Puissance mesurée < 100 W !");
            Serial.println("     Vérifiez que votre charge est bien allumée et consomme du courant.\n");
        }

        // Attente de 10 secondes tout en surveillant le maintien du relais
        Serial.println("  ⏳ Attente de 10 secondes d'observation...");
        for (int sec = 0; sec < 10; sec++) {
            delay(1000);
            // Sonde rapide de sécurité toutes les 2 secondes
            if (sec % 2 == 1) {
                device.captureMeasurement(telem);
                if (!telem.relayState) {
                    Serial.println("\n  ⚡ [SURCHARGE] Disjonction détectée pendant la phase d'attente !");
                    tripped = true;
                    break;
                }
            }
        }

        if (tripped) break;

        // --- ÉTAPE 4 : Calcul de la réduction de seuil ---
        float currentWatts = telem.powerWatts;
        float deltaP = (float)currentPmax - currentWatts;

        uint16_t reduction = 0;
        if (deltaP <= 10.0f) {
            reduction = 10;
        } else {
            reduction = (uint16_t)(deltaP / 2.0f);
            if (reduction < 10) reduction = 10;
        }

        uint16_t newPmax = (currentPmax > reduction) ? (currentPmax - reduction) : 50;

        Serial.printf("[ÉTAPE 4] Calcul de réduction de seuil :\n");
        Serial.printf("  Puissance mesurée : %.1f W  |  Seuil actuel : %u W  |  Delta = %.1f W\n", currentWatts, currentPmax, deltaP);
        Serial.printf("  -> Réduction appliquée : -%u W  ==> Nouveau seuil P_max : %u W\n", reduction, newPmax);

        // Application du nouveau seuil
        device.setOverloadPower(newPmax);
        currentPmax = newPmax;
        iteration++;
        delay(500);
    }

    // --- ÉTAPE 6 : Traitement de la surcharge atteinte ---
    Serial.println("\n================================================================================");
    Serial.println("  ⚡ ÉTAPE 6 : SEUIL DE SURCHARGE ATTEINT — LE RELAIS A DISJONCTÉ !              ");
    Serial.println("================================================================================");

    // Relevé final immédiat
    SEMTelemetry finalTelem;
    if (device.captureMeasurement(finalTelem) == SEM_OK) {
        const uint8_t* rawBuf = nullptr;
        size_t rawLen = device.getLastRxBuffer(&rawBuf);
        printRawTelemetryReport(99, currentPmax, finalTelem, rawBuf, rawLen);
    }

    // Restauration d'un seuil standard de sécurité (3680 W = 16A)
    Serial.println("[SÉCURITÉ] Restauration du seuil de surcharge nominal à 3680 W...");
    device.setOverloadPower(3680);
    delay(300);

    // Relâchement de la prise
    Serial.println("[BLE] Déconnexion et libération de la prise...");
    device.disconnect();
    Serial.println("[BLE] ✅ Prise déconnectée.");
    Serial.println("================================================================================");
    Serial.println("  TEST TERMINÉ AVEC SUCCÈS — ANALYSE DISPONIBLE DANS LA CONSOLE SÉRIE           ");
    Serial.println("================================================================================");
}

void loop() {
    delay(1000);
}
