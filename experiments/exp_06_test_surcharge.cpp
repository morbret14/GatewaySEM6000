/*
 * =====================================================================================
 * EXPÉRIENCE 06 : Test de Surcharge & Comportement de Word0 (FFF2) lors d'une Disjonction
 * =====================================================================================
 * Seuil configuré sur la prise : 1234 W (~5.3 A sous 230 V)
 *
 * Déroulement :
 *   1. L'ESP32 s'initialise et attend l'appui sur BOOT (GPIO 0).
 *   2. Connexion et authentification BLE à la prise SEM6000.
 *   3. Boucle de streaming temps réel (toutes les 500 ms) :
 *      - Lecture directe de FFF2 (Word0 = I_max en mA, Courant direct en mA)
 *      - Lecture de la télémesure 0x0400 (Puissance active, Tension, État du Relais)
 *      - Détection des événements :
 *          * Dépassement de seuil (I_mesuré > Word0)
 *          * Disjonction du relais (ON -> OFF)
 *          * Variation éventuelle de Word0
 *          * Réarmement du relais (OFF -> ON)
 *   4. L'utilisateur fait évoluer sa charge (> 1234 W) jusqu'à coupure, coupe sa charge,
 *      réarme la prise, puis fait une légère consommation.
 *   5. Appui sur le bouton BOOT pour terminer le test, déconnecter et libérer la prise.
 * =====================================================================================
 */

#include <Arduino.h>
#include "SEM6000Device.h"

static const char* TARGET_MAC = "94:a9:a8:1a:37:0c";
static const uint32_t TARGET_PIN = 0x00000000;
static const int BUTTON_BOOT_PIN = 0;

static SEM6000Device device(TARGET_MAC, TARGET_PIN);

static bool isMonitoring = false;
static bool previousRelayState = true;
static uint16_t previousWord0 = 0;

void waitUserPrompt(const char* message) {
    Serial.println("\n--------------------------------------------------------------------------------");
    Serial.print(" >>> ");
    Serial.println(message);
    Serial.println("     [Appuyez sur le bouton physique BOOT de l'ESP32 ou envoyez 'Entrée']");
    Serial.println("--------------------------------------------------------------------------------");

    while (Serial.available()) Serial.read();

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
        delay(50);
    }
    Serial.println(">>> Action validée !\n");
}

bool connectAndAuth() {
    Serial.printf("[BLE] Connexion à la prise %s...\n", TARGET_MAC);
    if (!device.connect(8000)) {
        Serial.println("[BLE] ÉCHEC de connexion !");
        return false;
    }
    Serial.println("[BLE] Connecté !");

    SEM6000Status st = device.authenticate32((uint32_t)TARGET_PIN);
    if (st != SEM_OK) {
        Serial.printf("[BLE] ÉCHEC Authentification ! Code: %d\n", st);
        device.disconnect();
        return false;
    }
    Serial.println("[BLE] Authentifié avec succès !");
    return true;
}

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_BOOT_PIN, INPUT_PULLUP);

    delay(1000);

    Serial.println("\n================================================================================");
    Serial.println("   Passerelle SEM6000 — EXP 06 : Surveillance Surcharge & Évolution Word0       ");
    Serial.println("   Seuil configuré sur la prise : 1234 W                                       ");
    Serial.println("================================================================================");

    NimBLEDevice::init("Gateway_Exp06");

    waitUserPrompt("L'ESP32 est prêt. Appuyez sur BOOT pour vous connecter et démarrer le flux");

    if (!connectAndAuth()) {
        Serial.println("[FATAL] Impossible d'établir la liaison BLE. Arrêt.");
        while (true) delay(1000);
    }

    // Lecture initiale de calibration
    uint16_t w0 = 0;
    uint8_t b2 = 0;
    float curA = 0;
    device.readRawFFF2(w0, b2, curA);
    previousWord0 = w0;

    SEMTelemetry t;
    if (device.captureMeasurement(t) == SEM_OK) {
        previousRelayState = t.relayState;
    }

    Serial.println("\n================================================================================");
    Serial.printf(" >>> STREAMING DÉMARRÉ — Word0 initial : %u mA (~%.1f W sous 230V)\n",
                  w0, (w0 * 230.0f) / 1000.0f);
    Serial.println("     1. Branchez votre charge et montez la puissance au-dessus de 1234 W.");
    Serial.println("     2. Observez la coupure (disjonction de protection).");
    Serial.println("     3. Éteignez votre appareil, réarmez la prise et re-consommez un peu.");
    Serial.println("     4. Appuyez sur le bouton physique BOOT pour stopper le test.");
    Serial.println("================================================================================\n");

    isMonitoring = true;
}

void loop() {
    if (!isMonitoring) {
        delay(500);
        return;
    }

    // Vérifier si l'utilisateur demande la fin du test (appui sur BOOT)
    if (digitalRead(BUTTON_BOOT_PIN) == LOW) {
        delay(150);
        while (digitalRead(BUTTON_BOOT_PIN) == LOW) delay(20);

        Serial.println("\n================================================================================");
        Serial.println(" >>> BOUTON BOOT DÉTECTÉ : ARRÊT DU TEST DEMANDÉ PAR L'UTILISATEUR              ");
        Serial.println("================================================================================");

        device.disconnect();
        Serial.println("[BLE] Prise déconnectée proprement. Fin de l'expérience 06.");
        isMonitoring = false;
        return;
    }

    // 1. Lecture de FFF2 (Word0 et courant brut direct)
    uint16_t word0 = 0;
    uint8_t byte2 = 0;
    float fff2CurrentA = 0.0f;
    bool fff2Ok = device.readRawFFF2(word0, byte2, fff2CurrentA);

    // 2. Lecture de la télémesure complète (Puissance, Tension, Relais)
    SEMTelemetry telemetry;
    bool teleOk = (device.captureMeasurement(telemetry) == SEM_OK);

    if (fff2Ok && teleOk) {
        uint32_t currentMA = (uint32_t)(telemetry.currentAmperes * 1000.0f);

        // Détection de changement d'état du relais
        if (previousRelayState == true && telemetry.relayState == false) {
            Serial.println("\n********************************************************************************");
            Serial.println(" !!! ALERTE : COUPURE PAR SURCHARGE DÉTECTÉE ! LE RELAIS S'EST OUVERT (OFF) !!! ");
            Serial.printf("     Puissance juste avant coupure : %.1f W | Courant mesuré : %.3f A\n",
                          telemetry.powerWatts, telemetry.currentAmperes);
            Serial.printf("     État de Word0 à la coupure : 0x%04X (%u mA)\n", word0, word0);
            Serial.println("********************************************************************************\n");
        } else if (previousRelayState == false && telemetry.relayState == true) {
            Serial.println("\n********************************************************************************");
            Serial.println(" >>> RÉARMEMENT DÉTECTÉ : LE RELAIS EST DE NOUVEAU FERMÉ (ON) !               ");
            Serial.printf("     État de Word0 au réarmement : 0x%04X (%u mA)\n", word0, word0);
            Serial.println("********************************************************************************\n");
        }
        previousRelayState = telemetry.relayState;

        // Détection de variation de Word0
        char word0Alert[40] = "";
        if (word0 != previousWord0) {
            snprintf(word0Alert, sizeof(word0Alert), " <<< WORD0 A CHANGÉ (%u->%u) !", previousWord0, word0);
            previousWord0 = word0;
        }

        // Marqueur de dépassement
        const char* overloadMarker = "";
        if (telemetry.relayState && word0 != 0xFFFF && currentMA > word0) {
            overloadMarker = " <<< DÉPASSEMENT SEUIL !";
        }

        // Ligne de streaming temps réel
        Serial.printf("[LIVE] Relais: %-3s | P: %6.1f W | U: %3d V | I_mesuré: %5.3f A (%4u mA) | Word0 (I_max): %5u mA%s%s\n",
                      telemetry.relayState ? "ON" : "OFF",
                      telemetry.powerWatts,
                      telemetry.voltageVolts,
                      telemetry.currentAmperes,
                      currentMA,
                      word0,
                      overloadMarker,
                      word0Alert);
    } else {
        Serial.println("[WARN] Trame BLE manquée lors de ce cycle...");
    }

    // Pause de 500ms découpée en tranches de 50ms pour réactivité instantanée du bouton BOOT
    for (int i = 0; i < 10; i++) {
        if (digitalRead(BUTTON_BOOT_PIN) == LOW) break;
        delay(50);
    }
}
