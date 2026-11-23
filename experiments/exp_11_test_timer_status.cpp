/**
 * @file exp_11_test_timer_status.cpp
 * @brief Expérience 11 : Détermination de la nature des octets [B11..B13] de la trame 0x0900 (Timer Status).
 * 
 * Objectifs :
 * 1. Synchroniser l'horloge interne de la prise avec une date propre (ex. 12:00:00).
 * 2. Armer un Timer de 60 secondes (cible 12:01:00 avec action ON).
 * 3. Interroger périodiquement la trame 0x0900 (à T=0s, 10s, 20s, 30s, 40s, 50s, puis après échéance à T=65s et 75s).
 * 4. Observer l'évolution des octets [B11..B13] :
 *    - Diminuent-ils en direct ? -> Compte à rebours temps réel (Countdown).
 *    - Restent-ils constants ? -> Durée initiale statique (Preset duration).
 * 5. Observer le comportement de la trame une fois le timer échu (Action repasse-t-elle à 0x00 ?).
 * 6. Nettoyer et libérer la prise.
 */

#include <Arduino.h>
#include "SEM6000Device.h"

// Prise de test par défaut (Théoule)
static const char* TARGET_MAC = "94:a9:a8:1a:37:0c";
// static const char* TARGET_MAC = "b0:b1:13:6b:43:f0"; // Chambre Cédric
static const uint32_t TARGET_PIN = 0x00000000;
static const int BUTTON_BOOT_PIN = 0;

static SEM6000Device device(TARGET_MAC, TARGET_PIN);

struct TimerSample {
    uint32_t elapsedMs;
    uint8_t action;
    uint8_t targetSec;
    uint8_t targetMin;
    uint8_t targetHour;
    uint8_t targetDay;
    uint8_t targetMonth;
    uint8_t targetYear;
    uint8_t b11;
    uint8_t b12;
    uint8_t b13;
    uint32_t totalSec;
};

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

TimerSample sampleTimer(uint32_t elapsedMs) {
    TimerSample s;
    memset(&s, 0, sizeof(s));
    s.elapsedMs = elapsedMs;

    SEMTimerStatus st;
    if (device.getTimerStatus(st) == SEM_OK) {
        const uint8_t* rawBuf = nullptr;
        size_t len = device.getLastRxBuffer(&rawBuf);
        if (rawBuf && len >= 18) {
            s.action      = rawBuf[4];
            s.targetSec   = rawBuf[5];
            s.targetMin   = rawBuf[6];
            s.targetHour  = rawBuf[7];
            s.targetDay   = rawBuf[8];
            s.targetMonth = rawBuf[9];
            s.targetYear  = rawBuf[10];
            s.b11         = rawBuf[11];
            s.b12         = rawBuf[12];
            s.b13         = rawBuf[13];
            s.totalSec    = (uint32_t)s.b11 * 3600 + (uint32_t)s.b12 * 60 + s.b13;
        }
    }
    return s;
}

void printSampleRow(int index, const TimerSample& s) {
    const char* actionStr = "INACTIF (00)";
    if (s.action == 0x01) actionStr = "ACTIF -> ON (01)";
    else if (s.action == 0x02) actionStr = "ACTIF -> OFF (02)";

    Serial.printf("  [%02d] |   T + %2lu s   | %-16s | 20%02u/%02u/%02u %02u:%02u:%02u | [0x%02X 0x%02X 0x%02X] | %02uh %02um %02us |  %4lu s\n",
                  index,
                  (unsigned long)(s.elapsedMs / 1000),
                  actionStr,
                  s.targetYear, s.targetMonth, s.targetDay, s.targetHour, s.targetMin, s.targetSec,
                  s.b11, s.b12, s.b13,
                  s.b11, s.b12, s.b13,
                  (unsigned long)s.totalSec);
}

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_BOOT_PIN, INPUT_PULLUP);

    uint32_t startWait = millis();
    while (!Serial && (millis() - startWait < 4000)) {
        delay(50);
    }
    delay(500);

    NimBLEDevice::init("Gateway_Exp11");

    Serial.println("\n\n");
    Serial.println("================================================================================");
    Serial.println("  ⏱️ EXPÉRIENCE 11 : ANALYSE DU STATUT TIMER (TRAME 0x0900)                     ");
    Serial.println("================================================================================");
    Serial.println("  Objectif : Déterminer si les octets [B11..B13] représentent :");
    Serial.println("             - Un compte à rebours dynamique décrémenté par la prise, ou");
    Serial.println("             - La durée statique initiale de consigne (preset).");
    Serial.printf("  Prise cible : %s\n", TARGET_MAC);
    Serial.println("================================================================================\n");

    waitUserPrompt("1. CONNEXION À LA PRISE",
                   "1. Assurez-vous que l'application smartphone est FERMÉE.\n"
                   "2. La prise doit être branchée et accessible en BLE.\n"
                   "3. Dès que vous êtes prêt :");

    Serial.printf("[BLE] Connexion à %s...\n", TARGET_MAC);
    if (!device.connect(8000)) {
        Serial.println("[BLE] ❌ ERREUR : Impossible de se connecter à la prise.");
        return;
    }
    Serial.println("[BLE] ✅ Connecté !");

    if (device.authenticate32((uint32_t)TARGET_PIN) != SEM_OK) {
        Serial.println("[BLE] ❌ ERREUR : Échec d'authentification.");
        device.disconnect();
        return;
    }
    Serial.println("[BLE] ✅ Authentifié avec succès !\n");

    // Éteindre le relais initialement pour pouvoir tester un timer d'allumage (ON)
    Serial.println("[INIT] Extinction préalable du relais pour test de bascule à ON...");
    device.setRelayState(false);
    device.stopTimer();
    delay(300);

    // Étape 2 : Calage de l'horloge
    Serial.println("[INIT] Synchronisation horloge prise sur repère fixe : 12:00:00 (le 23/11/2026)...");
    device.setDateTime(0, 0, 12, 23, 11, 2026);
    delay(300);

    // Étape 3 : Armement du Timer
    Serial.println("[TIMER] Armement d'un Timer de 60 secondes :");
    Serial.println("  -> Date cible : 12:01:00 le 23/11/2026");
    Serial.println("  -> Action     : Allumer le relais (ON = 0x01)");
    device.setTimerAbs(true, 0, 1, 12, 23, 11, 2026);
    uint32_t armTime = millis();
    delay(100);

    Serial.println("\n=====================================================================================================================");
    Serial.println("  #  | Temps écoulé | Action programmée| Cible absolue programmée | Octets bruts [B11..B13] | Durée H:M:S | Total Sec");
    Serial.println("-----+--------------+------------------+--------------------------+-------------------------+-------------+----------");

    // Collecte des échantillons
    TimerSample samples[8];
    int intervalsSeconds[] = { 0, 10, 20, 30, 40, 50, 65, 75 };
    int sampleCount = sizeof(intervalsSeconds) / sizeof(intervalsSeconds[0]);

    for (int i = 0; i < sampleCount; i++) {
        uint32_t targetElapsedMs = (uint32_t)intervalsSeconds[i] * 1000;
        while (millis() - armTime < targetElapsedMs) {
            delay(20);
        }

        samples[i] = sampleTimer(millis() - armTime);
        printSampleRow(i + 1, samples[i]);
    }
    Serial.println("=====================================================================================================================\n");

    // Analyse du verdict
    Serial.println("================================================================================");
    Serial.println("  📊 VERDICT & ANALYSE TECHNIQUE");
    Serial.println("================================================================================");

    bool isDecreasing = false;
    bool isConstant = true;

    for (int i = 1; i <= 5; i++) {
        if (samples[i].totalSec < samples[i - 1].totalSec) {
            isDecreasing = true;
            isConstant = false;
        }
    }

    if (isDecreasing) {
        Serial.println("  🎉 CONCLUSION : Les octets [B11..B13] forment un COMPTE À REBOURS DYNAMIQUE !");
        Serial.println("  -> La prise décompte en temps réel les secondes restantes jusqu'au déclenchement.");
        Serial.println("  -> L'ESP32 peut afficher le temps restant directement sans calcul horloge.");
    } else if (isConstant && samples[0].totalSec > 0) {
        Serial.println("  🎉 CONCLUSION : Les octets [B11..B13] forment un PRESET STATIQUE !");
        Serial.println("  -> La prise mémorise la durée initiale d'armement (ex. 60s) et la conserve figée.");
        Serial.println("  -> Pour connaître le temps restant, le client doit comparer l'heure actuelle avec la date cible.");
    }

    // Analyse post-échéance (T=65s et T=75s)
    Serial.println("\n  [COMPORTEMENT POST-ÉCHÉANCE (après 60s)] :");
    Serial.printf("  -> Échantillon T+65s : Action = 0x%02X, Durée = %lu s\n", samples[6].action, (unsigned long)samples[6].totalSec);
    Serial.printf("  -> Échantillon T+75s : Action = 0x%02X, Durée = %lu s\n", samples[7].action, (unsigned long)samples[7].totalSec);

    if (samples[6].action == 0x00) {
        Serial.println("  -> Constat : L'octet d'action repasse immédiatement à 0x00 dès que le timer est échu !");
    } else {
        Serial.printf("  -> Constat : L'action reste à 0x%02X même après franchissement.\n", samples[6].action);
    }

    // Vérification de l'état du relais
    SEMTelemetry telem;
    if (device.captureMeasurement(telem) == SEM_OK) {
        Serial.printf("  -> État final du relais après timer : %s (Attendu: ALLUMÉ / ON)\n", telem.relayState ? "ALLUMÉ (ON)" : "ÉTEINT (OFF)");
    }

    // Nettoyage final
    Serial.println("\n[FIN] Arrêt du timer et déconnexion...");
    device.stopTimer();
    device.setRelayState(false);
    device.disconnect();

    Serial.println("[BLE] ✅ Prise déconnectée et remise à zéro.");
    Serial.println("================================================================================");
}

void loop() {
    delay(1000);
}
