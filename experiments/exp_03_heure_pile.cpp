/*
 * =====================================================================================
 * EXPÉRIENCE 03 : Franchissement d'Heure Pile (18:59:50 -> 19:00:00)
 * =====================================================================================
 * Objectif :
 *   Valider l'hypothèse de basculement des historiques :
 *   1. Relever l'état initial des historiques (les 62 Wh sont dans Slot [23]).
 *   2. Injecter l'heure 18:59:50 (à 10 secondes de l'heure pile).
 *   3. Attendre 15 secondes (laisser le RTC franchir 19:00:00).
 *   4. Relever tous les historiques et observer le basculement :
 *      - Slot [23] -> Slot [22] (H-1) ?
 *      - Slot [23] remis à 0 Wh ?
 *      - Report des 62 Wh dans le jour courant Slot [29] ?
 *   5. Déconnexion propre pour libérer la prise.
 * =====================================================================================
 */

#include <Arduino.h>
#include "SEM6000Device.h"

static const char* TARGET_MAC = "94:a9:a8:1a:37:0c";
static const uint32_t TARGET_PIN = 0x00000000;
static const int BUTTON_BOOT_PIN = 0;

static SEM6000Device device(TARGET_MAC, TARGET_PIN);

struct Snapshot {
    SEMTelemetry telemetry;
    uint16_t fff2_word0;
    uint8_t fff2_byte2;
    float fff2_currentA;
    uint16_t hourly[24];
    uint32_t daily[30];
    uint32_t monthly[12];
};

static Snapshot snapBaseline;
static Snapshot snapPostRollover;

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

void printTelemetry(const Snapshot& s) {
    Serial.println("  [TÉLÉMESURE 0x0400]");
    Serial.printf("    Relais : %s | Tension : %d V | Fréquence : %d Hz\n",
                  s.telemetry.relayState ? "ON" : "OFF",
                  s.telemetry.voltageVolts,
                  s.telemetry.frequencyHz);
    Serial.printf("    Puissance active : %.3f W | Courant : %.3f A\n",
                  s.telemetry.powerWatts,
                  s.telemetry.currentAmperes);
    Serial.printf("    totalKwhWh brut : %lu (0x%08lX)\n",
                  (unsigned long)s.telemetry.totalKwhWh,
                  (unsigned long)s.telemetry.totalKwhWh);
    Serial.printf("  [DIAGNOSTIC FFF2]\n");
    Serial.printf("    Word0 : 0x%04X (%u) | Courant brut : %.3f A (octet2: 0x%02X)\n",
                  s.fff2_word0, s.fff2_word0, s.fff2_currentA, s.fff2_byte2);
}

void printHourly(const uint16_t cur[24], const uint16_t* prev = nullptr) {
    Serial.println("  [HISTORIQUE HORAIRE - 24H (Wh)]");
    for (int i = 0; i < 24; i++) {
        int hoursAgo = 23 - i;
        char tag[20] = "";
        if (i == 23) snprintf(tag, sizeof(tag), " [EN COURS]");
        else if (i == 22) snprintf(tag, sizeof(tag), " [H-1]");

        char deltaStr[40] = "";
        if (prev) {
            int32_t diff = (int32_t)cur[i] - (int32_t)prev[i];
            if (diff != 0) {
                snprintf(deltaStr, sizeof(deltaStr), " (Δ: %+ld Wh) <<< CHANGÉ", (long)diff);
            }
        }
        Serial.printf("    Slot [%02d] (H-%02d)%-12s : %5u Wh%s\n",
                      i, hoursAgo, tag, cur[i], deltaStr);
    }
}

void printDaily(const uint32_t cur[30], const uint32_t* prev = nullptr) {
    Serial.println("  [HISTORIQUE JOURNALIER - 30 JOURS (Wh)]");
    for (int i = 0; i < 30; i++) {
        int daysAgo = 29 - i;
        char tag[20] = "";
        if (i == 29) snprintf(tag, sizeof(tag), " [AUJOURD'HUI]");
        else if (i == 28) snprintf(tag, sizeof(tag), " [HIER]");

        char deltaStr[40] = "";
        if (prev) {
            int32_t diff = (int32_t)cur[i] - (int32_t)prev[i];
            if (diff != 0) {
                snprintf(deltaStr, sizeof(deltaStr), " (Δ: %+ld Wh) <<< CHANGÉ", (long)diff);
            }
        }
        Serial.printf("    Slot [%02d] (J-%02d)%-15s : %7lu Wh%s\n",
                      i, daysAgo, tag, (unsigned long)cur[i], deltaStr);
    }
}

void printMonthly(const uint32_t cur[12], const uint32_t* prev = nullptr) {
    Serial.println("  [HISTORIQUE MENSUEL - 12 MOIS (Wh)]");
    for (int i = 0; i < 12; i++) {
        int monthsAgo = 11 - i;
        char tag[20] = "";
        if (i == 11) snprintf(tag, sizeof(tag), " [MOIS COURANT]");
        else if (i == 10) snprintf(tag, sizeof(tag), " [M-1]");

        char deltaStr[40] = "";
        if (prev) {
            int32_t diff = (int32_t)cur[i] - (int32_t)prev[i];
            if (diff != 0) {
                snprintf(deltaStr, sizeof(deltaStr), " (Δ: %+ld Wh) <<< CHANGÉ", (long)diff);
            }
        }
        Serial.printf("    Slot [%02d] (M-%02d)%-16s : %7lu Wh%s\n",
                      i, monthsAgo, tag, (unsigned long)cur[i], deltaStr);
    }
}

bool captureFullSnapshot(Snapshot& snap) {
    if (!device.readRawFFF2(snap.fff2_word0, snap.fff2_byte2, snap.fff2_currentA)) {
        Serial.println("  [ERREUR] Échec lecture FFF2");
        return false;
    }
    if (device.captureMeasurement(snap.telemetry) != SEM_OK) {
        Serial.println("  [ERREUR] Échec captureMeasurement");
        return false;
    }
    if (device.requestDayHistory(snap.hourly) != SEM_OK) {
        Serial.println("  [ERREUR] Échec requestDayHistory");
        return false;
    }
    if (device.requestMonthHistory(snap.daily) != SEM_OK) {
        Serial.println("  [ERREUR] Échec requestMonthHistory");
        return false;
    }
    if (device.requestYearHistory(snap.monthly) != SEM_OK) {
        Serial.println("  [ERREUR] Échec requestYearHistory");
        return false;
    }
    return true;
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
    Serial.println("   Passerelle SEM6000 — EXP 03 : Franchissement Heure Pile (18:59:50 -> 19:00) ");
    Serial.println("================================================================================");

    NimBLEDevice::init("Gateway_Exp03");

    // =========================================================================
    // ÉTAPE 0 : ATTENTE UTILISATEUR
    // =========================================================================
    waitUserPrompt("L'ESP32 est prêt. Appuyez sur BOOT pour lancer le test");

    // =========================================================================
    // ÉTAPE 1 : LECTURE DE L'ÉTAT INITIAL (BASELINE)
    // =========================================================================
    Serial.println("================================================================================");
    Serial.println(" ÉTAPE 1 : LECTURE DE L'ÉTAT INITIAL (BASELINE)");
    Serial.println("================================================================================");

    if (!connectAndAuth()) {
        Serial.println("[FATAL] Échec connexion BLE.");
        return;
    }

    if (!captureFullSnapshot(snapBaseline)) {
        Serial.println("[FATAL] Échec capture historiques.");
        device.disconnect();
        return;
    }

    printTelemetry(snapBaseline);
    printHourly(snapBaseline.hourly);
    printDaily(snapBaseline.daily);
    printMonthly(snapBaseline.monthly);

    // =========================================================================
    // ÉTAPE 2 : INJECTION DE L'HEURE CIBLE (18:59:50)
    // =========================================================================
    Serial.println("\n================================================================================");
    Serial.println(" ÉTAPE 2 : INJECTION DE L'HEURE -> 18/11/2026 à 18:59:50");
    Serial.println("          (À 10 secondes de l'heure pile 19:00:00)");
    Serial.println("================================================================================");

    // Envoi de 18:59:50 le 18/11/2026
    SEM6000Status st = device.setDateTime(50, 59, 18, 18, 11, 2026);
    if (st == SEM_OK) {
        Serial.println("[RTC] Commande setDateTime(18:59:50) acceptée par la prise !");
    } else {
        Serial.printf("[RTC] ERREUR setDateTime (code: %d)\n", st);
        device.disconnect();
        return;
    }

    // =========================================================================
    // ÉTAPE 3 : ATTENTE DE 15 SECONDES (FRANCHISSEMENT DE 19:00:00)
    // =========================================================================
    Serial.println("\n================================================================================");
    Serial.println(" ÉTAPE 3 : ATTENTE DE 15 SECONDES DU FRANCHISSEMENT D'HEURE PILE (19:00:00)");
    Serial.println("================================================================================");

    for (int s = 15; s > 0; s--) {
        int simulatedSec = (50 + (15 - s)) % 60;
        int simulatedMin = (50 + (15 - s) >= 60) ? 0 : 59;
        int simulatedHour = (simulatedMin == 0) ? 19 : 18;

        Serial.printf("  T+%02ds -> Heure interne estimée prise: %02d:%02d:%02d (reste %2ds)\n",
                      15 - s, simulatedHour, simulatedMin, simulatedSec, s);
        delay(1000);
    }
    Serial.println("  >>> 15 secondes écoulées : 19:00:00 a été franchi !\n");

    // =========================================================================
    // ÉTAPE 4 : RELECTURE DES HISTORIQUES ET VÉRIFICATION DU BASCULEMENT
    // =========================================================================
    Serial.println("================================================================================");
    Serial.println(" ÉTAPE 4 : RELECTURE DES HISTORIQUES POST-HEURE PILE");
    Serial.println("================================================================================");

    if (!captureFullSnapshot(snapPostRollover)) {
        Serial.println("[ERREUR] Impossible de relire les historiques après franchissement.");
    } else {
        printTelemetry(snapPostRollover);
        printHourly(snapPostRollover.hourly, snapBaseline.hourly);
        printDaily(snapPostRollover.daily, snapBaseline.daily);
        printMonthly(snapPostRollover.monthly, snapBaseline.monthly);
    }

    // =========================================================================
    // ÉTAPE 5 : DÉCONNEXION PROPRE (LIBÉRATION DE LA PRISE)
    // =========================================================================
    device.disconnect();
    Serial.println("\n[BLE] Prise déconnectée proprement.");
    Serial.println("================================================================================");
    Serial.println("   EXPÉRIENCE 03 TERMINÉE AVEC SUCCÈS ! PRISE LIBÉRÉE                          ");
    Serial.println("================================================================================");
}

void loop() {
    delay(1000);
}
