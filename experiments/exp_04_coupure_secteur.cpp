/*
 * =====================================================================================
 * EXPÉRIENCE 04 : Test de Coupure Secteur (Amnésie et Préservation des Historiques)
 * =====================================================================================
 * Objectif :
 *   0-1. Se connecter et afficher l'état initial des historiques avant coupure.
 *   0-2. Se déconnecter de la prise et afficher "Débranchez la prise SVP".
 *   0-3. Attendre l'appui sur le bouton BOOT (la prise a été coupée >= 30s puis rebranchée).
 *     2. Se reconnecter en BLE SANS envoyer de date/heure (pas de setDateTime).
 *     3. Lire FFF2 : observer si la sentinelle 0xFFFF est présente.
 *     4. Lire les historiques : les données ont-elles été sauvegardées en Flash ou effacées ?
 *     5. Demander de consommer de l'énergie pendant environ 1 minute.
 *     6. Attendre l'appui sur le bouton BOOT.
 *     7. Lire et afficher les historiques pour voir dans quel slot la nouvelle énergie va.
 *     8. Déconnexion et fin du test.
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

static Snapshot snapBeforeCut;
static Snapshot snapPostCut;
static Snapshot snapPostLoad;

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
    Serial.printf("    Word0 : 0x%04X (%u)", s.fff2_word0, s.fff2_word0);
    if (s.fff2_word0 == 0xFFFF) {
        Serial.print(" <<< SENTINELLE AMNÉSIE / COUPURE SECTEUR (0xFFFF) DÉTECTÉE !");
    }
    Serial.println();
    Serial.printf("    Courant brut : %.3f A (octet2: 0x%02X)\n", s.fff2_currentA, s.fff2_byte2);
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

bool connectAndAuthWithRetry(int maxAttempts = 3) {
    for (int attempt = 1; attempt <= maxAttempts; attempt++) {
        Serial.printf("[BLE] Tentative %d/%d de connexion à la prise %s...\n",
                      attempt, maxAttempts, TARGET_MAC);
        if (device.connect(8000)) {
            Serial.println("[BLE] Connecté !");
            SEM6000Status st = device.authenticate32((uint32_t)TARGET_PIN);
            if (st == SEM_OK) {
                Serial.println("[BLE] Authentifié avec succès !");
                return true;
            } else {
                Serial.printf("[BLE] Échec authentification (code: %d)\n", st);
                device.disconnect();
            }
        } else {
            Serial.println("[BLE] Échec de connexion.");
        }
        if (attempt < maxAttempts) {
            Serial.println("[BLE] Nouvelle tentative dans 2 secondes...");
            delay(2000);
        }
    }
    return false;
}

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_BOOT_PIN, INPUT_PULLUP);

    delay(1000);

    Serial.println("\n================================================================================");
    Serial.println("   Passerelle SEM6000 — EXP 04 : Test de Coupure Secteur & Amnésie RTC          ");
    Serial.println("================================================================================");

    NimBLEDevice::init("Gateway_Exp04");

    // =========================================================================
    // POINT D'ARRÊT N°0 : Attente de l'utilisateur pour voir le début du test
    // =========================================================================
    waitUserPrompt("L'ESP32 est prêt. Appuyez sur BOOT pour lire l'état AVANT coupure (Étape 0-1)");

    // =========================================================================
    // ÉTAPE 0-1 : LECTURE DE L'ÉTAT INITIAL AVANT COUPURE
    // =========================================================================
    Serial.println("================================================================================");
    Serial.println(" ÉTAPE 0-1 : LECTURE DE L'ÉTAT AVANT COUPURE (BASELINE)");
    Serial.println("================================================================================");

    if (!connectAndAuthWithRetry(3)) {
        Serial.println("[FATAL] Impossible de se connecter à la prise avant coupure.");
        return;
    }

    if (!captureFullSnapshot(snapBeforeCut)) {
        Serial.println("[FATAL] Échec capture historiques avant coupure.");
        device.disconnect();
        return;
    }

    printTelemetry(snapBeforeCut);
    printHourly(snapBeforeCut.hourly);
    printDaily(snapBeforeCut.daily);
    printMonthly(snapBeforeCut.monthly);

    // =========================================================================
    // ÉTAPE 0-2 : DÉCONNEXION DE LA PRISE
    // =========================================================================
    device.disconnect();
    Serial.println("\n[BLE] Prise déconnectée proprement.");

    // =========================================================================
    // ÉTAPE 0-3 : ATTENTE DE LA COUPURE SECTEUR
    // =========================================================================
    Serial.println("\n================================================================================");
    Serial.println(" >>> DÉBRANCHEZ LA PRISE DU MUR SVP.                                            ");
    Serial.println("     1. Débranchez la prise SEM6000 de la prise murale.                         ");
    Serial.println("     2. Attendez au moins 30 secondes (pour vider les condensateurs internes).   ");
    Serial.println("     3. Rebranchez la prise au mur.                                             ");
    Serial.println("     4. Appuyez sur le bouton physique BOOT de l'ESP32 pour continuer.          ");
    Serial.println("================================================================================");

    waitUserPrompt("Avez-vous débranché (>=30s) puis rebranché la prise ? Appuyez sur BOOT");

    // =========================================================================
    // ÉTAPE 2, 3, 4 : RECONNEXION SANS REMISE À L'HEURE & ANALYSE AMNÉSIE
    // =========================================================================
    Serial.println("================================================================================");
    Serial.println(" ÉTAPE 2, 3, 4 : RECONNEXION SANS SYNCHRONISATION HORAIRE & LECTURE AMNÉSIE");
    Serial.println("================================================================================");

    if (!connectAndAuthWithRetry(4)) {
        Serial.println("[FATAL] Impossible de se reconnecter après coupure secteur.");
        return;
    }

    if (!captureFullSnapshot(snapPostCut)) {
        Serial.println("[FATAL] Échec capture historiques post-coupure.");
        device.disconnect();
        return;
    }

    Serial.println("\n--- RÉSULTATS POST-COUPURE SECTEUR ---");
    printTelemetry(snapPostCut);
    printHourly(snapPostCut.hourly, snapBeforeCut.hourly);
    printDaily(snapPostCut.daily, snapBeforeCut.daily);
    printMonthly(snapPostCut.monthly, snapBeforeCut.monthly);

    device.disconnect();
    Serial.println("\n[BLE] Prise déconnectée avant branchement de la charge.");

    // =========================================================================
    // ÉTAPE 5 & 6 : ATTENTE CONSOMMATION D'ÉNERGIE (~1 MINUTE)
    // =========================================================================
    Serial.println("\n================================================================================");
    Serial.println(" >>> ÉTAPE 5 : Branchez un consommateur d'énergie sur la prise SEM6000.         ");
    Serial.println("     Laissez-le fonctionner pendant environ 1 minute.                           ");
    Serial.println("     Appuyez sur le bouton physique BOOT de l'ESP32 dès que c'est fait.        ");
    Serial.println("================================================================================");

    waitUserPrompt("Avez-vous fait consommer la charge pendant ~1 minute ? Appuyez sur BOOT");

    // =========================================================================
    // ÉTAPE 7 : LECTURE APRÈS CONSOMMATION POST-COUPURE
    // =========================================================================
    Serial.println("================================================================================");
    Serial.println(" ÉTAPE 7 : RELECTURE DES HISTORIQUES APRÈS CONSOMMATION");
    Serial.println("================================================================================");

    if (!connectAndAuthWithRetry(3)) {
        Serial.println("[FATAL] Impossible de se reconnecter après charge.");
        return;
    }

    if (!captureFullSnapshot(snapPostLoad)) {
        Serial.println("[FATAL] Échec capture historiques post-charge.");
        device.disconnect();
        return;
    }

    Serial.println("\n--- RÉSULTATS APRÈS CHARGE POST-COUPURE ---");
    printTelemetry(snapPostLoad);
    printHourly(snapPostLoad.hourly, snapPostCut.hourly);
    printDaily(snapPostLoad.daily, snapPostCut.daily);
    printMonthly(snapPostLoad.monthly, snapPostCut.monthly);

    // =========================================================================
    // ÉTAPE 8 : DÉCONNEXION & FIN DU TEST
    // =========================================================================
    device.disconnect();
    Serial.println("\n[BLE] Prise déconnectée proprement.");
    Serial.println("================================================================================");
    Serial.println("   EXPÉRIENCE 04 TERMINÉE AVEC SUCCÈS ! PRISE LIBÉRÉE                          ");
    Serial.println("================================================================================");
}

void loop() {
    delay(1000);
}
