/*
 * =====================================================================================
 * EXPÉRIENCE 02 : Impact de la synchronisation date/heure sur les historiques SEM6000
 * Version Interactive Pas-à-Pas (Chaque étape attend le bouton BOOT / touche console)
 * =====================================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <esp_sntp.h>
#include "SEM6000Device.h"

static const char* TARGET_MAC = "94:a9:a8:1a:37:0c";
static const uint32_t TARGET_PIN = 0x00000000;
static const int BUTTON_BOOT_PIN = 0;

static const char* WIFI_SSID = "MORBRET_5G";
static const char* WIFI_PASS = "616463255C7EC3296F36EAA21F";

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
static Snapshot snapPostJump;
static Snapshot snapPostLoad;

// Fonction universelle d'attente utilisateur (Bouton BOOT ou touche dans le moniteur)
void waitUserPrompt(const char* message) {
    Serial.println("\n--------------------------------------------------------------------------------");
    Serial.print(" >>> ");
    Serial.println(message);
    Serial.println("     [Appuyez sur le bouton physique BOOT de l'ESP32 ou envoyez 'Entrée']");
    Serial.println("--------------------------------------------------------------------------------");

    // Vider le buffer série entrant
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

void syncNtpOrFallback() {
    Serial.println("\n[WIFI] Tentative de synchronisation NTP...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    unsigned long startWait = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startWait < 6000)) {
        delay(300);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("[WIFI] Connecté ! IP: ");
        Serial.println(WiFi.localIP());
        configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.nist.gov");
        time_t now = 0;
        int retries = 0;
        while (now < 1700000000 && retries++ < 10) {
            delay(500);
            time(&now);
        }
        struct tm t = *localtime(&now);
        Serial.printf("[NTP] Heure réelle obtenue : %02d/%02d/%04d %02d:%02d:%02d\n",
                      t.tm_mday, t.tm_mon + 1, t.tm_year + 1900,
                      t.tm_hour, t.tm_min, t.tm_sec);
    } else {
        Serial.println("[WIFI] Non connecté -> Application de l'heure système de repli (17:00:00)...");
        struct tm fb;
        fb.tm_year = 2026 - 1900;
        fb.tm_mon  = 11 - 1;
        fb.tm_mday = 18;
        fb.tm_hour = 17;
        fb.tm_min  = 0;
        fb.tm_sec  = 0;
        fb.tm_isdst = 0;
        time_t fbEpoch = mktime(&fb);
        struct timeval tv = { fbEpoch, 0 };
        settimeofday(&tv, nullptr);
    }

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    Serial.println("[WIFI] Radio Wi-Fi éteinte pour préserver le BLE.");
}

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_BOOT_PIN, INPUT_PULLUP);

    delay(1000);

    Serial.println("\n================================================================================");
    Serial.println("   Passerelle SEM6000 — EXP 02 INTERACTIVE : Impact Synchronisation Date/Heure ");
    Serial.println("================================================================================");

    syncNtpOrFallback();
    NimBLEDevice::init("Gateway_Exp02");

    // POINT D'ARRÊT N°0 : Attente de l'utilisateur pour voir le démarrage
    waitUserPrompt("L'ESP32 est prêt. Appuyez sur BOOT pour lancer l'ÉTAPE 1 (Lecture Baseline)");

    // =========================================================================
    // ÉTAPE 1 : ÉTAT INITIAL (BASELINE)
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

    // Déconnexion propre
    device.disconnect();
    Serial.println("[BLE] Prise déconnectée.");

    // POINT D'ARRÊT N°1 : Validation avant saut d'heure
    waitUserPrompt("Prenez note des valeurs ci-dessus. Appuyez sur BOOT pour passer à l'ÉTAPE 2 (+1h)");

    // =========================================================================
    // ÉTAPE 2 : AVANCER L'HEURE DE +1 HEURE
    // =========================================================================
    Serial.println("================================================================================");
    Serial.println(" ÉTAPE 2 : MODIFICATION DE L'HEURE -> SAUT DE +1 HEURE");
    Serial.println("================================================================================");

    if (!connectAndAuth()) {
        Serial.println("[FATAL] Échec reconnexion BLE.");
        return;
    }

    time_t now = time(nullptr);
    time_t jumpedTime = now + 3600;
    struct tm jt = *localtime(&jumpedTime);

    Serial.printf("[RTC] Envoi de la nouvelle heure (+1h) : %02d/%02d/%04d %02d:%02d:%02d\n",
                  jt.tm_mday, jt.tm_mon + 1, jt.tm_year + 1900,
                  jt.tm_hour, jt.tm_min, jt.tm_sec);

    SEM6000Status st = device.setDateTime(jt.tm_sec, jt.tm_min, jt.tm_hour,
                                         jt.tm_mday, jt.tm_mon + 1, jt.tm_year + 1900);
    if (st == SEM_OK) {
        Serial.println("[RTC] Commande setDateTime acceptée par la prise !");
    } else {
        Serial.printf("[RTC] ERREUR setDateTime (code: %d)\n", st);
    }

    Serial.println("[ATTENTE] Pause de 10 secondes pour laisser le microprogramme traiter...");
    for (int s = 10; s > 0; s--) {
        Serial.printf("  %d...", s);
        delay(1000);
    }
    Serial.println("\n");

    // =========================================================================
    // ÉTAPE 3 : LECTURE POST-SAUT D'HEURE & COMPARAISON AVEC BASELINE
    // =========================================================================
    Serial.println("================================================================================");
    Serial.println(" ÉTAPE 3 : RELECTURE POST-SAUT (Comparaison directe avec Étape 1)");
    Serial.println("================================================================================");

    if (!captureFullSnapshot(snapPostJump)) {
        Serial.println("[ERREUR] Impossible de lire les historiques après saut.");
    } else {
        printTelemetry(snapPostJump);
        printHourly(snapPostJump.hourly, snapBaseline.hourly);
        printDaily(snapPostJump.daily, snapBaseline.daily);
        printMonthly(snapPostJump.monthly, snapBaseline.monthly);
    }

    device.disconnect();
    Serial.println("\n[BLE] Prise déconnectée.");

    // POINT D'ARRÊT N°2 : Attente branchement de la charge
    waitUserPrompt("Branchez votre consommateur sur la prise. Laissez-le tourner, puis appuyez sur BOOT pour l'ÉTAPE 5");

    // =========================================================================
    // ÉTAPE 5 : RECONNEXION & LECTURE POST-CONSOMMATION
    // =========================================================================
    Serial.println("================================================================================");
    Serial.println(" ÉTAPE 5 : RELECTURE APRÈS CONSOMMATION (Comparaison avec Étape 3)");
    Serial.println("================================================================================");

    if (!connectAndAuth()) {
        Serial.println("[FATAL] Impossible de se reconnecter.");
        return;
    }

    if (!captureFullSnapshot(snapPostLoad)) {
        Serial.println("[ERREUR] Impossible de lire les historiques post-charge.");
    } else {
        printTelemetry(snapPostLoad);
        printHourly(snapPostLoad.hourly, snapPostJump.hourly);
        printDaily(snapPostLoad.daily, snapPostJump.daily);
        printMonthly(snapPostLoad.monthly, snapPostJump.monthly);
    }

    device.disconnect();
    Serial.println("\n[BLE] Prise déconnectée.");
    Serial.println("\n================================================================================");
    Serial.println("   EXPÉRIENCE 02 TERMINÉE AVEC SUCCÈS !                                        ");
    Serial.println("================================================================================");
}

void loop() {
    delay(1000);
}
