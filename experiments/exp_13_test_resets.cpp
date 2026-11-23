/**
 * @file exp_13_test_resets.cpp
 * @brief Expérience 13 : Validation empirique des commandes Reset Consommation (0x0F0002) et Factory Reset (0x0F0000).
 * 
 * Objectifs :
 * 0. Dump complet initial de la prise (Nom, S/N, FW/HW, Settings, Tarifs, Schedulers, Random, Timer, Historiques 24h/30j/12m, FFF2).
 * 1. Émission de resetConsumptionData() (0x0F0002).
 * 2. Dump complet post-reset consommation et comparaison différentielle : vérifier que SEULS les historiques sont remis à zéro.
 * 3. Validation par l'utilisateur (BOOT ou Entrée).
 * 4. Émission de factoryReset() (0x0F0000), gestion d'une éventuelle déconnexion/reboot BLE et reconnexion avec PIN usine.
 * 5. Dump complet post-factory reset et bilan exhaustif de ce qui a été réinitialisé en usine.
 * 6. Libération propre de la prise pour reconfiguration via l'app officielle.
 */

#include <Arduino.h>
#include <vector>
#include "SEM6000Device.h"

// Prise de test (Théoule)
static const char* TARGET_MAC = "94:a9:a8:1a:37:0c";
static const uint32_t DEFAULT_PIN = 0x00000000;
static const int BUTTON_BOOT_PIN = 0;

static SEM6000Device device(TARGET_MAC, DEFAULT_PIN);

struct PlugDump {
    String nameGATT;
    String serial;
    SEMDeviceInfo devInfo;
    SEMSettings settings;
    std::vector<SEMSchedulerSlot> schedulers;
    SEMRandomConfig randomMode;
    SEMTimerStatus timer;
    SEMTelemetry telemetry;
    uint16_t fff2Word0;
    uint8_t fff2CurByte;
    float fff2Amps;
    uint16_t hourlyWh[24];
    uint32_t dailyWh[30];
    uint32_t monthlyWh[12];
    bool valid;
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

bool ensureConnectedAndAuth() {
    if (device.isConnected() && device.isAuthorized()) {
        return true;
    }

    for (int attempt = 1; attempt <= 3; attempt++) {
        if (!device.isConnected()) {
            Serial.printf("[BLE] Connexion à %s (tentative %d/3)...\n", TARGET_MAC, attempt);
            if (!device.connect(8000)) {
                Serial.println("[BLE] ⚠️ Échec de connexion, nouvelle tentative dans 2s...");
                device.disconnect();
                delay(2000);
                continue;
            }
            Serial.println("[BLE] ✅ Connecté !");
        }

        if (!device.isAuthorized()) {
            Serial.println("[BLE] Authentification PIN 32 bits (0x00000000)...");
            SEM6000Status st = device.authenticate32((uint32_t)0x00000000);
            if (st != SEM_OK) {
                Serial.printf("[BLE] ⚠️ Échec authentification (code: %d)\n", (int)st);
                device.disconnect();
                delay(1500);
                continue;
            }
            Serial.println("[BLE] ✅ Authentifié avec succès !");
            return true;
        } else {
            return true;
        }
    }
    return false;
}

bool collectDump(PlugDump& dump) {
    memset(&dump, 0, sizeof(dump));
    dump.schedulers.clear();

    if (!ensureConnectedAndAuth()) return false;

    device.getDeviceNameGATT(dump.nameGATT);
    device.getSerialNumber(dump.serial);
    device.getDeviceVendorAndVersions(dump.devInfo);
    device.readRawFFF2(dump.fff2Word0, dump.fff2CurByte, dump.fff2Amps);
    device.requestSettings(dump.settings);
    device.getRandomMode(dump.randomMode);
    device.getTimerStatus(dump.timer);
    device.requestAllSchedulers(dump.schedulers);
    device.captureMeasurement(dump.telemetry);
    device.requestDayHistory(dump.hourlyWh);
    device.requestMonthHistory(dump.dailyWh);
    device.requestYearHistory(dump.monthlyWh);

    dump.valid = true;
    return true;
}

void printDump(const char* label, const PlugDump& d) {
    Serial.println("\n################################################################################");
    Serial.printf("  📋 %s\n", label);
    Serial.println("################################################################################");

    // 1. Identité & Versions
    Serial.println("\n--- [1. IDENTITÉ & CARACTÉRISTIQUES MATÉRIELLES] ---");
    Serial.printf("  Nom GATT (0x2A00)  : '%s'\n", d.nameGATT.c_str());
    Serial.printf("  Numéro de Série    : '%s'\n", d.serial.c_str());
    Serial.printf("  Fabricant          : %s\n", d.devInfo.vendor.c_str());
    Serial.printf("  Version Firmware   : %s\n", d.devInfo.firmwareVersion.c_str());
    Serial.printf("  Version Hardware   : %s (Byte16: 0x%02X)\n", d.devInfo.hardwareVersion.c_str(), d.devInfo.byte16);

    // 2. FFF2 & Télémétrie
    Serial.println("\n--- [2. DIAGNOSTIC LÉGER & TÉLÉMÉTRIE INSTANTANÉE] ---");
    Serial.printf("  Caractéristique FFF2: Word0 (Seuil mA) = %u mA  |  Word1 (Courant mA) = %.3f A (raw byte: 0x%02X)\n", 
                  d.fff2Word0, d.fff2Amps, d.fff2CurByte);
    Serial.printf("  Télémétrie 0x0400  : Relais = %s  |  P = %.2f W  |  U = %u V  |  I = %.3f A  |  F = %u Hz\n",
                  d.telemetry.relayState ? "🟢 ON" : "🔴 OFF",
                  d.telemetry.powerWatts, d.telemetry.voltageVolts, d.telemetry.currentAmperes, d.telemetry.frequencyHz);

    // 3. Paramètres & Tarifs
    Serial.println("\n--- [3. PARAMÈTRES GÉNÉRAUX & TARIFICATION (0x1000)] ---");
    Serial.printf("  Seuil Puissance Max: %u W\n", d.settings.maxPowerWatts);
    Serial.printf("  Tarif HP (Normal)  : %u centimes/kWh\n", d.settings.normalPriceCents);
    Serial.printf("  Tarif HC (Réduit)  : %u centimes/kWh\n", d.settings.reducedPriceCents);
    uint8_t hStart = d.settings.reducedPeriodStart / 60;
    uint8_t mStart = d.settings.reducedPeriodStart % 60;
    uint8_t hStop  = d.settings.reducedPeriodStop / 60;
    uint8_t mStop  = d.settings.reducedPeriodStop % 60;
    Serial.printf("  Période HC Active  : %s (%02dh%02d -> %02dh%02d, raw: %u -> %u min)\n",
                  d.settings.reducedModeActive ? "OUI" : "NON",
                  hStart, mStart, hStop, mStop,
                  d.settings.reducedPeriodStart, d.settings.reducedPeriodStop);
    Serial.printf("  Anneau LED         : 0x%02X\n", d.settings.ledStatus);

    // 4. Timer & Random
    Serial.println("\n--- [4. TIMER & MODE ALÉATOIRE] ---");
    Serial.printf("  Timer (0x0900)     : %s  |  Action: %s  |  Cible: 20%02d/%02d/%02d %02d:%02d:%02d  |  Preset: %lu s\n",
                  d.timer.active ? "ACTIF" : "INACTIF",
                  d.timer.actionTurnOn ? "ON" : "OFF",
                  d.timer.targetYear, d.timer.targetMonth, d.timer.targetDay,
                  d.timer.targetHour, d.timer.targetMin, d.timer.targetSec,
                  (unsigned long)d.timer.presetSeconds);
    Serial.printf("  Mode Aléatoire     : %s  |  Jours: 0x%02X  |  Plage: %02dh%02d -> %02dh%02d\n",
                  d.randomMode.active ? "ACTIF" : "INACTIF",
                  d.randomMode.dayMask,
                  d.randomMode.startHour, d.randomMode.startMin,
                  d.randomMode.endHour, d.randomMode.endMin);

    // 5. Programmations (Schedulers)
    Serial.println("\n--- [5. PROGRAMMATIONS (SCHEDULERS - 0x1400)] ---");
    Serial.printf("  Nombre total de Schedulers : %u\n", (unsigned int)d.schedulers.size());
    if (d.schedulers.empty()) {
        Serial.println("  -> Aucun scheduler configuré.");
    } else {
        for (size_t i = 0; i < d.schedulers.size(); i++) {
            const auto& s = d.schedulers[i];
            Serial.printf("  [Slot %02u] %s | Action: %s | Jours: 0x%02X | Heure: %02d:%02d | Date: 20%02d/%02d/%02d (sortKey: %u)\n",
                          s.slotId,
                          s.active ? "ACTIF  " : "INACTIF",
                          s.actionTurnOn ? "ON " : "OFF",
                          s.dayMask, s.hour, s.min,
                          s.year, s.month, s.day, s.sortKey);
        }
    }

    // 6. Historiques Énergétiques
    Serial.println("\n--- [6. HISTORIQUES ÉNERGÉTIQUES] ---");
    uint32_t total24h = 0;
    int nonZero24h = 0;
    for (int i = 0; i < 24; i++) {
        total24h += d.hourlyWh[i];
        if (d.hourlyWh[i] > 0) nonZero24h++;
    }
    Serial.printf("  Historique 24h (hourlyWh) : Total = %lu Wh  |  Slots non nuls = %d/24\n", (unsigned long)total24h, nonZero24h);
    if (nonZero24h > 0) {
        Serial.print("    Détail non nuls : ");
        for (int i = 0; i < 24; i++) {
            if (d.hourlyWh[i] > 0) Serial.printf("[H-%d: %uWh] ", 23 - i, d.hourlyWh[i]);
        }
        Serial.println();
    }

    uint32_t total30j = 0;
    int nonZero30j = 0;
    for (int i = 0; i < 30; i++) {
        uint32_t wh = d.dailyWh[i] & 0x00FFFFFF;
        total30j += wh;
        if (wh > 0) nonZero30j++;
    }
    Serial.printf("  Historique 30j (dailyWh)  : Total = %lu Wh  |  Jours non nuls = %d/30\n", (unsigned long)total30j, nonZero30j);
    if (nonZero30j > 0) {
        Serial.print("    Détail non nuls : ");
        for (int i = 0; i < 30; i++) {
            uint32_t wh = d.dailyWh[i] & 0x00FFFFFF;
            uint8_t ratio = (d.dailyWh[i] >> 24) & 0xFF;
            if (wh > 0) Serial.printf("[J-%d: %luWh (HC:%u%%)] ", 29 - i, (unsigned long)wh, ratio);
        }
        Serial.println();
    }

    uint32_t total12m = 0;
    int nonZero12m = 0;
    for (int i = 0; i < 12; i++) {
        uint32_t wh = d.monthlyWh[i] & 0x00FFFFFF;
        total12m += wh;
        if (wh > 0) nonZero12m++;
    }
    Serial.printf("  Historique 12m (monthlyWh): Total = %lu Wh  |  Mois non nuls = %d/12\n", (unsigned long)total12m, nonZero12m);
    if (nonZero12m > 0) {
        Serial.print("    Détail non nuls : ");
        for (int i = 0; i < 12; i++) {
            uint32_t wh = d.monthlyWh[i] & 0x00FFFFFF;
            uint8_t ratio = (d.monthlyWh[i] >> 24) & 0xFF;
            if (wh > 0) Serial.printf("[M-%d: %luWh (HC:%u%%)] ", 11 - i, (unsigned long)wh, ratio);
        }
        Serial.println();
    }
    Serial.println("################################################################################\n");
}

void compareDumps(const PlugDump& d0, const PlugDump& d1, const char* stepTitle) {
    Serial.println("\n================================================================================");
    Serial.printf("  🔍 ANALYSE DIFFÉRENTIELLE : %s\n", stepTitle);
    Serial.println("================================================================================");

    // Nom GATT
    if (d0.nameGATT == d1.nameGATT) {
        Serial.printf("  [NOM GATT]        : 🟢 INCHANGÉ ('%s')\n", d1.nameGATT.c_str());
    } else {
        Serial.printf("  [NOM GATT]        : 🔴 MODIFIÉ ('%s' -> '%s')\n", d0.nameGATT.c_str(), d1.nameGATT.c_str());
    }

    // Settings
    if (d0.settings.maxPowerWatts == d1.settings.maxPowerWatts &&
        d0.settings.normalPriceCents == d1.settings.normalPriceCents &&
        d0.settings.reducedPriceCents == d1.settings.reducedPriceCents &&
        d0.settings.reducedPeriodStart == d1.settings.reducedPeriodStart &&
        d0.settings.reducedPeriodStop == d1.settings.reducedPeriodStop &&
        d0.settings.ledStatus == d1.settings.ledStatus) {
        Serial.println("  [SETTINGS & PRIX] : 🟢 INCHANGÉS (Seuil, Tarifs HP/HC, Période HC, LED)");
    } else {
        Serial.println("  [SETTINGS & PRIX] : 🔴 MODIFIÉS :");
        if (d0.settings.maxPowerWatts != d1.settings.maxPowerWatts)
            Serial.printf("    - Seuil P_max   : %u W -> %u W\n", d0.settings.maxPowerWatts, d1.settings.maxPowerWatts);
        if (d0.settings.normalPriceCents != d1.settings.normalPriceCents)
            Serial.printf("    - Tarif Normal  : %u -> %u centimes\n", d0.settings.normalPriceCents, d1.settings.normalPriceCents);
        if (d0.settings.reducedPriceCents != d1.settings.reducedPriceCents)
            Serial.printf("    - Tarif Réduit  : %u -> %u centimes\n", d0.settings.reducedPriceCents, d1.settings.reducedPriceCents);
        if (d0.settings.reducedPeriodStart != d1.settings.reducedPeriodStart || d0.settings.reducedPeriodStop != d1.settings.reducedPeriodStop)
            Serial.printf("    - Période HC    : %u->%u -> %u->%u min\n", 
                          d0.settings.reducedPeriodStart, d0.settings.reducedPeriodStop,
                          d1.settings.reducedPeriodStart, d1.settings.reducedPeriodStop);
        if (d0.settings.ledStatus != d1.settings.ledStatus)
            Serial.printf("    - Anneau LED    : 0x%02X -> 0x%02X\n", d0.settings.ledStatus, d1.settings.ledStatus);
    }

    // Schedulers
    if (d0.schedulers.size() == d1.schedulers.size()) {
        Serial.printf("  [SCHEDULERS]      : 🟢 INCHANGÉS (Toujours %u schedulers)\n", (unsigned int)d1.schedulers.size());
    } else {
        Serial.printf("  [SCHEDULERS]      : 🔴 MODIFIÉS (%u schedulers -> %u schedulers)\n", 
                      (unsigned int)d0.schedulers.size(), (unsigned int)d1.schedulers.size());
    }

    // Mode Aléatoire
    if (d0.randomMode.active == d1.randomMode.active &&
        d0.randomMode.dayMask == d1.randomMode.dayMask &&
        d0.randomMode.startHour == d1.randomMode.startHour &&
        d0.randomMode.endHour == d1.randomMode.endHour) {
        Serial.println("  [MODE ALÉATOIRE]  : 🟢 INCHANGÉ");
    } else {
        Serial.printf("  [MODE ALÉATOIRE]  : 🔴 MODIFIÉ (Actif: %d -> %d)\n", d0.randomMode.active, d1.randomMode.active);
    }

    // Historiques 24h
    uint32_t t24_0 = 0, t24_1 = 0;
    for (int i = 0; i < 24; i++) { t24_0 += d0.hourlyWh[i]; t24_1 += d1.hourlyWh[i]; }
    if (t24_1 == 0 && t24_0 > 0) {
        Serial.printf("  [HISTORIQUE 24H]  : 🧹 EFFACÉ / REMIS À ZÉRO ! (%lu Wh -> 0 Wh)\n", (unsigned long)t24_0);
    } else if (t24_1 == t24_0) {
        Serial.printf("  [HISTORIQUE 24H]  : 🟢 INCHANGÉ (%lu Wh)\n", (unsigned long)t24_1);
    } else {
        Serial.printf("  [HISTORIQUE 24H]  : 🟡 PARTIEL (%lu Wh -> %lu Wh)\n", (unsigned long)t24_0, (unsigned long)t24_1);
    }

    // Historiques 30j
    uint32_t t30_0 = 0, t30_1 = 0;
    for (int i = 0; i < 30; i++) { t30_0 += (d0.dailyWh[i] & 0x00FFFFFF); t30_1 += (d1.dailyWh[i] & 0x00FFFFFF); }
    if (t30_1 == 0 && t30_0 > 0) {
        Serial.printf("  [HISTORIQUE 30J]  : 🧹 EFFACÉ / REMIS À ZÉRO ! (%lu Wh -> 0 Wh)\n", (unsigned long)t30_0);
    } else if (t30_1 == t30_0) {
        Serial.printf("  [HISTORIQUE 30J]  : 🟢 INCHANGÉ (%lu Wh)\n", (unsigned long)t30_1);
    } else {
        Serial.printf("  [HISTORIQUE 30J]  : 🟡 PARTIEL (%lu Wh -> %lu Wh)\n", (unsigned long)t30_0, (unsigned long)t30_1);
    }

    // Historiques 12m
    uint32_t t12_0 = 0, t12_1 = 0;
    for (int i = 0; i < 12; i++) { t12_0 += (d0.monthlyWh[i] & 0x00FFFFFF); t12_1 += (d1.monthlyWh[i] & 0x00FFFFFF); }
    if (t12_1 == 0 && t12_0 > 0) {
        Serial.printf("  [HISTORIQUE 12M]  : 🧹 EFFACÉ / REMIS À ZÉRO ! (%lu Wh -> 0 Wh)\n", (unsigned long)t12_0);
    } else if (t12_1 == t12_0) {
        Serial.printf("  [HISTORIQUE 12M]  : 🟢 INCHANGÉ (%lu Wh)\n", (unsigned long)t12_1);
    } else {
        Serial.printf("  [HISTORIQUE 12M]  : 🟡 PARTIEL (%lu Wh -> %lu Wh)\n", (unsigned long)t12_0, (unsigned long)t12_1);
    }

    Serial.println("================================================================================\n");
}

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_BOOT_PIN, INPUT_PULLUP);

    unsigned long startWait = millis();
    while (!Serial && (millis() - startWait < 4000));
    delay(500);

    // Initialisation indispensable de la pile Bluetooth NimBLE
    NimBLEDevice::init("Gateway_Exp13");

    Serial.println("\n\n================================================================================");
    Serial.println("  🚀 EXPÉRIENCE 13 : TEST COMPLET DES COMMANDES DE RÉINITIALISATION");
    Serial.println("     1. resetConsumptionData() (0x0F0002) - Remise à zéro des index énergétiques");
    Serial.println("     2. factoryReset()        (0x0F0000) - Réinitialisation usine globale");
    Serial.println("================================================================================");
    Serial.printf("  Cible : Prise %s  |  PIN : 0x%08lX\n", TARGET_MAC, (unsigned long)DEFAULT_PIN);

    // ========================================================================
    // ÉTAPE 0 : DUMP INITIAL COMPLET
    // ========================================================================
    waitUserPrompt("ÉTAPE 0 - DUMP INITIAL DE LA PRISE",
                   "Assurez-vous que la prise est alimentée et que vous avez injecté vos plannings/réglages.\n"
                   "Le programme va se connecter et capturer l'intégralité des données actuelles.");

    PlugDump dump0;
    Serial.println("[DUMP #0] Capture de l'état initial...");
    if (!collectDump(dump0)) {
        Serial.println("❌ Erreur critique lors de la collecte initiale ! Abandon.");
        return;
    }
    printDump("DUMP #0 : ÉTAT INITIAL COMPLET AVANT RESET", dump0);

    // ========================================================================
    // ÉTAPE 1 : RESET DES DONNÉES DE CONSOMMATION (0x0F0002)
    // ========================================================================
    waitUserPrompt("ÉTAPE 1 - COMMANDE RESET HISTORIQUES (0x0F0002)",
                   "Cette étape va envoyer la commande resetConsumptionData() (0x0F0002).\n"
                   "Les historiques 24h, 30j et 12m doivent être remis à zéro,\n"
                   "mais le Nom, Schedulers, Prix et Settings doivent rester intacts !");

    Serial.println("[RESET CONSO] Envoi de la commande resetConsumptionData()...");
    SEM6000Status stConso = device.resetConsumptionData();
    Serial.printf("[RESET CONSO] Code retour de la commande : %d (%s)\n", 
                  (int)stConso, (stConso == SEM_OK) ? "✅ SUCCÈS (SEM_OK)" : "❌ ÉCHEC");

    Serial.println("[RESET CONSO] Temporisation de 3 secondes pour l'écriture Flash...");
    delay(3000);

    // ========================================================================
    // ÉTAPE 2 : DUMP POST-RESET CONSOMMATION & ANALYSE DIFFÉRENTIELLE
    // ========================================================================
    PlugDump dump1;
    Serial.println("[DUMP #1] Capture de l'état après resetConsumptionData()...");
    if (!collectDump(dump1)) {
        Serial.println("❌ Erreur lors de la collecte post-reset consommation !");
    } else {
        printDump("DUMP #1 : ÉTAT APRÈS RESET DES HISTORIQUES DE CONSOMMATION", dump1);
        compareDumps(dump0, dump1, "DUMP #0 (Initial) vs DUMP #1 (Post-Reset Conso)");
    }

    // ========================================================================
    // ÉTAPE 3 : FACTORY RESET (0x0F0000)
    // ========================================================================
    waitUserPrompt("ÉTAPE 3 - FACTORY RESET (0x0F0000)",
                   "⚠️ ATTENTION : Cette étape va envoyer la commande factoryReset() (0x0F0000).\n"
                   "Toutes les programmations, tarifs, seuils et potentiellement le nom de la prise\n"
                   "seront remis aux valeurs d'usine par défaut !\n"
                   "Si la prise se déconnecte, le programme tentera une reconnexion automatique.");

    Serial.println("[FACTORY RESET] Envoi de la commande factoryReset()...");
    SEM6000Status stFactory = device.factoryReset();
    Serial.printf("[FACTORY RESET] Code retour de la commande : %d (%s)\n", 
                  (int)stFactory, (stFactory == SEM_OK) ? "✅ SUCCÈS (SEM_OK)" : "❌ ÉCHEC");

    Serial.println("[FACTORY RESET] Attente de 5 secondes (reboot / réinitialisation firmware prise)...");
    delay(5000);

    // Déconnexion propre de la session pour forcer un handshake neuf post-reset
    Serial.println("[BLE] Fermeture préventive de la session pour reconnexion fraîche...");
    device.disconnect();
    delay(2000);

    // ========================================================================
    // ÉTAPE 4 : DUMP POST-FACTORY RESET & ANALYSE DIFFÉRENTIELLE
    // ========================================================================
    PlugDump dump2;
    Serial.println("[DUMP #2] Capture de l'état après factoryReset()...");
    if (!collectDump(dump2)) {
        Serial.println("❌ Erreur lors de la collecte post-factory reset !");
    } else {
        printDump("DUMP #2 : ÉTAT APRÈS FACTORY RESET", dump2);
        compareDumps(dump0, dump2, "DUMP #0 (Initial) vs DUMP #2 (Post-Factory Reset)");
    }

    // ========================================================================
    // ÉTAPE 5 : DÉCONNEXION & BILAN
    // ========================================================================
    Serial.println("[FIN] Déconnexion propre de la prise...");
    device.disconnect();
    Serial.println("[BLE] ✅ Prise déconnectée.");

    Serial.println("\n================================================================================");
    Serial.println("  🎉 EXPÉRIENCE 13 TERMINÉE !");
    Serial.println("  Vous pouvez maintenant reprendre la main avec l'application mobile constructeur");
    Serial.println("  pour reconfigurer le nom de la prise et lui redonner ses paramètres.");
    Serial.println("================================================================================\n");
}

void loop() {
    delay(1000);
}
