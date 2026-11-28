/**
 * @file exp_23_validation_lib_v040.cpp
 * @brief Expérience 23 : Qualification officielle bout-en-bout de la librairie Couche 1 SEM6000Device v0.4.0
 *        ("La Grande Réunification : One Roof for Two Hardwares — HW 2.0 & HW 3.0"), incluant :
 *          - Volet 1 : Validation de toutes les méthodes de base + auto-guérison d'horloge (checkAndFixClockLoss)
 *          - Volet 2 : Stress-test & Alignement des Plannings (0x1300 / 0x1400 : création de 13 schedulers -> 12 acceptés
 *                      et 13e rejeté, dump 3 pages x 56 octets, modification de 2 slots, destruction et retour à 0)
 *          - Volet 3 : Test complet du Mode Aléatoire (0x1500 / 0x1600 : activation sur plage nocturne 22:00->05:00,
 *                      vérification après 3s, puis désactivation).
 * @requires SEM6000Device >= v0.4.0 (SEM6000_DEVICE_VERSION_HEX >= 0x000400)
 *
 * Topologie physique du banc en cascade (Daisy-Chain) :
 *   Secteur 230V ---> VÉLO (HW 3.0, a3:00:00:00:45:25) ---> THÉOULE (HW 2.0, 94:a9:a8:1a:37:0c) ---> Charge (Sèche-cheveux)
 *
 * Garde-fous temporels et électriques :
 *   - Vélo est testée en premier avec maintien strict de son relais à ON pour ne jamais couper l'alimentation de Théoule.
 *   - L'horloge est calée à 28/11/2026 09:15:00 (matin).
 *   - Tous les plannings (0x1300) et le mode aléatoire (0x1500) sont programmés dans la plage nocturne 22:00 -> 05:00
 *     (> 12 heures d'écart avec 09:15:00) pour garantir 0 commutation de relais intempestive.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <vector>
#include "SEM6000Device.h"

static_assert(SEM6000_DEVICE_VERSION_HEX >= 0x000400,
              "exp_23_validation_lib_v040 requiert SEM6000Device >= v0.4.0");

static const int   BUTTON_BOOT_PIN  = 0;
static const char* MAC_VELO_HW3     = "a3:00:00:00:45:25";
static const char* MAC_THEOULE_HW2  = "94:a9:a8:1a:37:0c";

static const char* statusToStr(SEM6000Status st) {
    switch (st) {
        case SEM_OK:                    return "SEM_OK";
        case SEM_ERR_NOT_CONNECTED:     return "SEM_ERR_NOT_CONNECTED";
        case SEM_ERR_AUTH_FAILED:       return "SEM_ERR_AUTH_FAILED";
        case SEM_ERR_INVALID_PARAM:     return "SEM_ERR_INVALID_PARAM";
        case SEM_ERR_BLE_WRITE_FAILED:  return "SEM_ERR_BLE_WRITE_FAILED";
        case SEM_ERR_TIMEOUT:           return "SEM_ERR_TIMEOUT";
        case SEM_ERR_RESPONSE_REJECTED: return "SEM_ERR_RESPONSE_REJECTED";
        case SEM_ERR_NOTIFICATION:      return "SEM_ERR_NOTIFICATION";
        case SEM_ERR_NOT_SUPPORTED:     return "SEM_ERR_NOT_SUPPORTED";
        default:                        return "UNKNOWN";
    }
}

static void printLastRxHex(const char* prefix, const SEM6000Device& dev) {
    const uint8_t* buf = nullptr;
    size_t len = dev.getLastRxBuffer(&buf);
    Serial.printf("%s (%2u o, CS_Valid=%s) : ", prefix, (unsigned)len, dev.wasLastChecksumValid() ? "OUI" : "NON");
    for (size_t i = 0; i < len; i++) {
        Serial.printf("%02X ", buf[i]);
    }
    Serial.println();
}

static void waitForBootButtonOrSerial(const char* promptMsg) {
    Serial.printf("\n  [%s] Appuyez sur BOOT (GPIO 0) ou envoyez 'Entrée'...\n", promptMsg);
    while (Serial.available()) Serial.read();
    while (true) {
        if (digitalRead(BUTTON_BOOT_PIN) == LOW) {
            delay(50);
            if (digitalRead(BUTTON_BOOT_PIN) == LOW) {
                while (digitalRead(BUTTON_BOOT_PIN) == LOW) delay(20);
                Serial.println(">>> Action validée (Bouton BOOT) !");
                return;
            }
        }
        if (Serial.available() > 0) {
            while (Serial.available()) Serial.read();
            Serial.println(">>> Action validée (Moniteur Série) !");
            return;
        }
        delay(30);
    }
}

static void dumpAllSchedulersDetailed(SEM6000Device& dev, const char* stepTitle, size_t& outCount) {
    Serial.printf("\n    📋 [DUMP SCHEDULERS : %s]\n", stepTitle);

    // 1. Lecture brute page par page pour inspecter l'alignement des trames 0x1400 (jusqu'à 56 octets/page)
    uint8_t totalCount = 0;
    SEM6000Status stP0 = dev.requestSchedulerPage(0, totalCount);
    printLastRxHex("       📥 Page 0 brute (0x1400)", dev);
    if (stP0 != SEM_OK) {
        Serial.printf("       ❌ Erreur requestSchedulerPage(0) : %s\n", statusToStr(stP0));
        outCount = 0;
        return;
    }

    const uint8_t* p0Buf = nullptr;
    size_t p0Len = dev.getLastRxBuffer(&p0Buf);
    size_t slotsInPage0 = (p0Len >= 8) ? ((p0Len - 8) / 12) : 0;

    if (slotsInPage0 < totalCount) {
        uint8_t totalPages = (totalCount + 3) / 4;
        for (uint8_t p = 1; p < totalPages; p++) {
            delay(80);
            uint8_t dummy = 0;
            dev.requestSchedulerPage(p, dummy);
            char lbl[40];
            snprintf(lbl, sizeof(lbl), "       📥 Page %u brute (0x1400)", (unsigned)p);
            printLastRxHex(lbl, dev);
        }
    }

    // 2. Lecture structurée via requestAllSchedulers()
    std::vector<SEMSchedulerSlot> list;
    SEM6000Status stAll = dev.requestAllSchedulers(list);
    outCount = list.size();
    Serial.printf("       -> requestAllSchedulers() = %s | Total annoncé=%u | Slots extraits=%u\n",
                  statusToStr(stAll), (unsigned)totalCount, (unsigned)list.size());

    if (!list.empty()) {
        Serial.println("       Idx | SlotId | État    | Action | DayMask | Date       | Heure | SortKey");
        Serial.println("       ----+--------+---------+--------+---------+------------+-------+--------");
        for (size_t i = 0; i < list.size(); i++) {
            const SEMSchedulerSlot& s = list[i];
            Serial.printf("       #%02u |   %2u   | %-7s |  %-3s   |  0x%02X   | 20%02u/%02u/%02u | %02u:%02u |   %3u\n",
                          (unsigned)(i + 1),
                          (unsigned)s.slotId,
                          s.active ? "ACTIF" : "INACTIF",
                          s.actionTurnOn ? "ON" : "OFF",
                          (unsigned)s.dayMask,
                          (unsigned)s.year, (unsigned)s.month, (unsigned)s.day,
                          (unsigned)s.hour, (unsigned)s.min,
                          (unsigned)s.sortKey);
        }
    }
}

struct DeviceAuditSummary {
    const char* label;
    const char* mac;
    bool        connected;
    bool        isHw3;
    bool        hasFFF2;
    uint16_t    fff2BeforeSync;
    uint16_t    fff2AfterSync;
    char        fwHwStr[64];
    char        gattName[32];
    SEM6000Status stAuth;
    SEM6000Status stClockCheck;
    bool          clockLossDetected;
    SEM6000Status st0400;
    bool          relayOn;
    float         powerW;
    uint32_t      totalKwhWh;
    SEM6000Status st1000;
    uint16_t      maxPowerW;
    SEM6000Status st0F00Led;
    SEM6000Status st0900Timer;
    uint32_t      timerPresetSec24b;
    SEM6000Status st1100Serial;
    char          serialStr[24];
    SEM6000Status st0A00;
    bool          cs0A00Valid;
    uint32_t      sum24hWh;
    uint16_t      h23Wh;
    SEM6000Status st0B00;
    bool          cs0B00Valid;
    uint32_t      sum30dWh;
    uint32_t      j29Wh;
    SEM6000Status st0C00;
    bool          cs0C00Valid;
    uint32_t      sum12mWh;
    uint32_t      m11Wh;

    // Résultats Volet 2 (Schedulers)
    size_t        schedCreatedOkCount;
    SEM6000Status stSched13th;
    size_t        schedCountAfter13;
    SEM6000Status stSchedMod1;
    SEM6000Status stSchedMod6;
    size_t        schedCountAfterMod;
    size_t        schedCountAfterDeleteAll;

    // Résultats Volet 3 (Random Mode)
    SEM6000Status stRandActivate;
    bool          randReadbackActive;
    uint8_t       randReadbackMask;
    SEM6000Status stRandDisable;
    bool          randFinalActive;
};

static void validateDeviceWithLibrary(const char* label, const char* mac, bool forceRelayOnForCascade, DeviceAuditSummary& out) {
    memset(&out, 0, sizeof(out));
    out.label = label;
    out.mac   = mac;

    Serial.println("\n========================================================================================================================");
    Serial.printf(" 🧪 QUALIFICATION COMPLÈTE SEM6000Device v%s SUR : %s (%s)\n",
                  SEM6000_DEVICE_VERSION_STR, label, mac);
    Serial.println("========================================================================================================================");

    SEM6000Device dev(mac);

    for (int attempt = 1; attempt <= 3; attempt++) {
        Serial.printf("  [1] dev.connect() — Tentative #%d...\n", attempt);
        if (dev.connect(6000)) {
            out.connected = true;
            break;
        }
        delay(600);
    }

    if (!out.connected) {
        Serial.println("  ❌ Échec de connexion BLE.");
        return;
    }

    out.isHw3   = dev.isHardwareV3();
    out.hasFFF2 = dev.hasCharacteristicFFF2();
    Serial.printf("  ✅ Connecté | isHardwareV3() = %s | hasCharacteristicFFF2() = %s\n",
                  out.isHw3 ? "TRUE (HW 3.0)" : "FALSE (HW 2.0)",
                  out.hasFFF2 ? "TRUE" : "FALSE");

    // ========================================================================================================
    // VOLET 1 : MÉTHODES DE BASE & AUTO-GUÉRISON D'HORLOGE (checkAndFixClockLoss)
    // ========================================================================================================
    Serial.println("\n  --------------------------------------------------------------------------------------------------------");
    Serial.println("  🔹 VOLET 1 : MÉTHODES DE BASE, AUTO-GUÉRISON D'HORLOGE & HISTORIQUES");
    Serial.println("  --------------------------------------------------------------------------------------------------------");

    // 2. Test readRawFFF2() AVANT authentification et AVANT checkAndFixClockLoss
    uint16_t w0 = 0;
    uint8_t  b2 = 0;
    float    ampsDirect = 0.0f;
    bool fff2Ok = dev.readRawFFF2(w0, b2, ampsDirect);
    out.fff2BeforeSync = w0;
    Serial.printf("  [2] dev.readRawFFF2(avant sync)  -> %s", fff2Ok ? "TRUE" : "FALSE (attendu sur HW 3.0)");
    if (fff2Ok) {
        Serial.printf(" (Word0=0x%04X=%u | I=%.3f A)\n", (unsigned)w0, (unsigned)w0, ampsDirect);
    } else {
        Serial.println();
    }

    // 3. Infos Constructeur & Versions (0xFFF1) + Nom GATT (0x2A00)
    SEMDeviceInfo info;
    SEM6000Status stInfo = dev.getDeviceVendorAndVersions(info);
    if (stInfo == SEM_OK) {
        snprintf(out.fwHwStr, sizeof(out.fwHwStr), "HW %s / FW %s (0x%02X)",
                 info.hardwareVersion.c_str(), info.firmwareVersion.c_str(), (unsigned)info.byte16);
    } else {
        snprintf(out.fwHwStr, sizeof(out.fwHwStr), "ERR");
    }
    Serial.printf("  [3] dev.getDeviceVendorAndVersions() -> %-10s | Vendor=\"%s\" | %s\n",
                  statusToStr(stInfo), info.vendor.c_str(), out.fwHwStr);

    String gName;
    SEM6000Status stName = dev.getDeviceNameGATT(gName);
    strncpy(out.gattName, (stName == SEM_OK) ? gName.c_str() : "ERR", sizeof(out.gattName) - 1);
    Serial.printf("  [4] dev.getDeviceNameGATT()      -> %-10s | Name=\"%s\"\n",
                  statusToStr(stName), out.gattName);

    // 4. Authentification 32 bits
    out.stAuth = dev.authenticate32((uint32_t)0x00000000);
    Serial.printf("  [5] dev.authenticate32(0)        -> %s\n", statusToStr(out.stAuth));
    if (out.stAuth != SEM_OK) {
        dev.disconnect();
        return;
    }

    // Si cette prise alimente la cascade (Vélo), s'assurer immédiatement que son relais est ON
    if (forceRelayOnForCascade) {
        SEMTelemetry telInit;
        if (dev.captureMeasurement(telInit) == SEM_OK && !telInit.relayState) {
            Serial.println("  ⚡ [CASCADE] Activation du relais de Vélo (ON) pour alimenter Théoule...");
            dev.setRelayState(true);
            delay(500);
        }
    }

    // 5. Détection et correction de perte d'horloge bi-génération (28/11/2026 09:15:00)
    out.stClockCheck = dev.checkAndFixClockLoss(0, 15, 9, 28, 11, 2026, &out.clockLossDetected);
    Serial.printf("  [6] dev.checkAndFixClockLoss()   -> %-10s | clockLossDetected=%s\n",
                  statusToStr(out.stClockCheck), out.clockLossDetected ? "OUI (Amnésie détectée & corrigée !)" : "NON (Horloge déjà active)");

    // On cale dans tous les cas l'horloge sur le créneau sécurisé du matin 28/11/2026 09:15:00
    // (loin de toute heure pile :00:00 et à >12h des plannings de test 22h00..23h00)
    SEM6000Status stSetDt = dev.setDateTime(0, 15, 9, 28, 11, 2026);
    Serial.printf("  [6b] dev.setDateTime(28/11 09:15)-> %s\n", statusToStr(stSetDt));

    if (out.hasFFF2) {
        uint16_t w0After = 0;
        uint8_t b2After = 0;
        float ampsAfter = 0.0f;
        dev.readRawFFF2(w0After, b2After, ampsAfter);
        out.fff2AfterSync = w0After;
        Serial.printf("  [6c] dev.readRawFFF2(après sync) -> Word0=0x%04X (%u mA) | Démasquage 0xFFFF réussi=%s\n",
                      (unsigned)w0After, (unsigned)w0After, (w0After != 0xFFFF) ? "OUI" : "NON");
    }

    // 6. Télémétrie 0x0400
    SEMTelemetry tel;
    out.st0400 = dev.captureMeasurement(tel);
    out.relayOn = tel.relayState;
    out.powerW  = tel.powerWatts;
    out.totalKwhWh = tel.totalKwhWh;
    Serial.printf("  [7] dev.captureMeasurement()     -> %-10s | Relais=%s | P=%.2f W | U=%u V | I=%.3f A | Word12_13=0x%04X | totalKwhWh=%lu Wh | CS_Valid=%s\n",
                  statusToStr(out.st0400), tel.relayState ? "ON" : "OFF", tel.powerWatts,
                  (unsigned)tel.voltageVolts, tel.currentAmperes, (unsigned)tel.unknownWord,
                  (unsigned long)tel.totalKwhWh, dev.wasLastChecksumValid() ? "OUI" : "NON");

    // 7. Paramètres 0x1000
    SEMSettings cfg;
    out.st1000 = dev.requestSettings(cfg);
    out.maxPowerW = cfg.maxPowerWatts;
    Serial.printf("  [8] dev.requestSettings()        -> %-10s | Pmax=%u W | LED=%u | Tarif HP/HC=%u/%u c | HC=%u (%04X->%04X) | CS_Valid=%s\n",
                  statusToStr(out.st1000), (unsigned)cfg.maxPowerWatts, (unsigned)cfg.ledStatus,
                  (unsigned)cfg.normalPriceCents, (unsigned)cfg.reducedPriceCents,
                  (unsigned)cfg.reducedModeActive, (unsigned)cfg.reducedPeriodStart, (unsigned)cfg.reducedPeriodStop,
                  dev.wasLastChecksumValid() ? "OUI" : "NON");

    // 8. Commande 3 octets CMD_SET_3B : setLedRing (en conservant l'état actuel de la LED)
    out.st0F00Led = dev.setLedRing(cfg.ledStatus != 0);
    Serial.printf("  [9] dev.setLedRing(actuel=%u)     -> %-10s (Validation CMD_SET_3B 9o HW2.0 / 8o HW3.0)\n",
                  (unsigned)(cfg.ledStatus != 0), statusToStr(out.st0F00Led));

    // 9. Test Timer (0x0800 / 0x0900) et décodage 24 bits en secondes (cible 28/11/2026 23:59:59 -> Δt attendu ~ 53099 s = 14h44m59s)
    SEM6000Status stArm = dev.setTimerAbs(true, 59, 59, 23, 28, 11, 2026);
    SEMTimerStatus tStArmed;
    out.st0900Timer = dev.getTimerStatus(tStArmed);
    out.timerPresetSec24b = tStArmed.presetSeconds;
    SEM6000Status stStop = dev.stopTimer();
    Serial.printf("  [10] Sonde Timer API (28/11 23:59:59) -> Arm=%s | Get=%s (preset24b=%lu s, attendu ~53099s) | Stop=%s\n",
                  statusToStr(stArm), statusToStr(out.st0900Timer),
                  (unsigned long)out.timerPresetSec24b, statusToStr(stStop));

    // 10. Numéro de Série (0x1100)
    String serial;
    uint32_t t0Serial = millis();
    out.st1100Serial = dev.getSerialNumber(serial);
    uint32_t dtSerial = millis() - t0Serial;
    strncpy(out.serialStr, (out.st1100Serial == SEM_OK) ? serial.c_str() : "N/A (HW3.0)", sizeof(out.serialStr) - 1);
    Serial.printf("  [11] dev.getSerialNumber()       -> %-21s (en %lu ms) | Serial=\"%s\"\n",
                  statusToStr(out.st1100Serial), (unsigned long)dtSerial, out.serialStr);

    // 11. Historiques 24h (0x0A00), 30j (0x0B00), 12m (0x0C00)
    uint16_t hourlyWh[24] = {0};
    out.st0A00 = dev.requestDayHistory(hourlyWh);
    out.cs0A00Valid = dev.wasLastChecksumValid();
    for (int i = 0; i < 24; i++) out.sum24hWh += hourlyWh[i];
    out.h23Wh = hourlyWh[23];
    Serial.printf("  [12] dev.requestDayHistory()     -> %-10s | Cumul24h=%5lu Wh | H[22..23]=%u/%u Wh | CS_Math_Valid=%s\n",
                  statusToStr(out.st0A00), (unsigned long)out.sum24hWh,
                  (unsigned)hourlyWh[22], (unsigned)hourlyWh[23],
                  out.cs0A00Valid ? "OUI" : "NON (Toléré HW3.0)");

    uint32_t dailyWh[30] = {0};
    out.st0B00 = dev.requestMonthHistory(dailyWh);
    out.cs0B00Valid = dev.wasLastChecksumValid();
    for (int i = 0; i < 30; i++) out.sum30dWh += dailyWh[i];
    out.j29Wh = dailyWh[29];
    Serial.printf("  [13] dev.requestMonthHistory()   -> %-10s | Cumul30j=%5lu Wh | J[28..29]=%lu/%lu Wh | CS_Math_Valid=%s\n",
                  statusToStr(out.st0B00), (unsigned long)out.sum30dWh,
                  (unsigned long)dailyWh[28], (unsigned long)dailyWh[29],
                  out.cs0B00Valid ? "OUI" : "NON (Toléré HW3.0)");

    uint32_t monthlyWh[12] = {0};
    out.st0C00 = dev.requestYearHistory(monthlyWh);
    out.cs0C00Valid = dev.wasLastChecksumValid();
    for (int i = 0; i < 12; i++) out.sum12mWh += monthlyWh[i];
    out.m11Wh = monthlyWh[11];
    Serial.printf("  [14] dev.requestYearHistory()    -> %-10s | Cumul12m=%5lu Wh | M[10..11]=%lu/%lu Wh | CS_Math_Valid=%s\n",
                  statusToStr(out.st0C00), (unsigned long)out.sum12mWh,
                  (unsigned long)monthlyWh[10], (unsigned long)monthlyWh[11],
                  out.cs0C00Valid ? "OUI" : "NON (Toléré HW3.0)");

    // ========================================================================================================
    // VOLET 2 : STRESS-TEST DES SCHEDULERS (0x1300 / 0x1400 : CRÉATION 13 SLOTS, DUMP, MODIF 2 SLOTS, PURGE)
    // ========================================================================================================
    Serial.println("\n  --------------------------------------------------------------------------------------------------------");
    Serial.println("  🔹 VOLET 2 : STRESS-TEST & ALIGNEMENT DES SCHEDULERS (0x1300 / 0x1400 — 12 MAX + 13e REJETÉ)");
    Serial.println("  --------------------------------------------------------------------------------------------------------");

    // 2A. État initial et purge éventuelle
    size_t initSchedCount = 0;
    dumpAllSchedulersDetailed(dev, "2A. État Initial avant création", initSchedCount);
    if (initSchedCount > 0) {
        std::vector<SEMSchedulerSlot> existing;
        dev.requestAllSchedulers(existing);
        for (size_t i = 0; i < existing.size(); i++) {
            dev.deleteScheduler(existing[i].slotId);
            delay(60);
        }
        dumpAllSchedulersDetailed(dev, "2A-bis. Après purge initiale", initSchedCount);
    }

    // 2B. Création de 13 Schedulers consécutifs (Slots 1 à 13)
    // Tous programmés à 22h01..22h13 (à +13h de l'horloge 09:15:00 -> aucun déclenchement possible)
    // Règle bi-génération : passer slotId = 0 en création (mode SCHED_ONCE / 0x00) pour que HW 2.0
    // (qui alloue 12..1 et édite en place si slotId existe déjà) comme HW 3.0 (qui alloue 0..11)
    // créent tous deux 12 nouveaux slots sans collision et rejettent le 13e !
    Serial.println("\n    🛠️ [2B. Création de 13 Schedulers (12 attendus OK avec slotId=0, 13e attendu REJETÉ)] :");
    for (uint8_t slot = 1; slot <= 13; slot++) {
        bool isWeekly  = (slot % 2 == 0);
        uint8_t dMask  = isWeekly ? (uint8_t)(0x01 << ((slot / 2) % 7)) : 0x00;
        bool isEnabled = (slot != 4 && slot != 9); // #4 et #9 créés INACTIFS pour tester l'octet active
        bool actTurnOn = true;                     // Action ON (sécurité cascade)
        uint8_t sHour  = 22;
        uint8_t sMin   = slot;                     // 22:01 à 22:13

        SEM6000Status stCreate = dev.setScheduler(SCHED_ONCE, 0, isEnabled, actTurnOn, dMask, 2026, 12, 25, sHour, sMin);
        const uint8_t* rxPtr = nullptr;
        size_t rxL = dev.getLastRxBuffer(&rxPtr);
        uint8_t rxStatusByte = (rxL >= 5) ? rxPtr[4] : 0xFF;

        Serial.printf("       -> Création #%02u (slotId=0, %s, %s, mask=0x%02X, 22:%02u) : %-25s (RX[4]=0x%02X)\n",
                      (unsigned)slot,
                      isEnabled ? "ACTIF  " : "INACTIF",
                      isWeekly  ? "HEBDO " : "UNIQUE",
                      (unsigned)dMask, (unsigned)sMin,
                      statusToStr(stCreate), (unsigned)rxStatusByte);

        if (slot <= 12 && stCreate == SEM_OK) {
            out.schedCreatedOkCount++;
        }
        if (slot == 13) {
            out.stSched13th = stCreate;
        }
        delay(60);
    }

    // 2C. Dump complet des plannings (12 schedulers)
    dumpAllSchedulersDetailed(dev, "2C. Après tentative de création de 13 Schedulers", out.schedCountAfter13);

    // 2D. Modification de 2 Schedulers existants (1er et 6e de la table réelle) avec le sous-opcode 0x01 (SCHED_REPEAT_WEEKLY = EDIT)
    std::vector<SEMSchedulerSlot> curSlots;
    dev.requestAllSchedulers(curSlots);
    uint8_t idModA = (!curSlots.empty())     ? curSlots[0].slotId : 1;
    uint8_t idModB = (curSlots.size() >= 6)  ? curSlots[5].slotId : 6;

    Serial.printf("\n    ✏️ [2D. Modification de 2 Schedulers existants (SlotId=%u -> 23:31 HEBDO 0x7F, SlotId=%u -> 23:36 INACTIF)] :\n",
                  (unsigned)idModA, (unsigned)idModB);
    out.stSchedMod1 = dev.setScheduler(SCHED_REPEAT_WEEKLY, idModA, true,  true, 0x7F, 2026, 12, 31, 23, 31);
    Serial.printf("       -> Modif SlotId=%2u (23:31, ACTIF, mask=0x7F)   : %s\n", (unsigned)idModA, statusToStr(out.stSchedMod1));
    delay(60);
    out.stSchedMod6 = dev.setScheduler(SCHED_REPEAT_WEEKLY, idModB, false, true, 0x00, 2026, 12, 31, 23, 36);
    Serial.printf("       -> Modif SlotId=%2u (23:36, INACTIF, mask=0x00) : %s\n", (unsigned)idModB, statusToStr(out.stSchedMod6));
    delay(60);

    dumpAllSchedulersDetailed(dev, "2D. Après modification de 2 Slots existants", out.schedCountAfterMod);

    // 2E. Destruction des 12 Schedulers un par un & vérification finale à 0
    Serial.println("\n    🗑️ [2E. Destruction des 12 Schedulers via deleteScheduler(slotId)] :");
    std::vector<SEMSchedulerSlot> toDelete;
    dev.requestAllSchedulers(toDelete);
    size_t delOk = 0;
    for (size_t i = 0; i < toDelete.size(); i++) {
        SEM6000Status stDel = dev.deleteScheduler(toDelete[i].slotId);
        if (stDel == SEM_OK) delOk++;
        delay(50);
    }
    Serial.printf("       -> suppressions confirmées SEM_OK : %u / %u\n", (unsigned)delOk, (unsigned)toDelete.size());

    dumpAllSchedulersDetailed(dev, "2E. Vérification Finale après destruction (Attendu : 0 scheduler)", out.schedCountAfterDeleteAll);

    // ========================================================================================================
    // VOLET 3 : TEST DU MODE ALÉATOIRE / RANDOM MODE (0x1500 / 0x1600) SUR PLAGE NOCTURNE (22:00 -> 05:00)
    // ========================================================================================================
    Serial.println("\n  --------------------------------------------------------------------------------------------------------");
    Serial.println("  🔹 VOLET 3 : TEST DU MODE ALÉATOIRE (0x1500 / 0x1600 — PLAGE NOCTURNE 22:00->05:00 HORS CRÉNEAU 09:15)");
    Serial.println("  --------------------------------------------------------------------------------------------------------");

    SEMRandomConfig rndInit;
    SEM6000Status stRnd0 = dev.getRandomMode(rndInit);
    printLastRxHex("    📥 [3A] getRandomMode(initial)", dev);
    Serial.printf("       -> Status=%s | active=%s | mask=0x%02X | plage=%02u:%02u -> %02u:%02u\n",
                  statusToStr(stRnd0), rndInit.active ? "ACTIF" : "INACTIF", (unsigned)rndInit.dayMask,
                  (unsigned)rndInit.startHour, (unsigned)rndInit.startMin,
                  (unsigned)rndInit.endHour, (unsigned)rndInit.endMin);

    // Activation pendant 3 secondes sur plage nocturne 22:00 -> 05:00 (tous les jours 0x7F)
    out.stRandActivate = dev.setRandomMode(true, 0x7F, 22, 0, 5, 0);
    Serial.printf("    📤 [3B] setRandomMode(ACTIF, mask=0x7F, 22:00->05:00) -> %s (Attente 3 secondes...)\n",
                  statusToStr(out.stRandActivate));
    delay(3000);

    SEMRandomConfig rndActive;
    SEM6000Status stRnd1 = dev.getRandomMode(rndActive);
    out.randReadbackActive = rndActive.active;
    out.randReadbackMask   = rndActive.dayMask;
    printLastRxHex("    📥 [3C] getRandomMode(après 3s)", dev);
    Serial.printf("       -> Status=%s | active=%s | mask=0x%02X | plage=%02u:%02u -> %02u:%02u\n",
                  statusToStr(stRnd1), rndActive.active ? "ACTIF (✅)" : "INACTIF (❌)", (unsigned)rndActive.dayMask,
                  (unsigned)rndActive.startHour, (unsigned)rndActive.startMin,
                  (unsigned)rndActive.endHour, (unsigned)rndActive.endMin);

    // Désactivation et retour à l'état initial inactif
    out.stRandDisable = dev.setRandomMode(false, rndInit.dayMask, rndInit.startHour, rndInit.startMin, rndInit.endHour, rndInit.endMin);
    Serial.printf("    📤 [3D] setRandomMode(INACTIF / Restauration) -> %s\n", statusToStr(out.stRandDisable));
    delay(200);

    SEMRandomConfig rndFinal;
    SEM6000Status stRnd2 = dev.getRandomMode(rndFinal);
    out.randFinalActive = rndFinal.active;
    printLastRxHex("    📥 [3E] getRandomMode(final)", dev);
    Serial.printf("       -> Status=%s | active=%s | mask=0x%02X | plage=%02u:%02u -> %02u:%02u\n",
                  statusToStr(stRnd2), rndFinal.active ? "ACTIF (❌)" : "INACTIF (✅)", (unsigned)rndFinal.dayMask,
                  (unsigned)rndFinal.startHour, (unsigned)rndFinal.startMin,
                  (unsigned)rndFinal.endHour, (unsigned)rndFinal.endMin);

    dev.disconnect();
    delay(400);
    Serial.printf("\n  [BLE] 🔌 Déconnecté proprement de %s.\n", label);
}

static void runValidationV040() {
    Serial.println("\n########################################################################################################################");
    Serial.printf(" 🏆 EXPÉRIENCE 23 : QUALIFICATION COMPLÈTE SEM6000Device v%s SUR CASCADE VÉLO (HW 3.0) -> THÉOULE (HW 2.0)\n",
                  SEM6000_DEVICE_VERSION_STR);
    Serial.println("########################################################################################################################");

    DeviceAuditSummary resHw3;
    DeviceAuditSummary resHw2;

    // 1. VÉLO (HW 3.0) EN PREMIER (tête de cascade : forceRelayOnForCascade = true)
    validateDeviceWithLibrary("1. TÊTE CASCADE : VÉLO (HW 3.0)", MAC_VELO_HW3, true, resHw3);
    delay(800);

    // 2. THÉOULE (HW 2.0) EN SECOND (alimentée par Vélo, vient d'être rebranchée -> test live 0xFFFF !)
    validateDeviceWithLibrary("2. AVAL CASCADE : THÉOULE (HW 2.0)", MAC_THEOULE_HW2, false, resHw2);

    Serial.println("\n========================================================================================================================");
    Serial.printf(" 📊 MATRICE DE QUALIFICATION FINALE — SEM6000Device v%s (THÉOULE HW 2.0 vs VÉLO HW 3.0)\n", SEM6000_DEVICE_VERSION_STR);
    Serial.println("========================================================================================================================");
    Serial.printf(" Méthode / Test API SEM6000Device     | %-36s | %-36s\n", resHw2.label, resHw3.label);
    Serial.println("--------------------------------------+--------------------------------------+--------------------------------------");
    Serial.printf(" connect() & isHardwareV3()           | %-5s / isHw3=%-5s                  | %-5s / isHw3=%-5s\n",
                  resHw2.connected ? "OK" : "FAIL", resHw2.isHw3 ? "TRUE" : "FALSE",
                  resHw3.connected ? "OK" : "FAIL", resHw3.isHw3 ? "TRUE" : "FALSE");
    Serial.printf(" FFF2 Avant / Après Sync              | 0x%04X -> 0x%04X                     | N/A (0xFFF2 absente)\n",
                  (unsigned)resHw2.fff2BeforeSync, (unsigned)resHw2.fff2AfterSync);
    Serial.printf(" getDeviceVendorAndVersions (0xFFF1)  | %-36s | %-36s\n", resHw2.fwHwStr, resHw3.fwHwStr);
    Serial.printf(" getDeviceNameGATT (0x2A00)           | %-36s | %-36s\n", resHw2.gattName, resHw3.gattName);
    Serial.printf(" authenticate32 (0x1700)              | %-36s | %-36s\n", statusToStr(resHw2.stAuth), statusToStr(resHw3.stAuth));
    Serial.printf(" checkAndFixClockLoss()               | %-16s (loss=%-3s)          | %-16s (loss=%-3s)\n",
                  statusToStr(resHw2.stClockCheck), resHw2.clockLossDetected ? "OUI" : "NON",
                  statusToStr(resHw3.stClockCheck), resHw3.clockLossDetected ? "OUI" : "NON");
    Serial.printf(" captureMeasurement (0x0400)          | %-6s (%-3s, P=%6.1fW, Tot=%5luWh) | %-6s (%-3s, P=%6.1fW, Tot=%5luWh)\n",
                  statusToStr(resHw2.st0400), resHw2.relayOn ? "ON" : "OFF", resHw2.powerW, (unsigned long)resHw2.totalKwhWh,
                  statusToStr(resHw3.st0400), resHw3.relayOn ? "ON" : "OFF", resHw3.powerW, (unsigned long)resHw3.totalKwhWh);
    Serial.printf(" requestSettings (0x1000, pad 20o)    | %-10s (Pmax=%4u W)             | %-10s (Pmax=%4u W)\n",
                  statusToStr(resHw2.st1000), (unsigned)resHw2.maxPowerW,
                  statusToStr(resHw3.st1000), (unsigned)resHw3.maxPowerW);
    Serial.printf(" setLedRing (0x0F0005, CMD_SET_3B)    | %-36s | %-36s\n",
                  statusToStr(resHw2.st0F00Led), statusToStr(resHw3.st0F00Led));
    Serial.printf(" getTimerStatus (0x0900, 24b sec)     | %-10s (Δt=%8lu s)           | %-10s (Δt=%8lu s)\n",
                  statusToStr(resHw2.st0900Timer), (unsigned long)resHw2.timerPresetSec24b,
                  statusToStr(resHw3.st0900Timer), (unsigned long)resHw3.timerPresetSec24b);
    Serial.printf(" getSerialNumber (0x1100)             | %-10s (%-22s)  | %-21s (%-11s)\n",
                  statusToStr(resHw2.st1100Serial), resHw2.serialStr,
                  statusToStr(resHw3.st1100Serial), resHw3.serialStr);
    Serial.printf(" requestDayHistory (0x0A00 - 24h)     | %-8s (H23=%4uWh, CS_Eq=%-3s)     | %-8s (H23=%4uWh, CS_Eq=%-3s)\n",
                  statusToStr(resHw2.st0A00), (unsigned)resHw2.h23Wh, resHw2.cs0A00Valid ? "OUI" : "NON",
                  statusToStr(resHw3.st0A00), (unsigned)resHw3.h23Wh, resHw3.cs0A00Valid ? "OUI" : "NON");
    Serial.printf(" requestMonthHistory (0x0B00 - 30j)   | %-8s (J29=%4luWh, CS_Eq=%-3s)     | %-8s (J29=%4luWh, CS_Eq=%-3s)\n",
                  statusToStr(resHw2.st0B00), (unsigned long)resHw2.j29Wh, resHw2.cs0B00Valid ? "OUI" : "NON",
                  statusToStr(resHw3.st0B00), (unsigned long)resHw3.j29Wh, resHw3.cs0B00Valid ? "OUI" : "NON");
    Serial.printf(" requestYearHistory (0x0C00 - 12m)    | %-8s (M11=%5luWh, CS_Eq=%-3s)    | %-8s (M11=%5luWh, CS_Eq=%-3s)\n",
                  statusToStr(resHw2.st0C00), (unsigned long)resHw2.m11Wh, resHw2.cs0C00Valid ? "OUI" : "NON",
                  statusToStr(resHw3.st0C00), (unsigned long)resHw3.m11Wh, resHw3.cs0C00Valid ? "OUI" : "NON");
    Serial.printf(" Schedulers : Créés (1..12) / Lus     | %2u créés OK / %2u lus en Flash        | %2u créés OK / %2u lus en Flash\n",
                  (unsigned)resHw2.schedCreatedOkCount, (unsigned)resHw2.schedCountAfter13,
                  (unsigned)resHw3.schedCreatedOkCount, (unsigned)resHw3.schedCountAfter13);
    Serial.printf(" Scheduler 13e (Attendu Rejeté)       | %-36s | %-36s\n",
                  statusToStr(resHw2.stSched13th), statusToStr(resHw3.stSched13th));
    Serial.printf(" Schedulers : Modif #1 & #6 / Restant | %-8s & %-8s (Total=%2u)       | %-8s & %-8s (Total=%2u)\n",
                  statusToStr(resHw2.stSchedMod1), statusToStr(resHw2.stSchedMod6), (unsigned)resHw2.schedCountAfterMod,
                  statusToStr(resHw3.stSchedMod1), statusToStr(resHw3.stSchedMod6), (unsigned)resHw3.schedCountAfterMod);
    Serial.printf(" Schedulers : Purge finale (Attendu 0)| Restant = %2u                         | Restant = %2u\n",
                  (unsigned)resHw2.schedCountAfterDeleteAll, (unsigned)resHw3.schedCountAfterDeleteAll);
    Serial.printf(" Random Mode : ON (3s) -> OFF final   | %-6s (lu=%-3s/0x%02X) -> %-6s (%-3s) | %-6s (lu=%-3s/0x%02X) -> %-6s (%-3s)\n",
                  statusToStr(resHw2.stRandActivate), resHw2.randReadbackActive ? "ON" : "OFF", (unsigned)resHw2.randReadbackMask,
                  statusToStr(resHw2.stRandDisable),  resHw2.randFinalActive ? "ON" : "OFF",
                  statusToStr(resHw3.stRandActivate), resHw3.randReadbackActive ? "ON" : "OFF", (unsigned)resHw3.randReadbackMask,
                  statusToStr(resHw3.stRandDisable),  resHw3.randFinalActive ? "ON" : "OFF");
    Serial.println("========================================================================================================================");
}

void setup() {
    pinMode(BUTTON_BOOT_PIN, INPUT_PULLUP);
    Serial.begin(115200);

    uint32_t tWait = millis();
    while (!Serial && (millis() - tWait < 3000)) {
        delay(50);
    }
    delay(400);

    WiFi.mode(WIFI_OFF);
    NimBLEDevice::init("ESP32S3-ValV040");
    NimBLEDevice::setMTU(160);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    waitForBootButtonOrSerial("LANCEMENT EXPÉRIENCE 23 : QUALIFICATION v0.4.0 SUR CASCADE VÉLO (HW3) -> THÉOULE (HW2)");
    runValidationV040();
}

void loop() {
    waitForBootButtonOrSerial("RELANCER EXPÉRIENCE 23 (QUALIFICATION v0.4.0)");
    runValidationV040();
}
