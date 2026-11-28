/**
 * @file exp_20_sonde_horloge_et_0100_hw3.cpp
 * @brief Expérience 20 : Auscultation de la RTC interne de Vélo (HW 3.0 / FW 1.30) via la Sonde Fantôme Timer
 *        (0x0800 / 0x0900) et test des variantes de la trame setDateTime (0x0100) avec octets [11..12] = 00 00 vs 01 00.
 *
 * Déroulement :
 * 1. ÉTAPE 1 (Lecture seule, état laissé par l'application officielle Voltcraft) :
 *    - Lecture des historiques 0x0A00 (24h), 0x0B00 (30j), 0x0C00 (12m) pour voir le décalage de 17h00 / 18h00.
 *    - Lecture de 0x0900 initial (avant tout armement).
 *    - Sonde Timer 1A : Cible = 26/03/2026 à 23:00:00 (date réelle de Mars 2026).
 *    - Sonde Timer 1B : Cible = 26/03/2026 à 23:30:00 (+30 min pour vérifier la linéarité de Δt).
 *    - Sonde Timer 1C : Cible = 26/11/2026 à 23:00:00 (date de Novembre 2026).
 * 2. ÉTAPE 2 (Test d'écriture setDateTime 0x0100 contrôlé par la Sonde Fantôme Timer) :
 *    - Essai 2A : Envoi 0x0100 standard ([B11..B12] = 00 00) à 14:00:00 (recul de ~3h45), puis Sonde Timer immédiate !
 *    - Essai 2B : Envoi 0x0100 avec [B11..B12] = 01 00 (hypothèse UTC+1 / flag 01 00) à 13:00:00, puis Sonde Timer immédiate !
 *    - Essai 2C : Envoi 0x0100 avec année sur 1 octet (26 = 0x1A en B9, 0x00 en B10) au cas où HW 3.0 utiliserait le même format
 *                 d'année 1 octet que 0x0800 (Timer) et 0x1300 (Scheduler), puis Sonde Timer immédiate !
 * 3. ÉTAPE 3 (Restauration de l'heure réelle exacte détectée à l'Étape 1 + vérification finale par Sonde Timer).
 */

#include <Arduino.h>
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <cstring>

static const int   BUTTON_BOOT_PIN = 0;
static const char* MAC_VELO_HW3    = "a3:00:00:00:45:25";

static const NimBLEUUID SERVICE_FFF0_UUID("0000fff0-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_REQ_FFF3_UUID("0000fff3-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_RES_FFF4_UUID("0000fff4-0000-1000-8000-00805f9b34fb");

static uint8_t           g_rxBuf[160];
static volatile size_t   g_rxLen      = 0;
static volatile uint16_t g_rxCmd      = 0;
static volatile bool     g_rxComplete = false;

static uint8_t calculateSemChecksum(const uint8_t* buf, size_t startIdx, size_t len) {
    uint32_t sum = 1;
    for (size_t i = startIdx; i < startIdx + len; i++) {
        sum += buf[i];
    }
    return (uint8_t)(sum & 0xFF);
}

static void printHexLine(const uint8_t* data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        Serial.printf("%02X ", data[i]);
    }
}

static void onNotifyFFF4(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t len, bool isNotify) {
    if (len == 0) return;

    if (pData[0] == 0x0F) {
        g_rxLen      = 0;
        g_rxComplete = false;
    }

    if (g_rxLen + len <= sizeof(g_rxBuf)) {
        memcpy(g_rxBuf + g_rxLen, pData, len);
        g_rxLen = g_rxLen + len;
    }

    if (g_rxLen >= 4) {
        uint16_t cmd = ((uint16_t)g_rxBuf[2] << 8) | g_rxBuf[3];
        if (cmd == 0x0400 && g_rxLen >= 19) {
            g_rxCmd      = cmd;
            g_rxComplete = true;
            return;
        }
        size_t expectedLen = (size_t)g_rxBuf[1] + 4;
        if (g_rxLen >= expectedLen && expectedLen >= 5) {
            if (g_rxBuf[expectedLen - 2] == 0xFF && g_rxBuf[expectedLen - 1] == 0xFF) {
                g_rxLen      = expectedLen;
                g_rxCmd      = cmd;
                g_rxComplete = true;
            }
        }
    }
}

static bool sendCmdWait(NimBLERemoteCharacteristic* pReq,
                        const uint8_t* frame,
                        size_t frameLen,
                        uint16_t expectedCmd,
                        uint32_t timeoutMs = 1200) {
    g_rxComplete = false;
    g_rxLen      = 0;
    g_rxCmd      = 0;

    if (!pReq->writeValue(frame, frameLen, false)) {
        return false;
    }

    uint32_t t0 = millis();
    while (millis() - t0 < timeoutMs) {
        if (g_rxComplete && g_rxCmd == expectedCmd) {
            return true;
        }
        delay(5);
    }
    return false;
}

static void dumpHistoriesSummary(NimBLERemoteCharacteristic* pReq, const char* title) {
    Serial.println("\n  ----------------------------------------------------------------------------------------------------------------");
    Serial.printf("  📊 ÉTAT DES HISTORIQUES : %s\n", title);
    Serial.println("  ----------------------------------------------------------------------------------------------------------------");

    // 1. 0x0A00 (24h)
    static const uint8_t cmd0A00[9] = { 0x0F, 0x05, 0x0A, 0x00, 0x00, 0x00, 0x0B, 0xFF, 0xFF };
    if (sendCmdWait(pReq, cmd0A00, sizeof(cmd0A00), 0x0A00, 1200) && g_rxLen >= 55) {
        uint8_t csExp = calculateSemChecksum(g_rxBuf, 2, g_rxLen - 5);
        uint8_t csGot = g_rxBuf[g_rxLen - 3];
        uint8_t diff  = (uint8_t)((csExp - csGot) & 0xFF);
        uint32_t sumWh = 0;
        char slotsBuf[192] = "";
        size_t pos = 0;
        for (int i = 0; i < 24; i++) {
            size_t idx = 4 + (i * 2);
            uint16_t wh = ((uint16_t)g_rxBuf[idx] << 8) | g_rxBuf[idx + 1];
            sumWh += wh;
            if (wh > 0 && pos + 16 < sizeof(slotsBuf)) {
                pos += snprintf(slotsBuf + pos, sizeof(slotsBuf) - pos, "[%02d]=%u ", i, (unsigned)wh);
            }
        }
        Serial.printf("  🕒 0x0A00 (24h) | Cumul=%4lu Wh | CS_Exp=0x%02X | CS_Got=0x%02X | Écart(Exp-Got)=0x%02X (%3u) | Slots>0: %s\n",
                      (unsigned long)sumWh, (unsigned)csExp, (unsigned)csGot, (unsigned)diff, (unsigned)diff, slotsBuf);
    }

    // 2. 0x0B00 (30j)
    static const uint8_t cmd0B00[9] = { 0x0F, 0x05, 0x0B, 0x00, 0x00, 0x00, 0x0C, 0xFF, 0xFF };
    if (sendCmdWait(pReq, cmd0B00, sizeof(cmd0B00), 0x0B00, 1200) && g_rxLen >= 127) {
        uint8_t csExp = calculateSemChecksum(g_rxBuf, 2, g_rxLen - 5);
        uint8_t csGot = g_rxBuf[g_rxLen - 3];
        uint8_t diff  = (uint8_t)((csExp - csGot) & 0xFF);
        uint32_t sumWh = 0;
        char slotsBuf[192] = "";
        size_t pos = 0;
        for (int i = 0; i < 30; i++) {
            size_t idx = 4 + (i * 4);
            uint32_t wh = ((uint32_t)g_rxBuf[idx] << 16) | ((uint32_t)g_rxBuf[idx + 1] << 8) | g_rxBuf[idx + 2];
            sumWh += wh;
            if (wh > 0 && pos + 18 < sizeof(slotsBuf)) {
                pos += snprintf(slotsBuf + pos, sizeof(slotsBuf) - pos, "[%02d]=%lu ", i, (unsigned long)wh);
            }
        }
        Serial.printf("  📅 0x0B00 (30j) | Cumul=%4lu Wh | CS_Exp=0x%02X | CS_Got=0x%02X | Écart(Exp-Got)=0x%02X (%3u) | Slots>0: %s\n",
                      (unsigned long)sumWh, (unsigned)csExp, (unsigned)csGot, (unsigned)diff, (unsigned)diff, slotsBuf);
    }

    // 3. 0x0C00 (12m) - Buffer large (256 chars) pour afficher TOUS les slots y compris [11] !
    static const uint8_t cmd0C00[9] = { 0x0F, 0x05, 0x0C, 0x00, 0x00, 0x00, 0x0D, 0xFF, 0xFF };
    if (sendCmdWait(pReq, cmd0C00, sizeof(cmd0C00), 0x0C00, 1200) && g_rxLen >= 55) {
        uint8_t csExp = calculateSemChecksum(g_rxBuf, 2, g_rxLen - 5);
        uint8_t csGot = g_rxBuf[g_rxLen - 3];
        uint8_t diff  = (uint8_t)((csExp - csGot) & 0xFF);
        uint32_t sumWh = 0;
        char slotsBuf[256] = "";
        size_t pos = 0;
        for (int i = 0; i < 12; i++) {
            size_t idx = 4 + (i * 4);
            uint32_t wh = ((uint32_t)g_rxBuf[idx] << 16) | ((uint32_t)g_rxBuf[idx + 1] << 8) | g_rxBuf[idx + 2];
            uint8_t  b3 = g_rxBuf[idx + 3];
            sumWh += wh;
            if (wh > 0 && pos + 22 < sizeof(slotsBuf)) {
                pos += snprintf(slotsBuf + pos, sizeof(slotsBuf) - pos, "[%02d]=%lu(%u%%) ", i, (unsigned long)wh, (unsigned)b3);
            }
        }
        Serial.printf("  📆 0x0C00 (12m) | Cumul=%4lu Wh | CS_Exp=0x%02X | CS_Got=0x%02X | Écart(Exp-Got)=0x%02X (%3u) | Slots>0: %s\n",
                      (unsigned long)sumWh, (unsigned)csExp, (unsigned)csGot, (unsigned)diff, (unsigned)diff, slotsBuf);
    }
}

// ============================================================================
// SONDE FANTÔME TIMER (0x0800 -> 0x0900 -> stopTimer)
// ============================================================================
struct TimerProbeResult {
    bool     okSet;
    bool     okGet;
    uint8_t  raw0900[24];
    size_t   rawLen;
    uint8_t  action;
    uint8_t  tSec, tMin, tHour, tDay, tMonth, tYear;
    uint8_t  pHour, pMin, pSec;
    uint32_t deltaSeconds;
    int32_t  deducedClockSecOfDay; // Heure interne déduite (en secondes depuis 00:00:00)
};

static void runFurtiveTimerProbe(NimBLERemoteCharacteristic* pReq,
                                 const char* probeTitle,
                                 bool relayCurrentlyOn,
                                 uint8_t targetSec,
                                 uint8_t targetMin,
                                 uint8_t targetHour,
                                 uint8_t targetDay,
                                 uint8_t targetMonth,
                                 uint16_t targetYear,
                                 TimerProbeResult& res) {
    memset(&res, 0, sizeof(res));

    // 1. Armement du Timer Absolu (0x0800) avec action identique à l'état actuel du relais
    uint8_t cmdArm[16] = {
        0x0F, 0x0C, 0x08, 0x00,
        (uint8_t)(relayCurrentlyOn ? 0x01 : 0x02),
        targetSec, targetMin, targetHour,
        targetDay, targetMonth, (uint8_t)(targetYear % 100),
        0x00, 0x00, 0x00, 0xFF, 0xFF
    };
    cmdArm[13] = calculateSemChecksum(cmdArm, 2, 11);

    Serial.printf("\n  🕵️‍♂️ [%s] -> Cible envoyée : %02u/%02u/%04u à %02u:%02u:%02u\n",
                  probeTitle,
                  (unsigned)targetDay, (unsigned)targetMonth, (unsigned)targetYear,
                  (unsigned)targetHour, (unsigned)targetMin, (unsigned)targetSec);
    Serial.print("     TX 0x0800 (Arm)  : ");
    printHexLine(cmdArm, sizeof(cmdArm));
    Serial.println();

    res.okSet = sendCmdWait(pReq, cmdArm, sizeof(cmdArm), 0x0800, 1000);
    Serial.print("     RX 0x0800 (Ack)  : ");
    printHexLine(g_rxBuf, g_rxLen);
    Serial.println();

    // 2. Lecture de getTimerStatus (0x0900) AVANT stopTimer !
    static const uint8_t cmdGet0900[9] = { 0x0F, 0x05, 0x09, 0x00, 0x00, 0x00, 0x0A, 0xFF, 0xFF };
    res.okGet = sendCmdWait(pReq, cmdGet0900, sizeof(cmdGet0900), 0x0900, 1000);
    if (res.okGet && g_rxLen >= 18) {
        res.rawLen = (g_rxLen <= sizeof(res.raw0900)) ? g_rxLen : sizeof(res.raw0900);
        memcpy(res.raw0900, g_rxBuf, res.rawLen);

        res.action = res.raw0900[4];
        res.tSec   = res.raw0900[5];
        res.tMin   = res.raw0900[6];
        res.tHour  = res.raw0900[7];
        res.tDay   = res.raw0900[8];
        res.tMonth = res.raw0900[9];
        res.tYear  = res.raw0900[10];
        res.pHour  = res.raw0900[11];
        res.pMin   = res.raw0900[12];
        res.pSec   = res.raw0900[13];
        res.deltaSeconds = (uint32_t)res.pHour * 3600UL + (uint32_t)res.pMin * 60UL + (uint32_t)res.pSec;

        int32_t targetSecOfDay = (int32_t)targetHour * 3600L + (int32_t)targetMin * 60L + (int32_t)targetSec;
        res.deducedClockSecOfDay = targetSecOfDay - (int32_t)(res.deltaSeconds % 86400UL);
        if (res.deducedClockSecOfDay < 0) res.deducedClockSecOfDay += 86400L;
    }

    Serial.print("     RX 0x0900 (Brut) : ");
    printHexLine(res.raw0900, res.rawLen);
    Serial.println();

    uint8_t dH = (uint8_t)(res.deducedClockSecOfDay / 3600L);
    uint8_t dM = (uint8_t)((res.deducedClockSecOfDay % 3600L) / 60L);
    uint8_t dS = (uint8_t)(res.deducedClockSecOfDay % 60L);

    Serial.printf("     -> Décodage 0x0900 : Action=0x%02X | Écho Cible=%02u/%02u/20%02u %02u:%02u:%02u | Δt retourné=[%02uh %02um %02us] (%lu s)\n",
                  (unsigned)res.action,
                  (unsigned)res.tDay, (unsigned)res.tMonth, (unsigned)res.tYear,
                  (unsigned)res.tHour, (unsigned)res.tMin, (unsigned)res.tSec,
                  (unsigned)res.pHour, (unsigned)res.pMin, (unsigned)res.pSec,
                  (unsigned long)res.deltaSeconds);
    Serial.printf("     ⏰ HEURE INTERNE DÉDUITE (Cible - Δt) : %02u:%02u:%02u (Si Δt > 24h : %lu jour(s) + %02uh %02um %02us)\n",
                  (unsigned)dH, (unsigned)dM, (unsigned)dS,
                  (unsigned long)(res.deltaSeconds / 86400UL),
                  (unsigned)((res.deltaSeconds % 86400UL) / 3600UL),
                  (unsigned)(((res.deltaSeconds % 86400UL) % 3600UL) / 60UL),
                  (unsigned)(res.deltaSeconds % 60UL));

    // 3. Désarmement immédiat (stopTimer)
    static const uint8_t cmdStop[16] = {
        0x0F, 0x0C, 0x08, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x09, 0xFF, 0xFF
    };
    sendCmdWait(pReq, cmdStop, sizeof(cmdStop), 0x0800, 1000);
}

// Envoi d'une trame 0x0100 personnalisée (pour tester B9..B12)
static bool sendCustom0100(NimBLERemoteCharacteristic* pReq,
                           const char* variantName,
                           uint8_t sec, uint8_t min, uint8_t hour,
                           uint8_t day, uint8_t month,
                           uint8_t b9, uint8_t b10, uint8_t b11, uint8_t b12) {
    uint8_t frame[16] = {
        0x0F, 0x0C, 0x01, 0x00,
        sec, min, hour, day, month,
        b9, b10, b11, b12,
        0x00, 0xFF, 0xFF
    };
    frame[13] = calculateSemChecksum(frame, 2, 11);

    Serial.printf("\n  🛠️ [%s]\n", variantName);
    Serial.print("     TX 0x0100 : ");
    printHexLine(frame, sizeof(frame));
    Serial.println();

    bool ok = sendCmdWait(pReq, frame, sizeof(frame), 0x0100, 1000);
    Serial.print("     RX 0x0100 : ");
    printHexLine(g_rxBuf, g_rxLen);
    Serial.printf(" (ok=%s)\n", ok ? "OUI" : "NON");
    return ok;
}

static void runExp20() {
    Serial.println("\n########################################################################################################################");
    Serial.printf(" 🕰️ EXPÉRIENCE 20 : SONDE FANTÔME TIMER (0x0800/0x0900) & ANALYSE DE 0x0100 SUR VÉLO HW 3.0 (%s)\n", MAC_VELO_HW3);
    Serial.println("########################################################################################################################");

    NimBLEDevice::setMTU(160);
    NimBLEClient* pClient = NimBLEDevice::createClient();
    pClient->setConnectTimeout(6000);
    NimBLEAddress addr(MAC_VELO_HW3, BLE_ADDR_PUBLIC);

    if (!pClient->connect(addr)) {
        Serial.println(" ❌ Échec connexion à Vélo (HW 3.0) !");
        NimBLEDevice::deleteClient(pClient);
        return;
    }

    NimBLERemoteService* pSvc = pClient->getService(SERVICE_FFF0_UUID);
    if (!pSvc) {
        pClient->disconnect();
        NimBLEDevice::deleteClient(pClient);
        return;
    }

    NimBLERemoteCharacteristic* pReq = pSvc->getCharacteristic(CHAR_REQ_FFF3_UUID);
    NimBLERemoteCharacteristic* pRes = pSvc->getCharacteristic(CHAR_RES_FFF4_UUID);
    if (!pReq || !pRes || !pRes->subscribe(true, onNotifyFFF4)) {
        pClient->disconnect();
        NimBLEDevice::deleteClient(pClient);
        return;
    }

    // Authentification 32 bits
    static const uint8_t cmdAuth[16] = {
        0x0F, 0x0C, 0x17, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x18, 0xFF, 0xFF
    };
    if (!sendCmdWait(pReq, cmdAuth, sizeof(cmdAuth), 0x1700, 1200)) {
        Serial.println(" ❌ Échec authentification 0x1700 !");
        pClient->disconnect();
        NimBLEDevice::deleteClient(pClient);
        return;
    }

    // Lecture Télémétrie 0x0400 initiale
    bool relayOn = true;
    static const uint8_t cmd0400[9] = { 0x0F, 0x05, 0x04, 0x00, 0x00, 0x00, 0x05, 0xFF, 0xFF };
    if (sendCmdWait(pReq, cmd0400, sizeof(cmd0400), 0x0400, 1000) && g_rxLen >= 19) {
        relayOn = (g_rxBuf[4] == 0x01);
        Serial.print("  📡 Télémétrie 0x0400 brute : ");
        printHexLine(g_rxBuf, g_rxLen);
        Serial.printf(" (Octets[12..13] = 0x%02X 0x%02X)\n", (unsigned)g_rxBuf[12], (unsigned)g_rxBuf[13]);
    }

    // =========================================================================
    // ÉTAPE 1 : LECTURE SEULE (ÉTAT LAISSÉ PAR L'APPLICATION VOLTCRAFT)
    // =========================================================================
    Serial.println("\n================================================================================================================");
    Serial.println(" 🔍 ÉTAPE 1 : AUSCULTATION EN LECTURE SEULE (APRÈS SYNCHRO PAR L'APPLICATION VOLTCRAFT)");
    Serial.println("================================================================================================================");

    dumpHistoriesSummary(pReq, "ÉTAT INITIAL (Avant tout setDateTime)");

    // Lecture 0x0900 à froid (pour voir ce qui réside dans le buffer Timer de la prise)
    static const uint8_t cmdGet0900[9] = { 0x0F, 0x05, 0x09, 0x00, 0x00, 0x00, 0x0A, 0xFF, 0xFF };
    if (sendCmdWait(pReq, cmdGet0900, sizeof(cmdGet0900), 0x0900, 1000)) {
        Serial.print("\n  🧊 Lecture 0x0900 à froid (sans armement) : ");
        printHexLine(g_rxBuf, g_rxLen);
        Serial.println();
    }

    TimerProbeResult p1A, p1B, p1C;
    // Test 1A : Cible 26/11/2026 à 23:00:00
    runFurtiveTimerProbe(pReq, "SONDE 1A (Cible 26/11/2026 23:00:00)", relayOn, 0, 0, 23, 26, 11, 2026, p1A);
    // Test 1B : Cible 26/11/2026 à 23:30:00 (+30 min pour vérifier si Δt augmente de +30 min = +1800 s)
    runFurtiveTimerProbe(pReq, "SONDE 1B (Cible 26/11/2026 23:30:00 -> +30 min)", relayOn, 0, 30, 23, 26, 11, 2026, p1B);
    // Test 1C : Cible 26/03/2026 à 23:00:00 (Mois = Mars = 03)
    runFurtiveTimerProbe(pReq, "SONDE 1C (Cible 26/03/2026 23:00:00 -> Mois 03)", relayOn, 0, 0, 23, 26, 3, 2026, p1C);

    // Déterminer quel mois (11 ou 03) a donné un Δt valide (< 24h = 86400s et > 0)
    uint8_t activeMonth = 11;
    uint8_t activeDay   = 26;
    TimerProbeResult refProbe = p1A;
    if (p1C.deltaSeconds > 0 && p1C.deltaSeconds < 86400UL && (p1A.deltaSeconds == 0 || p1A.deltaSeconds >= 86400UL)) {
        activeMonth = 3;
        refProbe    = p1C;
    }
    uint8_t initialH = (uint8_t)(refProbe.deducedClockSecOfDay / 3600L);
    uint8_t initialM = (uint8_t)((refProbe.deducedClockSecOfDay % 3600L) / 60L);
    uint8_t initialS = (uint8_t)(refProbe.deducedClockSecOfDay % 60L);

    Serial.printf("\n  📌 BILAN ÉTAPE 1 : Mois actif retenu = %02u/2026 | Heure interne mesurée = %02u:%02u:%02u | Diff(1B - 1A) = %ld s (Attendu: +1800 s)\n",
                  (unsigned)activeMonth, (unsigned)initialH, (unsigned)initialM, (unsigned)initialS,
                  (long)p1B.deltaSeconds - (long)p1A.deltaSeconds);

    // =========================================================================
    // ÉTAPE 2 : TEST DES VARIANTES DE SETDATETIME (0x0100) CONTRÔLÉ PAR SONDE TIMER
    // =========================================================================
    Serial.println("\n================================================================================================================");
    Serial.println(" 🧪 ÉTAPE 2 : TEST DES 3 VARIANTES DE SETDATETIME (0x0100) VÉRIFIÉES PAR LA SONDE TIMER");
    Serial.println("================================================================================================================");

    // Variante 2A : Format standard HW 2.0 -> Année 2026 sur 2 octets (0x07, 0xEA), B11..B12 = 0x00, 0x00
    // On tente de régler l'heure à 14:00:00 (pour une cible à 23:00:00 -> Δt attendu = 09h 00m 00s = 32400 s)
    sendCustom0100(pReq, "VARIANTE 2A : Format HW 2.0 standard (14:00:00, Year=07 EA, B11..12=00 00)",
                   0, 0, 14, activeDay, activeMonth, 0x07, 0xEA, 0x00, 0x00);
    TimerProbeResult p2A;
    runFurtiveTimerProbe(pReq, "VÉRIFICATION SONDE 2A (Attendu si 14:00:00 pris en compte : Δt = 09h 00m 00s)",
                         relayOn, 0, 0, 23, activeDay, activeMonth, 2026, p2A);

    // Variante 2B : Hypothèse Utilisateur -> B11..B12 = 0x01, 0x00 (comme dans 0x0400[12..13] !)
    // On tente de régler l'heure à 13:00:00 (pour une cible à 23:00:00 -> Δt attendu = 10h 00m 00s = 36000 s)
    sendCustom0100(pReq, "VARIANTE 2B : Avec [B11..B12] = 01 00 (13:00:00, Year=07 EA, B11..12=01 00)",
                   0, 0, 13, activeDay, activeMonth, 0x07, 0xEA, 0x01, 0x00);
    TimerProbeResult p2B;
    runFurtiveTimerProbe(pReq, "VÉRIFICATION SONDE 2B (Attendu si 13:00:00 pris en compte : Δt = 10h 00m 00s)",
                         relayOn, 0, 0, 23, activeDay, activeMonth, 2026, p2B);

    // Variante 2C : Année sur 1 octet (26 = 0x1A en B9, 0x00 en B10) comme dans 0x0800 (Timer) et 0x1300 (Scheduler)
    // On tente de régler l'heure à 12:00:00 (pour une cible à 23:00:00 -> Δt attendu = 11h 00m 00s = 39600 s)
    sendCustom0100(pReq, "VARIANTE 2C : Année 1 octet 0x1A (12:00:00, B9=1A, B10=00, B11..12=00 00)",
                   0, 0, 12, activeDay, activeMonth, 0x1A, 0x00, 0x00, 0x00);
    TimerProbeResult p2C;
    runFurtiveTimerProbe(pReq, "VÉRIFICATION SONDE 2C (Attendu si 12:00:00 pris en compte : Δt = 11h 00m 00s)",
                         relayOn, 0, 0, 23, activeDay, activeMonth, 2026, p2C);

    // =========================================================================
    // ÉTAPE 3 : RESTAURATION DE L'HEURE INITIALE & BILAN
    // =========================================================================
    Serial.println("\n================================================================================================================");
    Serial.println(" 🔄 ÉTAPE 3 : RESTAURATION DE L'HEURE INITIALE & VÉRIFICATION FINALE");
    Serial.println("================================================================================================================");
    sendCustom0100(pReq, "RESTAURATION HEURE INITIALE (Format 07 EA / 00 00)",
                   initialS, initialM, initialH, activeDay, activeMonth, 0x07, 0xEA, 0x00, 0x00);
    TimerProbeResult pRest;
    runFurtiveTimerProbe(pReq, "SONDE FINALE APRÈS RESTAURATION",
                         relayOn, 0, 0, 23, activeDay, activeMonth, 2026, pRest);

    dumpHistoriesSummary(pReq, "ÉTAT FINAL DES HISTORIQUES");

    pClient->disconnect();
    NimBLEDevice::deleteClient(pClient);
    Serial.println("\n[BLE] ✅ Expérience 20 terminée et déconnectée.");
    Serial.println("########################################################################################################################\n");
}

static void waitUserPrompt(const char* title, const char* instructions) {
    Serial.println("\n========================================================================================================================");
    Serial.printf(" >>> %s\n", title);
    Serial.println("------------------------------------------------------------------------------------------------------------------------");
    Serial.println(instructions);
    Serial.println("\n  👉 [Appuyez sur le bouton physique BOOT (GPIO 0) ou envoyez 'Entrée' en console]");
    Serial.println("========================================================================================================================\n");

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
        if (millis() - lastBeacon >= 4000) {
            Serial.printf("  [EN ATTENTE - %s] Appuyez sur BOOT ou 'Entrée'...\n", title);
            lastBeacon = millis();
        }
        delay(50);
    }
    Serial.println(">>> Action validée ! Démarrage...\n");
}

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_BOOT_PIN, INPUT_PULLUP);

    unsigned long startWait = millis();
    while (!Serial && (millis() - startWait < 3000)) {
        delay(50);
    }
    delay(500);

    WiFi.mode(WIFI_OFF);
    NimBLEDevice::init("Gateway_Exp20");

    Serial.println("\n\n========================================================================================================================");
    Serial.println("  🚀 EXPÉRIENCE 20 : SONDE FANTÔME TIMER (0x0800/0x0900) & ANALYSE DE 0x0100 SUR VÉLO (HW 3.0)");
    Serial.println("========================================================================================================================");

    waitUserPrompt("LANCEMENT EXPÉRIENCE 20 (SONDE HORLOGE & TEST 0x0100 SUR VÉLO HW 3.0)",
                   "Appuyez sur BOOT ou 'Entrée' pour interroger l'horloge interne de Vélo via la Sonde Timer\n"
                   "et tester les 3 variantes de setDateTime (0x0100) avec restauration automatique.");

    runExp20();

    Serial.println("\n========================================================================================================================");
    Serial.println("  🎉 EXPÉRIENCE 20 TERMINÉE ! (Appuyez sur BOOT ou 'Entrée' pour relancer)");
    Serial.println("========================================================================================================================\n");
}

void loop() {
    if (digitalRead(BUTTON_BOOT_PIN) == LOW) {
        delay(150);
        while (digitalRead(BUTTON_BOOT_PIN) == LOW) delay(20);
        runExp20();
    }
    if (Serial.available()) {
        while (Serial.available()) Serial.read();
        runExp20();
    }
    delay(50);
}
