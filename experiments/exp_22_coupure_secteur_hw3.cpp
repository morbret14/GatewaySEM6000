/**
 * @file exp_22_coupure_secteur_hw3.cpp
 * @brief Expérience 22 (v2) : Test de Micro-Coupure / Coupure Secteur sur Vélo (HW 3.0 / FW 1.30)
 *        avec vérification de :
 *        1. Persistance d'un Timer actif (0x0800), des Plannings (0x1400), du Mode Aléatoire (0x1600) et des autres caractéristiques GATT,
 *        2. Point a : Remise à l'heure après le constat à froid + attente de 30 secondes sous charge (suivi toutes les 5s),
 *        3. Point b : Cliché #3 et Bilan Comparatif à 3 colonnes (AVANT Coupure / APRÈS Coupure à froid / APRÈS Remise à l'heure + 30s)
 *           pour vérifier si H[23], J[29], M[11] étaient simplement masqués tant que l'horloge n'était pas remise à l'heure !
 */

#include <Arduino.h>
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <cstring>

static const int   BUTTON_BOOT_PIN = 0;
static const char* MAC_VELO_HW3    = "a3:00:00:00:45:25";

static const NimBLEUUID SERVICE_GAP_UUID("00001800-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_GAP_NAME_UUID("00002a00-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID Service_FFF0_UUID("0000fff0-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_FFF1_UUID("0000fff1-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_REQ_FFF3_UUID("0000fff3-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_RES_FFF4_UUID("0000fff4-0000-1000-8000-00805f9b34fb");

static uint8_t           g_rxBuf[160];
static volatile size_t   g_rxRawLen   = 0;
static volatile size_t   g_rxLen      = 0;
static volatile uint16_t g_rxCmd      = 0;
static volatile bool     g_rxComplete = false;

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

static uint8_t calculateSemChecksum(const uint8_t* buf, size_t startIdx, size_t len) {
    uint32_t sum = 1;
    for (size_t i = startIdx; i < startIdx + len; i++) {
        sum += buf[i];
    }
    return (uint8_t)(sum & 0xFF);
}

static void printHexBuffer(const char* prefix, const uint8_t* buf, size_t len) {
    Serial.printf("%s (%3u octets) : ", prefix, (unsigned)len);
    for (size_t i = 0; i < len; i++) {
        Serial.printf("%02X ", buf[i]);
    }
    Serial.println();
}

static void onNotifyFFF4(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t len, bool isNotify) {
    if (len == 0) return;

    if (pData[0] == 0x0F) {
        g_rxRawLen   = 0;
        g_rxLen      = 0;
        g_rxComplete = false;
    }

    if (g_rxRawLen + len <= sizeof(g_rxBuf)) {
        memcpy(g_rxBuf + g_rxRawLen, pData, len);
        g_rxRawLen = g_rxRawLen + len;
        g_rxLen    = g_rxRawLen;
    }

    if (g_rxRawLen >= 4) {
        uint16_t cmd = ((uint16_t)g_rxBuf[2] << 8) | g_rxBuf[3];
        if (cmd == 0x0400 && g_rxRawLen >= 19) {
            g_rxLen      = 19;
            g_rxCmd      = cmd;
            g_rxComplete = true;
            return;
        }
        size_t expectedLen = (size_t)g_rxBuf[1] + 4;
        if (g_rxRawLen >= expectedLen && expectedLen >= 5) {
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
                        uint32_t timeoutMs = 1500,
                        bool logRaw = true) {
    g_rxComplete = false;
    g_rxRawLen   = 0;
    g_rxLen      = 0;
    g_rxCmd      = 0;

    if (logRaw) {
        char txLabel[32];
        snprintf(txLabel, sizeof(txLabel), "     📤 TX 0x%04X", (unsigned)expectedCmd);
        printHexBuffer(txLabel, frame, frameLen);
    }

    if (!pReq->writeValue(frame, frameLen, false)) {
        Serial.printf("     ❌ Échec writeValue(0x%04X)\n", (unsigned)expectedCmd);
        return false;
    }

    uint32_t t0 = millis();
    while (millis() - t0 < timeoutMs) {
        if (g_rxComplete && g_rxCmd == expectedCmd) {
            if (logRaw) {
                char rxLabel[32];
                snprintf(rxLabel, sizeof(rxLabel), "     📥 RX 0x%04X", (unsigned)expectedCmd);
                printHexBuffer(rxLabel, g_rxBuf, g_rxRawLen);
            }
            return true;
        }
        delay(4);
    }
    Serial.printf("     ⏱️ TIMEOUT sur 0x%04X (reçu=%u octets)\n", (unsigned)expectedCmd, (unsigned)g_rxRawLen);
    return false;
}

static bool sendSetDateTime(NimBLERemoteCharacteristic* pReq,
                            uint8_t sec, uint8_t min, uint8_t hour,
                            uint8_t day, uint8_t month, uint16_t year) {
    uint8_t frame[16] = {
        0x0F, 0x0C, 0x01, 0x00,
        sec, min, hour, day, month,
        (uint8_t)(year >> 8), (uint8_t)(year & 0xFF),
        0x00, 0x00, 0x00, 0xFF, 0xFF
    };
    frame[13] = calculateSemChecksum(frame, 2, 11);
    return sendCmdWait(pReq, frame, sizeof(frame), 0x0100, 1000, true);
}

static uint8_t daysInMonth(uint8_t month, uint16_t year) {
    static const uint8_t kDays[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    if (month < 1 || month > 12) return 30;
    if (month == 2 && ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))) return 29;
    return kDays[month - 1];
}

static int32_t dateToDaysSince2000(uint8_t day, uint8_t month, uint16_t year) {
    if (year < 2000) year = 2000;
    int32_t totalDays = 0;
    for (uint16_t y = 2000; y < year; y++) {
        bool leap = ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0));
        totalDays += leap ? 366 : 365;
    }
    for (uint8_t m = 1; m < month; m++) {
        totalDays += daysInMonth(m, year);
    }
    totalDays += (int32_t)day - 1;
    return totalDays;
}

static void daysSince2000ToDate(int32_t daysSince2000, uint8_t& outDay, uint8_t& outMonth, uint16_t& outYear) {
    if (daysSince2000 < 0) daysSince2000 = 0;
    uint16_t y = 2000;
    while (true) {
        bool leap = ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0));
        int32_t dy = leap ? 366 : 365;
        if (daysSince2000 < dy) break;
        daysSince2000 -= dy;
        y++;
    }
    uint8_t m = 1;
    while (m <= 12) {
        uint8_t dm = daysInMonth(m, y);
        if (daysSince2000 < dm) break;
        daysSince2000 -= dm;
        m++;
    }
    outYear  = y;
    outMonth = m;
    outDay   = (uint8_t)(daysSince2000 + 1);
}

struct RtcProbeResult {
    bool     valid;
    bool     isReset2020;
    uint32_t rawDeltaSec24;
    uint8_t  rtcDay;
    uint8_t  rtcMonth;
    uint16_t rtcYear;
    uint8_t  rtcHour;
    uint8_t  rtcMin;
    uint8_t  rtcSec;
    char     summary[96];
};

static RtcProbeResult probeRtcWithTarget(NimBLERemoteCharacteristic* pReq,
                                         const char* label,
                                         uint8_t targetDay, uint8_t targetMonth, uint16_t targetYear,
                                         bool logRaw = true) {
    RtcProbeResult res;
    memset(&res, 0, sizeof(res));

    Serial.printf("\n  🕵️‍♂️ [%s] Cible Timer = %02u/%02u/%04u à 23:59:59\n",
                  label, (unsigned)targetDay, (unsigned)targetMonth, (unsigned)targetYear);

    uint8_t cmdArm[16] = {
        0x0F, 0x0C, 0x08, 0x00,
        0x01, // Relais ON
        59, 59, 23,
        targetDay, targetMonth, (uint8_t)(targetYear % 100),
        0x00, 0x00, 0x00, 0xFF, 0xFF
    };
    cmdArm[13] = calculateSemChecksum(cmdArm, 2, 11);
    sendCmdWait(pReq, cmdArm, sizeof(cmdArm), 0x0800, 1000, logRaw);

    static const uint8_t cmdGet0900[9] = { 0x0F, 0x05, 0x09, 0x00, 0x00, 0x00, 0x0A, 0xFF, 0xFF };
    if (sendCmdWait(pReq, cmdGet0900, sizeof(cmdGet0900), 0x0900, 1000, logRaw) && g_rxLen >= 18) {
        res.rawDeltaSec24 = ((uint32_t)g_rxBuf[11] << 16) | ((uint32_t)g_rxBuf[12] << 8) | (uint32_t)g_rxBuf[13];
    }

    static const uint8_t cmdStop[16] = {
        0x0F, 0x0C, 0x08, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x09, 0xFF, 0xFF
    };
    sendCmdWait(pReq, cmdStop, sizeof(cmdStop), 0x0800, 1000, logRaw);

    // Cas particulier découvert dans exp_22 : si cible=31/12/2026 et RTC=01/01/2020 00:00:xx (2557j = 0x0D2B0B7F -> 24b = 0x2B0B7F)
    if (targetDay == 31 && targetMonth == 12 && targetYear == 2026 &&
        res.rawDeltaSec24 <= 0x2B0B7FUL && res.rawDeltaSec24 > 0x2B0B7FUL - 3600UL) {
        uint32_t elapsedSince2020 = 0x2B0B7FUL - res.rawDeltaSec24;
        res.valid       = true;
        res.isReset2020 = true;
        res.rtcDay      = 1;
        res.rtcMonth    = 1;
        res.rtcYear     = 2020;
        res.rtcHour     = (uint8_t)(elapsedSince2020 / 3600UL);
        res.rtcMin      = (uint8_t)((elapsedSince2020 % 3600UL) / 60UL);
        res.rtcSec      = (uint8_t)(elapsedSince2020 % 60UL);
        snprintf(res.summary, sizeof(res.summary),
                 "01/01/2020 %02u:%02u:%02u [RESET USINE !] (Δt24b=0x%06lX)",
                 (unsigned)res.rtcHour, (unsigned)res.rtcMin, (unsigned)res.rtcSec,
                 (unsigned long)res.rawDeltaSec24);
    } else if (res.rawDeltaSec24 > 0 && res.rawDeltaSec24 < 0x800000UL) {
        int32_t deltaDays = (int32_t)(res.rawDeltaSec24 / 86400UL);
        int32_t remSec    = (int32_t)(res.rawDeltaSec24 % 86400UL);
        int32_t secOfDay  = 86399L - remSec;

        int32_t targetDays2000 = dateToDaysSince2000(targetDay, targetMonth, targetYear);
        int32_t rtcDays2000    = targetDays2000 - deltaDays;
        daysSince2000ToDate(rtcDays2000, res.rtcDay, res.rtcMonth, res.rtcYear);

        res.rtcHour = (uint8_t)(secOfDay / 3600L);
        res.rtcMin  = (uint8_t)((secOfDay % 3600L) / 60L);
        res.rtcSec  = (uint8_t)(secOfDay % 60L);
        res.valid   = true;

        snprintf(res.summary, sizeof(res.summary),
                 "%02u/%02u/%04u %02u:%02u:%02u (Δt=0x%06lX = %lu s)",
                 (unsigned)res.rtcDay, (unsigned)res.rtcMonth, (unsigned)res.rtcYear,
                 (unsigned)res.rtcHour, (unsigned)res.rtcMin, (unsigned)res.rtcSec,
                 (unsigned long)res.rawDeltaSec24, (unsigned long)res.rawDeltaSec24);
    } else {
        res.valid = false;
        snprintf(res.summary, sizeof(res.summary),
                 "Hors fenêtre (Δt24b=0x%06lX = %lu s)",
                 (unsigned long)res.rawDeltaSec24, (unsigned long)res.rawDeltaSec24);
    }

    Serial.printf("     ⏰ RTC DÉDUITE : %s\n", res.summary);
    return res;
}

struct CompleteSnapshot {
    char     title[64];
    char     fff1BeforeAuth[48];
    char     fff1AfterAuth[48];

    // 0x0400
    bool     relayOn;
    float    powerW;
    uint8_t  voltageV;
    float    currentA;
    uint8_t  freqHz;
    uint16_t word12_13;
    uint32_t totalWh0400;
    uint8_t  cs04_exp, cs04_got;

    // 0x1000
    uint8_t  raw1000[20];
    size_t   raw1000Len;

    // 0x0900 à froid (sans armement)
    uint8_t  raw0900Cold[18];
    size_t   raw0900ColdLen;
    uint32_t coldPreset24b;

    // 0x1400 (Scheduler) & 0x1600 (Random Mode)
    uint8_t  raw1400[32];
    size_t   raw1400Len;
    uint8_t  raw1600[20];
    size_t   raw1600Len;

    // 0x0A00 (24h)
    uint32_t sum24h_Wh;
    uint16_t h21_Wh, h22_Wh, h23_Wh;
    uint8_t  cs0A_exp, cs0A_got, cs0A_diffSub;
    char     slots24h[200];

    // 0x0B00 (30j)
    uint32_t sum30d_Wh;
    uint32_t j27_Wh, j28_Wh, j29_Wh;
    uint8_t  cs0B_exp, cs0B_got, cs0B_diffSub;
    char     slots30d[200];

    // 0x0C00 (12m)
    uint32_t sum12m_Wh;
    uint32_t m10_Wh, m11_Wh;
    uint8_t  m10_b3, m11_b3;
    uint8_t  cs0C_exp, cs0C_got, cs0C_diffSub;
    char     slots12m[256];

    // RTC déduite
    RtcProbeResult rtc;
};

static void readFFF1Hex(NimBLERemoteCharacteristic* pFFF1, char* outStr, size_t outLen) {
    if (!pFFF1 || !pFFF1->canRead()) {
        snprintf(outStr, outLen, "N/A");
        return;
    }
    std::string val = pFFF1->readValue();
    size_t pos = 0;
    for (size_t i = 0; i < val.size() && pos + 4 < outLen; i++) {
        pos += snprintf(outStr + pos, outLen - pos, "%02X ", (uint8_t)val[i]);
    }
    if (pos == 0) snprintf(outStr, outLen, "(vide, 0 octet)");
}

static void captureSnapshot(NimBLERemoteCharacteristic* pReq,
                            NimBLERemoteCharacteristic* pFFF1,
                            const char* title,
                            const char* fff1PreAuthStr,
                            CompleteSnapshot& out,
                            bool run2020DirectProbe = false) {
    memset(&out, 0, sizeof(out));
    strncpy(out.title, title, sizeof(out.title) - 1);
    strncpy(out.fff1BeforeAuth, fff1PreAuthStr ? fff1PreAuthStr : "N/A", sizeof(out.fff1BeforeAuth) - 1);
    readFFF1Hex(pFFF1, out.fff1AfterAuth, sizeof(out.fff1AfterAuth));

    Serial.println("\n========================================================================================================================");
    Serial.printf(" 📸 %s\n", out.title);
    Serial.println("========================================================================================================================");
    Serial.printf("  🔹 0xFFF1 (Avant Auth) : %s\n", out.fff1BeforeAuth);
    Serial.printf("  🔹 0xFFF1 (Après Auth) : %s\n", out.fff1AfterAuth);

    // 1. 0x0400 (Télémétrie)
    Serial.println("\n  --- [1. Télémétrie Live 0x0400] ---");
    static const uint8_t cmd0400[9] = { 0x0F, 0x05, 0x04, 0x00, 0x00, 0x00, 0x05, 0xFF, 0xFF };
    if (sendCmdWait(pReq, cmd0400, sizeof(cmd0400), 0x0400, 1200, true) && g_rxLen >= 19) {
        out.relayOn     = (g_rxBuf[4] == 0x01);
        uint32_t rawP   = ((uint32_t)g_rxBuf[5] << 16) | ((uint32_t)g_rxBuf[6] << 8) | g_rxBuf[7];
        out.powerW      = rawP / 1000.0f;
        out.voltageV    = g_rxBuf[8];
        uint16_t rawI   = ((uint16_t)g_rxBuf[9] << 8) | g_rxBuf[10];
        out.currentA    = rawI / 1000.0f;
        out.freqHz      = g_rxBuf[11];
        out.word12_13   = ((uint16_t)g_rxBuf[12] << 8) | g_rxBuf[13];
        out.totalWh0400 = ((uint32_t)g_rxBuf[14] << 24) | ((uint32_t)g_rxBuf[15] << 16) |
                          ((uint32_t)g_rxBuf[16] << 8)  | g_rxBuf[17];
        out.cs04_exp    = calculateSemChecksum(g_rxBuf, 2, 16);
        out.cs04_got    = g_rxBuf[18];

        Serial.printf("     -> Relais=%s | P=%.2f W | U=%u V | I=%.3f A | F=%u Hz | Octets[12..13]=0x%04X | totalKwhWh=%lu Wh | CS(Exp/Got)=0x%02X/0x%02X\n",
                      out.relayOn ? "ON" : "OFF", out.powerW, (unsigned)out.voltageV, out.currentA,
                      (unsigned)out.freqHz, (unsigned)out.word12_13, (unsigned long)out.totalWh0400,
                      (unsigned)out.cs04_exp, (unsigned)out.cs04_got);
    }

    // 2. 0x1000 (Paramètres)
    Serial.println("\n  --- [2. Paramètres 0x1000 (requestSettings)] ---");
    static const uint8_t cmd1000[9] = { 0x0F, 0x05, 0x10, 0x00, 0x00, 0x00, 0x11, 0xFF, 0xFF };
    if (sendCmdWait(pReq, cmd1000, sizeof(cmd1000), 0x1000, 1200, true)) {
        out.raw1000Len = (g_rxRawLen <= sizeof(out.raw1000)) ? g_rxRawLen : sizeof(out.raw1000);
        memcpy(out.raw1000, g_rxBuf, out.raw1000Len);
    }

    // 3. 0x0900 à froid (AVANT tout armement 0x0800 !)
    Serial.println("\n  --- [3. Statut Timer à Froid 0x0900 (SANS armement préalable)] ---");
    static const uint8_t cmd0900[9] = { 0x0F, 0x05, 0x09, 0x00, 0x00, 0x00, 0x0A, 0xFF, 0xFF };
    if (sendCmdWait(pReq, cmd0900, sizeof(cmd0900), 0x0900, 1200, true) && g_rxLen >= 18) {
        out.raw0900ColdLen = 18;
        memcpy(out.raw0900Cold, g_rxBuf, 18);
        out.coldPreset24b = ((uint32_t)g_rxBuf[11] << 16) | ((uint32_t)g_rxBuf[12] << 8) | g_rxBuf[13];
        Serial.printf("     -> Action=0x%02X | Écho Cible en mémoire=%02u/%02u/20%02u %02u:%02u:%02u | presetSeconds24b=%lu s\n",
                      (unsigned)g_rxBuf[4],
                      (unsigned)g_rxBuf[8], (unsigned)g_rxBuf[9], (unsigned)g_rxBuf[10],
                      (unsigned)g_rxBuf[7], (unsigned)g_rxBuf[6], (unsigned)g_rxBuf[5],
                      (unsigned long)out.coldPreset24b);
    }

    // 3bis. 0x1400 (Plannings / Scheduler page 0) & 0x1600 (Random Mode)
    Serial.println("\n  --- [3bis. Plannings 0x1400 (Page 0) & Mode Aléatoire 0x1600] ---");
    static const uint8_t cmd1400[10] = { 0x0F, 0x06, 0x14, 0x00, 0x00, 0x00, 0x00, 0x15, 0xFF, 0xFF };
    if (sendCmdWait(pReq, cmd1400, sizeof(cmd1400), 0x1400, 1200, true)) {
        out.raw1400Len = (g_rxRawLen <= sizeof(out.raw1400)) ? g_rxRawLen : sizeof(out.raw1400);
        memcpy(out.raw1400, g_rxBuf, out.raw1400Len);
    }
    static const uint8_t cmd1600[9] = { 0x0F, 0x05, 0x16, 0x00, 0x00, 0x00, 0x17, 0xFF, 0xFF };
    if (sendCmdWait(pReq, cmd1600, sizeof(cmd1600), 0x1600, 1200, true)) {
        out.raw1600Len = (g_rxRawLen <= sizeof(out.raw1600)) ? g_rxRawLen : sizeof(out.raw1600);
        memcpy(out.raw1600, g_rxBuf, out.raw1600Len);
    }

    // 4. 0x0A00 (24h)
    Serial.println("\n  --- [4. Historique Horaire 24 Heures (0x0A00)] ---");
    static const uint8_t cmd0A00[9] = { 0x0F, 0x05, 0x0A, 0x00, 0x00, 0x00, 0x0B, 0xFF, 0xFF };
    if (sendCmdWait(pReq, cmd0A00, sizeof(cmd0A00), 0x0A00, 1500, true) && g_rxLen >= 55) {
        out.cs0A_exp     = calculateSemChecksum(g_rxBuf, 2, g_rxLen - 5);
        out.cs0A_got     = g_rxBuf[g_rxLen - 3];
        out.cs0A_diffSub = (uint8_t)((out.cs0A_exp - out.cs0A_got) & 0xFF);
        size_t pos = 0;
        for (int i = 0; i < 24; i++) {
            size_t idx = 4 + (i * 2);
            uint16_t wh = ((uint16_t)g_rxBuf[idx] << 8) | g_rxBuf[idx + 1];
            out.sum24h_Wh += wh;
            if (i == 21) out.h21_Wh = wh;
            if (i == 22) out.h22_Wh = wh;
            if (i == 23) out.h23_Wh = wh;
            if (wh > 0 && pos + 16 < sizeof(out.slots24h)) {
                pos += snprintf(out.slots24h + pos, sizeof(out.slots24h) - pos, "[%02d]=%u ", i, (unsigned)wh);
            }
        }
        Serial.printf("     -> Cumul=%4lu Wh | H[21..23]=%u/%u/%u | CS_Exp=0x%02X | CS_Got=0x%02X | Écart(Exp-Got)=0x%02X (%3u) | Slots>0: %s\n",
                      (unsigned long)out.sum24h_Wh, (unsigned)out.h21_Wh, (unsigned)out.h22_Wh, (unsigned)out.h23_Wh,
                      (unsigned)out.cs0A_exp, (unsigned)out.cs0A_got, (unsigned)out.cs0A_diffSub, (unsigned)out.cs0A_diffSub,
                      out.slots24h);
    }

    // 5. 0x0B00 (30j)
    Serial.println("\n  --- [5. Historique Journalier 30 Jours (0x0B00)] ---");
    static const uint8_t cmd0B00[9] = { 0x0F, 0x05, 0x0B, 0x00, 0x00, 0x00, 0x0C, 0xFF, 0xFF };
    if (sendCmdWait(pReq, cmd0B00, sizeof(cmd0B00), 0x0B00, 1500, true) && g_rxLen >= 127) {
        out.cs0B_exp     = calculateSemChecksum(g_rxBuf, 2, g_rxLen - 5);
        out.cs0B_got     = g_rxBuf[g_rxLen - 3];
        out.cs0B_diffSub = (uint8_t)((out.cs0B_exp - out.cs0B_got) & 0xFF);
        size_t pos = 0;
        for (int i = 0; i < 30; i++) {
            size_t idx = 4 + (i * 4);
            uint32_t wh = ((uint32_t)g_rxBuf[idx] << 16) | ((uint32_t)g_rxBuf[idx + 1] << 8) | g_rxBuf[idx + 2];
            out.sum30d_Wh += wh;
            if (i == 27) out.j27_Wh = wh;
            if (i == 28) out.j28_Wh = wh;
            if (i == 29) out.j29_Wh = wh;
            if (wh > 0 && pos + 18 < sizeof(out.slots30d)) {
                pos += snprintf(out.slots30d + pos, sizeof(out.slots30d) - pos, "[%02d]=%lu ", i, (unsigned long)wh);
            }
        }
        Serial.printf("     -> Cumul=%4lu Wh | J[27..29]=%lu/%lu/%lu | CS_Exp=0x%02X | CS_Got=0x%02X | Écart(Exp-Got)=0x%02X (%3u) | Slots>0: %s\n",
                      (unsigned long)out.sum30d_Wh, (unsigned long)out.j27_Wh, (unsigned long)out.j28_Wh, (unsigned long)out.j29_Wh,
                      (unsigned)out.cs0B_exp, (unsigned)out.cs0B_got, (unsigned)out.cs0B_diffSub, (unsigned)out.cs0B_diffSub,
                      out.slots30d);
    }

    // 6. 0x0C00 (12m)
    Serial.println("\n  --- [6. Historique Mensuel 12 Mois (0x0C00)] ---");
    static const uint8_t cmd0C00[9] = { 0x0F, 0x05, 0x0C, 0x00, 0x00, 0x00, 0x0D, 0xFF, 0xFF };
    if (sendCmdWait(pReq, cmd0C00, sizeof(cmd0C00), 0x0C00, 1500, true) && g_rxLen >= 55) {
        out.cs0C_exp     = calculateSemChecksum(g_rxBuf, 2, g_rxLen - 5);
        out.cs0C_got     = g_rxBuf[g_rxLen - 3];
        out.cs0C_diffSub = (uint8_t)((out.cs0C_exp - out.cs0C_got) & 0xFF);
        size_t pos = 0;
        for (int i = 0; i < 12; i++) {
            size_t idx = 4 + (i * 4);
            uint32_t wh = ((uint32_t)g_rxBuf[idx] << 16) | ((uint32_t)g_rxBuf[idx + 1] << 8) | g_rxBuf[idx + 2];
            uint8_t  b3 = g_rxBuf[idx + 3];
            out.sum12m_Wh += wh;
            if (i == 10) { out.m10_Wh = wh; out.m10_b3 = b3; }
            if (i == 11) { out.m11_Wh = wh; out.m11_b3 = b3; }
            if (wh > 0 && pos + 22 < sizeof(out.slots12m)) {
                pos += snprintf(out.slots12m + pos, sizeof(out.slots12m) - pos, "[%02d]=%lu(%u%%) ", i, (unsigned long)wh, (unsigned)b3);
            }
        }
        Serial.printf("     -> Cumul=%4lu Wh | M[10..11]=%lu(%u%%)/%lu(%u%%) | CS_Exp=0x%02X | CS_Got=0x%02X | Écart(Exp-Got)=0x%02X (%3u) | Slots>0: %s\n",
                      (unsigned long)out.sum12m_Wh,
                      (unsigned long)out.m10_Wh, (unsigned)out.m10_b3,
                      (unsigned long)out.m11_Wh, (unsigned)out.m11_b3,
                      (unsigned)out.cs0C_exp, (unsigned)out.cs0C_got, (unsigned)out.cs0C_diffSub, (unsigned)out.cs0C_diffSub,
                      out.slots12m);
    }

    // 7. Sonde Timer Fantôme (0x0800 / 0x0900)
    Serial.println("\n  --- [7. Sonde Timer Fantôme (0x0800 / 0x0900) pour lecture de la RTC interne] ---");
    out.rtc = probeRtcWithTarget(pReq, "SONDE CIBLE 31/12/2026", 31, 12, 2026, true);
    if (run2020DirectProbe) {
        probeRtcWithTarget(pReq, "SONDE DIRECTE CIBLE 02/01/2020 (Vérif RTC=01/01/2020)", 2, 1, 2020, true);
    }
}

static bool connectAndAuthVelo(NimBLEClient*& outClient,
                               NimBLERemoteCharacteristic*& outReq,
                               NimBLERemoteCharacteristic*& outFFF1,
                               char* outFFF1PreAuth,
                               size_t outFFF1PreAuthLen) {
    NimBLEAddress addr(MAC_VELO_HW3, BLE_ADDR_PUBLIC);
    NimBLEClient* pClient = NimBLEDevice::getClientByPeerAddress(addr);
    if (!pClient) {
        pClient = NimBLEDevice::createClient();
    }

    pClient->setConnectTimeout(6000);
    bool connected = false;
    for (int attempt = 1; attempt <= 3; attempt++) {
        Serial.printf("  [BLE] Tentative de connexion #%d à Vélo (%s)...\n", attempt, MAC_VELO_HW3);
        if (pClient->connect(addr)) {
            connected = true;
            break;
        }
        delay(600);
    }

    if (!connected) {
        Serial.println("  ❌ Échec de connexion BLE à Vélo.");
        return false;
    }

    Serial.printf("  ✅ Connecté | MTU = %u octets | RSSI = %d dBm\n",
                  (unsigned)pClient->getMTU(), (int)pClient->getRssi());

    // Lecture optionnelle du nom GAP 0x2A00
    NimBLERemoteService* pGap = pClient->getService(SERVICE_GAP_UUID);
    if (pGap) {
        NimBLERemoteCharacteristic* pName = pGap->getCharacteristic(CHAR_GAP_NAME_UUID);
        if (pName && pName->canRead()) {
            std::string gName = pName->readValue();
            Serial.printf("  🔹 GAP Device Name (0x2A00) : \"%s\"\n", gName.c_str());
        }
    }

    NimBLERemoteService* pSvc = pClient->getService(Service_FFF0_UUID);
    if (!pSvc) {
        Serial.println("  ❌ Service 0xFFF0 introuvable.");
        pClient->disconnect();
        return false;
    }

    NimBLERemoteCharacteristic* pFFF1 = pSvc->getCharacteristic(CHAR_FFF1_UUID);
    NimBLERemoteCharacteristic* pReq  = pSvc->getCharacteristic(CHAR_REQ_FFF3_UUID);
    NimBLERemoteCharacteristic* pRes  = pSvc->getCharacteristic(CHAR_RES_FFF4_UUID);
    if (!pReq || !pRes) {
        Serial.println("  ❌ Caractéristiques 0xFFF3/0xFFF4 introuvables.");
        pClient->disconnect();
        return false;
    }

    readFFF1Hex(pFFF1, outFFF1PreAuth, outFFF1PreAuthLen);

    if (!pRes->subscribe(true, onNotifyFFF4)) {
        Serial.println("  ❌ Échec souscription notifications 0xFFF4.");
        pClient->disconnect();
        return false;
    }
    delay(150);

    Serial.println("  🔑 Authentification 32 bits (0x1700) :");
    static const uint8_t authFrame[16] = {
        0x0F, 0x0C, 0x17, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x18, 0xFF, 0xFF
    };
    if (!sendCmdWait(pReq, authFrame, sizeof(authFrame), 0x1700, 1500, true)) {
        Serial.println("  ❌ Échec authentification 32 bits.");
        pClient->disconnect();
        return false;
    }

    outClient = pClient;
    outReq    = pReq;
    outFFF1   = pFFF1;
    return true;
}

static void runExperiment22() {
    Serial.println("\n########################################################################################################################");
    Serial.printf(" ⚡ EXPÉRIENCE 22 (v2) : COUPURE SECTEUR + REMISE À L'HEURE (+30s) SUR VÉLO HW 3.0 (%s)\n", MAC_VELO_HW3);
    Serial.println("########################################################################################################################");

    CompleteSnapshot snapInitial;
    CompleteSnapshot snapBeforeCut;
    CompleteSnapshot snapAfterCutCold;
    CompleteSnapshot snapAfterResync30s;

    // ========================================================================================================
    // ÉTAPE 1 : CONNEXION INITIALE, CALAGE HORLOGE À 27/11/2026 16:15:00 & ARMEMENT D'UN TIMER ACTIF TÉMOIN
    // ========================================================================================================
    NimBLEClient* pClient = nullptr;
    NimBLERemoteCharacteristic* pReq  = nullptr;
    NimBLERemoteCharacteristic* pFFF1 = nullptr;
    char fff1PreAuth1[48] = {0};

    if (!connectAndAuthVelo(pClient, pReq, pFFF1, fff1PreAuth1, sizeof(fff1PreAuth1))) {
        return;
    }

    // S'assurer que le relais est ON
    static const uint8_t cmdRelayOn[10] = { 0x0F, 0x06, 0x03, 0x00, 0x01, 0x00, 0x00, 0x05, 0xFF, 0xFF };
    sendCmdWait(pReq, cmdRelayOn, sizeof(cmdRelayOn), 0x0300, 1000, true);
    delay(300);

    // Cliché #0 : État initial avant tout setDateTime
    captureSnapshot(pReq, pFFF1, "CLICHÉ #0 : État Initial avant calage 16:15:00", fff1PreAuth1, snapInitial, true);

    // Calage à 27/11/2026 16:15:00 (ou jour suivant si déjà > 16:15 le 27/11)
    uint8_t  targetDay   = 27;
    uint8_t  targetMonth = 11;
    uint16_t targetYear  = 2026;
    uint8_t  targetHour  = 16;
    uint8_t  targetMin   = 15;
    uint8_t  targetSec   = 0;

    if (snapInitial.rtc.valid && !snapInitial.rtc.isReset2020) {
        targetDay   = snapInitial.rtc.rtcDay;
        targetMonth = snapInitial.rtc.rtcMonth;
        targetYear  = snapInitial.rtc.rtcYear;

        int32_t currentSecOfDay = (int32_t)snapInitial.rtc.rtcHour * 3600L +
                                  (int32_t)snapInitial.rtc.rtcMin  * 60L +
                                  (int32_t)snapInitial.rtc.rtcSec;
        int32_t target16h15     = 16L * 3600L + 15L * 60L;

        if (currentSecOfDay > target16h15) {
            if (snapInitial.rtc.rtcMin < 30) {
                targetHour = snapInitial.rtc.rtcHour;
                targetMin  = 30;
            } else {
                int32_t d2000 = dateToDaysSince2000(targetDay, targetMonth, targetYear) + 1;
                daysSince2000ToDate(d2000, targetDay, targetMonth, targetYear);
            }
        }
    }

    Serial.println("\n  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    Serial.printf("  🛠️ [CALAGE HORLOGE AVANT COUPURE] Injection setDateTime(%02u/%02u/%04u %02u:%02u:%02u) + Attente 15s sous charge...\n",
                  (unsigned)targetDay, (unsigned)targetMonth, (unsigned)targetYear,
                  (unsigned)targetHour, (unsigned)targetMin, (unsigned)targetSec);
    sendSetDateTime(pReq, targetSec, targetMin, targetHour, targetDay, targetMonth, targetYear);

    for (int s = 5; s <= 15; s += 5) {
        delay(5000);
        static const uint8_t cmd0400[9] = { 0x0F, 0x05, 0x04, 0x00, 0x00, 0x00, 0x05, 0xFF, 0xFF };
        if (sendCmdWait(pReq, cmd0400, sizeof(cmd0400), 0x0400, 1000, false) && g_rxLen >= 19) {
            uint32_t rawP = ((uint32_t)g_rxBuf[5] << 16) | ((uint32_t)g_rxBuf[6] << 8) | g_rxBuf[7];
            uint32_t wh04 = ((uint32_t)g_rxBuf[14] << 24) | ((uint32_t)g_rxBuf[15] << 16) |
                            ((uint32_t)g_rxBuf[16] << 8)  | g_rxBuf[17];
            Serial.printf("     [+%2ds] P=%.1f W | 0400_totalKwhWh=%lu Wh\n", s, rawP / 1000.0f, (unsigned long)wh04);
        }
    }

    // Cliché #1 : Référence juste AVANT coupure secteur
    captureSnapshot(pReq, pFFF1, "CLICHÉ #1 : Référence AVANT Coupure Secteur (~16:15:15)", fff1PreAuth1, snapBeforeCut, false);

    // BONUS : On laisse un TIMER ACTIF (Action=0x01, cible 23:00:00 du même jour) armé avant de débrancher la prise
    // pour vérifier si un timer utilisateur programmé survit à une coupure secteur ou est effacé !
    Serial.println("\n  ⏲️ [TEST PERSISTANCE TIMER UTILISATEUR] Armement d'un Timer ACTIF (23:00:00) laissé actif avant la coupure :");
    uint8_t cmdLeaveTimerActive[16] = {
        0x0F, 0x0C, 0x08, 0x00,
        0x01, // ON
        0, 0, 23,
        targetDay, targetMonth, (uint8_t)(targetYear % 100),
        0x00, 0x00, 0x00, 0xFF, 0xFF
    };
    cmdLeaveTimerActive[13] = calculateSemChecksum(cmdLeaveTimerActive, 2, 11);
    sendCmdWait(pReq, cmdLeaveTimerActive, sizeof(cmdLeaveTimerActive), 0x0800, 1000, true);
    static const uint8_t cmdCheck0900[9] = { 0x0F, 0x05, 0x09, 0x00, 0x00, 0x00, 0x0A, 0xFF, 0xFF };
    sendCmdWait(pReq, cmdCheck0900, sizeof(cmdCheck0900), 0x0900, 1000, true);

    pClient->disconnect();
    delay(300);
    Serial.println("\n  [BLE] 🔌 Déconnecté proprement de Vélo (Timer utilisateur laissé ACTIF).");

    // ========================================================================================================
    // ÉTAPE 2 : ACTION MANUELLE DE COUPURE SECTEUR
    // ========================================================================================================
    Serial.println("\n************************************************************************************************************************");
    Serial.println(" ⚡ ACTION REQUISE : DÉBRANCHEZ LA PRISE VÉLO DU SECTEUR PENDANT ~10 SECONDES, PUIS REBRANCHEZ-LA !");
    Serial.println("************************************************************************************************************************");
    waitForBootButtonOrSerial("ÉTAPE 2 : PRISE VÉLO REBRANCHÉE APRÈS COUPURE SECTEUR -> LANCER AUSCULTATION À FROID");

    // ========================================================================================================
    // ÉTAPE 3 : AUSCULTATION À FROID APRÈS COUPURE SECTEUR (SANS SETDATETIME)
    // ========================================================================================================
    char fff1PreAuth2[48] = {0};
    if (!connectAndAuthVelo(pClient, pReq, pFFF1, fff1PreAuth2, sizeof(fff1PreAuth2))) {
        return;
    }

    captureSnapshot(pReq, pFFF1, "CLICHÉ #2 : État à Froid APRÈS Coupure Secteur (SANS setDateTime)", fff1PreAuth2, snapAfterCutCold, true);

    // ========================================================================================================
    // ÉTAPE 4 (POINTS a & b) : REMISE À L'HEURE (MÊME CRÉNEAU HORAIRE) + ATTENTE 30 SECONDES SOUS CHARGE
    // ========================================================================================================
    uint8_t resyncMin = (uint8_t)(targetMin + 1); // ex: 16:16:00 (même heure 16h, même jour, 1 min plus tard)
    Serial.println("\n  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    Serial.printf("  🔄 [POINT a : REMISE À L'HEURE POST-COUPURE] Injection setDateTime(%02u/%02u/%04u %02u:%02u:00) + Suivi 30s sous charge...\n",
                  (unsigned)targetDay, (unsigned)targetMonth, (unsigned)targetYear,
                  (unsigned)targetHour, (unsigned)resyncMin);
    sendSetDateTime(pReq, 0, resyncMin, targetHour, targetDay, targetMonth, targetYear);

    // Suivi live toutes les 5 secondes pendant 30 secondes pour observer si H[23], J[29], M[11] réapparaissent dès la remise à l'heure !
    for (int s = 5; s <= 30; s += 5) {
        delay(5000);
        float pW = 0.0f;
        uint32_t wh04 = 0;
        uint16_t h21 = 0, h22 = 0, h23 = 0;
        uint32_t j27 = 0, j28 = 0, j29 = 0;
        uint32_t m10 = 0, m11 = 0;
        uint8_t  m10b3 = 0, m11b3 = 0;

        static const uint8_t c04[9] = { 0x0F, 0x05, 0x04, 0x00, 0x00, 0x00, 0x05, 0xFF, 0xFF };
        if (sendCmdWait(pReq, c04, sizeof(c04), 0x0400, 1000, false) && g_rxLen >= 19) {
            uint32_t rawP = ((uint32_t)g_rxBuf[5] << 16) | ((uint32_t)g_rxBuf[6] << 8) | g_rxBuf[7];
            pW = rawP / 1000.0f;
            wh04 = ((uint32_t)g_rxBuf[14] << 24) | ((uint32_t)g_rxBuf[15] << 16) |
                   ((uint32_t)g_rxBuf[16] << 8)  | g_rxBuf[17];
        }
        static const uint8_t c0A[9] = { 0x0F, 0x05, 0x0A, 0x00, 0x00, 0x00, 0x0B, 0xFF, 0xFF };
        if (sendCmdWait(pReq, c0A, sizeof(c0A), 0x0A00, 1000, false) && g_rxLen >= 55) {
            h21 = ((uint16_t)g_rxBuf[4 + 21 * 2] << 8) | g_rxBuf[5 + 21 * 2];
            h22 = ((uint16_t)g_rxBuf[4 + 22 * 2] << 8) | g_rxBuf[5 + 22 * 2];
            h23 = ((uint16_t)g_rxBuf[4 + 23 * 2] << 8) | g_rxBuf[5 + 23 * 2];
        }
        static const uint8_t c0B[9] = { 0x0F, 0x05, 0x0B, 0x00, 0x00, 0x00, 0x0C, 0xFF, 0xFF };
        if (sendCmdWait(pReq, c0B, sizeof(c0B), 0x0B00, 1000, false) && g_rxLen >= 127) {
            j27 = ((uint32_t)g_rxBuf[4 + 27 * 4] << 16) | ((uint32_t)g_rxBuf[5 + 27 * 4] << 8) | g_rxBuf[6 + 27 * 4];
            j28 = ((uint32_t)g_rxBuf[4 + 28 * 4] << 16) | ((uint32_t)g_rxBuf[5 + 28 * 4] << 8) | g_rxBuf[6 + 28 * 4];
            j29 = ((uint32_t)g_rxBuf[4 + 29 * 4] << 16) | ((uint32_t)g_rxBuf[5 + 29 * 4] << 8) | g_rxBuf[6 + 29 * 4];
        }
        static const uint8_t c0C[9] = { 0x0F, 0x05, 0x0C, 0x00, 0x00, 0x00, 0x0D, 0xFF, 0xFF };
        if (sendCmdWait(pReq, c0C, sizeof(c0C), 0x0C00, 1000, false) && g_rxLen >= 55) {
            m10   = ((uint32_t)g_rxBuf[4 + 10 * 4] << 16) | ((uint32_t)g_rxBuf[5 + 10 * 4] << 8) | g_rxBuf[6 + 10 * 4];
            m10b3 = g_rxBuf[7 + 10 * 4];
            m11   = ((uint32_t)g_rxBuf[4 + 11 * 4] << 16) | ((uint32_t)g_rxBuf[5 + 11 * 4] << 8) | g_rxBuf[6 + 11 * 4];
            m11b3 = g_rxBuf[7 + 11 * 4];
        }

        Serial.printf("     [+%2ds] P=%5.1fW | 0400=%lu | H[21..23]=%u/%u/%u | J[27..29]=%lu/%lu/%lu | M[10..11]=%lu(%u%%)/%lu(%u%%)\n",
                      s, pW, (unsigned long)wh04,
                      (unsigned)h21, (unsigned)h22, (unsigned)h23,
                      (unsigned long)j27, (unsigned long)j28, (unsigned long)j29,
                      (unsigned long)m10, (unsigned)m10b3, (unsigned long)m11, (unsigned)m11b3);
    }

    // Cliché #3 : État complet APRÈS remise à l'heure + 30 secondes
    captureSnapshot(pReq, pFFF1, "CLICHÉ #3 : APRÈS Remise à l'heure + 30s sous charge", fff1PreAuth2, snapAfterResync30s, false);

    pClient->disconnect();
    delay(200);

    // ========================================================================================================
    // POINT b : TABLEAU COMPARATIF SYNTHÉTIQUE À 3 COLONNES
    // ========================================================================================================
    Serial.println("\n================================================================================================================================================");
    Serial.println(" 📊 BILAN COMPARATIF EXPÉRIENCE 22 (v2) : 1. AVANT COUPURE  |  2. APRÈS COUPURE À FROID  |  3. APRÈS REMISE À L'HEURE (+30s)");
    Serial.println("================================================================================================================================================");
    Serial.printf(" Indicateur                     | 1. AVANT Coupure (Cliché #1)        | 2. APRÈS Coupure à Froid (Cliché #2)| 3. APRÈS Remise à l'Heure +30s (#3)\n");
    Serial.println("--------------------------------+-------------------------------------+-------------------------------------+------------------------------------");
    Serial.printf(" Relais / Puissance (0x0400)    | %-3s / %7.2f W                     | %-3s / %7.2f W                     | %-3s / %7.2f W\n",
                  snapBeforeCut.relayOn ? "ON" : "OFF", snapBeforeCut.powerW,
                  snapAfterCutCold.relayOn ? "ON" : "OFF", snapAfterCutCold.powerW,
                  snapAfterResync30s.relayOn ? "ON" : "OFF", snapAfterResync30s.powerW);
    Serial.printf(" Octets[12..13] de 0x0400       | 0x%04X                              | 0x%04X                              | 0x%04X\n",
                  (unsigned)snapBeforeCut.word12_13,
                  (unsigned)snapAfterCutCold.word12_13,
                  (unsigned)snapAfterResync30s.word12_13);
    Serial.printf(" totalKwhWh (0x0400[14..17])    | %8lu Wh                         | %8lu Wh                         | %8lu Wh\n",
                  (unsigned long)snapBeforeCut.totalWh0400,
                  (unsigned long)snapAfterCutCold.totalWh0400,
                  (unsigned long)snapAfterResync30s.totalWh0400);
    Serial.printf(" 0x0900 à froid (Act / Cible)   | act=%02X %02u/%02u/20%02u %02u:%02u:%02u          | act=%02X %02u/%02u/20%02u %02u:%02u:%02u          | act=%02X %02u/%02u/20%02u %02u:%02u:%02u\n",
                  (unsigned)snapBeforeCut.raw0900Cold[4],
                  (unsigned)snapBeforeCut.raw0900Cold[8], (unsigned)snapBeforeCut.raw0900Cold[9], (unsigned)snapBeforeCut.raw0900Cold[10],
                  (unsigned)snapBeforeCut.raw0900Cold[7], (unsigned)snapBeforeCut.raw0900Cold[6], (unsigned)snapBeforeCut.raw0900Cold[5],
                  (unsigned)snapAfterCutCold.raw0900Cold[4],
                  (unsigned)snapAfterCutCold.raw0900Cold[8], (unsigned)snapAfterCutCold.raw0900Cold[9], (unsigned)snapAfterCutCold.raw0900Cold[10],
                  (unsigned)snapAfterCutCold.raw0900Cold[7], (unsigned)snapAfterCutCold.raw0900Cold[6], (unsigned)snapAfterCutCold.raw0900Cold[5],
                  (unsigned)snapAfterResync30s.raw0900Cold[4],
                  (unsigned)snapAfterResync30s.raw0900Cold[8], (unsigned)snapAfterResync30s.raw0900Cold[9], (unsigned)snapAfterResync30s.raw0900Cold[10],
                  (unsigned)snapAfterResync30s.raw0900Cold[7], (unsigned)snapAfterResync30s.raw0900Cold[6], (unsigned)snapAfterResync30s.raw0900Cold[5]);
    Serial.printf(" 0x0A00 (24h: H[21/22/23] & CS) | %u/%u/%u Wh (Cum=%lu, CS=%02X/%02X)     | %u/%u/%u Wh (Cum=%lu, CS=%02X/%02X)     | %u/%u/%u Wh (Cum=%lu, CS=%02X/%02X)\n",
                  (unsigned)snapBeforeCut.h21_Wh, (unsigned)snapBeforeCut.h22_Wh, (unsigned)snapBeforeCut.h23_Wh,
                  (unsigned long)snapBeforeCut.sum24h_Wh, (unsigned)snapBeforeCut.cs0A_exp, (unsigned)snapBeforeCut.cs0A_got,
                  (unsigned)snapAfterCutCold.h21_Wh, (unsigned)snapAfterCutCold.h22_Wh, (unsigned)snapAfterCutCold.h23_Wh,
                  (unsigned long)snapAfterCutCold.sum24h_Wh, (unsigned)snapAfterCutCold.cs0A_exp, (unsigned)snapAfterCutCold.cs0A_got,
                  (unsigned)snapAfterResync30s.h21_Wh, (unsigned)snapAfterResync30s.h22_Wh, (unsigned)snapAfterResync30s.h23_Wh,
                  (unsigned long)snapAfterResync30s.sum24h_Wh, (unsigned)snapAfterResync30s.cs0A_exp, (unsigned)snapAfterResync30s.cs0A_got);
    Serial.printf(" 0x0B00 (30j: J[27/28/29] & CS) | %lu/%lu/%lu Wh (Cum=%lu, CS=%02X/%02X)    | %lu/%lu/%lu Wh (Cum=%lu, CS=%02X/%02X)    | %lu/%lu/%lu Wh (Cum=%lu, CS=%02X/%02X)\n",
                  (unsigned long)snapBeforeCut.j27_Wh, (unsigned long)snapBeforeCut.j28_Wh, (unsigned long)snapBeforeCut.j29_Wh,
                  (unsigned long)snapBeforeCut.sum30d_Wh, (unsigned)snapBeforeCut.cs0B_exp, (unsigned)snapBeforeCut.cs0B_got,
                  (unsigned long)snapAfterCutCold.j27_Wh, (unsigned long)snapAfterCutCold.j28_Wh, (unsigned long)snapAfterCutCold.j29_Wh,
                  (unsigned long)snapAfterCutCold.sum30d_Wh, (unsigned)snapAfterCutCold.cs0B_exp, (unsigned)snapAfterCutCold.cs0B_got,
                  (unsigned long)snapAfterResync30s.j27_Wh, (unsigned long)snapAfterResync30s.j28_Wh, (unsigned long)snapAfterResync30s.j29_Wh,
                  (unsigned long)snapAfterResync30s.sum30d_Wh, (unsigned)snapAfterResync30s.cs0B_exp, (unsigned)snapAfterResync30s.cs0B_got);
    Serial.printf(" 0x0C00 (12m: M[10/11] & CS)    | %lu(%u%%)/%lu(%u%%) (Cum=%lu, CS=%02X/%02X) | %lu(%u%%)/%lu(%u%%) (Cum=%lu, CS=%02X/%02X) | %lu(%u%%)/%lu(%u%%) (Cum=%lu, CS=%02X/%02X)\n",
                  (unsigned long)snapBeforeCut.m10_Wh, (unsigned)snapBeforeCut.m10_b3,
                  (unsigned long)snapBeforeCut.m11_Wh, (unsigned)snapBeforeCut.m11_b3,
                  (unsigned long)snapBeforeCut.sum12m_Wh, (unsigned)snapBeforeCut.cs0C_exp, (unsigned)snapBeforeCut.cs0C_got,
                  (unsigned long)snapAfterCutCold.m10_Wh, (unsigned)snapAfterCutCold.m10_b3,
                  (unsigned long)snapAfterCutCold.m11_Wh, (unsigned)snapAfterCutCold.m11_b3,
                  (unsigned long)snapAfterCutCold.sum12m_Wh, (unsigned)snapAfterCutCold.cs0C_exp, (unsigned)snapAfterCutCold.cs0C_got,
                  (unsigned long)snapAfterResync30s.m10_Wh, (unsigned)snapAfterResync30s.m10_b3,
                  (unsigned long)snapAfterResync30s.m11_Wh, (unsigned)snapAfterResync30s.m11_b3,
                  (unsigned long)snapAfterResync30s.sum12m_Wh, (unsigned)snapAfterResync30s.cs0C_exp, (unsigned)snapAfterResync30s.cs0C_got);
    Serial.printf(" RTC Interne (Sonde Timer 24b)  | %-35s | %-35s | %s\n",
                  snapBeforeCut.rtc.summary, snapAfterCutCold.rtc.summary, snapAfterResync30s.rtc.summary);
    Serial.println("================================================================================================================================================");
    Serial.println("\n[BLE] ✅ Expérience 22 (v2) terminée.");
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
    NimBLEDevice::init("ESP32S3-Exp22v2");
    NimBLEDevice::setMTU(160);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    waitForBootButtonOrSerial("ÉTAPE 0 : LANCEMENT EXPÉRIENCE 22 v2 (CHARGE BRANCHÉE & RELAIS ON)");
    runExperiment22();
}

void loop() {
    waitForBootButtonOrSerial("RELANCER EXPÉRIENCE 22 v2");
    runExperiment22();
}
