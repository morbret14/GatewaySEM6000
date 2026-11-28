/**
 * @file exp_21_maitrise_rotations_et_cs_hw3.cpp
 * @brief Expérience 21 : Maîtrise des rotations Horaire (Hour), Journalière (Day) et Mensuelle (Month)
 *        sous charge marqueur sur Vélo (HW 3.0 / FW 1.30), avec Sonde Timer 24 bits en secondes
 *        et caractérisation complète des Checksums (0x0A00, 0x0B00, 0x0C00).
 *
 * Déroulement (~80 secondes au total, sous charge branchée pour générer des marqueurs Wh) :
 * - CLICHÉ #0 (État initial à froid) :
 *   Lecture Sonde Timer 24 bits + Audit complet 0x0A00 (24h), 0x0B00 (30j), 0x0C00 (12m).
 *   (Permet de vérifier si le passage naturel de 22:00:00 après exp_20 a bien décalé [01]->[00]).
 * - TEST 1 : ROTATION HORAIRE (26/11/2026 22:59:45 -> 23:00:10, suivi live 25 s) :
 *   Injection de 22:59:45 (15s avant l'heure pile) et suivi toutes les 5s jusqu'à 23:00:10.
 *   Audit complet post-rotation horaire (#1).
 * - TEST 2 : ROTATION JOURNALIÈRE / MINUIT (26/11/2026 23:59:45 -> 27/11/2026 00:00:10, suivi live 25 s) :
 *   Injection de 23:59:45 (15s avant Minuit) et suivi toutes les 5s jusqu'à 27/11/2026 00:00:10.
 *   Audit complet post-minuit (#2).
 * - TEST 3 : ROTATION MENSUELLE / FIN DE MOIS (30/11/2026 23:59:45 -> 01/12/2026 00:00:10, suivi live 25 s) :
 *   Injection de 30/11/2026 23:59:45 (15s avant la fin du mois) et suivi toutes les 5s jusqu'à 01/12/2026 00:00:10.
 *   Audit complet post-fin de mois (#3).
 * - TABLEAU SYNTHÉTIQUE FINAL des 4 Audits (Clichés #0, #1, #2, #3).
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
        delay(4);
    }
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
    return sendCmdWait(pReq, frame, sizeof(frame), 0x0100, 1000);
}

// Sonde Timer 24 bits Big-Endian en secondes : presetSeconds = (B11 << 16) | (B12 << 8) | B13
static void probeInternalClock24Bit(NimBLERemoteCharacteristic* pReq,
                                    uint8_t refDay, uint8_t refMonth, uint16_t refYear,
                                    char* outClockStr, size_t outClockLen,
                                    uint32_t* outRawDeltaSec = nullptr) {
    // On fixe la cible à 23:59:59 du jour refDay/refMonth/refYear (ou du lendemain si on est à 23h59)
    uint8_t cmdArm[16] = {
        0x0F, 0x0C, 0x08, 0x00,
        0x01, // Relais ON
        59, 59, 23,
        refDay, refMonth, (uint8_t)(refYear % 100),
        0x00, 0x00, 0x00, 0xFF, 0xFF
    };
    cmdArm[13] = calculateSemChecksum(cmdArm, 2, 11);
    sendCmdWait(pReq, cmdArm, sizeof(cmdArm), 0x0800, 800);

    static const uint8_t cmdGet0900[9] = { 0x0F, 0x05, 0x09, 0x00, 0x00, 0x00, 0x0A, 0xFF, 0xFF };
    uint32_t deltaSec24 = 0;
    if (sendCmdWait(pReq, cmdGet0900, sizeof(cmdGet0900), 0x0900, 800) && g_rxLen >= 18) {
        deltaSec24 = ((uint32_t)g_rxBuf[11] << 16) | ((uint32_t)g_rxBuf[12] << 8) | (uint32_t)g_rxBuf[13];
    }

    static const uint8_t cmdStop[16] = {
        0x0F, 0x0C, 0x08, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x09, 0xFF, 0xFF
    };
    sendCmdWait(pReq, cmdStop, sizeof(cmdStop), 0x0800, 800);

    if (outRawDeltaSec) *outRawDeltaSec = deltaSec24;

    int32_t targetSecOfDay = 23L * 3600L + 59L * 60L + 59L; // 86399
    int32_t clockSecOfDay  = targetSecOfDay - (int32_t)(deltaSec24 % 86400UL);
    if (clockSecOfDay < 0) clockSecOfDay += 86400L;

    uint8_t cH = (uint8_t)(clockSecOfDay / 3600L);
    uint8_t cM = (uint8_t)((clockSecOfDay % 3600L) / 60L);
    uint8_t cS = (uint8_t)(clockSecOfDay % 60L);
    snprintf(outClockStr, outClockLen, "%02u:%02u:%02u (Δt=%lus)", (unsigned)cH, (unsigned)cM, (unsigned)cS, (unsigned long)deltaSec24);
}

struct FullAudit {
    char     title[64];
    char     measuredClock[40];
    float    powerW;
    uint32_t totalWh0400;

    // 0x0A00 (24h)
    uint32_t sum24h_Wh;
    uint16_t h22_Wh, h23_Wh;
    uint8_t  cs0A_exp, cs0A_got, cs0A_diffSub;
    char     slots24h[200];

    // 0x0B00 (30j)
    uint32_t sum30d_Wh;
    uint32_t j28_Wh, j29_Wh;
    uint8_t  cs0B_exp, cs0B_got, cs0B_diffSub;
    char     slots30d[200];

    // 0x0C00 (12m)
    uint32_t sum12m_Wh;
    uint32_t m10_Wh, m11_Wh;
    uint8_t  cs0C_exp, cs0C_got, cs0C_diffSub;
    char     slots12m[256];
};

static void captureFullAudit(NimBLERemoteCharacteristic* pReq,
                             const char* title,
                             uint8_t probeRefDay, uint8_t probeRefMonth, uint16_t probeRefYear,
                             FullAudit& out) {
    memset(&out, 0, sizeof(out));
    strncpy(out.title, title, sizeof(out.title) - 1);

    probeInternalClock24Bit(pReq, probeRefDay, probeRefMonth, probeRefYear,
                            out.measuredClock, sizeof(out.measuredClock));

    // 0. 0x0400 (Télémétrie)
    static const uint8_t cmd0400[9] = { 0x0F, 0x05, 0x04, 0x00, 0x00, 0x00, 0x05, 0xFF, 0xFF };
    if (sendCmdWait(pReq, cmd0400, sizeof(cmd0400), 0x0400, 1000) && g_rxLen >= 19) {
        uint32_t rawP = ((uint32_t)g_rxBuf[5] << 16) | ((uint32_t)g_rxBuf[6] << 8) | g_rxBuf[7];
        out.powerW = rawP / 1000.0f;
        out.totalWh0400 = ((uint32_t)g_rxBuf[14] << 24) | ((uint32_t)g_rxBuf[15] << 16) |
                          ((uint32_t)g_rxBuf[16] << 8)  | g_rxBuf[17];
    }

    // 1. 0x0A00 (24h)
    static const uint8_t cmd0A00[9] = { 0x0F, 0x05, 0x0A, 0x00, 0x00, 0x00, 0x0B, 0xFF, 0xFF };
    if (sendCmdWait(pReq, cmd0A00, sizeof(cmd0A00), 0x0A00, 1200) && g_rxLen >= 55) {
        out.cs0A_exp     = calculateSemChecksum(g_rxBuf, 2, g_rxLen - 5);
        out.cs0A_got     = g_rxBuf[g_rxLen - 3];
        out.cs0A_diffSub = (uint8_t)((out.cs0A_exp - out.cs0A_got) & 0xFF);

        size_t pos = 0;
        for (int i = 0; i < 24; i++) {
            size_t idx = 4 + (i * 2);
            uint16_t wh = ((uint16_t)g_rxBuf[idx] << 8) | g_rxBuf[idx + 1];
            out.sum24h_Wh += wh;
            if (i == 22) out.h22_Wh = wh;
            if (i == 23) out.h23_Wh = wh;
            if (wh > 0 && pos + 16 < sizeof(out.slots24h)) {
                pos += snprintf(out.slots24h + pos, sizeof(out.slots24h) - pos, "[%02d]=%u ", i, (unsigned)wh);
            }
        }
    }

    // 2. 0x0B00 (30j)
    static const uint8_t cmd0B00[9] = { 0x0F, 0x05, 0x0B, 0x00, 0x00, 0x00, 0x0C, 0xFF, 0xFF };
    if (sendCmdWait(pReq, cmd0B00, sizeof(cmd0B00), 0x0B00, 1200) && g_rxLen >= 127) {
        out.cs0B_exp     = calculateSemChecksum(g_rxBuf, 2, g_rxLen - 5);
        out.cs0B_got     = g_rxBuf[g_rxLen - 3];
        out.cs0B_diffSub = (uint8_t)((out.cs0B_exp - out.cs0B_got) & 0xFF);

        size_t pos = 0;
        for (int i = 0; i < 30; i++) {
            size_t idx = 4 + (i * 4);
            uint32_t wh = ((uint32_t)g_rxBuf[idx] << 16) | ((uint32_t)g_rxBuf[idx + 1] << 8) | g_rxBuf[idx + 2];
            out.sum30d_Wh += wh;
            if (i == 28) out.j28_Wh = wh;
            if (i == 29) out.j29_Wh = wh;
            if (wh > 0 && pos + 18 < sizeof(out.slots30d)) {
                pos += snprintf(out.slots30d + pos, sizeof(out.slots30d) - pos, "[%02d]=%lu ", i, (unsigned long)wh);
            }
        }
    }

    // 3. 0x0C00 (12m)
    static const uint8_t cmd0C00[9] = { 0x0F, 0x05, 0x0C, 0x00, 0x00, 0x00, 0x0D, 0xFF, 0xFF };
    if (sendCmdWait(pReq, cmd0C00, sizeof(cmd0C00), 0x0C00, 1200) && g_rxLen >= 55) {
        out.cs0C_exp     = calculateSemChecksum(g_rxBuf, 2, g_rxLen - 5);
        out.cs0C_got     = g_rxBuf[g_rxLen - 3];
        out.cs0C_diffSub = (uint8_t)((out.cs0C_exp - out.cs0C_got) & 0xFF);

        size_t pos = 0;
        for (int i = 0; i < 12; i++) {
            size_t idx = 4 + (i * 4);
            uint32_t wh = ((uint32_t)g_rxBuf[idx] << 16) | ((uint32_t)g_rxBuf[idx + 1] << 8) | g_rxBuf[idx + 2];
            uint8_t  b3 = g_rxBuf[idx + 3];
            out.sum12m_Wh += wh;
            if (i == 10) out.m10_Wh = wh;
            if (i == 11) out.m11_Wh = wh;
            if (wh > 0 && pos + 22 < sizeof(out.slots12m)) {
                pos += snprintf(out.slots12m + pos, sizeof(out.slots12m) - pos, "[%02d]=%lu(%u%%) ", i, (unsigned long)wh, (unsigned)b3);
            }
        }
    }

    Serial.println("\n  ================================================================================================================");
    Serial.printf("  📸 %s | RTC Mesurée (Sonde 24b) : %s | P=%.1f W | 0400_Wh=%lu\n",
                  out.title, out.measuredClock, out.powerW, (unsigned long)out.totalWh0400);
    Serial.println("  ----------------------------------------------------------------------------------------------------------------");
    Serial.printf("  🕒 0x0A00 (24h) | Cumul=%4lu Wh | CS_Exp=0x%02X | CS_Got=0x%02X | Écart(Exp-Got)=0x%02X (%3u) | Slots>0: %s\n",
                  (unsigned long)out.sum24h_Wh, (unsigned)out.cs0A_exp, (unsigned)out.cs0A_got,
                  (unsigned)out.cs0A_diffSub, (unsigned)out.cs0A_diffSub, out.slots24h);
    Serial.printf("  📅 0x0B00 (30j) | Cumul=%4lu Wh | CS_Exp=0x%02X | CS_Got=0x%02X | Écart(Exp-Got)=0x%02X (%3u) | Slots>0: %s\n",
                  (unsigned long)out.sum30d_Wh, (unsigned)out.cs0B_exp, (unsigned)out.cs0B_got,
                  (unsigned)out.cs0B_diffSub, (unsigned)out.cs0B_diffSub, out.slots30d);
    Serial.printf("  📆 0x0C00 (12m) | Cumul=%4lu Wh | CS_Exp=0x%02X | CS_Got=0x%02X | Écart(Exp-Got)=0x%02X (%3u) | Slots>0: %s\n",
                  (unsigned long)out.sum12m_Wh, (unsigned)out.cs0C_exp, (unsigned)out.cs0C_got,
                  (unsigned)out.cs0C_diffSub, (unsigned)out.cs0C_diffSub, out.slots12m);
    Serial.println("  ================================================================================================================");
}

// Exécute une transition de 25 secondes (de XX:59:45 à (XX+1):00:10) avec suivi toutes les 5 secondes
static void runBoundaryTransition25s(NimBLERemoteCharacteristic* pReq,
                                     const char* stepTitle,
                                     uint8_t startSec, uint8_t startMin, uint8_t startHour,
                                     uint8_t startDay, uint8_t startMonth, uint16_t startYear) {
    Serial.println("\n  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    Serial.printf("  ⏳ %s : Injection setDateTime(%02u/%02u/%04u %02u:%02u:%02u) + Suivi 25s...\n",
                  stepTitle,
                  (unsigned)startDay, (unsigned)startMonth, (unsigned)startYear,
                  (unsigned)startHour, (unsigned)startMin, (unsigned)startSec);

    sendSetDateTime(pReq, startSec, startMin, startHour, startDay, startMonth, startYear);
    uint32_t t0 = millis();

    for (int step = 1; step <= 5; step++) {
        uint32_t targetMs = (uint32_t)step * 5000UL;
        while (millis() - t0 < targetMs) {
            delay(50);
        }

        // Lecture rapide 0x0400 + 0x0A00 + 0x0B00 + 0x0C00
        float pW = 0.0f;
        uint32_t wh0400 = 0;
        uint16_t h21 = 0, h22 = 0, h23 = 0;
        uint8_t cs0A_e = 0, cs0A_g = 0;
        uint32_t j28 = 0, j29 = 0;
        uint8_t cs0B_e = 0, cs0B_g = 0;
        uint32_t m10 = 0, m11 = 0;
        uint8_t cs0C_e = 0, cs0C_g = 0;

        static const uint8_t cmd0400[9] = { 0x0F, 0x05, 0x04, 0x00, 0x00, 0x00, 0x05, 0xFF, 0xFF };
        if (sendCmdWait(pReq, cmd0400, sizeof(cmd0400), 0x0400, 800) && g_rxLen >= 19) {
            uint32_t rawP = ((uint32_t)g_rxBuf[5] << 16) | ((uint32_t)g_rxBuf[6] << 8) | g_rxBuf[7];
            pW = rawP / 1000.0f;
            wh0400 = ((uint32_t)g_rxBuf[14] << 24) | ((uint32_t)g_rxBuf[15] << 16) |
                     ((uint32_t)g_rxBuf[16] << 8)  | g_rxBuf[17];
        }

        static const uint8_t cmd0A00[9] = { 0x0F, 0x05, 0x0A, 0x00, 0x00, 0x00, 0x0B, 0xFF, 0xFF };
        if (sendCmdWait(pReq, cmd0A00, sizeof(cmd0A00), 0x0A00, 800) && g_rxLen >= 55) {
            cs0A_e = calculateSemChecksum(g_rxBuf, 2, g_rxLen - 5);
            cs0A_g = g_rxBuf[g_rxLen - 3];
            h21 = ((uint16_t)g_rxBuf[4 + 21*2] << 8) | g_rxBuf[4 + 21*2 + 1];
            h22 = ((uint16_t)g_rxBuf[4 + 22*2] << 8) | g_rxBuf[4 + 22*2 + 1];
            h23 = ((uint16_t)g_rxBuf[4 + 23*2] << 8) | g_rxBuf[4 + 23*2 + 1];
        }

        static const uint8_t cmd0B00[9] = { 0x0F, 0x05, 0x0B, 0x00, 0x00, 0x00, 0x0C, 0xFF, 0xFF };
        if (sendCmdWait(pReq, cmd0B00, sizeof(cmd0B00), 0x0B00, 800) && g_rxLen >= 127) {
            cs0B_e = calculateSemChecksum(g_rxBuf, 2, g_rxLen - 5);
            cs0B_g = g_rxBuf[g_rxLen - 3];
            j28 = ((uint32_t)g_rxBuf[4 + 28*4] << 16) | ((uint32_t)g_rxBuf[4 + 28*4 + 1] << 8) | g_rxBuf[4 + 28*4 + 2];
            j29 = ((uint32_t)g_rxBuf[4 + 29*4] << 16) | ((uint32_t)g_rxBuf[4 + 29*4 + 1] << 8) | g_rxBuf[4 + 29*4 + 2];
        }

        static const uint8_t cmd0C00[9] = { 0x0F, 0x05, 0x0C, 0x00, 0x00, 0x00, 0x0D, 0xFF, 0xFF };
        if (sendCmdWait(pReq, cmd0C00, sizeof(cmd0C00), 0x0C00, 800) && g_rxLen >= 55) {
            cs0C_e = calculateSemChecksum(g_rxBuf, 2, g_rxLen - 5);
            cs0C_g = g_rxBuf[g_rxLen - 3];
            m10 = ((uint32_t)g_rxBuf[4 + 10*4] << 16) | ((uint32_t)g_rxBuf[4 + 10*4 + 1] << 8) | g_rxBuf[4 + 10*4 + 2];
            m11 = ((uint32_t)g_rxBuf[4 + 11*4] << 16) | ((uint32_t)g_rxBuf[4 + 11*4 + 1] << 8) | g_rxBuf[4 + 11*4 + 2];
        }

        Serial.printf("     [+%2ds] P=%5.1fW | 0400=%lu | H[21..23]=%u/%u/%u (CS:%02X/%02X) | J[28..29]=%lu/%lu (CS:%02X/%02X) | M[10..11]=%lu/%lu (CS:%02X/%02X)\n",
                      step * 5, pW, (unsigned long)wh0400,
                      (unsigned)h21, (unsigned)h22, (unsigned)h23, (unsigned)cs0A_e, (unsigned)cs0A_g,
                      (unsigned long)j28, (unsigned long)j29, (unsigned)cs0B_e, (unsigned)cs0B_g,
                      (unsigned long)m10, (unsigned long)m11, (unsigned)cs0C_e, (unsigned)cs0C_g);
    }
}

static void runExp21() {
    Serial.println("\n########################################################################################################################");
    Serial.printf(" 🔬 EXPÉRIENCE 21 : MAÎTRISE DES ROTATIONS (HOUR / DAY / MONTH) & CHECKSUMS SUR VÉLO HW 3.0 (%s)\n", MAC_VELO_HW3);
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

    // 1. Authentification 32 bits
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

    // 2. Forcer le relais à ON pour que la charge marqueur tire du courant
    static const uint8_t cmdRelayOn[10] = { 0x0F, 0x06, 0x03, 0x00, 0x01, 0x00, 0x00, 0x04, 0xFF, 0xFF };
    sendCmdWait(pReq, cmdRelayOn, sizeof(cmdRelayOn), 0x0300, 800);

    FullAudit audits[4];

    // =========================================================================
    // CLICHÉ #0 : ÉTAT INITIAL À FROID (VÉRIFICATION DU PASSAGE NATUREL DE 22:00:00)
    // =========================================================================
    captureFullAudit(pReq, "CLICHÉ #0 : État Initial à Froid", 26, 11, 2026, audits[0]);

    // =========================================================================
    // TEST 1 : ROTATION HORAIRE (26/11/2026 22:59:45 -> 23:00:10)
    // =========================================================================
    runBoundaryTransition25s(pReq, "TEST 1 : ROTATION HORAIRE (22:59:45 -> 23:00:10)", 45, 59, 22, 26, 11, 2026);
    captureFullAudit(pReq, "CLICHÉ #1 : Après Rotation HORAIRE (23:00:10)", 26, 11, 2026, audits[1]);

    // =========================================================================
    // TEST 2 : ROTATION JOURNALIÈRE / MINUIT (26/11/2026 23:59:45 -> 27/11/2026 00:00:10)
    // =========================================================================
    runBoundaryTransition25s(pReq, "TEST 2 : ROTATION JOURNALIÈRE / MINUIT (26/11 23:59:45 -> 27/11 00:00:10)", 45, 59, 23, 26, 11, 2026);
    captureFullAudit(pReq, "CLICHÉ #2 : Après Rotation JOURNALIÈRE (27/11 00:00:10)", 27, 11, 2026, audits[2]);

    // =========================================================================
    // TEST 3 : ROTATION MENSUELLE / FIN DE MOIS (30/11/2026 23:59:45 -> 01/12/2026 00:00:10)
    // =========================================================================
    runBoundaryTransition25s(pReq, "TEST 3 : ROTATION MENSUELLE / FIN DE MOIS (30/11 23:59:45 -> 01/12 00:00:10)", 45, 59, 23, 30, 11, 2026);
    captureFullAudit(pReq, "CLICHÉ #3 : Après Rotation MENSUELLE (01/12 00:00:10)", 1, 12, 2026, audits[3]);

    // =========================================================================
    // TABLEAU SYNTHÉTIQUE FINAL DES 4 AUDITS
    // =========================================================================
    Serial.println("\n========================================================================================================================");
    Serial.println(" 📊 TABLEAU SYNTHÉTIQUE D'EXPÉRIENCE 21 : ROTATIONS HOUR / DAY / MONTH & CHECKSUMS (VÉLO HW 3.0)");
    Serial.println("========================================================================================================================");
    Serial.println(" Cliché                                         | RTC Sonde 24b       | 0400_Wh | 0x0A00 (24h: Exp/Got/Δ) | 0x0B00 (30j: Exp/Got/Δ) | 0x0C00 (12m: Exp/Got/Δ)");
    Serial.println("------------------------------------------------+---------------------+---------+-------------------------+-------------------------+------------------------");
    for (int i = 0; i < 4; i++) {
        Serial.printf(" %-46s | %-19s | %7lu | %3luWh: %02X / %02X (-0x%02X) | %3luWh: %02X / %02X (-0x%02X) | %4luWh: %02X / %02X (-0x%02X)\n",
                      audits[i].title,
                      audits[i].measuredClock,
                      (unsigned long)audits[i].totalWh0400,
                      (unsigned long)audits[i].sum24h_Wh, (unsigned)audits[i].cs0A_exp, (unsigned)audits[i].cs0A_got, (unsigned)audits[i].cs0A_diffSub,
                      (unsigned long)audits[i].sum30d_Wh, (unsigned)audits[i].cs0B_exp, (unsigned)audits[i].cs0B_got, (unsigned)audits[i].cs0B_diffSub,
                      (unsigned long)audits[i].sum12m_Wh, (unsigned)audits[i].cs0C_exp, (unsigned)audits[i].cs0C_got, (unsigned)audits[i].cs0C_diffSub);
    }
    Serial.println("========================================================================================================================");

    pClient->disconnect();
    NimBLEDevice::deleteClient(pClient);
    Serial.println("\n[BLE] ✅ Expérience 21 terminée et déconnectée.");
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
    NimBLEDevice::init("Gateway_Exp21");

    Serial.println("\n\n========================================================================================================================");
    Serial.println("  🚀 EXPÉRIENCE 21 : MAÎTRISE DES ROTATIONS (HOUR / DAY / MONTH) SOUS CHARGE MARQUEUR (HW 3.0)");
    Serial.println("========================================================================================================================");

    waitUserPrompt("LANCEMENT EXPÉRIENCE 21 (AVEC CHARGE MARQUEUR BRANCHÉE SUR VÉLO)",
                   "Branchez votre charge sur 'Vélo' pour générer des marqueurs Wh pendant les transitions.\n"
                   "Appuyez sur BOOT ou 'Entrée' pour lancer les 3 rotations (Horaire, Minuit, Fin de Mois ~80 s).");

    runExp21();

    Serial.println("\n========================================================================================================================");
    Serial.println("  🎉 EXPÉRIENCE 21 TERMINÉE ! (Appuyez sur BOOT ou 'Entrée' pour relancer)");
    Serial.println("========================================================================================================================\n");
}

void loop() {
    if (digitalRead(BUTTON_BOOT_PIN) == LOW) {
        delay(150);
        while (digitalRead(BUTTON_BOOT_PIN) == LOW) delay(20);
        runExp21();
    }
    if (Serial.available()) {
        while (Serial.available()) Serial.read();
        runExp21();
    }
    delay(50);
}
