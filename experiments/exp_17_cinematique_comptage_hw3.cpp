/**
 * @file exp_17_cinematique_comptage_hw3.cpp
 * @brief Expérience 17 : Cinématique temps réel du comptage d'énergie sur Hardware 3.0 (FW 1.30).
 *
 * Objectifs :
 * 1. Observer ligne par ligne, côte à côte et à haute cadence (~1 Hz) pendant 75 secondes :
 *    - Puissance active P (W) & Courant I (mA) & Compteur 0x0400 (totalKwhWh)
 *    - Heure précédente H-1 (hourlyWh[22]) & Heure courante H (hourlyWh[23]) via 0x0A00
 *    - Jour précédent J-1 (dailyWh[28]) & Jour courant J (dailyWh[29] + ratio B3) via 0x0B00
 *    - Mois précédent M-1 (monthlyWh[10]) & Mois courant M (monthlyWh[11] + ratio B3) via 0x0C00
 * 2. Déterminer si le report de l'énergie consommée vers le Jour courant (0x0B00) et le Mois courant (0x0C00)
 *    sur HW 3.0 / FW 1.30 s'effectue :
 *    - Au fil de l'eau à chaque +1 Wh franchi ("pas de conso, pas de calcul"),
 *    - Sur une bascule de minute pile (XX:XX:00) ou période de 60 secondes,
 *    - Ou uniquement à l'heure pile (XX:00:00).
 * 3. Phase 2 (optionnelle sur appui BOOT/Entrée) : Test de franchissement d'heure pile (17:59:50 -> 18:00:15, 25 s)
 *    pour observer la rotation H[23] -> H[22] et vérifier l'absence de double comptage dans J[29] et M[11].
 */

#include <Arduino.h>
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <cstring>

static const int      BUTTON_BOOT_PIN = 0;
static const char*    MAC_VELO_HW3    = "a3:00:00:00:45:25";

static const NimBLEUUID SERVICE_FFF0_UUID("0000fff0-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_REQ_FFF3_UUID("0000fff3-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_RES_FFF4_UUID("0000fff4-0000-1000-8000-00805f9b34fb");

// Buffer de réception asynchrone rapide sans rejet de checksum sur 0x0C00
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
        // Cas spécial 0x0400 : 19 octets sans FF FF final
        if (cmd == 0x0400 && g_rxLen >= 19) {
            g_rxCmd      = cmd;
            g_rxComplete = true;
            return;
        }
        // Cas général : longueur logique = g_rxBuf[1] + 4 (supporte le padding 00 00 de 0x1000)
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

static bool sendFastCmd(NimBLERemoteCharacteristic* pReq,
                        const uint8_t* frame,
                        size_t frameLen,
                        uint16_t expectedCmd,
                        uint32_t timeoutMs = 1000) {
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
        delay(2);
    }
    return false;
}

struct FastSnapshot {
    uint32_t elapsedMs;
    uint32_t pollDurationMs;
    bool     ok0400;
    bool     ok0A00;
    bool     ok0B00;
    bool     ok0C00;

    // 0x0400
    bool     relayOn;
    float    powerW;
    uint8_t  voltageV;
    uint16_t currentmA;
    uint16_t word12_13;
    uint32_t totalWh0400;

    // 0x0A00 (24 heures)
    uint16_t hour21_Wh;
    uint16_t hour22_Wh; // H-1
    uint16_t hour23_Wh; // Heure courante
    uint32_t sum24h_Wh;
    uint8_t  cs0A_exp;
    uint8_t  cs0A_got;

    // 0x0B00 (30 jours)
    uint32_t day28_Wh;  // J-1
    uint8_t  day28_B3;
    uint32_t day29_Wh;  // Jour courant
    uint8_t  day29_B3;
    uint32_t sum30d_Wh;
    uint8_t  cs0B_exp;
    uint8_t  cs0B_got;

    // 0x0C00 (12 mois)
    uint32_t month10_Wh; // M-1
    uint8_t  month10_B3;
    uint32_t month11_Wh; // Mois courant
    uint8_t  month11_B3;
    uint32_t sum12m_Wh;
    uint8_t  cs0C_exp;
    uint8_t  cs0C_got;
};

static void captureFastSnapshot(NimBLERemoteCharacteristic* pReq, uint32_t tStartMs, FastSnapshot& s) {
    memset(&s, 0, sizeof(s));
    uint32_t tPoll0 = millis();
    s.elapsedMs = tPoll0 - tStartMs;

    // 1. Télémétrie 0x0400
    static const uint8_t cmd0400[9] = { 0x0F, 0x05, 0x04, 0x00, 0x00, 0x00, 0x05, 0xFF, 0xFF };
    if (sendFastCmd(pReq, cmd0400, sizeof(cmd0400), 0x0400, 800) && g_rxLen >= 19) {
        s.ok0400      = true;
        s.relayOn     = (g_rxBuf[4] == 0x01);
        uint32_t rawP = ((uint32_t)g_rxBuf[5] << 16) | ((uint32_t)g_rxBuf[6] << 8) | g_rxBuf[7];
        s.powerW      = rawP / 1000.0f;
        s.voltageV    = g_rxBuf[8];
        s.currentmA   = ((uint16_t)g_rxBuf[9] << 8) | g_rxBuf[10];
        s.word12_13   = ((uint16_t)g_rxBuf[12] << 8) | g_rxBuf[13];
        s.totalWh0400 = ((uint32_t)g_rxBuf[14] << 24) | ((uint32_t)g_rxBuf[15] << 16) |
                        ((uint32_t)g_rxBuf[16] << 8)  | g_rxBuf[17];
    }

    // 2. Historique 24h (0x0A00)
    static const uint8_t cmd0A00[9] = { 0x0F, 0x05, 0x0A, 0x00, 0x00, 0x00, 0x0B, 0xFF, 0xFF };
    if (sendFastCmd(pReq, cmd0A00, sizeof(cmd0A00), 0x0A00, 800) && g_rxLen >= 55) {
        s.ok0A00   = true;
        s.cs0A_exp = calculateSemChecksum(g_rxBuf, 2, g_rxLen - 5);
        s.cs0A_got = g_rxBuf[g_rxLen - 3];
        s.sum24h_Wh = 0;
        for (int i = 0; i < 24; i++) {
            size_t idx = 4 + (i * 2);
            uint16_t val = ((uint16_t)g_rxBuf[idx] << 8) | g_rxBuf[idx + 1];
            s.sum24h_Wh += val;
            if (i == 21) s.hour21_Wh = val;
            if (i == 22) s.hour22_Wh = val;
            if (i == 23) s.hour23_Wh = val;
        }
    }

    // 3. Historique 30j (0x0B00)
    static const uint8_t cmd0B00[9] = { 0x0F, 0x05, 0x0B, 0x00, 0x00, 0x00, 0x0C, 0xFF, 0xFF };
    if (sendFastCmd(pReq, cmd0B00, sizeof(cmd0B00), 0x0B00, 800) && g_rxLen >= 127) {
        s.ok0B00   = true;
        s.cs0B_exp = calculateSemChecksum(g_rxBuf, 2, g_rxLen - 5);
        s.cs0B_got = g_rxBuf[g_rxLen - 3];
        s.sum30d_Wh = 0;
        for (int i = 0; i < 30; i++) {
            size_t idx = 4 + (i * 4);
            uint32_t val = ((uint32_t)g_rxBuf[idx] << 16) | ((uint32_t)g_rxBuf[idx + 1] << 8) | g_rxBuf[idx + 2];
            uint8_t  b3  = g_rxBuf[idx + 3];
            s.sum30d_Wh += val;
            if (i == 28) { s.day28_Wh = val; s.day28_B3 = b3; }
            if (i == 29) { s.day29_Wh = val; s.day29_B3 = b3; }
        }
    }

    // 4. Historique 12m (0x0C00)
    static const uint8_t cmd0C00[9] = { 0x0F, 0x05, 0x0C, 0x00, 0x00, 0x00, 0x0D, 0xFF, 0xFF };
    if (sendFastCmd(pReq, cmd0C00, sizeof(cmd0C00), 0x0C00, 800) && g_rxLen >= 55) {
        s.ok0C00   = true;
        s.cs0C_exp = calculateSemChecksum(g_rxBuf, 2, g_rxLen - 5);
        s.cs0C_got = g_rxBuf[g_rxLen - 3];
        s.sum12m_Wh = 0;
        for (int i = 0; i < 12; i++) {
            size_t idx = 4 + (i * 4);
            uint32_t val = ((uint32_t)g_rxBuf[idx] << 16) | ((uint32_t)g_rxBuf[idx + 1] << 8) | g_rxBuf[idx + 2];
            uint8_t  b3  = g_rxBuf[idx + 3];
            s.sum12m_Wh += val;
            if (i == 10) { s.month10_Wh = val; s.month10_B3 = b3; }
            if (i == 11) { s.month11_Wh = val; s.month11_B3 = b3; }
        }
    }

    s.pollDurationMs = millis() - tPoll0;
}

static void waitUserPrompt(const char* title, const char* instructions) {
    Serial.println("\n========================================================================================");
    Serial.printf(" >>> %s\n", title);
    Serial.println("----------------------------------------------------------------------------------------");
    Serial.println(instructions);
    Serial.println("\n  👉 [Appuyez sur le bouton physique BOOT (GPIO 0) ou envoyez 'Entrée' en console]");
    Serial.println("========================================================================================\n");

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

static void runContinuousTrackingSession(uint8_t startHour,
                                         uint8_t startMin,
                                         uint8_t startSec,
                                         uint32_t durationSec,
                                         const char* phaseTitle) {
    Serial.println("\n########################################################################################################################");
    Serial.printf(" 🔬 %s (Durée: %lu s | Horloge injectée: %02u:%02u:%02u le 25/11/2026)\n",
                  phaseTitle, (unsigned long)durationSec,
                  (unsigned)startHour, (unsigned)startMin, (unsigned)startSec);
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

    // 1. Authentification PIN 00000000
    uint8_t cmdAuth[16] = { 0x0F, 0x0C, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xFF, 0xFF };
    if (!sendFastCmd(pReq, cmdAuth, sizeof(cmdAuth), 0x1700, 1000)) {
        Serial.println(" ❌ Échec authentification !");
        pClient->disconnect();
        NimBLEDevice::deleteClient(pClient);
        return;
    }

    // 2. Forcer le relais à ON au cas où
    uint8_t cmdRelayOn[10] = { 0x0F, 0x06, 0x03, 0x00, 0x01, 0x00, 0x00, 0x04, 0xFF, 0xFF };
    sendFastCmd(pReq, cmdRelayOn, sizeof(cmdRelayOn), 0x0300, 800);

    // 3. Snapshot AVANT setDateTime
    FastSnapshot snapPre;
    captureFastSnapshot(pReq, millis(), snapPre);
    Serial.printf(" [AVANT setDateTime] P=%.1f W (%u mA) | 0400_Wh=%lu | H[22]=%u H[23]=%u (Sum24=%lu) | J[28]=%lu J[29]=%lu (Sum30=%lu) | M[10]=%lu M[11]=%lu (Sum12=%lu)\n",
                  snapPre.powerW, (unsigned)snapPre.currentmA, (unsigned long)snapPre.totalWh0400,
                  (unsigned)snapPre.hour22_Wh, (unsigned)snapPre.hour23_Wh, (unsigned long)snapPre.sum24h_Wh,
                  (unsigned long)snapPre.day28_Wh, (unsigned long)snapPre.day29_Wh, (unsigned long)snapPre.sum30d_Wh,
                  (unsigned long)snapPre.month10_Wh, (unsigned long)snapPre.month11_Wh, (unsigned long)snapPre.sum12m_Wh);
    Serial.printf("                     Checksums -> 0A00: Exp=0x%02X Got=0x%02X | 0B00: Exp=0x%02X Got=0x%02X | 0C00: Exp=0x%02X Got=0x%02X (Diff=0x%02X)\n\n",
                  (unsigned)snapPre.cs0A_exp, (unsigned)snapPre.cs0A_got,
                  (unsigned)snapPre.cs0B_exp, (unsigned)snapPre.cs0B_got,
                  (unsigned)snapPre.cs0C_exp, (unsigned)snapPre.cs0C_got,
                  (unsigned)((snapPre.cs0C_exp - snapPre.cs0C_got) & 0xFF));

    // 4. Injection de l'horloge de référence : 25/11/2026 à startHour:startMin:startSec
    uint16_t year = 2026;
    uint8_t cmdDt[16] = {
        0x0F, 0x0C, 0x01, 0x00,
        startSec, startMin, startHour, 25, 11,
        (uint8_t)(year >> 8), (uint8_t)(year & 0xFF),
        0x00, 0x00, 0x00, 0xFF, 0xFF
    };
    cmdDt[13] = calculateSemChecksum(cmdDt, 2, 11);
    sendFastCmd(pReq, cmdDt, sizeof(cmdDt), 0x0100, 800);

    uint32_t tStart = millis();
    uint32_t baseTotalSec = (uint32_t)startHour * 3600UL + (uint32_t)startMin * 60UL + (uint32_t)startSec;

    Serial.println("  #  |  t (s) | Horloge Plug |  P (W)  | I (mA) | 0400_Wh | H-1[22] | Heure[23] | Jour[29] (HC%) | Mois[11] (HC%) | Poll | Événement Détecté");
    Serial.println("-----+--------+--------------+---------+--------+---------+---------+-----------+----------------+----------------+------+----------------------------------");

    FastSnapshot prev;
    bool hasPrev = false;
    uint32_t sampleIdx = 0;

    while ((millis() - tStart) < (durationSec * 1000UL)) {
        uint32_t loopT0 = millis();
        FastSnapshot cur;
        captureFastSnapshot(pReq, tStart, cur);
        sampleIdx++;

        uint32_t simSecTotal = (baseTotalSec + (cur.elapsedMs / 1000UL)) % 86400UL;
        uint8_t  simH = (uint8_t)(simSecTotal / 3600UL);
        uint8_t  simM = (uint8_t)((simSecTotal % 3600UL) / 60UL);
        uint8_t  simS = (uint8_t)(simSecTotal % 60UL);

        char eventBuf[96] = "-";
        if (hasPrev) {
            int32_t dH22  = (int32_t)cur.hour22_Wh   - (int32_t)prev.hour22_Wh;
            int32_t dH23  = (int32_t)cur.hour23_Wh   - (int32_t)prev.hour23_Wh;
            int32_t dJ29  = (int32_t)cur.day29_Wh    - (int32_t)prev.day29_Wh;
            int32_t dM11  = (int32_t)cur.month11_Wh  - (int32_t)prev.month11_Wh;
            int32_t d0400 = (int32_t)cur.totalWh0400 - (int32_t)prev.totalWh0400;

            if (dH22 != 0 || dH23 != 0 || dJ29 != 0 || dM11 != 0 || d0400 != 0) {
                snprintf(eventBuf, sizeof(eventBuf),
                         "⚡ ΔH23=%+ld ΔJ29=%+ld ΔM11=%+ld (ΔH22=%+ld Δ0400=%+ld)",
                         (long)dH23, (long)dJ29, (long)dM11, (long)dH22, (long)d0400);
            } else if (simS == 0) {
                snprintf(eventBuf, sizeof(eventBuf), "⏱️ Passage minute pile (%02u:%02u:00)", (unsigned)simH, (unsigned)simM);
            }
        } else {
            snprintf(eventBuf, sizeof(eventBuf), "🏁 État initial t=0");
        }

        Serial.printf(" %3lu | %5.1fs |   %02u:%02u:%02u   | %6.1fW | %5umA | %7lu | %5uWh |   %5uWh |   %6luWh(%2u%%) |   %6luWh(%2u%%) | %3lums| %s\n",
                      (unsigned long)sampleIdx,
                      cur.elapsedMs / 1000.0f,
                      (unsigned)simH, (unsigned)simM, (unsigned)simS,
                      cur.powerW,
                      (unsigned)cur.currentmA,
                      (unsigned long)cur.totalWh0400,
                      (unsigned)cur.hour22_Wh,
                      (unsigned)cur.hour23_Wh,
                      (unsigned long)cur.day29_Wh, (unsigned)cur.day29_B3,
                      (unsigned long)cur.month11_Wh, (unsigned)cur.month11_B3,
                      (unsigned long)cur.pollDurationMs,
                      eventBuf);

        prev = cur;
        hasPrev = true;

        // Cadencement à ~1 ligne par seconde (si le poll de 4 trames a pris ~150 ms, on attend ~850 ms)
        uint32_t spent = millis() - loopT0;
        if (spent < 1000) {
            delay(1000 - spent);
        }
    }

    Serial.println("-----+--------+--------------+---------+--------+---------+---------+-----------+----------------+----------------+------+----------------------------------");
    Serial.printf(" [FIN SESSION] Checksums finaux -> 0A00: Exp=0x%02X Got=0x%02X | 0B00: Exp=0x%02X Got=0x%02X | 0C00: Exp=0x%02X Got=0x%02X (Diff=0x%02X)\n",
                  (unsigned)prev.cs0A_exp, (unsigned)prev.cs0A_got,
                  (unsigned)prev.cs0B_exp, (unsigned)prev.cs0B_got,
                  (unsigned)prev.cs0C_exp, (unsigned)prev.cs0C_got,
                  (unsigned)((prev.cs0C_exp - prev.cs0C_got) & 0xFF));

    pClient->disconnect();
    NimBLEDevice::deleteClient(pClient);
    Serial.println("########################################################################################################################\n");
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
    NimBLEDevice::init("Gateway_Exp17");

    Serial.println("\n\n========================================================================================================================");
    Serial.println("  🚀 EXPÉRIENCE 17 : CINÉMATIQUE TEMPS RÉEL HEURE / JOUR / MOIS SUR VÉLO (HW 3.0 / FW 1.30)");
    Serial.println("========================================================================================================================");

    // VOLET 1 : 75 secondes en milieu d'heure (17:30:50 -> 17:32:05)
    // Franchit DEUX minutes piles (17:31:00 à t=+10s et 17:32:00 à t=+70s) et ~10 incréments de +1 Wh
    waitUserPrompt("VOLET 1 : TRACKING HAUTE CADENCE INTRA-HEURE (75 SECONDES : 17:30:50 -> 17:32:05)",
                   "Assurez-vous que votre charge (~480 W) est bien branchée sur 'Vélo' (a3:00:00:00:45:25).\n"
                   "L'horloge sera calée à 17:30:50 (franchissement de 17:31:00 à +10s et 17:32:00 à +70s).\n"
                   "Affichage côte à côte 1x/seconde : P, I, 0400_Wh, H-1[22], Heure[23], Jour[29], Mois[11] !");

    runContinuousTrackingSession(17, 30, 50, 75, "VOLET 1 : TRACKING INTRA-HEURE (75s : 17:30:50 -> 17:32:05)");

    // VOLET 2 (Optionnel) : 25 secondes autour de l'heure pile (17:59:50 -> 18:00:15)
    waitUserPrompt("VOLET 2 : TRACKING FRANCHISSEMENT D'HEURE PILE (25 SECONDES : 17:59:50 -> 18:00:15)",
                   "Appuyez sur BOOT ou 'Entrée' pour observer la bascule de l'heure pile 17:59:59 -> 18:00:00\n"
                   "sur Vélo (HW 3.0) et vérifier comment réagissent H-1[22], Heure[23], Jour[29] et Mois[11] !");

    runContinuousTrackingSession(17, 59, 50, 25, "VOLET 2 : FRANCHISSEMENT HEURE PILE (25s : 17:59:50 -> 18:00:15)");

    Serial.println("\n========================================================================================================================");
    Serial.println("  🎉 EXPÉRIENCE 17 TERMINÉE ! (Appuyez sur BOOT ou 'Entrée' pour relancer le Volet 1)");
    Serial.println("========================================================================================================================\n");
}

void loop() {
    if (digitalRead(BUTTON_BOOT_PIN) == LOW) {
        delay(150);
        while (digitalRead(BUTTON_BOOT_PIN) == LOW) delay(20);
        runContinuousTrackingSession(17, 30, 50, 75, "VOLET 1 (RELANCE) : TRACKING INTRA-HEURE (75s)");
    }
    if (Serial.available()) {
        while (Serial.available()) Serial.read();
        runContinuousTrackingSession(17, 30, 50, 75, "VOLET 1 (RELANCE) : TRACKING INTRA-HEURE (75s)");
    }
    delay(50);
}
