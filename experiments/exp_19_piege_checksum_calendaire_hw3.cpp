/**
 * @file exp_19_piege_checksum_calendaire_hw3.cpp
 * @brief Expérience 19 : Démonstration expérimentale et prédiction du bug de Checksum Calendaire
 *        du Firmware 1.30 (Hardware 3.0) sur les 3 historiques glissants :
 *        - 0x0A00 (24 heures)
 *        - 0x0B00 (30 jours)
 *        - 0x0C00 (12 mois)
 *
 * Protocole à puissance nulle (P = 0 W) sur "Vélo / Wattmètre mobile" (a3:00:00:00:45:25) :
 * - Cliché #0 : État initial intact (26/11/2026) avant toute écriture d'horloge.
 * - Cliché #1 : Phase 1 — Envoi direct de setDateTime(25/11/2026 13:30:00) sans franchissement de frontière 59:59 -> 00:00.
 * - Cliché #2 : Phase 2A — Avant passage de Minuit (setDateTime 26/11/2026 23:59:50).
 * - Cliché #3 : Phase 2A — Après passage naturel de Minuit (26/11/2026 23:59:58 -> attente 4s -> 27/11/2026 00:00:02).
 *               -> Prédiction 0x0A00 : Les 9 Wh passent dans "Hier", Aujourd'hui=0 Wh => CS_Reçu(0x0A00) tombe à 0x0B (0x01 + 0x0A + 0x00) !
 * - Cliché #4 : Phase 2B — Avant passage de Fin de Mois (setDateTime 30/11/2026 23:59:50).
 * - Cliché #5 : Phase 2B — Après passage naturel de Fin de Mois (30/11/2026 23:59:58 -> attente 4s -> 01/12/2026 00:00:02).
 *               -> Prédiction 0x0B00 : Les 45 Wh et 9 Wh passent dans "Mois-1 (Novembre)", Décembre=0 Wh => Le bug apparaît sur 0x0B00 !
 *                  CS_Reçu(0x0B00) quitte 0x42 et tombe à 0x0C (0x01 + 0x0B + 0x00) !
 * - Cliché #6 : Phase 2C — Après passage naturel du Nouvel An (31/12/2026 23:59:58 -> attente 4s -> 01/01/2027 00:00:02).
 *               -> Observation de CS_Reçu(0x0C00) lorsque tous les mois non nuls basculent dans l'année précédente (2026) et que 2027 est à 0 Wh !
 * - Restauration finale : Remise de l'horloge à 26/11/2026 15:45:00.
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
        delay(5);
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

struct HistoryAudit {
    char     label[64];
    char     clockContext[32];

    // 0x0A00 (24h)
    uint32_t sum24h_Wh;
    uint8_t  cs0A_exp;
    uint8_t  cs0A_got;
    uint8_t  cs0A_diffSub; // (Exp - Got) & 0xFF
    uint8_t  cs0A_diffAdd; // (Got - Exp) & 0xFF
    char     nonZero24h[96];

    // 0x0B00 (30j)
    uint32_t sum30d_Wh;
    uint8_t  cs0B_exp;
    uint8_t  cs0B_got;
    uint8_t  cs0B_diffSub;
    uint8_t  cs0B_diffAdd;
    char     nonZero30d[96];

    // 0x0C00 (12m)
    uint32_t sum12m_Wh;
    uint8_t  cs0C_exp;
    uint8_t  cs0C_got;
    uint8_t  cs0C_diffSub;
    uint8_t  cs0C_diffAdd;
    char     nonZero12m[128];
};

static void captureHistoryAudit(NimBLERemoteCharacteristic* pReq,
                                const char* label,
                                const char* clockContext,
                                HistoryAudit& out) {
    memset(&out, 0, sizeof(out));
    strncpy(out.label, label, sizeof(out.label) - 1);
    strncpy(out.clockContext, clockContext, sizeof(out.clockContext) - 1);

    Serial.println("\n  ================================================================================================================");
    Serial.printf("  📸 %s  [Horloge Prise : %s]\n", label, clockContext);
    Serial.println("  ----------------------------------------------------------------------------------------------------------------");

    // 1. 0x0A00 (24 heures)
    static const uint8_t cmd0A00[9] = { 0x0F, 0x05, 0x0A, 0x00, 0x00, 0x00, 0x0B, 0xFF, 0xFF };
    if (sendCmdWait(pReq, cmd0A00, sizeof(cmd0A00), 0x0A00, 1200) && g_rxLen >= 55) {
        out.cs0A_exp     = calculateSemChecksum(g_rxBuf, 2, g_rxLen - 5);
        out.cs0A_got     = g_rxBuf[g_rxLen - 3];
        out.cs0A_diffSub = (uint8_t)((out.cs0A_exp - out.cs0A_got) & 0xFF);
        out.cs0A_diffAdd = (uint8_t)((out.cs0A_got - out.cs0A_exp) & 0xFF);

        size_t pos = 0;
        for (int i = 0; i < 24; i++) {
            size_t idx = 4 + (i * 2);
            uint16_t wh = ((uint16_t)g_rxBuf[idx] << 8) | g_rxBuf[idx + 1];
            out.sum24h_Wh += wh;
            if (wh > 0 && pos + 16 < sizeof(out.nonZero24h)) {
                pos += snprintf(out.nonZero24h + pos, sizeof(out.nonZero24h) - pos, "[%02d]=%u ", i, (unsigned)wh);
            }
        }
        if (pos == 0) strncpy(out.nonZero24h, "(tous à 0)", sizeof(out.nonZero24h) - 1);

        Serial.printf("  🕒 0x0A00 (24h) | Cumul=%4lu Wh | CS_Exp=0x%02X | CS_Got=0x%02X | Écart (Exp-Got)=0x%02X (%3u) | Slots>0: %s\n",
                      (unsigned long)out.sum24h_Wh,
                      (unsigned)out.cs0A_exp, (unsigned)out.cs0A_got,
                      (unsigned)out.cs0A_diffSub, (unsigned)out.cs0A_diffSub,
                      out.nonZero24h);
    }

    // 2. 0x0B00 (30 jours)
    static const uint8_t cmd0B00[9] = { 0x0F, 0x05, 0x0B, 0x00, 0x00, 0x00, 0x0C, 0xFF, 0xFF };
    if (sendCmdWait(pReq, cmd0B00, sizeof(cmd0B00), 0x0B00, 1200) && g_rxLen >= 127) {
        out.cs0B_exp     = calculateSemChecksum(g_rxBuf, 2, g_rxLen - 5);
        out.cs0B_got     = g_rxBuf[g_rxLen - 3];
        out.cs0B_diffSub = (uint8_t)((out.cs0B_exp - out.cs0B_got) & 0xFF);
        out.cs0B_diffAdd = (uint8_t)((out.cs0B_got - out.cs0B_exp) & 0xFF);

        size_t pos = 0;
        for (int i = 0; i < 30; i++) {
            size_t idx = 4 + (i * 4);
            uint32_t wh = ((uint32_t)g_rxBuf[idx] << 16) | ((uint32_t)g_rxBuf[idx + 1] << 8) | g_rxBuf[idx + 2];
            out.sum30d_Wh += wh;
            if (wh > 0 && pos + 18 < sizeof(out.nonZero30d)) {
                pos += snprintf(out.nonZero30d + pos, sizeof(out.nonZero30d) - pos, "[%02d]=%lu ", i, (unsigned long)wh);
            }
        }
        if (pos == 0) strncpy(out.nonZero30d, "(tous à 0)", sizeof(out.nonZero30d) - 1);

        Serial.printf("  📅 0x0B00 (30j) | Cumul=%4lu Wh | CS_Exp=0x%02X | CS_Got=0x%02X | Écart (Exp-Got)=0x%02X (%3u) | Slots>0: %s\n",
                      (unsigned long)out.sum30d_Wh,
                      (unsigned)out.cs0B_exp, (unsigned)out.cs0B_got,
                      (unsigned)out.cs0B_diffSub, (unsigned)out.cs0B_diffSub,
                      out.nonZero30d);
    }

    // 3. 0x0C00 (12 mois)
    static const uint8_t cmd0C00[9] = { 0x0F, 0x05, 0x0C, 0x00, 0x00, 0x00, 0x0D, 0xFF, 0xFF };
    if (sendCmdWait(pReq, cmd0C00, sizeof(cmd0C00), 0x0C00, 1200) && g_rxLen >= 55) {
        out.cs0C_exp     = calculateSemChecksum(g_rxBuf, 2, g_rxLen - 5);
        out.cs0C_got     = g_rxBuf[g_rxLen - 3];
        out.cs0C_diffSub = (uint8_t)((out.cs0C_exp - out.cs0C_got) & 0xFF);
        out.cs0C_diffAdd = (uint8_t)((out.cs0C_got - out.cs0C_exp) & 0xFF);

        size_t pos = 0;
        for (int i = 0; i < 12; i++) {
            size_t idx = 4 + (i * 4);
            uint32_t wh = ((uint32_t)g_rxBuf[idx] << 16) | ((uint32_t)g_rxBuf[idx + 1] << 8) | g_rxBuf[idx + 2];
            uint8_t  b3 = g_rxBuf[idx + 3];
            out.sum12m_Wh += wh;
            if (wh > 0 && pos + 20 < sizeof(out.nonZero12m)) {
                pos += snprintf(out.nonZero12m + pos, sizeof(out.nonZero12m) - pos, "[%02d]=%lu(%u%%) ", i, (unsigned long)wh, (unsigned)b3);
            }
        }
        if (pos == 0) strncpy(out.nonZero12m, "(tous à 0)", sizeof(out.nonZero12m) - 1);

        Serial.printf("  📆 0x0C00 (12m) | Cumul=%4lu Wh | CS_Exp=0x%02X | CS_Got=0x%02X | Écart (Exp-Got)=0x%02X (%3u) | Slots>0: %s\n",
                      (unsigned long)out.sum12m_Wh,
                      (unsigned)out.cs0C_exp, (unsigned)out.cs0C_got,
                      (unsigned)out.cs0C_diffSub, (unsigned)out.cs0C_diffSub,
                      out.nonZero12m);
    }
}

static void runExp19() {
    Serial.println("\n########################################################################################################################");
    Serial.printf(" 🧪 EXPÉRIENCE 19 : PIÉGEAGE & PRÉDICTION DU CHECKSUM CALENDAIRE SUR VÉLO HW 3.0 (%s)\n", MAC_VELO_HW3);
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

    HistoryAudit audits[7];

    // =========================================================================
    // CLICHÉ #0 : ÉTAT INITIAL INTACT (26/11/2026)
    // =========================================================================
    captureHistoryAudit(pReq, "CLICHÉ #0 : État Initial Intact", "26/11/2026 ~13h30", audits[0]);

    // =========================================================================
    // CLICHÉ #1 : PHASE 1 — SETDATETIME DIRECT SANS FRANCHISSEMENT (25/11/2026 13:30:00)
    // =========================================================================
    Serial.println("\n  >>> [Action Phase 1] Injection directe setDateTime(25/11/2026 13:30:00) SANS franchir 59:59 -> 00:00...");
    sendSetDateTime(pReq, 0, 30, 13, 25, 11, 2026);
    delay(300);
    captureHistoryAudit(pReq, "CLICHÉ #1 : Après setDateTime direct (sans bascule)", "25/11/2026 13:30:00", audits[1]);

    // =========================================================================
    // CLICHÉS #2 & #3 : PHASE 2A — BASCULE DE MINUIT (26/11 23:59:58 -> 27/11 00:00:02)
    // =========================================================================
    Serial.println("\n  >>> [Action Phase 2A-1] Positionnement à 10s de Minuit : setDateTime(26/11/2026 23:59:50)...");
    sendSetDateTime(pReq, 50, 59, 23, 26, 11, 2026);
    delay(300);
    captureHistoryAudit(pReq, "CLICHÉ #2 : AVANT Minuit (26/11/2026)", "26/11/2026 23:59:50", audits[2]);

    Serial.println("\n  >>> [Action Phase 2A-2] Armement à 2s de Minuit : setDateTime(26/11/2026 23:59:58) + Attente 4s pour franchir 00:00:00...");
    sendSetDateTime(pReq, 58, 59, 23, 26, 11, 2026);
    delay(4200);
    captureHistoryAudit(pReq, "CLICHÉ #3 : APRÈS Minuit (27/11/2026)", "27/11/2026 00:00:02", audits[3]);
    Serial.printf("  🎯 VÉRIFICATION PRÉDICTION TEST A (0x0A00) : CS_Got attendu = 0x0B | CS_Got observé = 0x%02X -> %s\n",
                  (unsigned)audits[3].cs0A_got,
                  (audits[3].cs0A_got == 0x0B) ? "✅ PRÉDICTION VALIDÉE À 100 % !" : "🔍 Autre valeur observée");

    // =========================================================================
    // CLICHÉS #4 & #5 : PHASE 2B — BASCULE DE FIN DE MOIS (30/11 23:59:58 -> 01/12 00:00:02)
    // =========================================================================
    Serial.println("\n  >>> [Action Phase 2B-1] Positionnement à 10s de la Fin de Mois : setDateTime(30/11/2026 23:59:50)...");
    sendSetDateTime(pReq, 50, 59, 23, 30, 11, 2026);
    delay(300);
    captureHistoryAudit(pReq, "CLICHÉ #4 : AVANT Fin de Mois (30/11/2026)", "30/11/2026 23:59:50", audits[4]);

    Serial.println("\n  >>> [Action Phase 2B-2] Armement à 2s de la Fin de Mois : setDateTime(30/11/2026 23:59:58) + Attente 4s pour franchir 01/12/2026 00:00:00...");
    sendSetDateTime(pReq, 58, 59, 23, 30, 11, 2026);
    delay(4200);
    captureHistoryAudit(pReq, "CLICHÉ #5 : APRÈS Fin de Mois (01/12/2026)", "01/12/2026 00:00:02", audits[5]);
    Serial.printf("  🎯 VÉRIFICATION PRÉDICTION TEST B (0x0B00) : CS_Got attendu = 0x0C (bug déclenché sur 30j !) | CS_Got observé = 0x%02X -> %s\n",
                  (unsigned)audits[5].cs0B_got,
                  (audits[5].cs0B_got == 0x0C) ? "✅ PRÉDICTION VALIDÉE À 100 % !" : "🔍 Autre valeur observée");

    // =========================================================================
    // CLICHÉ #6 : PHASE 2C — BASCULE DU NOUVEL AN (31/12/2026 23:59:58 -> 01/01/2027 00:00:02)
    // =========================================================================
    Serial.println("\n  >>> [Action Phase 2C] Armement à 2s du Nouvel An : setDateTime(31/12/2026 23:59:58) + Attente 4s pour franchir 01/01/2027 00:00:00...");
    sendSetDateTime(pReq, 58, 59, 23, 31, 12, 2026);
    delay(4200);
    captureHistoryAudit(pReq, "CLICHÉ #6 : APRÈS Nouvel An (01/01/2027)", "01/01/2027 00:00:02", audits[6]);
    Serial.printf("  🎯 VÉRIFICATION PRÉDICTION TEST C (0x0C00) : CS_Got attendu = 0x0D (si 2027=0 Wh seul sommé) | CS_Got observé = 0x%02X\n",
                  (unsigned)audits[6].cs0C_got);

    // =========================================================================
    // TABLEAU RÉCAPITULATIF GÉNÉRAL DES 7 CLICHÉS
    // =========================================================================
    Serial.println("\n========================================================================================================================");
    Serial.println(" 📊 TABLEAU SYNTHÉTIQUE DES 7 CLICHÉS D'EXPÉRIENCE 19 (VÉLO HW 3.0 / FW 1.30)");
    Serial.println("========================================================================================================================");
    Serial.println(" Cliché                                      | Horloge Prise       | 0x0A00 (24h: Exp/Got/Δ) | 0x0B00 (30j: Exp/Got/Δ) | 0x0C00 (12m: Exp/Got/Δ)");
    Serial.println("---------------------------------------------+---------------------+-------------------------+-------------------------+------------------------");
    for (int i = 0; i < 7; i++) {
        Serial.printf(" %-43s | %-19s | %2luWh: %02X / %02X (-0x%02X)  | %2luWh: %02X / %02X (-0x%02X)  | %4luWh: %02X / %02X (-0x%02X)\n",
                      audits[i].label,
                      audits[i].clockContext,
                      (unsigned long)audits[i].sum24h_Wh, (unsigned)audits[i].cs0A_exp, (unsigned)audits[i].cs0A_got, (unsigned)audits[i].cs0A_diffSub,
                      (unsigned long)audits[i].sum30d_Wh, (unsigned)audits[i].cs0B_exp, (unsigned)audits[i].cs0B_got, (unsigned)audits[i].cs0B_diffSub,
                      (unsigned long)audits[i].sum12m_Wh, (unsigned)audits[i].cs0C_exp, (unsigned)audits[i].cs0C_got, (unsigned)audits[i].cs0C_diffSub);
    }
    Serial.println("========================================================================================================================");

    pClient->disconnect();
    NimBLEDevice::deleteClient(pClient);
    Serial.println("\n[BLE] ✅ Expérience 19 terminée et déconnectée.");
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
    NimBLEDevice::init("Gateway_Exp19");

    Serial.println("\n\n========================================================================================================================");
    Serial.println("  🚀 EXPÉRIENCE 19 : PIÉGEAGE & PRÉDICTION DU CHECKSUM CALENDAIRE SUR VÉLO (HW 3.0 / FW 1.30)");
    Serial.println("========================================================================================================================");

    waitUserPrompt("LANCEMENT EXPÉRIENCE 19 (SANS CHARGE BRANCHÉE : P = 0 W)",
                   "Assurez-vous qu'aucune charge ne tire sur 'Vélo' (P = 0 W).\n"
                   "Appuyez sur BOOT ou 'Entrée' pour exécuter les 7 clichés de bascules calendaires (durée ~20 s).");

    runExp19();

    Serial.println("\n========================================================================================================================");
    Serial.println("  🎉 EXPÉRIENCE 19 TERMINÉE ! (Appuyez sur BOOT ou 'Entrée' pour relancer)");
    Serial.println("========================================================================================================================\n");
}

void loop() {
    if (digitalRead(BUTTON_BOOT_PIN) == LOW) {
        delay(150);
        while (digitalRead(BUTTON_BOOT_PIN) == LOW) delay(20);
        runExp19();
    }
    if (Serial.available()) {
        while (Serial.available()) Serial.read();
        runExp19();
    }
    delay(50);
}
