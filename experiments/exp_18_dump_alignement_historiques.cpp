/**
 * @file exp_18_dump_alignement_historiques.cpp
 * @brief Expérience 18 : Dump intégral et vérification d'alignement octet par octet
 *        des notifications brutes d'historiques 0x0A00 (24h), 0x0B00 (30j) et 0x0C00 (12m).
 *
 * Comparaison côte à côte :
 * 1. Témoin Hardware 2.0 / Firmware 1.24 : "Chambre Cédric" (b0:b1:13:6b:43:f0)
 * 2. Cible  Hardware 3.0 / Firmware 1.30 : "Vélo / Wattmètre mobile" (a3:00:00:00:45:25)
 *
 * Pour chaque commande (0x0A00, 0x0B00, 0x0C00) :
 * - Capture et affichage de chaque paquet BLE individuel (chunk) tel que livré par NimBLE
 *   (sans aucune troncature, permettant de voir si la trame est fragmentée ou comporte des octets en plus).
 * - Vérification de l'alignement structurel : En-tête [0..3], tous les slots de données,
 *   octet de Checksum [L-3], marqueur de fin [L-2..L-1] (FF FF), et éventuels octets résiduels au-delà.
 * - Suivi pas à pas de la somme cumulée (Checksum) slot par slot, et calcul de l'écart additif
 *   (Reçu - Attendu) mod 256.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <cstring>

static const int   BUTTON_BOOT_PIN = 0;
static const char* MAC_CEDRIC_HW2  = "b0:b1:13:6b:43:f0";
static const char* MAC_VELO_HW3    = "a3:00:00:00:45:25";

static const NimBLEUUID SERVICE_FFF0_UUID("0000fff0-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_REQ_FFF3_UUID("0000fff3-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_RES_FFF4_UUID("0000fff4-0000-1000-8000-00805f9b34fb");

// Stockage des paquets (chunks) individuels reçus pour une commande
struct BleChunk {
    uint32_t timestampMs;
    size_t   len;
    uint8_t  data[160];
};

static BleChunk          g_chunks[16];
static volatile size_t   g_chunkCount = 0;
static uint8_t           g_fullRx[256];
static volatile size_t   g_fullRxLen  = 0;
static volatile uint32_t g_lastRxMs   = 0;

static void onRawNotifyFFF4(NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t len, bool isNotify) {
    uint32_t now = millis();
    g_lastRxMs = now;

    if (g_chunkCount < 16) {
        size_t idx = g_chunkCount;
        g_chunks[idx].timestampMs = now;
        g_chunks[idx].len = (len <= sizeof(g_chunks[idx].data)) ? len : sizeof(g_chunks[idx].data);
        memcpy(g_chunks[idx].data, pData, g_chunks[idx].len);
        g_chunkCount = idx + 1;
    }

    if (g_fullRxLen + len <= sizeof(g_fullRx)) {
        memcpy(g_fullRx + g_fullRxLen, pData, len);
        g_fullRxLen = g_fullRxLen + len;
    }
}

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

static bool sendAndCaptureAllChunks(NimBLERemoteCharacteristic* pReq,
                                    const uint8_t* cmdFrame,
                                    size_t cmdLen,
                                    uint32_t quietWaitMs = 450,
                                    uint32_t maxTimeoutMs = 2500) {
    g_chunkCount = 0;
    g_fullRxLen  = 0;
    g_lastRxMs   = millis();

    if (!pReq->writeValue(cmdFrame, cmdLen, false)) {
        Serial.println("  ❌ Échec writeValue sur 0xFFF3 !");
        return false;
    }

    uint32_t t0 = millis();
    // On attend au moins 1 paquet, puis on attend un silence radio de quietWaitMs
    // pour être 100 % sûr de capturer d'éventuels octets ou paquets supplémentaires !
    while (millis() - t0 < maxTimeoutMs) {
        if (g_chunkCount > 0 && (millis() - g_lastRxMs >= quietWaitMs)) {
            return true;
        }
        delay(10);
    }
    return (g_chunkCount > 0);
}

static void dumpChunksAndHeader(const char* cmdTitle, const uint8_t* txFrame, size_t txLen, uint32_t tSendMs) {
    Serial.println("\n  ======================================================================================================");
    Serial.printf("  📌 %s\n", cmdTitle);
    Serial.println("  ------------------------------------------------------------------------------------------------------");
    Serial.printf("  📤 TX (0xFFF3) [%2u octets] : ", (unsigned)txLen);
    printHexLine(txFrame, txLen);
    Serial.println();

    Serial.printf("  📥 Notifications reçues sur 0xFFF4 : %u paquet(s) BLE | Total = %u octets\n",
                  (unsigned)g_chunkCount, (unsigned)g_fullRxLen);

    for (size_t c = 0; c < g_chunkCount; c++) {
        Serial.printf("     -> Paquet #%u (+%3lu ms, %3u octets) : ",
                      (unsigned)(c + 1),
                      (unsigned long)(g_chunks[c].timestampMs - tSendMs),
                      (unsigned)g_chunks[c].len);
        printHexLine(g_chunks[c].data, g_chunks[c].len);
        Serial.println();
    }

    if (g_fullRxLen >= 2) {
        uint8_t startByte  = g_fullRx[0];
        uint8_t lenField   = g_fullRx[1];
        size_t  logicalLen = (size_t)lenField + 4;
        Serial.printf("  📐 Analyse de longueur : Byte[0]=0x%02X | Byte[1]=0x%02X (%u) -> Longueur logique attendue (Byte[1]+4) = %u octets | Reçu = %u octets",
                      (unsigned)startByte, (unsigned)lenField, (unsigned)lenField,
                      (unsigned)logicalLen, (unsigned)g_fullRxLen);
        if (g_fullRxLen == logicalLen) {
            Serial.println(" (✅ Exactement aligné)");
        } else if (g_fullRxLen > logicalLen) {
            Serial.printf(" (⚠️ +%u octet(s) supplémentaires après la trame logique !)\n",
                          (unsigned)(g_fullRxLen - logicalLen));
        } else {
            Serial.printf(" (❌ Trame plus courte de %u octet(s) !)\n",
                          (unsigned)(logicalLen - g_fullRxLen));
        }
    }
}

// ============================================================================
// ANALYSE DÉTAILLÉE D'UN HISTORIQUE 2 OCTETS/SLOT : 0x0A00 (24 HEURES)
// ============================================================================
static void analyzeHistory24h(const char* plugLabel) {
    if (g_fullRxLen < 7) return;

    uint32_t runningSum = 1; // Seed 0x01
    Serial.println("\n  --- [ALIGNEMENT OCTET PAR OCTET : 0x0A00 (24 HEURES x 2 OCTETS)] ---");
    Serial.printf("  En-tête : [0]=0x%02X (Start) | [1]=0x%02X (Len=%u) | [2..3]=0x%02X 0x%02X (Cmd 0x0A00)\n",
                  (unsigned)g_fullRx[0], (unsigned)g_fullRx[1], (unsigned)g_fullRx[1],
                  (unsigned)g_fullRx[2], (unsigned)g_fullRx[3]);

    runningSum += g_fullRx[2] + g_fullRx[3];
    Serial.printf("  Seed + Cmd[2..3] : 0x01 + 0x%02X + 0x%02X = 0x%04lX (CS partiel = 0x%02X)\n\n",
                  (unsigned)g_fullRx[2], (unsigned)g_fullRx[3],
                  (unsigned long)runningSum, (unsigned)(runningSum & 0xFF));

    Serial.println("  Slot  | Heure rel. | Index Octets | Hex [B0 B1] | Énergie (Wh) | ΔSomme | Somme16 | CS Cumulé (8b)");
    Serial.println(" -------+------------+--------------+-------------+--------------+--------+---------+---------------");

    uint32_t totalWh24 = 0;
    for (int i = 0; i < 24; i++) {
        size_t idx = 4 + (i * 2);
        if (idx + 1 >= g_fullRxLen) break;

        uint8_t  b0  = g_fullRx[idx];
        uint8_t  b1  = g_fullRx[idx + 1];
        uint16_t wh  = ((uint16_t)b0 << 8) | b1;
        uint16_t dS  = (uint16_t)b0 + (uint16_t)b1;
        runningSum  += dS;
        totalWh24   += wh;

        Serial.printf("   [%02d] |    H-%02d    |   [%02u..%02u]   |   %02X %02X     |   %6u Wh  | +0x%02X  | 0x%04lX  |     0x%02X\n",
                      i, 23 - i, (unsigned)idx, (unsigned)(idx + 1),
                      (unsigned)b0, (unsigned)b1, (unsigned)wh,
                      (unsigned)dS, (unsigned long)runningSum, (unsigned)(runningSum & 0xFF));
    }

    Serial.println(" -------+------------+--------------+-------------+--------------+--------+---------+---------------");
    size_t csIdx = 52; // 4 + 24*2 = 52
    if (g_fullRxLen >= 55) {
        uint8_t csExp  = (uint8_t)(runningSum & 0xFF);
        uint8_t csGot  = g_fullRx[csIdx];
        uint8_t diffAdd = (uint8_t)((csGot - csExp) & 0xFF);
        Serial.printf("  Pied de trame : [%u]=0x%02X (Checksum Reçu) | [%u..%u]=%02X %02X (Marqueur Fin)\n",
                      (unsigned)csIdx, (unsigned)csGot,
                      (unsigned)(csIdx + 1), (unsigned)(csIdx + 2),
                      (unsigned)g_fullRx[csIdx + 1], (unsigned)g_fullRx[csIdx + 2]);
        if (g_fullRxLen > 55) {
            Serial.print("  ⚠️ Octets supplémentaires après [54] : ");
            printHexLine(g_fullRx + 55, g_fullRxLen - 55);
            Serial.println();
        }
        Serial.printf("  📊 BILAN 0x0A00 (%s) : Cumul 24h = %lu Wh | CS Attendu = 0x%02X | CS Reçu = 0x%02X | Écart (Reçu - Attendu) = +0x%02X (%u)\n",
                      plugLabel, (unsigned long)totalWh24,
                      (unsigned)csExp, (unsigned)csGot, (unsigned)diffAdd, (unsigned)diffAdd);
    }
}

// ============================================================================
// ANALYSE DÉTAILLÉE D'UN HISTORIQUE 4 OCTETS/SLOT : 0x0B00 (30J) OU 0x0C00 (12M)
// ============================================================================
static void analyzeHistory4Bytes(const char* plugLabel,
                                 const char* cmdName,
                                 int numSlots,
                                 const char* unitPrefix) {
    size_t expectedTotalLen = 4 + (size_t)numSlots * 4 + 3; // 127 pour 30j, 55 pour 12m
    if (g_fullRxLen < 7) return;

    uint32_t runningSumAll   = 1; // Seed 0x01 + tous les octets B0..B3
    uint32_t runningSumNoB3  = 1; // Seed 0x01 + seulement B0..B2 (sans B3)
    uint32_t sumB3Only       = 0;

    Serial.printf("\n  --- [ALIGNEMENT OCTET PAR OCTET : %s (%d SLOTS x 4 OCTETS)] ---\n", cmdName, numSlots);
    Serial.printf("  En-tête : [0]=0x%02X (Start) | [1]=0x%02X (Len=%u) | [2..3]=0x%02X 0x%02X (Cmd)\n",
                  (unsigned)g_fullRx[0], (unsigned)g_fullRx[1], (unsigned)g_fullRx[1],
                  (unsigned)g_fullRx[2], (unsigned)g_fullRx[3]);

    runningSumAll  += g_fullRx[2] + g_fullRx[3];
    runningSumNoB3 += g_fullRx[2] + g_fullRx[3];
    Serial.printf("  Seed + Cmd[2..3] : 0x01 + 0x%02X + 0x%02X = 0x%04lX (CS partiel = 0x%02X)\n\n",
                  (unsigned)g_fullRx[2], (unsigned)g_fullRx[3],
                  (unsigned long)runningSumAll, (unsigned)(runningSumAll & 0xFF));

    Serial.println("  Slot  | Période | Index Octets  | Hex [B0 B1 B2 B3] | Énergie (Wh) | B3 (Hex/Dec) | ΔSomme | Somme16 | CS Cumulé (8b)");
    Serial.println(" -------+---------+---------------+-------------------+--------------+--------------+--------+---------+---------------");

    uint32_t totalWh = 0;
    for (int i = 0; i < numSlots; i++) {
        size_t idx = 4 + (size_t)(i * 4);
        if (idx + 3 >= g_fullRxLen) break;

        uint8_t  b0  = g_fullRx[idx];
        uint8_t  b1  = g_fullRx[idx + 1];
        uint8_t  b2  = g_fullRx[idx + 2];
        uint8_t  b3  = g_fullRx[idx + 3];
        uint32_t wh  = ((uint32_t)b0 << 16) | ((uint32_t)b1 << 8) | b2;
        uint16_t dS  = (uint16_t)b0 + (uint16_t)b1 + (uint16_t)b2 + (uint16_t)b3;

        runningSumAll  += dS;
        runningSumNoB3 += ((uint16_t)b0 + (uint16_t)b1 + (uint16_t)b2);
        sumB3Only      += b3;
        totalWh        += wh;

        Serial.printf("   [%02d] |  %s-%02d   |  [%03u..%03u]   |  %02X %02X %02X %02X      |  %8lu Wh |  0x%02X (%3u%%) | +0x%03X | 0x%04lX  |     0x%02X\n",
                      i, unitPrefix, (numSlots - 1) - i,
                      (unsigned)idx, (unsigned)(idx + 3),
                      (unsigned)b0, (unsigned)b1, (unsigned)b2, (unsigned)b3,
                      (unsigned long)wh, (unsigned)b3, (unsigned)b3,
                      (unsigned)dS, (unsigned long)runningSumAll, (unsigned)(runningSumAll & 0xFF));
    }

    Serial.println(" -------+---------+---------------+-------------------+--------------+--------------+--------+---------+---------------");
    size_t csIdx = 4 + (size_t)numSlots * 4;
    if (g_fullRxLen >= expectedTotalLen) {
        uint8_t csExp    = (uint8_t)(runningSumAll & 0xFF);
        uint8_t csGot    = g_fullRx[csIdx];
        uint8_t diffAdd  = (uint8_t)((csGot - csExp) & 0xFF);
        uint8_t diffSub  = (uint8_t)((csExp - csGot) & 0xFF);

        Serial.printf("  Pied de trame : [%u]=0x%02X (Checksum Reçu) | [%u..%u]=%02X %02X (Marqueur Fin)\n",
                      (unsigned)csIdx, (unsigned)csGot,
                      (unsigned)(csIdx + 1), (unsigned)(csIdx + 2),
                      (unsigned)g_fullRx[csIdx + 1], (unsigned)g_fullRx[csIdx + 2]);

        if (g_fullRxLen > expectedTotalLen) {
            Serial.printf("  ⚠️ Octets supplémentaires après [%u] : ", (unsigned)(expectedTotalLen - 1));
            printHexLine(g_fullRx + expectedTotalLen, g_fullRxLen - expectedTotalLen);
            Serial.println();
        }

        Serial.printf("  📊 BILAN %s (%s) :\n", cmdName, plugLabel);
        Serial.printf("     * Cumul Énergie       = %lu Wh (Somme des octets B3 = %lu = 0x%02X)\n",
                      (unsigned long)totalWh, (unsigned long)sumB3Only, (unsigned)(sumB3Only & 0xFF));
        Serial.printf("     * CS Attendu (std)    = 0x%02X (Somme16 complète = 0x%04lX = %lu)\n",
                      (unsigned)csExp, (unsigned long)runningSumAll, (unsigned long)runningSumAll);
        Serial.printf("     * CS Attendu (sans B3)= 0x%02X\n", (unsigned)(runningSumNoB3 & 0xFF));
        Serial.printf("     * CS Reçu (Got)       = 0x%02X\n", (unsigned)csGot);
        Serial.printf("     * Écart Additif       : (0x1%02X - 0x%02X) & 0xFF = +0x%02X (%u en décimal)\n",
                      (unsigned)csGot, (unsigned)csExp, (unsigned)diffAdd, (unsigned)diffAdd);
        Serial.printf("     * Écart Soustractif   : (0x1%02X - 0x%02X) & 0xFF = -0x%02X (%u en décimal)\n",
                      (unsigned)csExp, (unsigned)csGot, (unsigned)diffSub, (unsigned)diffSub);
    }
}

static void inspectPlugHistories(const char* plugLabel, const char* macStr) {
    Serial.println("\n########################################################################################################################");
    Serial.printf(" 🔬 DUMP INTÉGRAL & ALIGNEMENT HISTORIQUES : %s (%s)\n", plugLabel, macStr);
    Serial.println("########################################################################################################################");

    NimBLEDevice::setMTU(160);
    NimBLEClient* pClient = NimBLEDevice::createClient();
    pClient->setConnectTimeout(6000);
    NimBLEAddress addr(macStr, BLE_ADDR_PUBLIC);

    uint32_t t0 = millis();
    if (!pClient->connect(addr)) {
        Serial.printf(" ❌ Impossible de se connecter à %s (%s)\n", plugLabel, macStr);
        NimBLEDevice::deleteClient(pClient);
        return;
    }
    Serial.printf(" ✅ Connecté en %lu ms | MTU négocié : %u octets | RSSI : %d dBm\n",
                  (unsigned long)(millis() - t0),
                  (unsigned)pClient->getMTU(),
                  pClient->getRssi());

    NimBLERemoteService* pSvc = pClient->getService(SERVICE_FFF0_UUID);
    if (!pSvc) {
        Serial.println(" ❌ Service 0xFFF0 introuvable !");
        pClient->disconnect();
        NimBLEDevice::deleteClient(pClient);
        return;
    }

    NimBLERemoteCharacteristic* pReq = pSvc->getCharacteristic(CHAR_REQ_FFF3_UUID);
    NimBLERemoteCharacteristic* pRes = pSvc->getCharacteristic(CHAR_RES_FFF4_UUID);
    if (!pReq || !pRes || !pRes->subscribe(true, onRawNotifyFFF4)) {
        Serial.println(" ❌ Impossible de souscrire à 0xFFF4 !");
        pClient->disconnect();
        NimBLEDevice::deleteClient(pClient);
        return;
    }

    // 1. Authentification 32 bits (0x1700)
    static const uint8_t cmdAuth[16] = {
        0x0F, 0x0C, 0x17, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x18, 0xFF, 0xFF
    };
    uint32_t tSend = millis();
    if (!sendAndCaptureAllChunks(pReq, cmdAuth, sizeof(cmdAuth), 300, 1500)) {
        Serial.println(" ❌ Échec authentification 0x1700 !");
        pClient->disconnect();
        NimBLEDevice::deleteClient(pClient);
        return;
    }
    dumpChunksAndHeader("0. Authentification 32 bits (0x1700)", cmdAuth, sizeof(cmdAuth), tSend);

    // 2. Télémétrie instantanée (0x0400)
    static const uint8_t cmd0400[9] = { 0x0F, 0x05, 0x04, 0x00, 0x00, 0x00, 0x05, 0xFF, 0xFF };
    tSend = millis();
    if (sendAndCaptureAllChunks(pReq, cmd0400, sizeof(cmd0400), 300, 1500) && g_fullRxLen >= 19) {
        dumpChunksAndHeader("0bis. Télémétrie Live (0x0400)", cmd0400, sizeof(cmd0400), tSend);
        uint32_t rawP = ((uint32_t)g_fullRx[5] << 16) | ((uint32_t)g_fullRx[6] << 8) | g_fullRx[7];
        uint32_t totWh = ((uint32_t)g_fullRx[14] << 24) | ((uint32_t)g_fullRx[15] << 16) |
                         ((uint32_t)g_fullRx[16] << 8)  | g_fullRx[17];
        uint8_t csExp04 = calculateSemChecksum(g_fullRx, 2, 16);
        Serial.printf("  -> Relais=%s | P=%.2f W | U=%u V | totalKwhWh=%lu Wh | CS_Exp=0x%02X CS_Got=0x%02X\n",
                      (g_fullRx[4] == 0x01) ? "ON" : "OFF",
                      rawP / 1000.0f, (unsigned)g_fullRx[8], (unsigned long)totWh,
                      (unsigned)csExp04, (unsigned)g_fullRx[18]);
    }

    // 3. Historique 24 Heures (0x0A00)
    static const uint8_t cmd0A00[9] = { 0x0F, 0x05, 0x0A, 0x00, 0x00, 0x00, 0x0B, 0xFF, 0xFF };
    tSend = millis();
    if (sendAndCaptureAllChunks(pReq, cmd0A00, sizeof(cmd0A00), 450, 2500)) {
        dumpChunksAndHeader("1. requestDayHistory — Historique Horaire 24 Heures (0x0A00)", cmd0A00, sizeof(cmd0A00), tSend);
        analyzeHistory24h(plugLabel);
    }

    // 4. Historique 30 Jours (0x0B00)
    static const uint8_t cmd0B00[9] = { 0x0F, 0x05, 0x0B, 0x00, 0x00, 0x00, 0x0C, 0xFF, 0xFF };
    tSend = millis();
    if (sendAndCaptureAllChunks(pReq, cmd0B00, sizeof(cmd0B00), 450, 2500)) {
        dumpChunksAndHeader("2. requestMonthHistory — Historique Journalier 30 Jours (0x0B00)", cmd0B00, sizeof(cmd0B00), tSend);
        analyzeHistory4Bytes(plugLabel, "0x0B00 (30 JOURS)", 30, "J");
    }

    // 5. Historique 12 Mois (0x0C00)
    static const uint8_t cmd0C00[9] = { 0x0F, 0x05, 0x0C, 0x00, 0x00, 0x00, 0x0D, 0xFF, 0xFF };
    tSend = millis();
    if (sendAndCaptureAllChunks(pReq, cmd0C00, sizeof(cmd0C00), 450, 2500)) {
        dumpChunksAndHeader("3. requestYearHistory — Historique Mensuel 12 Mois (0x0C00)", cmd0C00, sizeof(cmd0C00), tSend);
        analyzeHistory4Bytes(plugLabel, "0x0C00 (12 MOIS)", 12, "M");
    }

    pClient->disconnect();
    NimBLEDevice::deleteClient(pClient);
    Serial.printf("\n[BLE] ✅ Inspection %s terminée et déconnectée.\n", plugLabel);
    Serial.println("########################################################################################################################\n");
    delay(500);
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
    NimBLEDevice::init("Gateway_Exp18");

    Serial.println("\n\n========================================================================================================================");
    Serial.println("  🚀 EXPÉRIENCE 18 : DUMP COMPLET & ALIGNEMENT DES NOTIFICATIONS 0x0A00 (24h), 0x0B00 (30j), 0x0C00 (12m)");
    Serial.println("========================================================================================================================");

    waitUserPrompt("LANCEMENT EXPÉRIENCE 18 (TÉMOIN CHAMBRE CÉDRIC HW 2.0 vs CIBLE VÉLO HW 3.0)",
                   "Appuyez sur BOOT ou 'Entrée' pour capturer et disséquer les notifications brutes\n"
                   "0x0A00 (24h), 0x0B00 (30j) et 0x0C00 (12m) sur Chambre Cédric (HW 2.0) puis Vélo (HW 3.0).");

    inspectPlugHistories("TÉMOIN CHAMBRE CÉDRIC (HW 2.0 / FW 1.24)", MAC_CEDRIC_HW2);
    inspectPlugHistories("CIBLE VÉLO / WATTMÈTRE MOBILE (HW 3.0 / FW 1.30)", MAC_VELO_HW3);

    Serial.println("\n========================================================================================================================");
    Serial.println("  🎉 EXPÉRIENCE 18 TERMINÉE ! (Appuyez sur BOOT ou 'Entrée' pour relancer)");
    Serial.println("========================================================================================================================\n");
}

void loop() {
    if (digitalRead(BUTTON_BOOT_PIN) == LOW) {
        delay(150);
        while (digitalRead(BUTTON_BOOT_PIN) == LOW) delay(20);
        inspectPlugHistories("TÉMOIN CHAMBRE CÉDRIC (HW 2.0 / FW 1.24)", MAC_CEDRIC_HW2);
        inspectPlugHistories("CIBLE VÉLO / WATTMÈTRE MOBILE (HW 3.0 / FW 1.30)", MAC_VELO_HW3);
    }
    if (Serial.available()) {
        while (Serial.available()) Serial.read();
        inspectPlugHistories("TÉMOIN CHAMBRE CÉDRIC (HW 2.0 / FW 1.24)", MAC_CEDRIC_HW2);
        inspectPlugHistories("CIBLE VÉLO / WATTMÈTRE MOBILE (HW 3.0 / FW 1.30)", MAC_VELO_HW3);
    }
    delay(50);
}
