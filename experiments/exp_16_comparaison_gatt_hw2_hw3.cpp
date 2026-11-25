/**
 * @file exp_16_comparaison_gatt_hw2_hw3.cpp
 * @brief Expérience 16 : Analyse différentielle des générations Hardware 2.0 (FW 1.24) vs Hardware 3.0 (FW 1.30).
 *
 * Objectifs :
 * 1. Dump comparatif intégral des Services & Caractéristiques GATT (UUID, Handle, Propriétés,
 *    Longueur et Valeur brute Hex/ASCII) entre :
 *    - Référence Hardware 2.0 / Firmware 1.24 : "Chambre Cédric" (b0:b1:13:6b:43:f0)
 *    - Cible     Hardware 3.0 / Firmware 1.30 : "Vélo / Wattmètre mobile" (a3:00:00:00:45:25)
 *    -> Permet de voir exactement ce qu'est devenue la caractéristique 0xFFF2 sur HW 3.0 !
 * 2. Sniffer brut des notifications 0xFFF4 sur "Vélo" (HW 3.0 / FW 1.30) :
 *    Affiche chaque paquet BLE reçu (longueur exacte + octets hex) sans filtrage pour :
 *    - 0x1700 (authenticate32)
 *    - 0x1000 (requestSettings)
 *    - 0x0400 (captureMeasurement)
 *    - 0x0F0005 (setLedRing ON)
 *    - 0x1100 (getSerialNumber)
 *    - 0x0C00 (requestYearHistory 12m)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <vector>
#include <cstring>

static const int      BUTTON_BOOT_PIN = 0;
static const char*    MAC_CEDRIC_HW2  = "b0:b1:13:6b:43:f0";
static const char*    MAC_VELO_HW3    = "a3:00:00:00:45:25";

static const NimBLEUUID SERVICE_FFF0_UUID("0000fff0-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_REQ_FFF3_UUID("0000fff3-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_RES_FFF4_UUID("0000fff4-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_FFF2_UUID("0000fff2-0000-1000-8000-00805f9b34fb");

// ============================================================================
// UTILITAIRES D'AFFICHAGE
// ============================================================================
static void printHexAndAscii(const uint8_t* data, size_t len) {
    if (!data || len == 0) {
        Serial.print("(0 octet / vide)");
        return;
    }
    Serial.printf("(%u octets) HEX=[ ", (unsigned)len);
    for (size_t i = 0; i < len; i++) {
        Serial.printf("%02X ", data[i]);
    }
    Serial.print("] ASCII='");
    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];
        Serial.print((c >= 32 && c <= 126) ? c : '.');
    }
    Serial.print("'");
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

// ============================================================================
// 1. DUMP INTÉGRAL DE LA TABLE GATT D'UNE PRISE (SERVICES, CHARS, VALEURS)
// ============================================================================
static void dumpFullGattTable(const char* label, const char* macStr) {
    Serial.println("\n########################################################################################");
    Serial.printf(" 📋 DUMP INTÉGRAL TABLE GATT : %s (%s)\n", label, macStr);
    Serial.println("########################################################################################");

    NimBLEDevice::setMTU(160);
    NimBLEClient* pClient = NimBLEDevice::createClient();
    pClient->setConnectTimeout(6000);
    NimBLEAddress addr(macStr, BLE_ADDR_PUBLIC);

    uint32_t t0 = millis();
    if (!pClient->connect(addr)) {
        Serial.printf(" ❌ Impossible de se connecter à %s (%s)\n", label, macStr);
        NimBLEDevice::deleteClient(pClient);
        return;
    }
    Serial.printf(" ✅ Connecté en %lu ms | MTU négocié : %u octets | RSSI : %d dBm\n",
                  (unsigned long)(millis() - t0),
                  (unsigned)pClient->getMTU(),
                  pClient->getRssi());

    const std::vector<NimBLERemoteService*>& services = pClient->getServices(true);
    Serial.printf(" -> Nombre de services GATT découverts : %u\n", (unsigned)services.size());

    for (size_t s = 0; s < services.size(); s++) {
        NimBLERemoteService* pSvc = services[s];
        if (!pSvc) continue;

        Serial.printf("\n  [SERVICE #%u] UUID: %s (Handles: 0x%04X .. 0x%04X)\n",
                      (unsigned)(s + 1),
                      pSvc->getUUID().toString().c_str(),
                      (unsigned)pSvc->getStartHandle(),
                      (unsigned)pSvc->getEndHandle());

        const std::vector<NimBLERemoteCharacteristic*>& chars = pSvc->getCharacteristics(true);
        for (size_t c = 0; c < chars.size(); c++) {
            NimBLERemoteCharacteristic* pChar = chars[c];
            if (!pChar) continue;

            char props[32];
            snprintf(props, sizeof(props), "%s%s%s%s%s",
                     pChar->canRead()            ? "READ "   : "",
                     pChar->canWrite()           ? "WRITE "  : "",
                     pChar->canWriteNoResponse() ? "WR_NR "  : "",
                     pChar->canNotify()          ? "NOTIFY " : "",
                     pChar->canIndicate()        ? "INDIC "  : "");

            Serial.printf("    * Char 0x%04X | UUID: %-36s | Props: [%-22s]\n",
                          (unsigned)pChar->getHandle(),
                          pChar->getUUID().toString().c_str(),
                          props);

            if (pChar->canRead()) {
                std::string val = pChar->readValue();
                Serial.print("      -> Lecture directe : ");
                printHexAndAscii(reinterpret_cast<const uint8_t*>(val.data()), val.size());
                Serial.println();
            }
        }
    }

    pClient->disconnect();
    NimBLEDevice::deleteClient(pClient);
    Serial.println("########################################################################################\n");
    delay(400);
}

// ============================================================================
// 2. SNIFFER BRUT DES NOTIFICATIONS FFF4 & TEST FFF2 SUR VÉLO (HW 3.0)
// ============================================================================
static uint8_t calculateSemChecksum(const uint8_t* buf, size_t startIdx, size_t len) {
    uint32_t sum = 1;
    for (size_t i = startIdx; i < startIdx + len; i++) {
        sum += buf[i];
    }
    return (uint8_t)(sum & 0xFF);
}

static volatile uint32_t g_notifChunkCount = 0;
static volatile uint32_t g_lastNotifMs     = 0;

static void sendRawCmdAndSniff(NimBLERemoteCharacteristic* pReq,
                               NimBLERemoteCharacteristic* pFFF2,
                               const char* cmdTitle,
                               const uint8_t* frame,
                               size_t frameLen,
                               uint32_t waitMs = 800) {
    Serial.printf("\n  >>> COMMANDE : %s\n", cmdTitle);
    Serial.print("      TX (0xFFF3) : ");
    printHexAndAscii(frame, frameLen);
    Serial.println();

    g_notifChunkCount = 0;
    g_lastNotifMs     = millis();

    if (!pReq->writeValue(frame, frameLen, false)) {
        Serial.println("      ❌ Échec writeValue sur 0xFFF3 !");
        return;
    }

    uint32_t t0 = millis();
    while (millis() - t0 < waitMs) {
        delay(20);
    }

    if (g_notifChunkCount == 0) {
        Serial.printf("      ⚠️ Aucune notification reçue sur 0xFFF4 après %lu ms !\n", (unsigned long)waitMs);
    }

    if (pFFF2 && pFFF2->canRead()) {
        std::string v2 = pFFF2->readValue();
        Serial.print("      Lecture 0xFFF2 après commande : ");
        printHexAndAscii(reinterpret_cast<const uint8_t*>(v2.data()), v2.size());
        Serial.println();
    }
}

static void runSnifferOnVeloHW3() {
    Serial.println("\n########################################################################################");
    Serial.printf(" 🔬 SNIFFER BRUT PROTOCOLE 0xFFF3/0xFFF4 & 0xFFF2 SUR VÉLO HW 3.0 (%s)\n", MAC_VELO_HW3);
    Serial.println("########################################################################################");

    NimBLEDevice::setMTU(160);
    NimBLEClient* pClient = NimBLEDevice::createClient();
    pClient->setConnectTimeout(6000);
    NimBLEAddress addr(MAC_VELO_HW3, BLE_ADDR_PUBLIC);

    if (!pClient->connect(addr)) {
        Serial.println(" ❌ Échec connexion à Vélo (HW 3.0) !");
        NimBLEDevice::deleteClient(pClient);
        return;
    }
    Serial.printf(" ✅ Connecté à Vélo (HW 3.0) | MTU = %u octets\n", (unsigned)pClient->getMTU());

    NimBLERemoteService* pSvc = pClient->getService(SERVICE_FFF0_UUID);
    if (!pSvc) {
        Serial.println(" ❌ Service 0xFFF0 introuvable !");
        pClient->disconnect();
        NimBLEDevice::deleteClient(pClient);
        return;
    }

    NimBLERemoteCharacteristic* pReq  = pSvc->getCharacteristic(CHAR_REQ_FFF3_UUID);
    NimBLERemoteCharacteristic* pRes  = pSvc->getCharacteristic(CHAR_RES_FFF4_UUID);
    NimBLERemoteCharacteristic* pFFF2 = pSvc->getCharacteristic(CHAR_FFF2_UUID);

    Serial.printf(" -> Caractéristique 0xFFF2 : %s (canRead=%s, canNotify=%s)\n",
                  pFFF2 ? "TROUVÉE" : "ABSENTE (nullptr)",
                  (pFFF2 && pFFF2->canRead()) ? "OUI" : "NON",
                  (pFFF2 && pFFF2->canNotify()) ? "OUI" : "NON");

    if (!pReq || !pRes) {
        pClient->disconnect();
        NimBLEDevice::deleteClient(pClient);
        return;
    }

    auto rawNotifyCb = [](NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t len, bool isNotify) {
        g_notifChunkCount = g_notifChunkCount + 1;
        g_lastNotifMs     = millis();
        Serial.printf("      RX #%lu (0xFFF4) : ", (unsigned long)g_notifChunkCount);
        printHexAndAscii(pData, len);
        Serial.println();
    };

    pRes->subscribe(true, rawNotifyCb);

    if (pFFF2 && pFFF2->canNotify()) {
        auto fff2NotifyCb = [](NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t len, bool isNotify) {
            Serial.print("      🔔 NOTIF SUR 0xFFF2 : ");
            printHexAndAscii(pData, len);
            Serial.println();
        };
        pFFF2->subscribe(true, fff2NotifyCb);
    }

    // 1. Authentification PIN 00000000 (0x1700)
    uint8_t cmdAuth[16] = { 0x0F, 0x0C, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0xFF, 0xFF };
    sendRawCmdAndSniff(pReq, pFFF2, "1. Authentification 32-bit (0x1700)", cmdAuth, sizeof(cmdAuth), 600);

    // 2. Request Settings (0x1000)
    uint8_t cmdSettings[9] = { 0x0F, 0x05, 0x10, 0x00, 0x00, 0x00, 0x11, 0xFF, 0xFF };
    sendRawCmdAndSniff(pReq, pFFF2, "2. Request Settings (0x1000)", cmdSettings, sizeof(cmdSettings), 600);

    // 3. Capture Measurement (0x0400)
    uint8_t cmdMeas[9] = { 0x0F, 0x05, 0x04, 0x00, 0x00, 0x00, 0x05, 0xFF, 0xFF };
    sendRawCmdAndSniff(pReq, pFFF2, "3. Capture Measurement (0x0400)", cmdMeas, sizeof(cmdMeas), 600);

    // 4. Set LED Ring ON (0x0F0005)
    uint8_t cmdLedOn[13] = { 0x0F, 0x09, 0x0F, 0x00, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x16, 0xFF, 0xFF };
    cmdLedOn[10] = calculateSemChecksum(cmdLedOn, 2, 8);
    sendRawCmdAndSniff(pReq, pFFF2, "4. Set LED Ring ON (0x0F0005)", cmdLedOn, sizeof(cmdLedOn), 600);

    // 5. Get Serial Number (0x1100)
    uint8_t cmdSerial[9] = { 0x0F, 0x05, 0x11, 0x00, 0x00, 0x00, 0x12, 0xFF, 0xFF };
    sendRawCmdAndSniff(pReq, pFFF2, "5. Get Serial Number (0x1100)", cmdSerial, sizeof(cmdSerial), 600);

    // 6. Request Year History 12m (0x0C00)
    uint8_t cmdYear[9] = { 0x0F, 0x05, 0x0C, 0x00, 0x00, 0x00, 0x0D, 0xFF, 0xFF };
    sendRawCmdAndSniff(pReq, pFFF2, "6. Request Year History 12m (0x0C00)", cmdYear, sizeof(cmdYear), 1200);

    pClient->disconnect();
    NimBLEDevice::deleteClient(pClient);
    Serial.println("\n[BLE] ✅ Sniffer Vélo (HW 3.0) terminé et déconnecté.");
    Serial.println("########################################################################################\n");
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
    NimBLEDevice::init("Gateway_Exp16");

    Serial.println("\n\n========================================================================================");
    Serial.println("  🚀 EXPÉRIENCE 16 : COMPARAISON GATT (HW 2.0 vs HW 3.0) & SNIFFER BRUT FFF4/FFF2");
    Serial.println("========================================================================================");

    waitUserPrompt("LANCEMENT EXPÉRIENCE 16 (CHAMBRE CÉDRIC HW 2.0 vs VÉLO HW 3.0)",
                   "Appuyez sur BOOT ou envoyez 'Entrée' pour comparer les tables GATT\n"
                   "de Chambre Cédric (HW 2.0) et Vélo (HW 3.0) et sniffer les trames brutes !");

    dumpFullGattTable("CHAMBRE CÉDRIC (Référence HW 2.0 / FW 1.24)", MAC_CEDRIC_HW2);
    dumpFullGattTable("VÉLO / WATTMÈTRE MOBILE (Cible HW 3.0 / FW 1.30)", MAC_VELO_HW3);
    runSnifferOnVeloHW3();

    Serial.println("\n========================================================================================");
    Serial.println("  🎉 EXPÉRIENCE 16 TERMINÉE ! (Appuyez sur BOOT ou 'Entrée' pour relancer)");
    Serial.println("========================================================================================\n");
}

void loop() {
    if (digitalRead(BUTTON_BOOT_PIN) == LOW) {
        delay(150);
        while (digitalRead(BUTTON_BOOT_PIN) == LOW) delay(20);
        dumpFullGattTable("CHAMBRE CÉDRIC (Référence HW 2.0 / FW 1.24)", MAC_CEDRIC_HW2);
        dumpFullGattTable("VÉLO / WATTMÈTRE MOBILE (Cible HW 3.0 / FW 1.30)", MAC_VELO_HW3);
        runSnifferOnVeloHW3();
    }
    if (Serial.available()) {
        while (Serial.available()) Serial.read();
        dumpFullGattTable("CHAMBRE CÉDRIC (Référence HW 2.0 / FW 1.24)", MAC_CEDRIC_HW2);
        dumpFullGattTable("VÉLO / WATTMÈTRE MOBILE (Cible HW 3.0 / FW 1.30)", MAC_VELO_HW3);
        runSnifferOnVeloHW3();
    }
    delay(50);
}
