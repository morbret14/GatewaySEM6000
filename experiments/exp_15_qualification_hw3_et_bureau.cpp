/**
 * @file exp_15_qualification_hw3_et_bureau.cpp
 * @brief Expérience 15 :
 *        Volet 1 - Enquête sur la prise "Bureau" (b0:b1:13:6b:2d:d7) :
 *                  * Scan BLE sans aucun filtre (tous les appareils reçus autour du Bureau)
 *                  * Tentative de connexion GATT directe forcée sur b0:b1:13:6b:2d:d7 (PUBLIC & RANDOM)
 *        Volet 2 - Qualification complète de SEM6000Device v0.3.8 sur la génération Hardware 3.0 / Firmware 1.30
 *                  Cible : Prise "Vélo / Wattmètre mobile" (a3:00:00:00:45:25)
 *                  * Connexion & Authentification 32 bits
 *                  * Inspection brute de FFF1, 1100 (S/N), 1000 (Settings), 0400 (Télémétrie : octets 13-14 et 15-18)
 *                  * Élucidation du comportement de 0xFFF2 sur HW 3.0 / FW 1.30 :
 *                    - Lecture initiale avant setDateTime()
 *                    - Test des 4 combinaisons (Relais OFF/ON x Anneau LED OFF/ON)
 *                    - Test après synchronisation d'horloge setDateTime()
 *                    - Restauration de l'état initial
 *                  * Validation de la Sonde Temporelle Furtive (setTimerAbs -> getTimerStatus -> stopTimer) sur HW 3.0
 *                  * Validation des lectures Planning (0x1400), Mode Aléatoire (0x1600) et Historiques (24h, 30j, 12m).
 */

#include <Arduino.h>
#include <WiFi.h>
#include <NimBLEDevice.h>
#include <vector>
#include <cstring>
#include "SEM6000Device.h"

static const int         BUTTON_BOOT_PIN = 0;
static const char*       MAC_BUREAU      = "b0:b1:13:6b:2d:d7";
static const char*       MAC_VELO_HW3    = "a3:00:00:00:45:25";
static const uint32_t    DEFAULT_PIN     = 0x00000000;

static const NimBLEUUID SERVICE_GAP_UUID("00001800-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_GAP_NAME_UUID("00002a00-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID SERVICE_FFF0_UUID("0000fff0-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_FFF1_UUID("0000fff1-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_FFF2_UUID("0000fff2-0000-1000-8000-00805f9b34fb");

struct KnownPlugEntry {
    const char* name;
    const char* mac;
};

static const KnownPlugEntry KNOWN_PLUGS[] = {
    { "Bureau",              "b0:b1:13:6b:2d:d7" },
    { "Chambre Cedric",      "b0:b1:13:6b:43:f0" },
    { "Congelateur cuisine", "94:a9:a8:18:db:fd" },
    { "Lave linge (HW3)",    "a3:00:00:00:47:93" },
    { "Lave Vaisselle",      "94:a9:a8:18:db:e5" },
    { "Salon",               "b0:b1:13:6c:0c:ba" },
    { "Seche linge (HW3)",   "a3:00:00:00:40:59" },
    { "Smart EQ",            "b0:b1:13:6c:2e:20" },
    { "Velo (HW3)",          "a3:00:00:00:45:25" },
    { "Theoule",             "94:a9:a8:1a:37:0c" }
};
static const size_t KNOWN_PLUG_COUNT = sizeof(KNOWN_PLUGS) / sizeof(KNOWN_PLUGS[0]);

static const char* identifyKnownMac(const char* macStr) {
    for (size_t i = 0; i < KNOWN_PLUG_COUNT; i++) {
        if (strcasecmp(macStr, KNOWN_PLUGS[i].mac) == 0) {
            return KNOWN_PLUGS[i].name;
        }
    }
    return nullptr;
}

static void printHexBuffer(const char* prefix, const uint8_t* buf, size_t len) {
    Serial.print(prefix);
    if (!buf || len == 0) {
        Serial.println("(vide)");
        return;
    }
    for (size_t i = 0; i < len; i++) {
        Serial.printf("%02X ", buf[i]);
    }
    Serial.printf(" (%u octets)\n", (unsigned)len);
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

static void runVolet1EnqueteBureau() {
    Serial.println("########################################################################################");
    Serial.println(" 🕵️‍♂️ VOLET 1 : ENQUÊTE SUR LA PRISE 'BUREAU' (b0:b1:13:6b:2d:d7)");
    Serial.println("########################################################################################");

    Serial.println("\n--- [1A. SCAN BLE 'FILET DÉRIVANT' SANS FILTRE (8 secondes)] ---");
    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setActiveScan(true);
    pScan->setInterval(50);
    pScan->setWindow(50);
    pScan->setDuplicateFilter(1);
    pScan->setMaxResults(0xFF);

    NimBLEScanResults results = pScan->getResults(8000, false);
    int totalDevs = results.getCount();
    Serial.printf(" -> %d périphérique(s) BLE distinct(s) détecté(s) autour du Bureau :\n\n", totalDevs);
    Serial.println(" #  | MAC               | Typ | dBm  | FFF0 | Identité Connue      | Nom Annoncé      | Mfg Data (Hex)");
    Serial.println("----+-------------------+-----+------+------+----------------------+------------------+--------------------");

    bool bureauSeenInScan = false;
    for (int i = 0; i < totalDevs; i++) {
        const NimBLEAdvertisedDevice* pDev = results.getDevice(i);
        if (!pDev) continue;

        std::string macStd = pDev->getAddress().toString();
        uint8_t addrType   = pDev->getAddressType();
        int rssi           = pDev->getRSSI();
        bool hasFFF0       = (pDev->haveServiceUUID() && pDev->isAdvertisingService(SERVICE_FFF0_UUID));
        std::string name   = pDev->haveName() ? pDev->getName() : "";
        const char* known  = identifyKnownMac(macStd.c_str());

        if (strcasecmp(macStd.c_str(), MAC_BUREAU) == 0) {
            bureauSeenInScan = true;
        }

        char mfgHex[33] = {0};
        if (pDev->haveManufacturerData()) {
            std::string mfg = pDev->getManufacturerData();
            size_t n = mfg.size();
            if (n > 16) n = 16;
            for (size_t b = 0; b < n; b++) {
                snprintf(mfgHex + (b * 2), 3, "%02X", (uint8_t)mfg[b]);
            }
        }

        Serial.printf(" %2d | %-17s | %-3s | %4d | %-4s | %-20.20s | %-16.16s | %s\n",
                      i + 1,
                      macStd.c_str(),
                      (addrType == 0) ? "PUB" : "RND",
                      rssi,
                      hasFFF0 ? "OUI" : "non",
                      known ? known : "(inconnu)",
                      !name.empty() ? name.c_str() : "(vide)",
                      (mfgHex[0] != '\0') ? mfgHex : "-");
    }
    pScan->clearResults();

    Serial.println("\n--- [1B. TENTATIVE DE CONNEXION GATT DIRECTE FORCÉE SUR 'BUREAU' (b0:b1:13:6b:2d:d7)] ---");
    Serial.printf(" -> Vue dans le scan préalable : %s\n", bureauSeenInScan ? "OUI 🟢" : "NON 🔴 (Tentative forcée quand même !)");

    for (uint8_t tryType = 0; tryType <= 1; tryType++) {
        Serial.printf(" -> Essai connexion directe vers %s en mode %s (timeout 5s)...",
                      MAC_BUREAU, (tryType == 0) ? "PUBLIC(0)" : "RANDOM(1)");

        NimBLEClient* pClient = NimBLEDevice::createClient();
        pClient->setConnectTimeout(5000);
        NimBLEAddress addr(MAC_BUREAU, tryType);

        uint32_t t0 = millis();
        if (pClient->connect(addr)) {
            uint32_t dt = millis() - t0;
            Serial.printf(" ✅ CONNECTÉ en %lu ms (RSSI: %d dBm) !\n", (unsigned long)dt, pClient->getRssi());

            NimBLERemoteService* pGap = pClient->getService(SERVICE_GAP_UUID);
            if (pGap) {
                NimBLERemoteCharacteristic* pName = pGap->getCharacteristic(CHAR_GAP_NAME_UUID);
                if (pName && pName->canRead()) {
                    Serial.printf("    * Nom GATT (0x2A00) : '%s'\n", pName->readValue().c_str());
                }
            }
            NimBLERemoteService* pSvc = pClient->getService(SERVICE_FFF0_UUID);
            if (pSvc) {
                NimBLERemoteCharacteristic* pFFF1 = pSvc->getCharacteristic(CHAR_FFF1_UUID);
                if (pFFF1 && pFFF1->canRead()) {
                    std::string v1 = pFFF1->readValue();
                    printHexBuffer("    * Trame 0xFFF1 brute : ", reinterpret_cast<const uint8_t*>(v1.data()), v1.size());
                }
                NimBLERemoteCharacteristic* pFFF2 = pSvc->getCharacteristic(CHAR_FFF2_UUID);
                if (pFFF2 && pFFF2->canRead()) {
                    std::string v2 = pFFF2->readValue();
                    printHexBuffer("    * Trame 0xFFF2 brute : ", reinterpret_cast<const uint8_t*>(v2.data()), v2.size());
                }
            }
            pClient->disconnect();
            NimBLEDevice::deleteClient(pClient);
            break;
        } else {
            uint32_t dt = millis() - t0;
            Serial.printf(" ❌ Échec (%lu ms)\n", (unsigned long)dt);
            NimBLEDevice::deleteClient(pClient);
        }
        delay(200);
    }
    Serial.println("########################################################################################\n");
}

static void inspectFFF2AndTelemetry(SEM6000Device& dev, const char* stepLabel) {
    uint16_t word0 = 0;
    uint8_t  curByte = 0;
    float    ampsDirect = 0.0f;
    bool okFFF2 = dev.readRawFFF2(word0, curByte, ampsDirect);

    SEMTelemetry tel;
    memset(&tel, 0, sizeof(tel));
    SEM6000Status stTel = dev.captureMeasurement(tel);
    const uint8_t* rxBuf = nullptr;
    size_t rxLen = dev.getLastRxBuffer(&rxBuf);

    Serial.printf("  [%s]\n", stepLabel);
    Serial.printf("    -> Lecture 0xFFF2 : %s | Word0 (Imax) = %5u mA (0x%04X) | Word1 (Imes) = %6.3f A (byte2=0x%02X)\n",
                  okFFF2 ? "OK" : "ERR",
                  (unsigned)word0, (unsigned)word0, ampsDirect, (unsigned)curByte);
    if (stTel == SEM_OK) {
        Serial.printf("    -> Télémétrie 0400: Relais=%s | P=%.2f W | U=%u V | I=%.3f A | F=%u Hz | Octets[12..13]=0x%04X | totalKwhWh=%lu\n",
                      tel.relayState ? "🟢 ON " : "🔴 OFF",
                      tel.powerWatts, (unsigned)tel.voltageVolts, tel.currentAmperes, (unsigned)tel.frequencyHz,
                      (unsigned)tel.unknownWord, (unsigned long)tel.totalKwhWh);
        printHexBuffer("    -> Trame 0x0400   : ", rxBuf, rxLen);
    } else {
        Serial.printf("    -> Télémétrie 0400: ❌ Erreur code %d\n", (int)stTel);
    }
}

static void runVolet2QualificationHW3() {
    Serial.println("\n########################################################################################");
    Serial.printf(" 🔬 VOLET 2 : QUALIFICATION HARDWARE 3.0 / FIRMWARE 1.30 SUR '%s'\n", MAC_VELO_HW3);
    Serial.println("########################################################################################");

    SEM6000Device velo(MAC_VELO_HW3, DEFAULT_PIN);

    Serial.println("\n--- [2A. CONNEXION GATT & AUTHENTIFICATION 32 BITS] ---");
    uint32_t t0 = millis();
    if (!velo.connect(8000)) {
        Serial.printf(" ❌ Échec de connexion à la prise Vélo (%s) ! Vérifiez qu'elle est bien branchée à portée.\n", MAC_VELO_HW3);
        return;
    }
    Serial.printf(" ✅ Connecté à %s en %lu ms !\n", MAC_VELO_HW3, (unsigned long)(millis() - t0));

    uint16_t preAuthW0 = 0;
    uint8_t  preAuthB2 = 0;
    float    preAuthAmps = 0.0f;
    velo.readRawFFF2(preAuthW0, preAuthB2, preAuthAmps);
    Serial.printf(" -> Lecture 0xFFF2 AVANT Auth : Word0 = %u mA (0x%04X) | Word1 = %.3f A\n",
                  (unsigned)preAuthW0, (unsigned)preAuthW0, preAuthAmps);

    SEM6000Status stAuth = velo.authenticate32((uint32_t)0x00000000);
    Serial.printf(" -> authenticate32(0x00000000) : code %d (%s)\n",
                  (int)stAuth, (stAuth == SEM_OK) ? "✅ SEM_OK" : "❌ ECHEC");
    if (stAuth != SEM_OK) {
        velo.disconnect();
        return;
    }

    Serial.println("\n--- [2B. IDENTITÉ, VERSIONS (0xFFF1), NUMÉRO DE SÉRIE (0x1100) & SETTINGS (0x1000)] ---");
    String gattName;
    if (velo.getDeviceNameGATT(gattName) == SEM_OK) {
        Serial.printf(" -> Nom GATT (0x2A00)      : '%s'\n", gattName.c_str());
    }

    SEMDeviceInfo devInfo;
    if (velo.getDeviceVendorAndVersions(devInfo) == SEM_OK) {
        const uint8_t* rxBuf = nullptr;
        size_t rxLen = velo.getLastRxBuffer(&rxBuf);
        Serial.printf(" -> Fabricant (0xFFF1)     : '%s' | FW: %s | HW: %s | Byte16: 0x%02X\n",
                      devInfo.vendor.c_str(), devInfo.firmwareVersion.c_str(),
                      devInfo.hardwareVersion.c_str(), (unsigned)devInfo.byte16);
        printHexBuffer(" -> Trame 0xFFF1 brute     : ", rxBuf, rxLen);
    }

    String serialStr;
    SEM6000Status stSn = velo.getSerialNumber(serialStr);
    {
        const uint8_t* rxBuf = nullptr;
        size_t rxLen = velo.getLastRxBuffer(&rxBuf);
        Serial.printf(" -> Numéro de Série (1100) : code %d | '%s'\n", (int)stSn, serialStr.c_str());
        printHexBuffer(" -> Trame 0x1100 brute     : ", rxBuf, rxLen);
    }

    SEMSettings initialSettings;
    memset(&initialSettings, 0, sizeof(initialSettings));
    SEM6000Status stSet = velo.requestSettings(initialSettings);
    {
        const uint8_t* rxBuf = nullptr;
        size_t rxLen = velo.getLastRxBuffer(&rxBuf);
        Serial.printf(" -> Settings (0x1000)      : code %d | Pmax=%u W | HP=%u cts | HC=%u cts | ModeHC=%u (%u->%u min) | LED=0x%02X\n",
                      (int)stSet,
                      (unsigned)initialSettings.maxPowerWatts,
                      (unsigned)initialSettings.normalPriceCents,
                      (unsigned)initialSettings.reducedPriceCents,
                      (unsigned)initialSettings.reducedModeActive,
                      (unsigned)initialSettings.reducedPeriodStart,
                      (unsigned)initialSettings.reducedPeriodStop,
                      (unsigned)initialSettings.ledStatus);
        printHexBuffer(" -> Trame 0x1000 brute     : ", rxBuf, rxLen);
    }

    SEMTelemetry initialTel;
    memset(&initialTel, 0, sizeof(initialTel));
    velo.captureMeasurement(initialTel);
    bool initialRelay = initialTel.relayState;
    bool initialLed   = (initialSettings.ledStatus != 0);

    Serial.println("\n--- [2C. ÉLUCIDATION DE 0xFFF2 SUR HW 3.0 (4 COMBINAISONS RELAIS x LED + SETDATETIME)] ---");
    inspectFFF2AndTelemetry(velo, "ÉTAT INITIAL (Avant toute modification)");

    Serial.println("\n -> Test Combinaison 1 : setRelayState(OFF) + setLedRing(OFF)...");
    SEM6000Status stR1 = velo.setRelayState(false);
    SEM6000Status stL1 = velo.setLedRing(false);
    Serial.printf("    Codes retour SET : setRelayState(false)=%d | setLedRing(false)=%d\n", (int)stR1, (int)stL1);
    delay(400);
    inspectFFF2AndTelemetry(velo, "COMBI 1 : Relais OFF + LED OFF");

    Serial.println("\n -> Test Combinaison 2 : setRelayState(OFF) + setLedRing(ON)...");
    SEM6000Status stL2 = velo.setLedRing(true);
    Serial.printf("    Code retour SET  : setLedRing(true)=%d\n", (int)stL2);
    delay(400);
    inspectFFF2AndTelemetry(velo, "COMBI 2 : Relais OFF + LED ON");

    Serial.println("\n -> Test Combinaison 3 : setRelayState(ON) + setLedRing(OFF)...");
    SEM6000Status stR3 = velo.setRelayState(true);
    SEM6000Status stL3 = velo.setLedRing(false);
    Serial.printf("    Codes retour SET : setRelayState(true)=%d | setLedRing(false)=%d\n", (int)stR3, (int)stL3);
    delay(600);
    inspectFFF2AndTelemetry(velo, "COMBI 3 : Relais ON + LED OFF");

    Serial.println("\n -> Test Combinaison 4 : setRelayState(ON) + setLedRing(ON)...");
    SEM6000Status stL4 = velo.setLedRing(true);
    Serial.printf("    Code retour SET  : setLedRing(true)=%d\n", (int)stL4);
    delay(600);
    inspectFFF2AndTelemetry(velo, "COMBI 4 : Relais ON + LED ON");

    Serial.println("\n -> Test 5 : Envoi setDateTime(0, 30, 17, 25, 11, 2026) sur HW 3.0...");
    SEM6000Status stDt = velo.setDateTime(0, 30, 17, 25, 11, 2026);
    Serial.printf("    Code retour SET  : setDateTime()=%d (%s)\n",
                  (int)stDt, (stDt == SEM_OK) ? "✅ SEM_OK" : "❌ ECHEC");
    delay(400);
    inspectFFF2AndTelemetry(velo, "APRÈS setDateTime() (Relais ON + LED ON)");

    Serial.printf("\n -> Restauration de l'état initial (Relais=%s, LED=%s)...\n",
                  initialRelay ? "ON" : "OFF", initialLed ? "ON" : "OFF");
    velo.setLedRing(initialLed);
    velo.setRelayState(initialRelay);
    delay(400);
    inspectFFF2AndTelemetry(velo, "ÉTAT RESTAURÉ");

    Serial.println("\n--- [2D. TEST DE LA SONDE TEMPORELLE FURTIVE (TIMER 0x0800 / 0x0900) SUR HW 3.0] ---");
    SEMTimerStatus timSt;
    memset(&timSt, 0, sizeof(timSt));
    SEM6000Status stTim0 = velo.getTimerStatus(timSt);
    Serial.printf(" -> Timer initial (0x0900) : code %d | Actif=%s\n",
                  (int)stTim0, timSt.active ? "OUI" : "NON");
    if (!timSt.active) {
        SEM6000Status stArm = velo.setTimerAbs(initialRelay, 0, 30, 18, 25, 11, 2026);
        SEMTimerStatus probeTim;
        memset(&probeTim, 0, sizeof(probeTim));
        SEM6000Status stRead = velo.getTimerStatus(probeTim);
        const uint8_t* rxTim = nullptr;
        size_t rxTimLen = velo.getLastRxBuffer(&rxTim);
        uint8_t b11 = (rxTim && rxTimLen >= 14) ? rxTim[11] : 0;
        uint8_t b12 = (rxTim && rxTimLen >= 14) ? rxTim[12] : 0;
        uint8_t b13 = (rxTim && rxTimLen >= 14) ? rxTim[13] : 0;
        SEM6000Status stStop = velo.stopTimer();

        Serial.printf(" -> Armement furtif (+1h)  : setTimerAbs=%d | getTimerStatus=%d | stopTimer=%d\n",
                      (int)stArm, (int)stRead, (int)stStop);
        Serial.printf(" -> Durée Δt retournée par HW 3.0 : [%02uh %02um %02us] = %lu secondes (Attendu: ~3598 s)\n",
                      (unsigned)b11, (unsigned)b12, (unsigned)b13, (unsigned long)probeTim.presetSeconds);
        printHexBuffer(" -> Trame 0x0900 brute     : ", rxTim, rxTimLen);
    }

    Serial.println("\n--- [2E. QUALIFICATION DES LECTURES PLANNINGS, RANDOM & HISTORIQUES SUR HW 3.0] ---");
    SEMRandomConfig rndCfg;
    memset(&rndCfg, 0, sizeof(rndCfg));
    SEM6000Status stRnd = velo.getRandomMode(rndCfg);
    Serial.printf(" -> getRandomMode (0x1600)       : code %d (%s) | Actif=%s\n",
                  (int)stRnd, (stRnd == SEM_OK) ? "✅ SEM_OK" : "❌ ECHEC", rndCfg.active ? "OUI" : "NON");

    std::vector<SEMSchedulerSlot> scheds;
    SEM6000Status stSch = velo.requestAllSchedulers(scheds);
    Serial.printf(" -> requestAllSchedulers (0x1400): code %d (%s) | %u programmation(s) lue(s)\n",
                  (int)stSch, (stSch == SEM_OK) ? "✅ SEM_OK" : "❌ ECHEC", (unsigned)scheds.size());

    uint16_t hourlyWh[24] = {0};
    SEM6000Status stH24 = velo.requestDayHistory(hourlyWh);
    uint32_t sum24 = 0;
    for (int i = 0; i < 24; i++) sum24 += hourlyWh[i];
    Serial.printf(" -> requestDayHistory (0x0A00)   : code %d (%s) | Cumul 24h = %lu Wh (Slot[23]=%u Wh)\n",
                  (int)stH24, (stH24 == SEM_OK) ? "✅ SEM_OK" : "❌ ECHEC",
                  (unsigned long)sum24, (unsigned)hourlyWh[23]);

    uint32_t dailyWh[30] = {0};
    SEM6000Status stD30 = velo.requestMonthHistory(dailyWh);
    uint32_t sum30 = 0;
    for (int i = 0; i < 30; i++) sum30 += (dailyWh[i] & 0x00FFFFFF);
    Serial.printf(" -> requestMonthHistory (0x0B00) : code %d (%s) | Cumul 30j = %lu Wh (Jour[29]=%lu Wh)\n",
                  (int)stD30, (stD30 == SEM_OK) ? "✅ SEM_OK" : "❌ ECHEC",
                  (unsigned long)sum30, (unsigned long)(dailyWh[29] & 0x00FFFFFF));

    uint32_t monthlyWh[12] = {0};
    SEM6000Status stM12 = velo.requestYearHistory(monthlyWh);
    uint32_t sum12 = 0;
    for (int i = 0; i < 12; i++) sum12 += (monthlyWh[i] & 0x00FFFFFF);
    Serial.printf(" -> requestYearHistory (0x0C00)  : code %d (%s) | Cumul 12m = %lu Wh (Mois[11]=%lu Wh)\n",
                  (int)stM12, (stM12 == SEM_OK) ? "✅ SEM_OK" : "❌ ECHEC",
                  (unsigned long)sum12, (unsigned long)(monthlyWh[11] & 0x00FFFFFF));

    velo.disconnect();
    Serial.println("\n[BLE] ✅ Prise Vélo (HW 3.0) déconnectée proprement.");
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
    NimBLEDevice::init("Gateway_Exp15");

    Serial.println("\n\n========================================================================================");
    Serial.println("  🚀 EXPÉRIENCE 15 : ENQUÊTE PRISE 'BUREAU' & QUALIFICATION HARDWARE 3.0 / FW 1.30");
    Serial.println("========================================================================================");

    waitUserPrompt("DÉMARRAGE EXPÉRIENCE 15 (VOLET 1 : BUREAU + VOLET 2 : VÉLO HW 3.0)",
                   "1. Assurez-vous que la prise 'Vélo / Wattmètre mobile' (a3:00:00:00:45:25)\n"
                   "   est branchée dans Chambre Cédric (ou dans le Bureau).\n"
                   "2. Appuyez sur BOOT ou envoyez 'Entrée' pour lancer l'audit complet !");

    runVolet1EnqueteBureau();
    runVolet2QualificationHW3();

    Serial.println("\n========================================================================================");
    Serial.println("  🎉 EXPÉRIENCE 15 TERMINÉE ! (Appuyez sur BOOT ou 'Entrée' pour relancer si besoin)");
    Serial.println("========================================================================================\n");
}

void loop() {
    if (digitalRead(BUTTON_BOOT_PIN) == LOW) {
        delay(150);
        while (digitalRead(BUTTON_BOOT_PIN) == LOW) delay(20);
        runVolet1EnqueteBureau();
        runVolet2QualificationHW3();
    }
    if (Serial.available()) {
        while (Serial.available()) Serial.read();
        runVolet1EnqueteBureau();
        runVolet2QualificationHW3();
    }
    delay(50);
}
