/**
 * @file exp_14_mobile_radio_survey.cpp
 * @brief Expérience 14 (v2) : Campagne Mobile de Cartographie Radio (60s BLE + Sonde F auto + Wi-Fi en dBm)
 *        Parcours guidé sur 7 emplacements :
 *          1. 01_Sous-sol
 *          2. 02_Garage
 *          3. 03_Cuisine
 *          4. 04_Salle_a_manger_Salon
 *          5. 05_Bureau
 *          6. 06_Palier_etage
 *          7. 07_Chambre_Cedric
 *
 * Déroulement automatique à chaque point d'arrêt (~1m15s à 1m30s par point) :
 *   1/3 Écoute BLE pure pendant 60 secondes (Wi-Fi OFF, 100% duty-cycle, ~1000 paquets attendus/prise).
 *       Calcul Moy dBm, Pwr dBm (mW -> dBm), Eff dBm (intégrant le taux de perte de paquets), Min/Max, Sigma.
 *   2/3 Sonde GATT active automatique ("Commande F") sans code PIN sur chaque prise entendue :
 *       Connexion courte (timeout 4s), lecture Nom GATT (0x2A00), Vendor/FW/HW (0xFFF1) et Seuil/Courant (0xFFF2).
 *   3/3 Scan passif Wi-Fi 2.4 GHz (2s) pour mesurer Morbret_5G et les AP voisins en dBm.
 *   -> Enregistrement intégral en mémoire Flash FFat (/exp14_reports.txt et /exp14_survey.csv).
 */

#include <Arduino.h>
#include <WiFi.h>
#include <FFat.h>
#include <NimBLEDevice.h>
#include <cmath>
#include <cstring>

// ============================================================================
// 1. CONSTANTES & CONFIGURATION MATÉRIELLE
// ============================================================================
static const int      BUTTON_BOOT_PIN          = 0;
static const uint32_t BLE_SURVEY_WINDOW_MS     = 60000; // 60 secondes (1 min) de scan BLE pur par point
static const uint32_t NOMINAL_ADV_INTERVAL_MS  = 60;    // Cadence nominale SEM6000 découverte en exp_14 v1
static const uint32_t GATT_PROBE_TIMEOUT_MS    = 4000;  // Timeout connexion GATT courte (4s max par prise)
static const char*    TARGET_WIFI_SSID         = "MORBRET_5G";
static const char*    TARGET_WIFI_PREFIX       = "MORBRET";
static const char*    CSV_FILE_PATH            = "/exp14_survey.csv";
static const char*    REPORT_FILE_PATH         = "/exp14_reports.txt";

static const NimBLEUUID SERVICE_GAP_UUID("00001800-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_GAP_NAME_UUID("00002a00-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID SERVICE_FFF0_UUID("0000fff0-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_FFF1_UUID("0000fff1-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_FFF2_UUID("0000fff2-0000-1000-8000-00805f9b34fb");

// Parcours pré-enregistré dans l'ordre exact demandé
static const char* PLANNED_ROUTE[] = {
    "01_Sous-sol",
    "02_Garage",
    "03_Cuisine",
    "04_Salle_a_manger_Salon",
    "05_Bureau",
    "06_Palier_etage",
    "07_Chambre_Cedric"
};
static const size_t PLANNED_ROUTE_COUNT = sizeof(PLANNED_ROUTE) / sizeof(PLANNED_ROUTE[0]);

// ============================================================================
// 2. RÉPERTOIRE STATIQUE DES 10 PRISES CONNUES DU PARC
// ============================================================================
struct KnownPlugDef {
    const char* name;
    const char* mac; // minuscule "xx:xx:xx:xx:xx:xx"
};

static const KnownPlugDef KNOWN_PLUGS[] = {
    { "Bureau",              "b0:b1:13:6b:2d:d7" },
    { "Chambre Cedric",      "b0:b1:13:6b:43:f0" },
    { "Congelateur cuisine", "94:a9:a8:18:db:fd" },
    { "Lave linge",          "a3:00:00:00:47:93" },
    { "Lave Vaisselle",      "94:a9:a8:18:db:e5" },
    { "Salon",               "b0:b1:13:6c:0c:ba" },
    { "Seche linge",         "a3:00:00:00:40:59" },
    { "Smart EQ",            "b0:b1:13:6c:2e:20" },
    { "Velo",                "a3:00:00:00:45:25" },
    { "Theoule",             "94:a9:a8:1a:37:0c" }
};
static const size_t KNOWN_PLUG_COUNT  = sizeof(KNOWN_PLUGS) / sizeof(KNOWN_PLUGS[0]);
static const size_t MAX_TRACKED_SLOTS = 24; // 10 connues + jusqu'à 14 découvertes

// ============================================================================
// 3. STRUCTURES DE MESURE STATIQUES (ZÉRO ALLOCATION DYNAMIQUE EN BOUCLE)
// ============================================================================
struct DeviceRadioStats {
    bool     inUse;
    bool     isKnown;
    char     knownName[24];
    char     mac[18];
    uint8_t  addrType;
    bool     hasFFF0;
    char     advName[24];
    char     mfgHex[41];     // Jusqu'à 20 octets en hex
    char     payloadHex[65]; // Jusqu'à 32 octets en hex

    uint32_t pktCount;
    int32_t  sumDbm;
    int64_t  sumSqDbm;
    double   sumMilliwatts;
    int8_t   minDbm;
    int8_t   maxDbm;
    uint32_t firstSeenMs;
    uint32_t lastSeenMs;

    // Résultats de la sonde active GATT sans PIN ("Commande F")
    bool     probeAttempted;
    bool     probeConnected;
    uint32_t probeConnMs;
    int      probeConnRssi;
    char     gattName[24];
    char     vendor[8];
    uint8_t  fwMaj;
    uint8_t  fwMin;
    uint8_t  hwMaj;
    uint8_t  hwMin;
    uint16_t fff2ImaxMa;
    uint16_t fff2ImesMa;
};

struct WifiSurveyResult {
    bool    targetFound;
    char    targetSsid[33];
    int32_t targetDbm;
    int32_t targetChannel;
    char    bestSsid[33];
    int32_t bestDbm;
    int32_t bestChannel;
    int32_t totalAp24GHz;
    int32_t ch1Count;
    int32_t ch6Count;
    int32_t ch11Count;
};

// Verrou inter-cœurs (Core 0 NimBLE callback <-> Core 1 Application)
static portMUX_TYPE g_statsMux = portMUX_INITIALIZER_UNLOCKED;

class SpinLockGuard {
public:
    explicit SpinLockGuard(portMUX_TYPE* mux) : m_mux(mux) { portENTER_CRITICAL(m_mux); }
    ~SpinLockGuard() { portEXIT_CRITICAL(m_mux); }
private:
    portMUX_TYPE* m_mux;
};

static DeviceRadioStats  g_slots[MAX_TRACKED_SLOTS];
static volatile uint32_t g_totalBlePacketsInWindow = 0;
static uint32_t          g_pointCounter            = 0;
static bool              g_ffatMounted             = false;

// ============================================================================
// 4. UTILITAIRES DE FORMATAGE & INITIALISATION DES SLOTS
// ============================================================================
static const char* getNextPlannedLabel(char* fallbackBuf, size_t fallbackLen) {
    if (g_pointCounter < PLANNED_ROUTE_COUNT) {
        return PLANNED_ROUTE[g_pointCounter];
    }
    snprintf(fallbackBuf, fallbackLen, "%02lu_POINT_%02lu",
             (unsigned long)(g_pointCounter + 1), (unsigned long)(g_pointCounter + 1));
    return fallbackBuf;
}

static void bytesToHex(const uint8_t* data, size_t len, char* outHex, size_t outMaxLen) {
    if (!data || len == 0 || outMaxLen < 3) {
        if (outMaxLen > 0) outHex[0] = '\0';
        return;
    }
    size_t maxBytes = (outMaxLen - 1) / 2;
    if (len > maxBytes) len = maxBytes;
    for (size_t i = 0; i < len; i++) {
        snprintf(outHex + (i * 2), 3, "%02X", data[i]);
    }
    outHex[len * 2] = '\0';
}

static void resetSlotsForNewWindow() {
    SpinLockGuard lock(&g_statsMux);
    memset(g_slots, 0, sizeof(g_slots));
    g_totalBlePacketsInWindow = 0;

    for (size_t i = 0; i < KNOWN_PLUG_COUNT; i++) {
        g_slots[i].inUse   = true;
        g_slots[i].isKnown = true;
        strncpy(g_slots[i].knownName, KNOWN_PLUGS[i].name, sizeof(g_slots[i].knownName) - 1);
        strncpy(g_slots[i].mac, KNOWN_PLUGS[i].mac, sizeof(g_slots[i].mac) - 1);
        g_slots[i].minDbm  = 0;
        g_slots[i].maxDbm  = -127;
    }
}

static int findSlotByMacLocked(const char* macStr) {
    for (size_t i = 0; i < MAX_TRACKED_SLOTS; i++) {
        if (g_slots[i].inUse && strcasecmp(g_slots[i].mac, macStr) == 0) {
            return (int)i;
        }
    }
    return -1;
}

static void enrichSlotMetadataLocked(DeviceRadioStats& slot, const NimBLEAdvertisedDevice* pDev) {
    slot.addrType = pDev->getAddressType();

    if (pDev->haveServiceUUID() && pDev->isAdvertisingService(SERVICE_FFF0_UUID)) {
        slot.hasFFF0 = true;
    }

    if (pDev->haveName()) {
        std::string nm = pDev->getName();
        if (!nm.empty()) {
            strncpy(slot.advName, nm.c_str(), sizeof(slot.advName) - 1);
            slot.advName[sizeof(slot.advName) - 1] = '\0';
        }
    }

    if (pDev->haveManufacturerData() && slot.mfgHex[0] == '\0') {
        std::string mfg = pDev->getManufacturerData();
        bytesToHex(reinterpret_cast<const uint8_t*>(mfg.data()), mfg.size(), slot.mfgHex, sizeof(slot.mfgHex));
    }

    const std::vector<uint8_t>& pay = pDev->getPayload();
    if (!pay.empty() && pay.size() > strlen(slot.payloadHex) / 2) {
        bytesToHex(pay.data(), pay.size(), slot.payloadHex, sizeof(slot.payloadHex));
    }
}

// ============================================================================
// 5. CALLBACKS DE SCAN NIMBLE (CORE 0 -> SECTION CRITIQUE COURTE)
// ============================================================================
class SurveyScanCallbacks : public NimBLEScanCallbacks {
public:
    void onDiscovered(const NimBLEAdvertisedDevice* pDev) override {
        if (!pDev) return;
        std::string macStd = pDev->getAddress().toString();
        int8_t dbm = pDev->getRSSI();
        uint32_t nowMs = millis();

        bool hasFFF0 = (pDev->haveServiceUUID() && pDev->isAdvertisingService(SERVICE_FFF0_UUID));
        bool looksLikeSem = hasFFF0;
        if (!looksLikeSem && pDev->haveName()) {
            std::string nm = pDev->getName();
            if (nm.find("Voltcraft") != std::string::npos || nm.find("SEM") != std::string::npos) {
                looksLikeSem = true;
            }
        }

        double mw = pow(10.0, (double)dbm / 10.0);

        SpinLockGuard lock(&g_statsMux);
        g_totalBlePacketsInWindow = g_totalBlePacketsInWindow + 1;

        int idx = findSlotByMacLocked(macStd.c_str());
        if (idx < 0 && looksLikeSem) {
            for (size_t i = KNOWN_PLUG_COUNT; i < MAX_TRACKED_SLOTS; i++) {
                if (!g_slots[i].inUse) {
                    g_slots[i].inUse   = true;
                    g_slots[i].isKnown = false;
                    snprintf(g_slots[i].knownName, sizeof(g_slots[i].knownName), "INCONNUE_%02u", (unsigned)(i - KNOWN_PLUG_COUNT + 1));
                    strncpy(g_slots[i].mac, macStd.c_str(), sizeof(g_slots[i].mac) - 1);
                    g_slots[i].minDbm  = 0;
                    g_slots[i].maxDbm  = -127;
                    idx = (int)i;
                    break;
                }
            }
        }

        if (idx >= 0) {
            DeviceRadioStats& s = g_slots[idx];
            if (s.pktCount == 0) {
                s.firstSeenMs = nowMs;
                s.minDbm      = dbm;
                s.maxDbm      = dbm;
            } else {
                if (dbm < s.minDbm) s.minDbm = dbm;
                if (dbm > s.maxDbm) s.maxDbm = dbm;
            }
            s.lastSeenMs     = nowMs;
            s.pktCount++;
            s.sumDbm        += (int32_t)dbm;
            s.sumSqDbm      += (int64_t)dbm * (int64_t)dbm;
            s.sumMilliwatts += mw;

            enrichSlotMetadataLocked(s, pDev);
        }
    }

    void onResult(const NimBLEAdvertisedDevice* pDev) override {
        if (!pDev) return;
        std::string macStd = pDev->getAddress().toString();

        bool hasFFF0 = (pDev->haveServiceUUID() && pDev->isAdvertisingService(SERVICE_FFF0_UUID));
        bool looksLikeSem = hasFFF0;
        if (!looksLikeSem && pDev->haveName()) {
            std::string nm = pDev->getName();
            if (nm.find("Voltcraft") != std::string::npos || nm.find("SEM") != std::string::npos) {
                looksLikeSem = true;
            }
        }

        SpinLockGuard lock(&g_statsMux);
        int idx = findSlotByMacLocked(macStd.c_str());
        if (idx < 0 && looksLikeSem) {
            int8_t dbm = pDev->getRSSI();
            uint32_t nowMs = millis();
            double mw = pow(10.0, (double)dbm / 10.0);
            for (size_t i = KNOWN_PLUG_COUNT; i < MAX_TRACKED_SLOTS; i++) {
                if (!g_slots[i].inUse) {
                    g_slots[i].inUse         = true;
                    g_slots[i].isKnown       = false;
                    snprintf(g_slots[i].knownName, sizeof(g_slots[i].knownName), "INCONNUE_%02u", (unsigned)(i - KNOWN_PLUG_COUNT + 1));
                    strncpy(g_slots[i].mac, macStd.c_str(), sizeof(g_slots[i].mac) - 1);
                    g_slots[i].firstSeenMs   = nowMs;
                    g_slots[i].lastSeenMs    = nowMs;
                    g_slots[i].minDbm        = dbm;
                    g_slots[i].maxDbm        = dbm;
                    g_slots[i].pktCount      = 1;
                    g_slots[i].sumDbm        = dbm;
                    g_slots[i].sumSqDbm      = (int64_t)dbm * (int64_t)dbm;
                    g_slots[i].sumMilliwatts = mw;
                    idx = (int)i;
                    break;
                }
            }
        }
        if (idx >= 0) {
            enrichSlotMetadataLocked(g_slots[idx], pDev);
        }
    }
};

static SurveyScanCallbacks g_scanCallbacks;

// ============================================================================
// 6. SONDE ACTIVE GATT SANS PIN (2A00 + FFF1 + FFF2) SUR LES PRISES ENTENDUES
// ============================================================================
static void runActiveGattProbeOnHeardDevices() {
    int heardToProbe = 0;
    for (size_t i = 0; i < MAX_TRACKED_SLOTS; i++) {
        if (g_slots[i].inUse && g_slots[i].pktCount > 0) heardToProbe++;
    }

    Serial.printf("    2/3 Sonde active GATT sans PIN ('Commande F') sur %d prise(s) entendue(s)...\n", heardToProbe);
    if (heardToProbe == 0) return;

    int cur = 0;
    for (size_t i = 0; i < MAX_TRACKED_SLOTS; i++) {
        DeviceRadioStats& s = g_slots[i];
        if (!s.inUse || s.pktCount == 0) continue;

        cur++;
        s.probeAttempted = true;
        s.probeConnected = false;
        float avgDbm = (float)s.sumDbm / (float)s.pktCount;

        Serial.printf("        [%d/%d] Sonde GATT -> %-20.20s (%s, Moy: %.1f dBm)...",
                      cur, heardToProbe, s.knownName, s.mac, avgDbm);

        NimBLEClient* pClient = NimBLEDevice::createClient();
        pClient->setConnectTimeout(GATT_PROBE_TIMEOUT_MS);
        NimBLEAddress addr(s.mac, s.addrType);

        uint32_t t0 = millis();
        if (!pClient->connect(addr)) {
            uint32_t dt = millis() - t0;
            Serial.printf(" ❌ Echec (%lu ms)\n", (unsigned long)dt);
            NimBLEDevice::deleteClient(pClient);
            delay(100);
            continue;
        }

        s.probeConnMs    = millis() - t0;
        s.probeConnected = true;
        s.probeConnRssi  = pClient->getRssi();

        // 1. Lecture optionnelle du Nom GATT personnalisé (Service 0x1800 / Char 0x2A00)
        NimBLERemoteService* pGap = pClient->getService(SERVICE_GAP_UUID);
        if (pGap) {
            NimBLERemoteCharacteristic* pNameChar = pGap->getCharacteristic(CHAR_GAP_NAME_UUID);
            if (pNameChar && pNameChar->canRead()) {
                std::string gName = pNameChar->readValue();
                if (!gName.empty()) {
                    strncpy(s.gattName, gName.c_str(), sizeof(s.gattName) - 1);
                    s.gattName[sizeof(s.gattName) - 1] = '\0';
                }
            }
        }

        // 2. Lecture de FFF1 (Vendor + FW/HW) et FFF2 (Seuil mA + Courant mA)
        NimBLERemoteService* pSvc = pClient->getService(SERVICE_FFF0_UUID);
        if (pSvc) {
            NimBLERemoteCharacteristic* pFFF1 = pSvc->getCharacteristic(CHAR_FFF1_UUID);
            if (pFFF1 && pFFF1->canRead()) {
                std::string v1 = pFFF1->readValue();
                if (v1.size() >= 16) {
                    memcpy(s.vendor, v1.data(), 6);
                    s.vendor[6] = '\0';
                    s.fwMaj = (uint8_t)v1[11];
                    s.fwMin = (uint8_t)v1[12];
                    s.hwMaj = (uint8_t)v1[13];
                    s.hwMin = (uint8_t)v1[14];
                }
            }

            NimBLERemoteCharacteristic* pFFF2 = pSvc->getCharacteristic(CHAR_FFF2_UUID);
            if (pFFF2 && pFFF2->canRead()) {
                std::string v2 = pFFF2->readValue();
                if (v2.size() >= 4) {
                    s.fff2ImaxMa = ((uint16_t)(uint8_t)v2[0] << 8) | (uint8_t)v2[1];
                    s.fff2ImesMa = ((uint16_t)(uint8_t)v2[2] << 8) | (uint8_t)v2[3];
                }
            }
        }

        pClient->disconnect();
        NimBLEDevice::deleteClient(pClient);

        Serial.printf(" ✅ OK (%lu ms, %d dBm) | GATT='%s' | %s FW %u.%u | Imax=%u mA, I=%u mA\n",
                      (unsigned long)s.probeConnMs,
                      s.probeConnRssi,
                      (s.gattName[0] != '\0') ? s.gattName : "?",
                      (s.vendor[0] != '\0') ? s.vendor : "?",
                      (unsigned)s.fwMaj, (unsigned)s.fwMin,
                      (unsigned)s.fff2ImaxMa, (unsigned)s.fff2ImesMa);
        delay(150);
    }
}

// ============================================================================
// 7. SCAN PASSIF WI-FI 2.4 GHz EN dBm (EXÉCUTÉ HORS FENÊTRE BLE)
// ============================================================================
static void runPassiveWifiSurvey(WifiSurveyResult& out) {
    memset(&out, 0, sizeof(out));
    out.targetDbm = -127;
    out.bestDbm   = -127;
    strncpy(out.targetSsid, "NON_RECU", sizeof(out.targetSsid) - 1);
    strncpy(out.bestSsid, "AUCUN", sizeof(out.bestSsid) - 1);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);
    delay(50);

    int n = WiFi.scanNetworks(false, false);
    out.totalAp24GHz = (n > 0) ? n : 0;

    for (int i = 0; i < n; i++) {
        char ssidBuf[33];
        strncpy(ssidBuf, WiFi.SSID(i).c_str(), sizeof(ssidBuf) - 1);
        ssidBuf[sizeof(ssidBuf) - 1] = '\0';
        int32_t rssi   = WiFi.RSSI(i);
        int32_t ch     = WiFi.channel(i);

        if (ch >= 1 && ch <= 3)       out.ch1Count++;
        else if (ch >= 4 && ch <= 8)  out.ch6Count++;
        else if (ch >= 9 && ch <= 14) out.ch11Count++;

        if (rssi > out.bestDbm) {
            out.bestDbm     = rssi;
            out.bestChannel = ch;
            strncpy(out.bestSsid, ssidBuf, sizeof(out.bestSsid) - 1);
            out.bestSsid[sizeof(out.bestSsid) - 1] = '\0';
        }

        if (strcasecmp(ssidBuf, TARGET_WIFI_SSID) == 0 ||
            strcasestr(ssidBuf, TARGET_WIFI_PREFIX) != nullptr) {
            if (!out.targetFound || rssi > out.targetDbm) {
                out.targetFound   = true;
                out.targetDbm     = rssi;
                out.targetChannel = ch;
                strncpy(out.targetSsid, ssidBuf, sizeof(out.targetSsid) - 1);
                out.targetSsid[sizeof(out.targetSsid) - 1] = '\0';
            }
        }
    }

    WiFi.scanDelete();
    WiFi.mode(WIFI_OFF);
    delay(50);
}

// ============================================================================
// 8. EXÉCUTION COMPLÈTE D'UN POINT DE MESURE (60s BLE + SONDE F AUTO + WI-FI)
// ============================================================================
static void runSurveyAtPoint(const char* locationLabel) {
    g_pointCounter++;
    resetSlotsForNewWindow();

    WiFi.mode(WIFI_OFF);
    delay(50);

    const uint32_t expectedNominalPkts = BLE_SURVEY_WINDOW_MS / NOMINAL_ADV_INTERVAL_MS; // 1000 paquets sur 60s

    Serial.println("\n================================================================================");
    Serial.printf(" 📡 DÉMARRAGE POINT #%02lu : \"%s\"\n", (unsigned long)g_pointCounter, locationLabel);
    Serial.printf("    1/3 Écoute BLE pure pendant %lu secondes (Wi-Fi OFF, ~%lu paquets attendus/prise)...\n",
                  (unsigned long)(BLE_SURVEY_WINDOW_MS / 1000),
                  (unsigned long)expectedNominalPkts);
    Serial.println("================================================================================");

    NimBLEScan* pScan = NimBLEDevice::getScan();
    pScan->setScanCallbacks(&g_scanCallbacks, true); // wantDuplicates = true
    pScan->setActiveScan(true);
    pScan->setInterval(50);
    pScan->setWindow(50);
    pScan->setMaxResults(0);

    pScan->start(BLE_SURVEY_WINDOW_MS, false, true);

    uint32_t startMs = millis();
    uint32_t nextReportMs = 5000; // Point de progression toutes les 5 secondes sur 60s
    while (pScan->isScanning() && (millis() - startMs < BLE_SURVEY_WINDOW_MS + 500)) {
        uint32_t elapsed = millis() - startMs;
        if (elapsed >= nextReportMs) {
            uint32_t remSec = (BLE_SURVEY_WINDOW_MS > elapsed) ? ((BLE_SURVEY_WINDOW_MS - elapsed + 500) / 1000) : 0;
            int heardSoFar = 0;
            for (size_t i = 0; i < MAX_TRACKED_SLOTS; i++) {
                if (g_slots[i].inUse && g_slots[i].pktCount > 0) heardSoFar++;
            }
            Serial.printf("    ⏳ [%2lu s restantes] Trames BLE : %4lu | Prises entendues : %d\n",
                          (unsigned long)remSec,
                          (unsigned long)g_totalBlePacketsInWindow,
                          heardSoFar);
            nextReportMs += 5000;
        }
        delay(50);
    }
    pScan->stop();
    pScan->clearResults();
    delay(150);

    // Phase 2 : Exécution AUTOMATIQUE de la sonde F (GATT sans PIN : 2A00 + FFF1 + FFF2)
    runActiveGattProbeOnHeardDevices();

    // Phase 3 : Scan passif Wi-Fi 2.4 GHz
    Serial.println("    3/3 Scan passif Wi-Fi 2.4 GHz (2 secondes)...");
    WifiSurveyResult wifiRes;
    runPassiveWifiSurvey(wifiRes);

    // Construction et sauvegarde du rapport
    char lineBuf[320];
    File repFile;
    if (g_ffatMounted) {
        repFile = FFat.open(REPORT_FILE_PATH, FILE_APPEND);
    }

    auto printBoth = [&](const char* txt) {
        Serial.print(txt);
        if (repFile) repFile.print(txt);
    };

    printBoth("\n########################################################################################################################\n");
    snprintf(lineBuf, sizeof(lineBuf),
             " 📍 POINT #%02lu : \"%s\"  (Fenêtre BLE: %lu s | Total trames BLE: %lu | Nominal/prise: %lu)\n",
             (unsigned long)g_pointCounter, locationLabel,
             (unsigned long)(BLE_SURVEY_WINDOW_MS / 1000),
             (unsigned long)g_totalBlePacketsInWindow,
             (unsigned long)expectedNominalPkts);
    printBoth(lineBuf);

    if (wifiRes.targetFound) {
        snprintf(lineBuf, sizeof(lineBuf),
                 " 📶 Wi-Fi Cible [%s] : %ld dBm (Canal %ld) | AP 2.4GHz vus: %ld (Ch1:%ld Ch6:%ld Ch11:%ld)\n",
                 wifiRes.targetSsid, (long)wifiRes.targetDbm, (long)wifiRes.targetChannel,
                 (long)wifiRes.totalAp24GHz, (long)wifiRes.ch1Count, (long)wifiRes.ch6Count, (long)wifiRes.ch11Count);
    } else {
        snprintf(lineBuf, sizeof(lineBuf),
                 " 📶 Wi-Fi Cible [%s*] : NON REÇU EN 2.4GHz | Meilleur AP 2.4GHz: '%s' (%ld dBm, Ch.%ld) | Total APs: %ld\n",
                 TARGET_WIFI_PREFIX, wifiRes.bestSsid, (long)wifiRes.bestDbm, (long)wifiRes.bestChannel,
                 (long)wifiRes.totalAp24GHz);
    }
    printBoth(lineBuf);
    printBoth("########################################################################################################################\n");
    printBoth(" Prise                | MAC               | Paq  | PRR % | Int.Moy | Moy dBm | Pwr dBm | Eff dBm | Min/Max   | Sig | Sonde F (GATT sans PIN)\n");
    printBoth("----------------------+-------------------+------+-------+---------+---------+---------+---------+-----------+-----+------------------------\n");

    File csvFile;
    if (g_ffatMounted) {
        bool exists = FFat.exists(CSV_FILE_PATH);
        csvFile = FFat.open(CSV_FILE_PATH, FILE_APPEND);
        if (csvFile && !exists) {
            csvFile.println("PointID;Lieu;Prise;Connue;MAC;AddrType;HasFFF0;AdvName;MfgHex;Paquets;PRR_Pct;Intervalle_ms;Moy_dBm;MoyPwr_dBm;Eff_dBm;Min_dBm;Max_dBm;Sigma_dB;GATT_OK;GATT_Conn_ms;GATT_Conn_dBm;GATT_Name;Vendor;FW;HW;FFF2_Imax_mA;FFF2_Imes_mA;WiFi_Target_SSID;WiFi_Target_dBm;WiFi_Best_SSID;WiFi_Best_dBm;WiFi_Best_Ch;WiFi_TotalAPs");
        }
    }

    int heardCount = 0;
    int gattOkCount = 0;

    for (size_t i = 0; i < MAX_TRACKED_SLOTS; i++) {
        const DeviceRadioStats& s = g_slots[i];
        if (!s.inUse) continue;

        float    avgDbm   = 0.0f;
        float    pwrDbm   = 0.0f;
        float    effDbm   = 0.0f;
        float    sigmaDb  = 0.0f;
        float    prrPct   = 0.0f;
        uint32_t avgIntMs = 0;

        if (s.pktCount > 0) {
            heardCount++;
            if (s.probeConnected) gattOkCount++;

            prrPct = (100.0f * (float)s.pktCount) / (float)expectedNominalPkts;
            if (prrPct > 100.0f) prrPct = 100.0f;

            avgDbm = (float)s.sumDbm / (float)s.pktCount;
            double avgMw = s.sumMilliwatts / (double)s.pktCount;
            pwrDbm = (avgMw > 0.0) ? (float)(10.0 * log10(avgMw)) : avgDbm;

            // Puissance effective sur l'ensemble des paquets attendus : sumMilliwatts / expectedNominalPkts
            double effMw = s.sumMilliwatts / (double)expectedNominalPkts;
            effDbm = (effMw > 0.0) ? (float)(10.0 * log10(effMw)) : -110.0f;

            double meanSq = (double)s.sumSqDbm / (double)s.pktCount;
            double var = meanSq - ((double)avgDbm * (double)avgDbm);
            sigmaDb = (var > 0.0) ? (float)sqrt(var) : 0.0f;

            if (s.pktCount > 1 && s.lastSeenMs > s.firstSeenMs) {
                avgIntMs = (s.lastSeenMs - s.firstSeenMs) / (s.pktCount - 1);
            } else {
                avgIntMs = BLE_SURVEY_WINDOW_MS;
            }

            char probeSummary[64];
            if (s.probeConnected) {
                snprintf(probeSummary, sizeof(probeSummary),
                         "✅ %lums (%ddBm) Imax=%umA I=%umA",
                         (unsigned long)s.probeConnMs, s.probeConnRssi,
                         (unsigned)s.fff2ImaxMa, (unsigned)s.fff2ImesMa);
            } else {
                snprintf(probeSummary, sizeof(probeSummary), "❌ ECHEC CONNEXION (>4s)");
            }

            snprintf(lineBuf, sizeof(lineBuf),
                     " %-20.20s | %-17s | %4lu | %5.1f | %4lu ms |  %6.1f |  %6.1f |  %6.1f | %4d/%-4d | %3.1f | %s\n",
                     s.knownName, s.mac,
                     (unsigned long)s.pktCount,
                     prrPct,
                     (unsigned long)avgIntMs,
                     avgDbm, pwrDbm, effDbm,
                     (int)s.minDbm, (int)s.maxDbm,
                     sigmaDb,
                     probeSummary);
            printBoth(lineBuf);
        } else {
            snprintf(lineBuf, sizeof(lineBuf),
                     " %-20.20s | %-17s |    0 |   0.0 |    -    |     -   |     -   |     -   |   - / -   |  -  | ⚫ HORS PORTEE\n",
                     s.knownName, s.mac);
            printBoth(lineBuf);
        }

        if (csvFile) {
            char fwStr[12] = "";
            char hwStr[12] = "";
            if (s.probeConnected) {
                snprintf(fwStr, sizeof(fwStr), "%u.%u", (unsigned)s.fwMaj, (unsigned)s.fwMin);
                snprintf(hwStr, sizeof(hwStr), "%u.%u", (unsigned)s.hwMaj, (unsigned)s.hwMin);
            }
            csvFile.printf("%lu;%s;%s;%d;%s;%s;%d;%s;%s;%lu;%.1f;%lu;%.2f;%.2f;%.2f;%d;%d;%.2f;%d;%lu;%d;%s;%s;%s;%s;%u;%u;%s;%ld;%s;%ld;%ld;%ld\n",
                           (unsigned long)g_pointCounter,
                           locationLabel,
                           s.knownName,
                           s.isKnown ? 1 : 0,
                           s.mac,
                           (s.pktCount > 0) ? ((s.addrType == 0) ? "PUBLIC" : "RANDOM") : "NONE",
                           s.hasFFF0 ? 1 : 0,
                           (s.advName[0] != '\0') ? s.advName : "",
                           s.mfgHex,
                           (unsigned long)s.pktCount,
                           prrPct,
                           (unsigned long)avgIntMs,
                           avgDbm,
                           pwrDbm,
                           effDbm,
                           (s.pktCount > 0) ? (int)s.minDbm : 0,
                           (s.pktCount > 0) ? (int)s.maxDbm : 0,
                           sigmaDb,
                           s.probeConnected ? 1 : 0,
                           (unsigned long)s.probeConnMs,
                           s.probeConnected ? s.probeConnRssi : 0,
                           s.gattName,
                           s.vendor,
                           fwStr,
                           hwStr,
                           (unsigned)s.fff2ImaxMa,
                           (unsigned)s.fff2ImesMa,
                           wifiRes.targetSsid,
                           (long)wifiRes.targetDbm,
                           wifiRes.bestSsid,
                           (long)wifiRes.bestDbm,
                           (long)wifiRes.bestChannel,
                           (long)wifiRes.totalAp24GHz);
        }
    }

    printBoth("------------------------------------------------------------------------------------------------------------------------\n");
    snprintf(lineBuf, sizeof(lineBuf),
             " 📊 Bilan Point #%02lu (%s) : %d prise(s) entendue(s) en ADV | %d prise(s) connectée(s) en GATT (Sonde F).\n",
             (unsigned long)g_pointCounter, locationLabel, heardCount, gattOkCount);
    printBoth(lineBuf);
    printBoth("########################################################################################################################\n");

    if (csvFile) csvFile.close();
    if (repFile) repFile.close();

    if (g_ffatMounted) {
        Serial.printf(" 💾 [FLASH] Point #%02lu (\"%s\") sauvegardé dans %s et %s !\n",
                      (unsigned long)g_pointCounter, locationLabel, CSV_FILE_PATH, REPORT_FILE_PATH);
    }
}

// ============================================================================
// 9. GESTION DES FICHIERS FLASH (DUMP & EFFACEMENT)
// ============================================================================
static void dumpSavedFilesToSerial() {
    if (!g_ffatMounted) {
        Serial.println("❌ Partition FFat non montée !");
        return;
    }

    Serial.println("\n==================== 📄 DÉBUT DUMP RAPPORTS (exp14_reports.txt) ====================");
    if (FFat.exists(REPORT_FILE_PATH)) {
        File f = FFat.open(REPORT_FILE_PATH, FILE_READ);
        while (f && f.available()) {
            Serial.write(f.read());
        }
        if (f) f.close();
    } else {
        Serial.println("(Aucun rapport enregistré)");
    }
    Serial.println("\n==================== 📄 FIN DUMP RAPPORTS ====================\n");

    Serial.println("==================== 📊 DÉBUT EXPORT CSV (exp14_survey.csv) ====================");
    if (FFat.exists(CSV_FILE_PATH)) {
        File f = FFat.open(CSV_FILE_PATH, FILE_READ);
        while (f && f.available()) {
            Serial.write(f.read());
        }
        if (f) f.close();
    } else {
        Serial.println("(Aucun fichier CSV enregistré)");
    }
    Serial.println("==================== 📊 FIN EXPORT CSV ====================\n");
}

static void eraseSavedFiles() {
    if (!g_ffatMounted) return;
    FFat.remove(CSV_FILE_PATH);
    FFat.remove(REPORT_FILE_PATH);
    g_pointCounter = 0;
    Serial.println("\n🧹 [FLASH] Fichiers CSV et Rapports effacés ! Compteur remis à 0 pour la nouvelle campagne.");
}

// ============================================================================
// 10. MENU PRINCIPAL & ATTENTE HYBRIDE (PARCOURS PRÉ-PROGRAMMÉ)
// ============================================================================
static void printPromptBanner() {
    char fallbackBuf[32];
    const char* nextStation = getNextPlannedLabel(fallbackBuf, sizeof(fallbackBuf));

    Serial.println("\n----------------------------------------------------------------------------------------");
    Serial.printf(" 🧭 POINTS ENREGISTRÉS : %lu/%u  |  PROCHAINE ÉTAPE PRÉVUE : \"%s\"\n",
                  (unsigned long)g_pointCounter, (unsigned)PLANNED_ROUTE_COUNT, nextStation);
    Serial.println("----------------------------------------------------------------------------------------");
    Serial.printf("   👉 [Bouton BOOT ou Entrée vide] : Lancer \"%s\" (60s BLE + Sonde F auto + Wi-Fi)\n", nextStation);
    Serial.println("   👉 [Tapez un autre nom+Entrée]  : Lancer avec un nom personnalisé");
    Serial.println("   👉 [F + Entrée]                 : Relancer uniquement la Sonde F sur le dernier point");
    Serial.println("   👉 [D + Entrée]                 : Afficher (Dump) tous les rapports + le CSV complet");
    Serial.println("   👉 [E + Entrée]                 : Effacer la Flash et repartir au Point #01 (01_Sous-sol)");
    Serial.println("----------------------------------------------------------------------------------------");
}

static bool isDumpCommand(const char* cmd) {
    return (strcasecmp(cmd, "d") == 0 || strcasecmp(cmd, "dd") == 0 || strcasecmp(cmd, "dump") == 0);
}

static bool isEraseCommand(const char* cmd) {
    return (strcasecmp(cmd, "e") == 0 || strcasecmp(cmd, "ee") == 0 || strcasecmp(cmd, "erase") == 0);
}

static bool isProbeCommand(const char* cmd) {
    return (strcasecmp(cmd, "f") == 0 || strcasecmp(cmd, "ff") == 0);
}

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_BOOT_PIN, INPUT_PULLUP);

    unsigned long startWait = millis();
    while (!Serial && (millis() - startWait < 3000)) {
        delay(50);
    }
    delay(500);

    Serial.println("\n\n========================================================================================");
    Serial.println("  📡 EXPÉRIENCE 14 (v2) : SONDEUR RADIO MOBILE (60s BLE + Sonde F auto + Wi-Fi)");
    Serial.println("     Parcours guidé : 1.Sous-sol -> 2.Garage -> 3.Cuisine -> 4.SAM/Salon");
    Serial.println("                      -> 5.Bureau -> 6.Palier étage -> 7.Chambre Cédric");
    Serial.println("========================================================================================");

    if (FFat.begin(true)) {
        g_ffatMounted = true;
        if (FFat.exists(REPORT_FILE_PATH)) {
            File f = FFat.open(REPORT_FILE_PATH, FILE_READ);
            char scanBuf[128];
            while (f && f.available()) {
                size_t rLen = f.readBytesUntil('\n', scanBuf, sizeof(scanBuf) - 1);
                scanBuf[rLen] = '\0';
                if (strstr(scanBuf, "POINT #") != nullptr) {
                    g_pointCounter++;
                }
            }
            if (f) f.close();
        }
        Serial.printf("  💾 Flash FFat montée : %lu Ko libres | %lu point(s) actuellement en mémoire Flash\n",
                      (unsigned long)(FFat.freeBytes() / 1024),
                      (unsigned long)g_pointCounter);
    } else {
        Serial.println("  ⚠️ Impossible de monter FFat (fonctionnement en affichage console uniquement).");
    }

    WiFi.mode(WIFI_OFF);
    NimBLEDevice::init("SEM6000_Survey14");

    resetSlotsForNewWindow();
    printPromptBanner();
}

void loop() {
    static char inputBuf[64];
    static size_t inputPos = 0;

    // 1. Bouton physique BOOT -> Lance automatiquement la prochaine étape du parcours pré-programmé
    if (digitalRead(BUTTON_BOOT_PIN) == LOW) {
        delay(120);
        while (digitalRead(BUTTON_BOOT_PIN) == LOW) {
            delay(20);
        }
        char fallbackBuf[32];
        const char* label = getNextPlannedLabel(fallbackBuf, sizeof(fallbackBuf));
        runSurveyAtPoint(label);
        inputPos = 0;
        printPromptBanner();
        return;
    }

    // 2. Saisie série (Console Smartphone OTG ou PC)
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n') {
            inputBuf[inputPos] = '\0';

            while (inputPos > 0 && (inputBuf[inputPos - 1] == ' ' || inputBuf[inputPos - 1] == '\t')) {
                inputBuf[--inputPos] = '\0';
            }

            if (inputPos == 0) {
                char fallbackBuf[32];
                const char* label = getNextPlannedLabel(fallbackBuf, sizeof(fallbackBuf));
                runSurveyAtPoint(label);
                printPromptBanner();
            } else if (isDumpCommand(inputBuf)) {
                dumpSavedFilesToSerial();
                printPromptBanner();
            } else if (isEraseCommand(inputBuf)) {
                eraseSavedFiles();
                printPromptBanner();
            } else if (isProbeCommand(inputBuf)) {
                runActiveGattProbeOnHeardDevices();
                printPromptBanner();
            } else {
                for (size_t i = 0; i < inputPos; i++) {
                    if (inputBuf[i] == ';') inputBuf[i] = '_';
                }
                runSurveyAtPoint(inputBuf);
                printPromptBanner();
            }

            inputPos = 0;
            delay(20);
            while (Serial.available()) {
                char peekC = (char)Serial.peek();
                if (peekC == '\r' || peekC == '\n') Serial.read();
                else break;
            }
            return;
        } else if (inputPos < sizeof(inputBuf) - 1) {
            inputBuf[inputPos++] = c;
        }
    }

    delay(30);
}
