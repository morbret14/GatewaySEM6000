/*
 * SEM6000Device.h - Version 0.4.0 — "La Grande Réunification (One Roof for Two Hardwares : HW 2.0 & HW 3.0)"
 * Interface unifiée BLE Bi-Génération pour Voltcraft SEM6000-FR :
 *   - Hardware 2.0 / Firmware 1.24 (MTU 23, caractéristique 0xFFF2 présente)
 *   - Hardware 3.0 / Firmware 1.30 (MTU 160, sans 0xFFF2, compteur Flash totalKwhWh actif)
 * Support hybride : PIN BCD (0..9999) & PIN32 Binaire (0x00000000..0xFFFFFFFF)
 *
 * [Limites connues / TODO avant publication open-source publique] :
 *   - Sur HW 3.0 / FW 1.30, l'octet de checksum des historiques 0x0A00 (24h, omission des premiers slots de gauche)
 *     et 0x0C00 (12m, biais calendaire lié à la date RTC) diverge de la formule standard de HW 2.0.
 *     La validation sur HW 3.0 s'appuie sur le paquet BLE monolithique (MTU 160, CRC-24 Link Layer) et la
 *     vérification structurelle stricte (0x0F, longueur logique Byte[1]+4 = 55, marqueur de fin FF FF).
 *     L'état de conformité mathématique de la dernière checksum reçue est exposé via wasLastChecksumValid().
 */

#ifndef SEM6000_DEVICE_H
#define SEM6000_DEVICE_H

#define SEM6000_DEVICE_VERSION_MAJOR 0
#define SEM6000_DEVICE_VERSION_MINOR 4
#define SEM6000_DEVICE_VERSION_PATCH 0
#define SEM6000_DEVICE_VERSION_HEX   0x000400
#define SEM6000_DEVICE_VERSION_STR   "0.4.0"

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <vector>
#include <ctime>

enum SEM6000Status {
    SEM_OK = 0,
    SEM_ERR_NOT_CONNECTED,
    SEM_ERR_AUTH_FAILED,
    SEM_ERR_INVALID_PARAM,
    SEM_ERR_BLE_WRITE_FAILED,
    SEM_ERR_TIMEOUT,
    SEM_ERR_RESPONSE_REJECTED,
    SEM_ERR_NOTIFICATION,
    SEM_ERR_NOT_SUPPORTED
};

enum SEMCmdType {
    CMD_GET,
    CMD_SET_2B,
    CMD_SET_3B
};

enum SEMSchedulerMode {
    SCHED_ONCE = 0x00,
    SCHED_REPEAT_WEEKLY = 0x01
};

struct SEMSettings {
    uint8_t reducedModeActive;
    uint8_t normalPriceCents;
    uint8_t reducedPriceCents;
    uint16_t reducedPeriodStart;
    uint16_t reducedPeriodStop;
    uint8_t ledStatus;
    uint16_t maxPowerWatts;
};

struct SEMTimerStatus {
    bool active;
    bool actionTurnOn;
    uint8_t targetSec;
    uint8_t targetMin;
    uint8_t targetHour;
    uint8_t targetDay;
    uint8_t targetMonth;
    uint8_t targetYear;
    uint32_t presetSeconds;
};

struct SEMSchedulerSlot {
    uint8_t slotId;
    bool active;
    bool actionTurnOn;
    uint8_t dayMask;
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sortKey;
};

struct SEMRandomConfig {
    bool active;
    uint8_t dayMask;
    uint8_t startHour;
    uint8_t startMin;
    uint8_t endHour;
    uint8_t endMin;
};

struct SEMTelemetry {
    bool relayState;
    float powerWatts;
    uint8_t voltageVolts;
    float currentAmperes;
    uint8_t frequencyHz;
    uint16_t unknownWord;
    uint32_t totalKwhWh;
};

struct SEMDeviceInfo {
    String vendor;
    String firmwareVersion;
    String hardwareVersion;
    uint8_t byte16;
};

class SEM6000Device {
public:
    SEM6000Device(const char* macAddress, uint32_t pin = 0);
    ~SEM6000Device();

    bool connect(uint32_t timeoutMs = 5000);
    void disconnect();
    bool isConnected() const { return m_connected; }
    bool isAuthorized() const { return m_authorized; }

    // --- Identification Matérielle Bi-Génération (v0.4.0) ---
    bool hasCharacteristicFFF2() const { return m_pCharFFF2 != nullptr; }
    bool isHardwareV3() const { return m_connected && (m_pCharFFF2 == nullptr); }
    bool wasLastChecksumValid() const { return m_lastChecksumValid; }

    // --- 1. PIN & Authentification ---
    // Méthodes historiques (PIN BCD 0..9999)
    SEM6000Status authenticate(uint32_t pin);
    SEM6000Status changePin(uint32_t oldPin, uint32_t newPin);

    // PIN Binaire 32 bits (0x00000000..0xFFFFFFFF)
    SEM6000Status authenticate32(const uint8_t pin[4]);
    SEM6000Status authenticate32(uint32_t pinRaw);
    SEM6000Status changePin32(const uint8_t oldPin[4], const uint8_t newPin[4]);
    SEM6000Status changePin32(uint32_t oldPinRaw, uint32_t newPinRaw);

    SEM6000Status resetPinToDefault();

    // --- 2. Configuration & Paramètres ---
    SEM6000Status setDateTime(uint8_t sec, uint8_t min, uint8_t hour, uint8_t day, uint8_t month, uint16_t year);
    SEM6000Status requestSettings(SEMSettings& settings);
    SEM6000Status setLedRing(bool enable);
    SEM6000Status setOverloadPower(uint16_t maxWatts);
    SEM6000Status setPrices(float standardCents, float reducedCents);
    SEM6000Status setReducedPeriod(uint8_t onOff, uint8_t startHour, uint8_t startMin, uint8_t endHour, uint8_t endMin);

    // --- 3. Commande Relais ---
    SEM6000Status setRelayState(bool turnOn);

    // --- 4. Timer ---
    SEM6000Status getTimerStatus(SEMTimerStatus& status);
    SEM6000Status setTimerAbs(bool turnOn, uint8_t sec, uint8_t min, uint8_t hour, uint8_t day, uint8_t month, uint16_t year);
    SEM6000Status setTimerRel(bool turnOn, uint32_t durationSeconds, const struct tm& currentSystemTime);
    SEM6000Status stopTimer();

    // --- 5. Plannings (Scheduler) ---
    SEM6000Status requestSchedulerPage(uint8_t page, uint8_t& totalCountOut);
    SEM6000Status requestAllSchedulers(std::vector<SEMSchedulerSlot>& list);
    SEM6000Status setScheduler(SEMSchedulerMode mode, uint8_t slotId, bool enable, bool turnOn, uint8_t dayMask, uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min);
    SEM6000Status deleteScheduler(uint8_t slotId);
    SEM6000Status resetScheduler();

    // --- 6. Mode Aléatoire (Présence) ---
    SEM6000Status getRandomMode(SEMRandomConfig& config);
    SEM6000Status setRandomMode(bool active, uint8_t dayMask, uint8_t startHour, uint8_t startMin, uint8_t endHour, uint8_t endMin);

    // --- 7. Télémétrie & Historiques ---
    SEM6000Status captureMeasurement(SEMTelemetry& telemetry);
    SEM6000Status requestDayHistory(uint16_t hourlyWh[24]);
    SEM6000Status requestMonthHistory(uint32_t dailyWh[30]);
    SEM6000Status requestYearHistory(uint32_t monthlyWh[12]);
    SEM6000Status resetConsumptionData();
    SEM6000Status factoryReset();

    // --- 8. Infos Device & Nom GATT ---
    SEM6000Status getDeviceVendorAndVersions(SEMDeviceInfo& info);
    SEM6000Status getDeviceNameGATT(String& name);
    SEM6000Status setDeviceName(const String& name);
    SEM6000Status getSerialNumber(String& serial);

    // --- Outils d'inspection & Horloge ---
    bool readRawFFF2(uint16_t& word0, uint8_t& currentByte, float& currentAmpsDirect);
    SEM6000Status checkAndFixClockLoss(uint8_t sec, uint8_t min, uint8_t hour, uint8_t day, uint8_t month, uint16_t year, bool* outClockLossDetected = nullptr);
    size_t getLastRxBuffer(const uint8_t** outBufPtr) const { *outBufPtr = m_rxBuffer; return m_rxLength; }

private:
    String m_macAddress;
    uint32_t m_pin;
    bool m_connected;
    bool m_authorized;

    NimBLEClient* m_pClient;
    NimBLERemoteCharacteristic* m_pCharReq;
    NimBLERemoteCharacteristic* m_pCharRes;
    NimBLERemoteCharacteristic* m_pCharFFF2;
    NimBLERemoteCharacteristic* m_pCharFFF1;
    NimBLERemoteCharacteristic* m_pCharName;

    uint8_t m_rxBuffer[160];
    size_t m_rxLength;
    volatile bool m_rxReady;
    volatile bool m_lastChecksumValid;
    volatile uint16_t m_expectedCmd;

    uint8_t calculateChecksum(const uint8_t* buffer, size_t startIndex, size_t length);
    void handleNotification(NimBLERemoteCharacteristic* pRemoteChar, uint8_t* pData, size_t length);
    SEM6000Status sendCommandAndWaitResponse(const uint8_t* cmdFrame, size_t cmdLen, uint16_t expectedCmd, SEMCmdType type, uint32_t timeoutMs = 2000);
    void parseSchedulerBlocks(const uint8_t* buffer, size_t len, std::vector<SEMSchedulerSlot>& list);
};

#endif // SEM6000_DEVICE_H
