/*
 * SEM6000Device.cpp - Version 0.3.8
 * Interface unifiée BLE pour Voltcraft SEM6000-FR
 */

#include "SEM6000Device.h"

static const NimBLEUUID SERVICE_FFF0_UUID("0000fff0-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_REQ_UUID("0000fff3-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_RES_UUID("0000fff4-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_FFF2_UUID("0000fff2-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_FFF1_UUID("0000fff1-0000-1000-8000-00805f9b34fb");

static const NimBLEUUID SERVICE_GENERIC_ACCESS("00001800-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_DEVICE_NAME("00002a00-0000-1000-8000-00805f9b34fb");

SEM6000Device::SEM6000Device(const char* macAddress, uint32_t pin)
    : m_macAddress(macAddress), m_pin(pin), m_connected(false), m_authorized(false),
      m_pClient(nullptr), m_pCharReq(nullptr), m_pCharRes(nullptr), m_pCharFFF2(nullptr),
      m_pCharFFF1(nullptr), m_pCharName(nullptr),
      m_rxLength(0), m_rxReady(false) {}

SEM6000Device::~SEM6000Device() {
    disconnect();
}

uint8_t SEM6000Device::calculateChecksum(const uint8_t* buffer, size_t startIndex, size_t length) {
    uint32_t sum = 1;
    for (size_t i = startIndex; i < startIndex + length; i++) {
        sum += buffer[i];
    }
    return (uint8_t)(sum & 0xFF);
}

void SEM6000Device::handleNotification(NimBLERemoteCharacteristic* pRemoteChar, uint8_t* pData, size_t length) {
    if (length == 0) return;

    if (pData[0] == 0x0F) {
        m_rxLength = 0;
        m_rxReady = false;
    }

    if (m_rxLength + length <= sizeof(m_rxBuffer)) {
        memcpy(m_rxBuffer + m_rxLength, pData, length);
        m_rxLength += length;
    }

    if (m_rxLength >= 5 && m_rxBuffer[m_rxLength - 2] == 0xFF && m_rxBuffer[m_rxLength - 1] == 0xFF) {
        uint8_t expectedCS = calculateChecksum(m_rxBuffer, 2, m_rxLength - 5);
        uint8_t receivedCS = m_rxBuffer[m_rxLength - 3];

        if (expectedCS != receivedCS) {
            Serial.printf("[SEM6000] CHECKSUM INVALID ! Expected: 0x%02X | Got: 0x%02X (Total: %d bytes)\n", 
                          expectedCS, receivedCS, m_rxLength);
            return;
        }

        m_rxReady = true;
    }
    else if (m_rxLength == 19 && m_rxBuffer[0] == 0x0F && m_rxBuffer[2] == 0x04) {
        uint8_t expectedCS = calculateChecksum(m_rxBuffer, 2, 16);
        uint8_t receivedCS = m_rxBuffer[18];

        if (expectedCS == receivedCS) {
            m_rxReady = true;
        }
    }
}

bool SEM6000Device::connect(uint32_t timeoutMs) {
    if (m_connected) return true;

    NimBLEDevice::setMTU(160);
    m_pClient = NimBLEDevice::createClient();

    NimBLEAddress address(m_macAddress.c_str(), BLE_ADDR_PUBLIC);

    if (!m_pClient->connect(address)) {
        NimBLEDevice::deleteClient(m_pClient);
        m_pClient = nullptr;
        return false;
    }

    NimBLERemoteService* pServiceFFF0 = m_pClient->getService(SERVICE_FFF0_UUID);
    if (!pServiceFFF0) {
        m_pClient->disconnect();
        NimBLEDevice::deleteClient(m_pClient);
        m_pClient = nullptr;
        return false;
    }

    m_pCharReq  = pServiceFFF0->getCharacteristic(CHAR_REQ_UUID);
    m_pCharRes  = pServiceFFF0->getCharacteristic(CHAR_RES_UUID);
    m_pCharFFF2 = pServiceFFF0->getCharacteristic(CHAR_FFF2_UUID);
    m_pCharFFF1 = pServiceFFF0->getCharacteristic(CHAR_FFF1_UUID);

    if (!m_pCharReq || !m_pCharRes) {
        m_pClient->disconnect();
        NimBLEDevice::deleteClient(m_pClient);
        m_pClient = nullptr;
        return false;
    }

    NimBLERemoteService* pService1800 = m_pClient->getService(SERVICE_GENERIC_ACCESS);
    if (pService1800) {
        m_pCharName = pService1800->getCharacteristic(CHAR_DEVICE_NAME);
    }

    auto notifyCallback = [this](NimBLERemoteCharacteristic* pChar, uint8_t* pData, size_t len, bool isNotify) {
        this->handleNotification(pChar, pData, len);
    };

    if (!m_pCharRes->subscribe(true, notifyCallback)) {
        m_pClient->disconnect();
        NimBLEDevice::deleteClient(m_pClient);
        m_pClient = nullptr;
        return false;
    }

    m_connected = true;
    return true;
}

void SEM6000Device::disconnect() {
    if (m_pClient && m_connected) {
        m_pClient->disconnect();
        NimBLEDevice::deleteClient(m_pClient);
    }
    m_pClient = nullptr;
    m_connected = false;
    m_authorized = false;
}

SEM6000Status SEM6000Device::sendCommandAndWaitResponse(const uint8_t* cmdFrame, size_t cmdLen, uint16_t expectedCmd, SEMCmdType type, uint32_t timeoutMs) {
    if (!m_connected) return SEM_ERR_NOT_CONNECTED;

    m_rxReady = false;
    m_rxLength = 0;

    if (!m_pCharReq->writeValue(cmdFrame, cmdLen, false)) {
        return SEM_ERR_BLE_WRITE_FAILED;
    }

    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        if (m_rxReady) {
            m_rxReady = false;

            uint16_t receivedCmd = ((uint16_t)m_rxBuffer[2] << 8) | m_rxBuffer[3];
            if (receivedCmd != expectedCmd) {
                continue;
            }

            if (type == CMD_SET_2B) {
                if (m_rxLength < 8) return SEM_ERR_NOTIFICATION;
                return (m_rxBuffer[4] == 0x00) ? SEM_OK : SEM_ERR_RESPONSE_REJECTED;
            } 
            else if (type == CMD_SET_3B) {
                if (m_rxLength < 9) return SEM_ERR_NOTIFICATION;
                return (m_rxBuffer[5] == 0x00) ? SEM_OK : SEM_ERR_RESPONSE_REJECTED;
            } 
            else {
                return SEM_OK;
            }
        }
        delay(10);
    }
    return SEM_ERR_TIMEOUT;
}

// --- 1. PIN & Authentification ---

SEM6000Status SEM6000Device::authenticate(uint32_t pin) {
    if (pin > 9999) return SEM_ERR_INVALID_PARAM;
    uint8_t pinBytes[4] = {
        (uint8_t)((pin / 1000) % 10),
        (uint8_t)((pin / 100) % 10),
        (uint8_t)((pin / 10) % 10),
        (uint8_t)(pin % 10)
    };
    return authenticate32(pinBytes);
}

SEM6000Status SEM6000Device::changePin(uint32_t oldPin, uint32_t newPin) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;
    if (oldPin > 9999 || newPin > 9999 || oldPin == newPin) return SEM_ERR_INVALID_PARAM;
    uint8_t oldBytes[4] = {
        (uint8_t)((oldPin / 1000) % 10), (uint8_t)((oldPin / 100) % 10),
        (uint8_t)((oldPin / 10) % 10),   (uint8_t)(oldPin % 10)
    };
    uint8_t newBytes[4] = {
        (uint8_t)((newPin / 1000) % 10), (uint8_t)((newPin / 100) % 10),
        (uint8_t)((newPin / 10) % 10),   (uint8_t)(newPin % 10)
    };
    return changePin32(oldBytes, newBytes);
}

SEM6000Status SEM6000Device::authenticate32(const uint8_t pin[4]) {
    if (!m_connected) return SEM_ERR_NOT_CONNECTED;

    uint8_t frame[16] = {
        0x0F, 0x0C, 0x17, 0x00, 0x00,
        pin[0], pin[1], pin[2], pin[3],
        0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF
    };
    frame[13] = calculateChecksum(frame, 2, 11);

    SEM6000Status status = sendCommandAndWaitResponse(frame, sizeof(frame), 0x1700, CMD_SET_2B);
    if (status == SEM_OK) m_authorized = true;
    return status;
}

SEM6000Status SEM6000Device::authenticate32(uint32_t pinRaw) {
    uint8_t pin[4] = {
        (uint8_t)((pinRaw >> 24) & 0xFF),
        (uint8_t)((pinRaw >> 16) & 0xFF),
        (uint8_t)((pinRaw >> 8) & 0xFF),
        (uint8_t)(pinRaw & 0xFF)
    };
    return authenticate32(pin);
}

SEM6000Status SEM6000Device::changePin32(const uint8_t oldPin[4], const uint8_t newPin[4]) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;

    uint8_t frame[16] = {
        0x0F, 0x0C, 0x17, 0x00, 0x01,
        newPin[0], newPin[1], newPin[2], newPin[3],
        oldPin[0], oldPin[1], oldPin[2], oldPin[3],
        0x00, 0xFF, 0xFF
    };
    frame[13] = calculateChecksum(frame, 2, 11);

    SEM6000Status status = sendCommandAndWaitResponse(frame, sizeof(frame), 0x1700, CMD_SET_2B);
    if (status == SEM_OK && m_rxBuffer[5] != 0x01) {
        return SEM_ERR_RESPONSE_REJECTED;
    }
    return status;
}

SEM6000Status SEM6000Device::changePin32(uint32_t oldPinRaw, uint32_t newPinRaw) {
    uint8_t oldPin[4] = {
        (uint8_t)((oldPinRaw >> 24) & 0xFF), (uint8_t)((oldPinRaw >> 16) & 0xFF),
        (uint8_t)((oldPinRaw >> 8) & 0xFF),  (uint8_t)(oldPinRaw & 0xFF)
    };
    uint8_t newPin[4] = {
        (uint8_t)((newPinRaw >> 24) & 0xFF), (uint8_t)((newPinRaw >> 16) & 0xFF),
        (uint8_t)((newPinRaw >> 8) & 0xFF),  (uint8_t)(newPinRaw & 0xFF)
    };
    return changePin32(oldPin, newPin);
}

SEM6000Status SEM6000Device::resetPinToDefault() {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;

    uint8_t frame[16] = {
        0x0F, 0x0C, 0x17, 0x00, 0x02,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0xFF, 0xFF
    };
    frame[13] = calculateChecksum(frame, 2, 11);

    SEM6000Status status = sendCommandAndWaitResponse(frame, sizeof(frame), 0x1700, CMD_SET_2B);
    if (status == SEM_OK && m_rxBuffer[5] != 0x02) {
        return SEM_ERR_RESPONSE_REJECTED;
    }
    return status;
}

// --- 2. Configuration & Paramètres ---
SEM6000Status SEM6000Device::setDateTime(uint8_t sec, uint8_t min, uint8_t hour, uint8_t day, uint8_t month, uint16_t year) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;
    uint8_t frame[16] = {
        0x0F, 0x0C, 0x01, 0x00, sec, min, hour, day, month,
        (uint8_t)(year >> 8), (uint8_t)(year & 0xFF), 0x00, 0x00, 0x00, 0xFF, 0xFF
    };
    frame[13] = calculateChecksum(frame, 2, 11);
    return sendCommandAndWaitResponse(frame, sizeof(frame), 0x0100, CMD_SET_2B);
}

SEM6000Status SEM6000Device::requestSettings(SEMSettings& settings) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;
    uint8_t frame[] = { 0x0F, 0x05, 0x10, 0x00, 0x00, 0x00, 0x11, 0xFF, 0xFF };

    SEM6000Status status = sendCommandAndWaitResponse(frame, sizeof(frame), 0x1000, CMD_GET);
    if (status == SEM_OK) {
        if (m_rxLength >= 18) {
            settings.reducedModeActive  = m_rxBuffer[4];
            settings.normalPriceCents   = m_rxBuffer[5];
            settings.reducedPriceCents  = m_rxBuffer[6];
            settings.reducedPeriodStart = ((uint16_t)m_rxBuffer[7] << 8) | m_rxBuffer[8];
            settings.reducedPeriodStop  = ((uint16_t)m_rxBuffer[9] << 8) | m_rxBuffer[10];
            settings.ledStatus          = m_rxBuffer[11];
            settings.maxPowerWatts      = ((uint16_t)m_rxBuffer[13] << 8) | m_rxBuffer[14];
        } else {
            status = SEM_ERR_NOTIFICATION;
        }
    }
    return status;
}

SEM6000Status SEM6000Device::setLedRing(bool enable) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;
    uint8_t frame[13] = { 0x0F, 0x09, 0x0F, 0x00, 0x05, (uint8_t)(enable ? 0x01 : 0x00), 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF };
    frame[10] = calculateChecksum(frame, 2, 8);
    return sendCommandAndWaitResponse(frame, sizeof(frame), 0x0F00, CMD_SET_3B);
}

SEM6000Status SEM6000Device::setOverloadPower(uint16_t maxWatts) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;
    uint8_t frame[11] = { 0x0F, 0x07, 0x05, 0x00, (uint8_t)(maxWatts >> 8), (uint8_t)(maxWatts & 0xFF), 0x00, 0x00, 0x00, 0xFF, 0xFF };
    frame[8] = calculateChecksum(frame, 2, 6);
    return sendCommandAndWaitResponse(frame, sizeof(frame), 0x0500, CMD_SET_2B);
}

SEM6000Status SEM6000Device::setPrices(float standardCents, float reducedCents) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;
    uint8_t frame[11] = { 0x0F, 0x09, 0x0F, 0x00, 0x04, (uint8_t)standardCents, (uint8_t)reducedCents, 0x00, 0x00, 0xFF, 0xFF };
    frame[8] = calculateChecksum(frame, 2, 6);
    return sendCommandAndWaitResponse(frame, sizeof(frame), 0x0F00, CMD_SET_3B);
}

SEM6000Status SEM6000Device::setReducedPeriod(uint8_t onOff, uint8_t startHour, uint8_t startMin, uint8_t endHour, uint8_t endMin) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;
    uint8_t frame[13] = { 0x0F, 0x09, 0x0F, 0x00, 0x01, onOff, startHour, startMin, endHour, endMin, 0x00, 0xFF, 0xFF };
    frame[10] = calculateChecksum(frame, 2, 8);
    return sendCommandAndWaitResponse(frame, sizeof(frame), 0x0F00, CMD_SET_3B);
}

// --- 3. Commande Relais ---
SEM6000Status SEM6000Device::setRelayState(bool turnOn) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;
    uint8_t frame[10] = { 0x0F, 0x06, 0x03, 0x00, (uint8_t)(turnOn ? 0x01 : 0x00), 0x00, 0x00, 0x00, 0xFF, 0xFF };
    frame[7] = calculateChecksum(frame, 2, 5);
    return sendCommandAndWaitResponse(frame, sizeof(frame), 0x0300, CMD_SET_2B);
}

// --- 4. Timer ---
SEM6000Status SEM6000Device::getTimerStatus(SEMTimerStatus& status) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;

    uint8_t frame[] = { 0x0F, 0x05, 0x09, 0x00, 0x00, 0x00, 0x0A, 0xFF, 0xFF };
    SEM6000Status res = sendCommandAndWaitResponse(frame, sizeof(frame), 0x0900, CMD_GET);

    if (res == SEM_OK && m_rxLength >= 18) {
        uint8_t action = m_rxBuffer[4];
        if (action == 0x01 || action == 0x02) {
            status.active       = true;
            status.actionTurnOn = (action == 0x01);
            status.targetSec    = m_rxBuffer[5];
            status.targetMin    = m_rxBuffer[6];
            status.targetHour   = m_rxBuffer[7];
            status.targetDay    = m_rxBuffer[8];
            status.targetMonth  = m_rxBuffer[9];
            status.targetYear   = m_rxBuffer[10];

            uint8_t pHour = m_rxBuffer[11];
            uint8_t pMin  = m_rxBuffer[12];
            uint8_t pSec  = m_rxBuffer[13];
            status.presetSeconds = (uint32_t)pHour * 3600 + (uint32_t)pMin * 60 + pSec;
        } else {
            status.active        = false;
            status.actionTurnOn  = false;
            status.presetSeconds = 0;
        }
    } else {
        res = SEM_ERR_NOTIFICATION;
    }
    return res;
}

SEM6000Status SEM6000Device::setTimerAbs(bool turnOn, uint8_t sec, uint8_t min, uint8_t hour, uint8_t day, uint8_t month, uint16_t year) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;

    uint8_t frame[16] = {
        0x0F, 0x0C, 0x08, 0x00,
        (uint8_t)(turnOn ? 0x01 : 0x02),
        sec, min, hour, day, month, (uint8_t)(year % 100),
        0x00, 0x00, 0x00, 0xFF, 0xFF
    };

    frame[13] = calculateChecksum(frame, 2, 11);
    return sendCommandAndWaitResponse(frame, sizeof(frame), 0x0800, CMD_SET_2B);
}

SEM6000Status SEM6000Device::setTimerRel(bool turnOn, uint32_t durationSeconds, const struct tm& currentSystemTime) {
    struct tm targetTime = currentSystemTime;
    time_t targetEpoch   = mktime(&targetTime) + durationSeconds;
    struct tm* t = localtime(&targetEpoch);

    return setTimerAbs(turnOn, t->tm_sec, t->tm_min, t->tm_hour, t->tm_mday, t->tm_mon + 1, t->tm_year + 1900);
}

SEM6000Status SEM6000Device::stopTimer() {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;

    uint8_t frame[16] = {
        0x0F, 0x0C, 0x08, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x09,
        0xFF, 0xFF
    };

    return sendCommandAndWaitResponse(frame, sizeof(frame), 0x0800, CMD_SET_2B);
}

// --- 5. Plannings (Scheduler) ---
SEM6000Status SEM6000Device::requestSchedulerPage(uint8_t page, uint8_t& totalCountOut) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;

    uint8_t frame[10] = {
        0x0F, 0x06, 0x14, 0x00,
        page,
        0x00, 0x00,
        0x00,
        0xFF, 0xFF
    };

    frame[7] = calculateChecksum(frame, 2, 5);
    SEM6000Status st = sendCommandAndWaitResponse(frame, sizeof(frame), 0x1400, CMD_GET);

    if (st == SEM_OK && m_rxLength >= 5) {
        totalCountOut = m_rxBuffer[4];
    } else {
        totalCountOut = 0;
    }
    return st;
}

void SEM6000Device::parseSchedulerBlocks(const uint8_t* buffer, size_t len, std::vector<SEMSchedulerSlot>& list) {
    if (len < 8) return;

    size_t payloadLen = len - 8;
    size_t numBlocks = payloadLen / 12;

    for (size_t i = 0; i < numBlocks; i++) {
        size_t idx = 5 + (i * 12);
        SEMSchedulerSlot slot;
        slot.slotId       = buffer[idx];
        slot.active       = (buffer[idx + 1] == 0x01);
        slot.actionTurnOn = (buffer[idx + 2] == 0x01);
        slot.dayMask      = buffer[idx + 3];
        slot.year         = buffer[idx + 4];
        slot.month        = buffer[idx + 5];
        slot.day          = buffer[idx + 6];
        slot.hour         = buffer[idx + 7];
        slot.min          = buffer[idx + 8];
        slot.sortKey      = buffer[idx + 11];

        list.push_back(slot);
    }
}

SEM6000Status SEM6000Device::requestAllSchedulers(std::vector<SEMSchedulerSlot>& list) {
    list.clear();
    uint8_t totalSchedulers = 0;

    SEM6000Status st = requestSchedulerPage(0, totalSchedulers);
    if (st != SEM_OK) return st;

    if (totalSchedulers == 0) return SEM_OK;

    parseSchedulerBlocks(m_rxBuffer, m_rxLength, list);

    uint8_t totalPages = (totalSchedulers + 3) / 4;

    for (uint8_t page = 1; page < totalPages; page++) {
        delay(100);
        uint8_t dummyTotal = 0;
        st = requestSchedulerPage(page, dummyTotal);
        if (st == SEM_OK) {
            parseSchedulerBlocks(m_rxBuffer, m_rxLength, list);
        } else {
            return st;
        }
    }

    return SEM_OK;
}

SEM6000Status SEM6000Device::setScheduler(SEMSchedulerMode mode, uint8_t slotId, bool enable, bool turnOn, uint8_t dayMask, uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;

    uint8_t frame[19] = {
        0x0F, 0x0F, 0x13, 0x00,
        (uint8_t)mode,
        slotId,
        (uint8_t)(enable ? 0x01 : 0x00),
        (uint8_t)(turnOn ? 0x01 : 0x00),
        dayMask,
        (uint8_t)(year % 100),
        month,
        day,
        hour,
        min,
        0x00, 0x00,
        0x00,
        0xFF, 0xFF
    };

    frame[16] = calculateChecksum(frame, 2, 14);
    return sendCommandAndWaitResponse(frame, sizeof(frame), 0x1300, CMD_SET_2B);
}

SEM6000Status SEM6000Device::deleteScheduler(uint8_t slotId) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;

    uint8_t frame[19] = {
        0x0F, 0x0F, 0x13, 0x00,
        0x02,
        slotId,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00,
        0xFF, 0xFF
    };

    frame[16] = calculateChecksum(frame, 2, 14);
    return sendCommandAndWaitResponse(frame, sizeof(frame), 0x1300, CMD_SET_2B);
}

SEM6000Status SEM6000Device::resetScheduler() {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;
    uint8_t frame[] = { 0x0F, 0x06, 0x13, 0x00, 0x02, 0x00, 0x00, 0xFF, 0xFF };
    frame[5] = calculateChecksum(frame, 2, 3);
    return sendCommandAndWaitResponse(frame, sizeof(frame), 0x1300, CMD_SET_2B);
}

// --- 6. Mode Aléatoire ---
SEM6000Status SEM6000Device::getRandomMode(SEMRandomConfig& config) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;
    uint8_t frame[] = { 0x0F, 0x05, 0x16, 0x00, 0x00, 0x00, 0x17, 0xFF, 0xFF };

    SEM6000Status st = sendCommandAndWaitResponse(frame, sizeof(frame), 0x1600, CMD_GET);
    if (st == SEM_OK) {
        if (m_rxLength >= 15) {
            config.active    = (m_rxBuffer[4] == 0x01);
            config.dayMask   = m_rxBuffer[5];
            config.startHour = m_rxBuffer[6];
            config.startMin  = m_rxBuffer[7];
            config.endHour   = m_rxBuffer[8];
            config.endMin    = m_rxBuffer[9];
        } else {
            st = SEM_ERR_NOTIFICATION;
        }
    }
    return st;
}

SEM6000Status SEM6000Device::setRandomMode(bool active, uint8_t dayMask, uint8_t startHour, uint8_t startMin, uint8_t endHour, uint8_t endMin) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;

    uint8_t frame[15] = {
        0x0F, 0x0B, 0x15, 0x00,
        (uint8_t)(active ? 0x01 : 0x00),
        dayMask,
        startHour,
        startMin,
        endHour,
        endMin,
        0x00, 0x00,
        0x00,
        0xFF, 0xFF
    };

    frame[12] = calculateChecksum(frame, 2, 10);
    return sendCommandAndWaitResponse(frame, sizeof(frame), 0x1500, CMD_SET_2B);
}

// --- 7. Télémétrie & Historiques ---
SEM6000Status SEM6000Device::captureMeasurement(SEMTelemetry& telemetry) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;
    uint8_t frame[] = { 0x0F, 0x05, 0x04, 0x00, 0x00, 0x00, 0x05, 0xFF, 0xFF };

    SEM6000Status status = sendCommandAndWaitResponse(frame, sizeof(frame), 0x0400, CMD_GET);

    if (status == SEM_OK && m_rxLength >= 19) {
        telemetry.relayState     = (m_rxBuffer[4] == 0x01);
        uint32_t rawPwr          = ((uint32_t)m_rxBuffer[5] << 16) | ((uint32_t)m_rxBuffer[6] << 8) | m_rxBuffer[7];
        telemetry.powerWatts     = rawPwr / 1000.0f;
        telemetry.voltageVolts   = m_rxBuffer[8];
        uint16_t rawCurrent      = ((uint16_t)m_rxBuffer[9] << 8) | m_rxBuffer[10];
        telemetry.currentAmperes = rawCurrent / 1000.0f;
        telemetry.frequencyHz    = m_rxBuffer[11];
        telemetry.unknownWord    = ((uint16_t)m_rxBuffer[12] << 8) | m_rxBuffer[13];
        telemetry.totalKwhWh     = ((uint32_t)m_rxBuffer[14] << 24) | ((uint32_t)m_rxBuffer[15] << 16) | ((uint32_t)m_rxBuffer[16] << 8) | m_rxBuffer[17];
    } else {
        status = SEM_ERR_NOTIFICATION;
    }
    return status;
}

SEM6000Status SEM6000Device::requestDayHistory(uint16_t hourlyWh[24]) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;
    memset(hourlyWh, 0, 24 * sizeof(uint16_t));

    uint8_t frame[] = { 0x0F, 0x05, 0x0A, 0x00, 0x00, 0x00, 0x0B, 0xFF, 0xFF };
    SEM6000Status st = sendCommandAndWaitResponse(frame, sizeof(frame), 0x0A00, CMD_GET, 5000);

    if (st == SEM_OK && m_rxLength >= 55) {
        for (int i = 0; i < 24; i++) {
            size_t idx = 4 + (i * 2);
            hourlyWh[i] = ((uint16_t)m_rxBuffer[idx] << 8) | m_rxBuffer[idx + 1];
        }
    } else {
        st = SEM_ERR_NOTIFICATION;
    }
    return st;
}

SEM6000Status SEM6000Device::requestMonthHistory(uint32_t dailyWh[30]) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;
    memset(dailyWh, 0, 30 * sizeof(uint32_t));

    uint8_t frame[] = { 0x0F, 0x05, 0x0B, 0x00, 0x00, 0x00, 0x0C, 0xFF, 0xFF };
    SEM6000Status st = sendCommandAndWaitResponse(frame, sizeof(frame), 0x0B00, CMD_GET, 5000);

    if (st == SEM_OK && m_rxLength >= 127) {
        for (int i = 0; i < 30; i++) {
            size_t idx = 4 + (i * 4);
            dailyWh[i] = ((uint32_t)m_rxBuffer[idx] << 16) | ((uint32_t)m_rxBuffer[idx + 1] << 8) | m_rxBuffer[idx + 2];
        }
    } else {
        st = SEM_ERR_NOTIFICATION;
    }
    return st;
}

SEM6000Status SEM6000Device::requestYearHistory(uint32_t monthlyWh[12]) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;
    memset(monthlyWh, 0, 12 * sizeof(uint32_t));

    uint8_t frame[] = { 0x0F, 0x05, 0x0C, 0x00, 0x00, 0x00, 0x0D, 0xFF, 0xFF };
    SEM6000Status st = sendCommandAndWaitResponse(frame, sizeof(frame), 0x0C00, CMD_GET, 5000);

    if (st == SEM_OK && m_rxLength >= 55) {
        for (int i = 0; i < 12; i++) {
            size_t idx = 4 + (i * 4);
            monthlyWh[i] = ((uint32_t)m_rxBuffer[idx] << 16) | ((uint32_t)m_rxBuffer[idx + 1] << 8) | m_rxBuffer[idx + 2];
        }
    } else {
        st = SEM_ERR_NOTIFICATION;
    }
    return st;
}

SEM6000Status SEM6000Device::resetConsumptionData() {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;
    uint8_t frame[13] = { 0x0F, 0x09, 0x0F, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF };
    frame[10] = calculateChecksum(frame, 2, 8);
    return sendCommandAndWaitResponse(frame, sizeof(frame), 0x0F00, CMD_SET_3B);
}

SEM6000Status SEM6000Device::factoryReset() {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;
    uint8_t frame[13] = { 0x0F, 0x09, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF };
    frame[10] = calculateChecksum(frame, 2, 8);
    return sendCommandAndWaitResponse(frame, sizeof(frame), 0x0F00, CMD_SET_3B);
}

// --- 8. Infos Device & Nom GATT ---
SEM6000Status SEM6000Device::getDeviceVendorAndVersions(SEMDeviceInfo& info) {
    if (!m_pCharFFF1 || !m_pCharFFF1->canRead()) return SEM_ERR_BLE_WRITE_FAILED;

    std::string val = m_pCharFFF1->readValue();
    m_rxLength = val.length();
    if (m_rxLength > 0 && m_rxLength <= sizeof(m_rxBuffer)) {
        memcpy(m_rxBuffer, val.data(), m_rxLength);
    } else {
        m_rxLength = 0;
        return SEM_ERR_NOTIFICATION;
    }

    if (val.length() >= 16) {
        info.vendor = String(val.substr(0, 6).c_str());

        uint8_t fwMajor = (uint8_t)val[11];
        uint8_t fwMinor = (uint8_t)val[12];
        info.firmwareVersion = String(fwMajor) + "." + String(fwMinor);

        uint8_t hwMajor = (uint8_t)val[13];
        uint8_t hwMinor = (uint8_t)val[14];
        info.hardwareVersion = String(hwMajor) + "." + String(hwMinor);

        info.byte16 = (uint8_t)val[15];
        return SEM_OK;
    }
    return SEM_ERR_NOTIFICATION;
}

SEM6000Status SEM6000Device::getDeviceNameGATT(String& name) {
    if (!m_pCharName || !m_pCharName->canRead()) return SEM_ERR_BLE_WRITE_FAILED;

    std::string val = m_pCharName->readValue();
    name = String(val.c_str());
    return SEM_OK;
}

SEM6000Status SEM6000Device::setDeviceName(const String& name) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;

    uint8_t frame[27] = { 0x0F, 0x17, 0x02, 0x00 };
    memset(frame + 4, 0, 20);

    size_t copyLen = min((size_t)19, name.length());
    memcpy(frame + 4, name.c_str(), copyLen);

    frame[24] = calculateChecksum(frame, 2, 22);
    frame[25] = 0xFF;
    frame[26] = 0xFF;

    return sendCommandAndWaitResponse(frame, sizeof(frame), 0x0200, CMD_SET_2B);
}

SEM6000Status SEM6000Device::getSerialNumber(String& serial) {
    if (!m_authorized) return SEM_ERR_AUTH_FAILED;

    uint8_t frame[] = { 0x0F, 0x05, 0x11, 0x00, 0x00, 0x00, 0x12, 0xFF, 0xFF };
    SEM6000Status st = sendCommandAndWaitResponse(frame, sizeof(frame), 0x1100, CMD_GET);

    if (st == SEM_OK && m_rxLength >= 25) {
        char temp[17];
        for (int i = 0; i < 16; i++) {
            uint8_t b = m_rxBuffer[4 + i];
            temp[i] = (b == 0x00) ? '-' : (char)b;
        }
        temp[16] = '\0';
        serial = String(temp);
    } else {
        st = SEM_ERR_NOTIFICATION;
    }
    return st;
}

bool SEM6000Device::readRawFFF2(uint16_t& word0, uint8_t& currentByte, float& currentAmpsDirect) {
    if (!m_pCharFFF2 || !m_pCharFFF2->canRead()) return false;
    std::string value = m_pCharFFF2->readValue();
    if (value.length() >= 4) {
        word0 = ((uint8_t)value[0] << 8) | (uint8_t)value[1];
        currentByte = (uint8_t)value[2];
        uint16_t rawmA = ((uint16_t)value[2] << 8) | (uint8_t)value[3];
        currentAmpsDirect = rawmA / 1000.0f;
        return true;
    }
    return false;
}

SEM6000Status SEM6000Device::checkAndFixClockLoss(uint8_t sec, uint8_t min, uint8_t hour, uint8_t day, uint8_t month, uint16_t year) {
    uint16_t word0 = 0;
    uint8_t b2 = 0;
    float currentA = 0.0f;

    if (readRawFFF2(word0, b2, currentA)) {
        if (word0 == 0xFFFF) {
            Serial.println("[SEM6000] AMNÉSIE HORLOGE DÉTECTÉE SUR FFF2 ! AUTO-CORRECTION...");
            return setDateTime(sec, min, hour, day, month, year);
        }
    }
    return SEM_OK;
}
