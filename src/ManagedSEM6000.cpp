/**
 * @file ManagedSEM6000.cpp
 * @brief Implémentation de la Couche 2 ManagedSEM6000 v0.1.0.
 * @version 0.1.0
 * @requires SEM6000Device >= v0.4.0 (SEM6000_DEVICE_VERSION_HEX >= 0x000400)
 */

#include "ManagedSEM6000.h"
#include <cstring>
#include <cstdio>

ManagedSEM6000::ManagedSEM6000(const char* label, const char* macAddress, uint32_t pin)
    : m_pin(pin),
      m_device(macAddress, pin),
      m_nvsInitialized(false),
      m_perpetualIndexWh(0),
      m_lastSavedIndexWh(0),
      m_lastNvsSaveMillis(0),
      m_lastTotalKwh0400(0),
      m_lastH23(0),
      m_powerOutageCount(0),
      m_savedWhAcrossOutages(0),
      m_isHw3(false),
      m_settingsCached(false),
      m_cachedMaxPowerW(3680),
      m_lastHavenSyncHour(-1),
      m_lastRolloverHourHw2(-1),
      m_lastMeasuredDriftSec(0),
      m_totalPollCount(0),
      m_successPollCount(0) {
    strncpy(m_label, label ? label : "SEM6000", sizeof(m_label) - 1);
    m_label[sizeof(m_label) - 1] = '\0';

    strncpy(m_mac, macAddress ? macAddress : "00:00:00:00:00:00", sizeof(m_mac) - 1);
    m_mac[sizeof(m_mac) - 1] = '\0';

    buildNvsNamespaceFromMac(m_mac);
}

ManagedSEM6000::~ManagedSEM6000() {
    m_device.disconnect();
}

void ManagedSEM6000::buildNvsNamespaceFromMac(const char* mac) {
    // Extrait les 6 derniers caractères hexadécimaux de l'adresse MAC pour former "sem_XXXXXX" (10 caractères <= 15 max NVS)
    char hexOnly[13];
    size_t idx = 0;
    for (size_t i = 0; mac[i] != '\0' && idx < 12; i++) {
        char c = mac[i];
        if ((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F')) {
            hexOnly[idx++] = (c >= 'A' && c <= 'F') ? (char)(c + 32) : c;
        }
    }
    hexOnly[idx] = '\0';
    const char* suffix = (idx >= 6) ? (hexOnly + idx - 6) : hexOnly;
    snprintf(m_nvsNamespace, sizeof(m_nvsNamespace), "sem_%s", suffix);
}

bool ManagedSEM6000::begin() {
    Preferences prefs;
    if (!prefs.begin(m_nvsNamespace, true)) { // Lecture seule
        m_nvsInitialized = false;
        return false;
    }

    m_nvsInitialized = prefs.getBool("init", false);
    if (m_nvsInitialized) {
        m_perpetualIndexWh     = prefs.getULong("perp_wh", 0);
        m_lastTotalKwh0400     = prefs.getULong("last_0400", 0);
        m_lastH23              = prefs.getUShort("last_h23", 0);
        m_powerOutageCount     = prefs.getULong("outages", 0);
        m_savedWhAcrossOutages = prefs.getULong("saved_wh", 0);
        m_lastSavedIndexWh     = m_perpetualIndexWh;
        m_lastNvsSaveMillis    = millis();
    }
    prefs.end();
    return true;
}

bool ManagedSEM6000::saveToNvs(bool forceNow) {
    if (!m_nvsInitialized) return false;

    uint32_t nowMs = millis();
    uint32_t unsavedDelta = (m_perpetualIndexWh >= m_lastSavedIndexWh)
                                ? (m_perpetualIndexWh - m_lastSavedIndexWh)
                                : 0;
    bool intervalElapsed = (nowMs - m_lastNvsSaveMillis) >= NVS_SAVE_INTERVAL_MS;

    if (!forceNow && unsavedDelta < NVS_SAVE_DELTA_WH_THRESHOLD && !intervalElapsed) {
        return false;
    }

    Preferences prefs;
    if (!prefs.begin(m_nvsNamespace, false)) {
        return false;
    }

    prefs.putBool("init", true);
    prefs.putULong("perp_wh", m_perpetualIndexWh);
    prefs.putULong("last_0400", m_lastTotalKwh0400);
    prefs.putUShort("last_h23", m_lastH23);
    prefs.putULong("outages", m_powerOutageCount);
    prefs.putULong("saved_wh", m_savedWhAcrossOutages);
    prefs.end();

    m_lastSavedIndexWh  = m_perpetualIndexWh;
    m_lastNvsSaveMillis = nowMs;
    return true;
}

bool ManagedSEM6000::resetNvsBaseline() {
    Preferences prefs;
    if (!prefs.begin(m_nvsNamespace, false)) {
        return false;
    }
    prefs.clear();
    prefs.end();

    m_nvsInitialized       = false;
    m_perpetualIndexWh     = 0;
    m_lastSavedIndexWh     = 0;
    m_lastTotalKwh0400     = 0;
    m_lastH23              = 0;
    m_powerOutageCount     = 0;
    m_savedWhAcrossOutages = 0;
    m_lastRolloverHourHw2  = -1;
    return true;
}

bool ManagedSEM6000::measureClockDriftFurtive(const struct tm& ntpTime, bool currentRelayOn, int32_t& outDriftSec) {
    // 1. Politesse d'accès (Règle 4 Section 2.6 SSOT) : ne jamais écraser un timer utilisateur actif
    SEMTimerStatus stInit;
    if (m_device.getTimerStatus(stInit) != SEM_OK) {
        return false;
    }
    if (stInit.active) {
        return false;
    }

    // 2. Calcul d'une cible à exactement T_NTP + 24h (86400 s)
    struct tm tmCopy = ntpTime;
    time_t ntpEpoch = mktime(&tmCopy);
    if (ntpEpoch <= 0) {
        return false;
    }
    time_t targetEpoch = ntpEpoch + 86400;
    struct tm targetTm;
    localtime_r(&targetEpoch, &targetTm);

    // 3. Armement furtif avec la même action que l'état actuel du relais (risque nul)
    SEM6000Status stArm = m_device.setTimerAbs(
        currentRelayOn,
        (uint8_t)targetTm.tm_sec,
        (uint8_t)targetTm.tm_min,
        (uint8_t)targetTm.tm_hour,
        (uint8_t)targetTm.tm_mday,
        (uint8_t)(targetTm.tm_mon + 1),
        (uint16_t)(targetTm.tm_year + 1900));

    if (stArm != SEM_OK) {
        m_device.stopTimer();
        return false;
    }

    // 4. Lecture de presetSeconds (24 bits Big-Endian) puis désarmement immédiat
    SEMTimerStatus stArmed;
    SEM6000Status stGet = m_device.getTimerStatus(stArmed);
    m_device.stopTimer();

    if (stGet != SEM_OK || stArmed.presetSeconds == 0) {
        return false;
    }

    // Dérive = T_prise - T_NTP = 86400 - presetSeconds
    outDriftSec = 86400L - (int32_t)stArmed.presetSeconds;
    m_lastMeasuredDriftSec = outDriftSec;
    return true;
}

void ManagedSEM6000::refreshHw3RamSentinel(const struct tm& ntpTime, bool currentRelayOn) {
    SEMTimerStatus stInit;
    if (m_device.getTimerStatus(stInit) == SEM_OK && !stInit.active) {
        if (stInit.targetDay == 0 && stInit.targetMonth == 0) {
            m_device.setTimerAbs(
                currentRelayOn,
                0, 0, 0,
                (uint8_t)ntpTime.tm_mday,
                (uint8_t)(ntpTime.tm_mon + 1),
                (uint16_t)(ntpTime.tm_year + 1900));
            m_device.stopTimer();
        }
    }
}

bool ManagedSEM6000::setRelayStateImmediate(bool turnOn) {
    bool ok = false;
    for (int attempt = 1; attempt <= 2; attempt++) {
        if (m_device.connect(5000)) {
            if (m_device.authenticate32(m_pin) == SEM_OK) {
                if (m_device.setRelayState(turnOn) == SEM_OK) {
                    ok = true;
                }
            }
            m_device.disconnect();
            break;
        }
        delay(200);
    }
    return ok;
}

bool ManagedSEM6000::pollCycle(const struct tm& ntpTime, bool forceDriftProbe, ManagedPollReport& outReport) {
    memset(&outReport, 0, sizeof(outReport));
    m_totalPollCount++;

    // 1. Connexion BLE (jusqu'à 2 tentatives)
    bool connected = false;
    for (int attempt = 1; attempt <= 2; attempt++) {
        if (m_device.connect(5000)) {
            connected = true;
            break;
        }
        delay(250);
    }

    if (!connected) {
        return false;
    }

    m_isHw3 = m_device.isHardwareV3();
    outReport.isHw3 = m_isHw3;

    // 2. Lecture rapide 0xFFF2 avant authentification sur HW 2.0
    uint16_t fff2Word0 = 0;
    uint8_t  fff2Byte2 = 0;
    float    fff2Amps  = 0.0f;
    if (!m_isHw3) {
        m_device.readRawFFF2(fff2Word0, fff2Byte2, fff2Amps);
    }

    // 3. Authentification PIN 32 bits
    if (m_device.authenticate32(m_pin) != SEM_OK) {
        m_device.disconnect();
        return false;
    }

    // 4. Détection et guérison immédiate de perte d'horloge / micro-coupure secteur (Règle 3 SSOT)
    bool clockLoss = false;
    SEM6000Status stClock = m_device.checkAndFixClockLoss(
        (uint8_t)ntpTime.tm_sec,
        (uint8_t)ntpTime.tm_min,
        (uint8_t)ntpTime.tm_hour,
        (uint8_t)ntpTime.tm_mday,
        (uint8_t)(ntpTime.tm_mon + 1),
        (uint16_t)(ntpTime.tm_year + 1900),
        &clockLoss);

    if (stClock != SEM_OK) {
        m_device.disconnect();
        return false;
    }

    if (clockLoss) {
        outReport.clockLossDetectedAndFixed = true;
        m_powerOutageCount++;
        m_lastMeasuredDriftSec = 0;
        // Relire FFF2 après démasquage 0xFFFF sur HW 2.0
        if (!m_isHw3) {
            m_device.readRawFFF2(fff2Word0, fff2Byte2, fff2Amps);
        }
    }

    outReport.fff2ImaxMa = fff2Word0;

    // 5. Capture de la télémétrie instantanée 0x0400
    SEMTelemetry tel;
    if (m_device.captureMeasurement(tel) != SEM_OK) {
        m_device.disconnect();
        return false;
    }

    outReport.relayOn         = tel.relayState;
    outReport.powerWatts      = tel.powerWatts;
    outReport.voltageVolts    = tel.voltageVolts;
    outReport.currentAmperes  = tel.currentAmperes;
    outReport.rawTotalKwh0400 = tel.totalKwhWh;

    // 6. Mise en cache de P_max (0x1000) au premier poll ou après une coupure secteur
    if (!m_settingsCached || clockLoss) {
        SEMSettings cfg;
        if (m_device.requestSettings(cfg) == SEM_OK) {
            m_cachedMaxPowerW = (cfg.maxPowerWatts > 0) ? cfg.maxPowerWatts : 3680;
            m_settingsCached  = true;
        }
    }
    outReport.maxPowerWatts = m_cachedMaxPowerW;

    // Calcul du taux de charge (%) et de l'état de surcharge
    if (!m_isHw3 && fff2Word0 > 0 && fff2Word0 != 0xFFFF) {
        float currentMa = tel.currentAmperes * 1000.0f;
        outReport.loadPercent = (currentMa * 100.0f) / (float)fff2Word0;
    } else if (m_cachedMaxPowerW > 0) {
        outReport.loadPercent = (tel.powerWatts * 100.0f) / (float)m_cachedMaxPowerW;
    }
    if (outReport.loadPercent >= 100.0f) {
        outReport.loadState = LOAD_OVERLOAD_100;
    } else if (outReport.loadPercent >= 85.0f) {
        outReport.loadState = LOAD_WARNING_85;
    } else {
        outReport.loadState = LOAD_NOMINAL;
    }

    // 7. Sonde Temporelle Furtive & Synchronisation en « Havre de Paix » (HH:30)
    bool isHavenWindow = (ntpTime.tm_min == 30 && m_lastHavenSyncHour != (int8_t)ntpTime.tm_hour);
    if (!clockLoss && (forceDriftProbe || isHavenWindow)) {
        int32_t driftSec = 0;
        if (measureClockDriftFurtive(ntpTime, tel.relayState, driftSec)) {
            outReport.driftMeasuredThisCycle = true;
            if (isHavenWindow) {
                m_lastHavenSyncHour = (int8_t)ntpTime.tm_hour;
                if (driftSec >= HAVEN_DRIFT_THRESHOLD_SEC || driftSec <= -HAVEN_DRIFT_THRESHOLD_SEC) {
                    if (m_device.setDateTime(
                            (uint8_t)ntpTime.tm_sec,
                            (uint8_t)ntpTime.tm_min,
                            (uint8_t)ntpTime.tm_hour,
                            (uint8_t)ntpTime.tm_mday,
                            (uint8_t)(ntpTime.tm_mon + 1),
                            (uint16_t)(ntpTime.tm_year + 1900)) == SEM_OK) {
                        if (m_isHw3) {
                            refreshHw3RamSentinel(ntpTime, tel.relayState);
                        }
                        outReport.havenSyncPerformed = true;
                        m_lastMeasuredDriftSec = 0;
                    }
                }
            }
        }
    }
    outReport.lastMeasuredDriftSec = m_lastMeasuredDriftSec;

    // 8. Lecture de l'historique horaire 24h (0x0A00) pour l'accumulateur différentiel H[23] / H[22]
    uint16_t hourlyWh[24] = {0};
    if (m_device.requestDayHistory(hourlyWh) != SEM_OK) {
        m_device.disconnect();
        return false;
    }

    m_device.disconnect();

    uint32_t sum24h = 0;
    for (int i = 0; i < 24; i++) {
        sum24h += hourlyWh[i];
    }
    uint16_t h22 = hourlyWh[22];
    uint16_t h23 = hourlyWh[23];

    outReport.rawH22Wh    = h22;
    outReport.rawH23Wh    = h23;
    outReport.rawSum24hWh = sum24h;
    outReport.rawPlugApparentWh = m_isHw3 ? (tel.totalKwhWh + h23) : sum24h;

    // 9. Algorithme Différentiel Monotone de l'Index Perpétuel (Section 3 SSOT)
    int32_t deltaWh = 0;
    bool forceSave = false;

    if (!m_nvsInitialized) {
        // Initialisation de la référence (1er lancement ou après resetNvsBaseline)
        m_lastTotalKwh0400  = m_isHw3 ? tel.totalKwhWh : 0;
        m_lastH23           = h23;
        m_perpetualIndexWh  = outReport.rawPlugApparentWh;
        m_lastRolloverHourHw2 = (int8_t)ntpTime.tm_hour;
        m_nvsInitialized    = true;
        forceSave           = true;
    } else if (clockLoss) {
        // Micro-coupure secteur sur la prise : H[23] en RAM a été perdu côté prise,
        // mais m_lastH23 était DÉJÀ acquis dans m_perpetualIndexWh !
        if (m_isHw3 && tel.totalKwhWh > m_lastTotalKwh0400) {
            // Si une heure pile avait été franchie avant la coupure secteur
            int32_t closedDelta = (int32_t)(tel.totalKwhWh - m_lastTotalKwh0400) - (int32_t)m_lastH23;
            if (closedDelta > 0) {
                deltaWh += closedDelta;
            }
            m_lastTotalKwh0400 = tel.totalKwhWh;
        } else if (m_lastH23 > 0) {
            // Les Wh de l'heure entamée avant la coupure sont sauvés par notre index NVS !
            m_savedWhAcrossOutages += m_lastH23;
        }
        // Remise à zéro de la référence RAM m_lastH23 sans jamais reculer m_perpetualIndexWh
        m_lastH23 = 0;
        deltaWh  += (int32_t)h23;
        m_perpetualIndexWh += (uint32_t)deltaWh;
        m_lastH23 = h23;
        m_lastRolloverHourHw2 = (int8_t)ntpTime.tm_hour;
        forceSave = true;
    } else if (m_isHw3) {
        // Régime nominal Hardware 3.0 : couple (m_lastTotalKwh0400, m_lastH23)
        if (tel.totalKwhWh > m_lastTotalKwh0400) {
            // Franchissement d'une (ou plusieurs) heures piles : totalKwhWh a intégré H_fermée
            int32_t rolloverDelta = (int32_t)(tel.totalKwhWh - m_lastTotalKwh0400)
                                  + (int32_t)h23
                                  - (int32_t)m_lastH23;
            if (rolloverDelta < 0) rolloverDelta = 0;
            deltaWh = rolloverDelta;
            m_perpetualIndexWh += (uint32_t)deltaWh;
            m_lastTotalKwh0400  = tel.totalKwhWh;
            m_lastH23           = h23;
            outReport.hourlyRolloverDetected = true;
            forceSave = true;
        } else {
            // Même heure en cours (totalKwhWh constant)
            if (h23 >= m_lastH23) {
                deltaWh = (int32_t)(h23 - m_lastH23);
                m_perpetualIndexWh += (uint32_t)deltaWh;
                m_lastH23 = h23;
            } else {
                // Baisse de H[23] à totalKwhWh constant (ex. rotation d'une heure à 0 Wh ou reset RAM)
                m_savedWhAcrossOutages += m_lastH23;
                deltaWh = (int32_t)h23;
                m_perpetualIndexWh += (uint32_t)deltaWh;
                m_lastH23 = h23;
                forceSave = true;
            }
        }
    } else {
        // Régime nominal Hardware 2.0 (totalKwhWh == 0 permanent)
        bool hourChangedByClock = (m_lastRolloverHourHw2 >= 0 &&
                                   (int8_t)ntpTime.tm_hour != m_lastRolloverHourHw2 &&
                                   ntpTime.tm_sec >= 12);

        if (h23 < m_lastH23 || hourChangedByClock) {
            // Franchissement d'heure pile : l'heure fermée a glissé dans H[22]
            int32_t endOfHourRemainder = (int32_t)h22 - (int32_t)m_lastH23;
            if (endOfHourRemainder < 0) endOfHourRemainder = 0;
            deltaWh = endOfHourRemainder + (int32_t)h23;
            m_perpetualIndexWh += (uint32_t)deltaWh;
            m_lastH23 = h23;
            m_lastRolloverHourHw2 = (int8_t)ntpTime.tm_hour;
            outReport.hourlyRolloverDetected = true;
            forceSave = true;
        } else {
            if (m_lastRolloverHourHw2 < 0) {
                m_lastRolloverHourHw2 = (int8_t)ntpTime.tm_hour;
            }
            deltaWh = (int32_t)(h23 - m_lastH23);
            m_perpetualIndexWh += (uint32_t)deltaWh;
            m_lastH23 = h23;
        }
    }

    outReport.nvsSavedThisCycle    = saveToNvs(forceSave);
    outReport.deltaWhThisCycle     = deltaWh;
    outReport.perpetualIndexWh     = m_perpetualIndexWh;
    outReport.savedWhAcrossOutages = m_savedWhAcrossOutages;
    outReport.totalOutageCount     = m_powerOutageCount;
    outReport.pollSuccess          = true;

    m_successPollCount++;
    return true;
}
