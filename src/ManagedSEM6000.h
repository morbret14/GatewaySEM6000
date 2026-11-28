/**
 * @file ManagedSEM6000.h
 * @brief Couche 2 : Machine à états, surveillance d'horloge NTP, détection/guérison de micro-coupures
 *        secteur et calcul différentiel de l'Index Perpétuel Monotone en NVS (Preferences) pour prises
 *        Voltcraft SEM6000-FR Bi-Génération (Hardware 2.0 / FW 1.24 & Hardware 3.0 / FW 1.30).
 * @version 0.1.0
 * @requires SEM6000Device >= v0.4.0 (SEM6000_DEVICE_VERSION_HEX >= 0x000400)
 *
 * Conformité GEMINI.md Section 5 :
 *   - Composition forte : encapsule en membre privé une instance SEM6000Device.
 *   - Zéro usage de la classe Arduino String (buffers fixes char[] exclusivement).
 *   - Zéro allocation dynamique dans la boucle de scrutation pollCycle().
 */

#ifndef MANAGED_SEM6000_H
#define MANAGED_SEM6000_H

#include <Arduino.h>
#include <Preferences.h>
#include <ctime>
#include "SEM6000Device.h"

static_assert(SEM6000_DEVICE_VERSION_HEX >= 0x000400,
              "ManagedSEM6000 v0.1.0 requiert SEM6000Device >= v0.4.0");

#define MANAGED_SEM6000_VERSION_MAJOR 0
#define MANAGED_SEM6000_VERSION_MINOR 1
#define MANAGED_SEM6000_VERSION_PATCH 0
#define MANAGED_SEM6000_VERSION_HEX   0x000100
#define MANAGED_SEM6000_VERSION_STR   "0.1.0"

enum ManagedLoadState : uint8_t {
    LOAD_NOMINAL = 0,
    LOAD_WARNING_85,
    LOAD_OVERLOAD_100
};

struct ManagedPollReport {
    bool             pollSuccess;
    bool             isHw3;
    bool             clockLossDetectedAndFixed;
    bool             hourlyRolloverDetected;
    bool             havenSyncPerformed;
    bool             driftMeasuredThisCycle;
    bool             nvsSavedThisCycle;
    bool             relayOn;
    float            powerWatts;
    uint8_t          voltageVolts;
    float            currentAmperes;
    uint16_t         maxPowerWatts;
    uint16_t         fff2ImaxMa;
    float            loadPercent;
    ManagedLoadState loadState;
    uint32_t         rawTotalKwh0400;
    uint16_t         rawH22Wh;
    uint16_t         rawH23Wh;
    uint32_t         rawSum24hWh;
    uint32_t         rawPlugApparentWh;
    int32_t          deltaWhThisCycle;
    uint32_t         perpetualIndexWh;
    uint32_t         savedWhAcrossOutages;
    uint32_t         totalOutageCount;
    int32_t          lastMeasuredDriftSec;
};

class ManagedSEM6000 {
public:
    ManagedSEM6000(const char* label, const char* macAddress, uint32_t pin = (uint32_t)0x00000000);
    ~ManagedSEM6000();

    /**
     * @brief Initialise le namespace NVS de la prise et restaure l'Index Perpétuel s'il existe.
     */
    bool begin();

    /**
     * @brief Exécute un cycle complet de scrutation BLE séquentiel (Connect -> Auth -> Check Clock Loss
     *        -> Havre de Paix / Sonde dérive -> Télémétrie 0x0400 -> Historique 24h 0x0A00 -> Calcul Index
     *        Perpétuel -> Sauvegarde NVS conditionnelle -> Disconnect).
     * @param ntpTime         Heure NTP courante (dans le référentiel monotone UTC+1 fixe).
     * @param forceDriftProbe Si true, exécute la Sonde Temporelle Furtive (0x0800/0x0900) durant ce cycle.
     * @param outReport       Structure de rapport remplie avec toutes les métriques du cycle.
     * @return true si le cycle BLE s'est déroulé avec succès, false en cas d'erreur de connexion/protocole.
     */
    bool pollCycle(const struct tm& ntpTime, bool forceDriftProbe, ManagedPollReport& outReport);

    /**
     * @brief Force immédiatement la sauvegarde de l'état courant en mémoire Flash NVS.
     */
    bool saveToNvs(bool forceNow = true);

    /**
     * @brief Réinitialise la base NVS de la prise : au prochain pollCycle(), l'index perpétuel
     *        sera réaligné sur l'index apparent actuel de la prise et les compteurs de coupures remis à 0.
     */
    bool resetNvsBaseline();

    /**
     * @brief Commande ponctuelle du relais (ex. pour simuler une micro-coupure amont sur Vélo).
     */
    bool setRelayStateImmediate(bool turnOn);

    // --- Accesseurs constants (zéro String) ---
    const char* getLabel() const { return m_label; }
    const char* getMacAddress() const { return m_mac; }
    const char* getNvsNamespace() const { return m_nvsNamespace; }
    bool        isNvsInitialized() const { return m_nvsInitialized; }
    bool        isHardwareV3() const { return m_isHw3; }
    uint32_t    getPerpetualIndexWh() const { return m_perpetualIndexWh; }
    uint32_t    getLastSavedIndexWh() const { return m_lastSavedIndexWh; }
    uint32_t    getLastTotalKwh0400() const { return m_lastTotalKwh0400; }
    uint16_t    getLastH23Wh() const { return m_lastH23; }
    uint32_t    getPowerOutageCount() const { return m_powerOutageCount; }
    uint32_t    getSavedWhAcrossOutages() const { return m_savedWhAcrossOutages; }
    int32_t     getLastMeasuredDriftSec() const { return m_lastMeasuredDriftSec; }
    uint32_t    getTotalPollCount() const { return m_totalPollCount; }
    uint32_t    getSuccessPollCount() const { return m_successPollCount; }

private:
    static const uint32_t NVS_SAVE_DELTA_WH_THRESHOLD = 10;
    static const uint32_t NVS_SAVE_INTERVAL_MS        = 30UL * 60UL * 1000UL; // 30 minutes
    static const int32_t  HAVEN_DRIFT_THRESHOLD_SEC   = 3;                    // Recalage si |dérive| >= 3 s

    char          m_label[24];
    char          m_mac[18];
    char          m_nvsNamespace[16];
    uint32_t      m_pin;

    // Composition forte Couche 1
    SEM6000Device m_device;

    // État persistant NVS
    bool          m_nvsInitialized;
    uint32_t      m_perpetualIndexWh;
    uint32_t      m_lastSavedIndexWh;
    uint32_t      m_lastNvsSaveMillis;
    uint32_t      m_lastTotalKwh0400;
    uint16_t      m_lastH23;
    uint32_t      m_powerOutageCount;
    uint32_t      m_savedWhAcrossOutages;

    // État volatile de session
    bool          m_isHw3;
    bool          m_settingsCached;
    uint16_t      m_cachedMaxPowerW;
    int8_t        m_lastHavenSyncHour;
    int8_t        m_lastRolloverHourHw2;
    int32_t       m_lastMeasuredDriftSec;
    uint32_t      m_totalPollCount;
    uint32_t      m_successPollCount;

    void buildNvsNamespaceFromMac(const char* mac);
    bool measureClockDriftFurtive(const struct tm& ntpTime, bool currentRelayOn, int32_t& outDriftSec);
    void refreshHw3RamSentinel(const struct tm& ntpTime, bool currentRelayOn);
};

#endif // MANAGED_SEM6000_H
