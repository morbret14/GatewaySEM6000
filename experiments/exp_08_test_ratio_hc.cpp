/*
 * =====================================================================================
 * EXPÉRIENCE 08 : Test du Rôle de l'Octet B3 & Détection du Ratio Heures Creuses (HC)
 * =====================================================================================
 * Cible : Prise Voltcraft SEM6000-FR (MAC: 94:a9:a8:1a:37:0c)
 *
 * Objectif :
 *   Déterminer si le 4ᵉ octet (B3) de chaque slot journalier (dans requestMonthHistory)
 *   représente le ratio / pourcentage d'énergie consommée en Heures Creuses (HC).
 *
 * Protocole :
 *   0. Configuration de la prise :
 *      - Plage HC active : 22h00 à 06h00 (setReducedPeriod)
 *      - Tarifs : 25 cts HP / 15 cts HC (setPrices)
 *   1. Relevé de référence initial (B0, B1, B2, B3 pour les jours J-0 et J-1).
 *   2. Calage de l'horloge à 21h58 (Jour J). Relais ON.
 *   3. Consommation sous charge pendant 4 minutes réelles :
 *      - De 21h58 à 22h00 (2 minutes) : Heures Pleines (HP) -> ~50% de l'énergie.
 *      - De 22h00 à 22h02 (2 minutes) : Heures Creuses (HC) -> ~50% de l'énergie.
 *   4. À 22h02 : Coupure du relais (relais OFF).
 *   5. Bond temporel à 23h59:00 sur le même jour.
 *   6. Attente passive de 70 secondes pour franchir Minuit (passage au jour J+1).
 *   7. Relecture de l'historique : inspection de Slot [28] (le jour qui vient d'être scellé).
 *      Vérification de la valeur de l'octet B3 :
 *      - Si B3 ≈ 50 (0x32)  -> Pourcentage direct HC (0..100) !
 *      - Si B3 ≈ 128 (0x80) -> Ratio 8 bits HC (0..255) !
 * =====================================================================================
 */

#include <Arduino.h>
#include "SEM6000Device.h"

static const char* TARGET_MAC = "94:a9:a8:1a:37:0c";
static const uint32_t TARGET_PIN = 0x00000000;
static const int BUTTON_BOOT_PIN = 0;

static SEM6000Device device(TARGET_MAC, TARGET_PIN);

// Données d'inspection brute
struct RawDailySlot {
    uint32_t wh;
    uint8_t  b0;
    uint8_t  b1;
    uint8_t  b2;
    uint8_t  b3;
};

void waitUserPrompt(const char* title, const char* instructions) {
    Serial.println("\n================================================================================");
    Serial.printf(" >>> ÉTAPE : %s\n", title);
    Serial.println("--------------------------------------------------------------------------------");
    Serial.println(instructions);
    Serial.println("\n  👉 [Appuyez sur le bouton physique BOOT de l'ESP32 ou envoyez 'Entrée' en console]");
    Serial.println("================================================================================\n");

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
        if (millis() - lastBeacon >= 3000) {
            Serial.printf("  [EN ATTENTE - %s] Appuyez sur le bouton BOOT (GPIO 0) ou 'Entrée'...\n", title);
            lastBeacon = millis();
        }
        delay(50);
    }
    Serial.println(">>> Action validée par l'utilisateur ! Démarrage de l'étape...\n");
}

bool connectAndAuth() {
    Serial.printf("[BLE] Tentative de connexion à la prise %s...\n", TARGET_MAC);
    if (!device.connect(8000)) {
        Serial.println("[BLE] ❌ ÉCHEC de connexion ! (Vérifiez que l'application smartphone est bien fermée)");
        return false;
    }
    Serial.println("[BLE] ✅ Connecté !");

    SEM6000Status st = device.authenticate32((uint32_t)TARGET_PIN);
    if (st != SEM_OK) {
        Serial.printf("[BLE] ❌ ÉCHEC Authentification ! Code: %d\n", st);
        device.disconnect();
        return false;
    }
    Serial.println("[BLE] ✅ Authentifié avec succès (PIN 32-bit: 0x00000000) !\n");
    return true;
}

RawDailySlot getRawDailySlot(int slotIndex) {
    RawDailySlot slot = {0, 0, 0, 0, 0};
    uint32_t dummyDaily[30];
    if (device.requestMonthHistory(dummyDaily) == SEM_OK) {
        const uint8_t* rawBuf = nullptr;
        size_t len = device.getLastRxBuffer(&rawBuf);
        if (rawBuf && len >= 127 && slotIndex >= 0 && slotIndex < 30) {
            size_t idx = 4 + (slotIndex * 4);
            slot.b0 = rawBuf[idx];
            slot.b1 = rawBuf[idx + 1];
            slot.b2 = rawBuf[idx + 2];
            slot.b3 = rawBuf[idx + 3];
            slot.wh = ((uint32_t)slot.b0 << 16) | ((uint32_t)slot.b1 << 8) | slot.b2;
        }
    }
    return slot;
}

void printRawSlot(const char* label, int slotIndex, const RawDailySlot& s) {
    Serial.printf("  %-18s (Slot [%02d]) : %6lu Wh  |  Octets bruts : [0x%02X 0x%02X 0x%02X]  Octet B3 : 0x%02X (%3u dec)\n",
                  label, slotIndex, (unsigned long)s.wh, s.b0, s.b1, s.b2, s.b3, s.b3);
}

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_BOOT_PIN, INPUT_PULLUP);

    // Attente active de la connexion du moniteur série USB CDC
    uint32_t startWait = millis();
    while (!Serial && (millis() - startWait < 4000)) {
        delay(50);
    }
    delay(500);

    // Initialisation de la pile NimBLE
    NimBLEDevice::init("Gateway_Exp08");

    Serial.println("\n\n");
    Serial.println("================================================================================");
    Serial.println("  🌟 EXPÉRIENCE 08 : ANALYSE DE L'OCTET B3 & DÉTECTION DU RATIO HC (50/50)      ");
    Serial.println("================================================================================");
    Serial.println("  Objectif : Percer le mystère du 4ème octet (B3) des historiques journaliers   ");
    Serial.println("             en consommant 50% en Heures Pleines et 50% en Heures Creuses.      ");
    Serial.println("================================================================================\n");

    waitUserPrompt("1. PRÉPARATION MATÉRIELLE & DÉMARRAGE",
                   "1. Branchez votre charge (ex. appareil 500W ou ampoule).\n"
                   "2. Assurez-vous que la prise SEM6000 est bien sous tension.\n"
                   "3. Dès que vous êtes prêt :");

    // Connexion
    if (!connectAndAuth()) {
        Serial.println("[FATAL] Impossible d'établir la liaison BLE. Arrêt.");
        while (true) delay(1000);
    }

    // =================================================================================
    // CONFIGURATION DES HEURES CREUSES
    // =================================================================================
    Serial.println("[CONFIG] Configuration du mode Heures Creuses (HC)...");
    Serial.println("  -> Plage HC : 22h00 à 06h00 (active = 1)");
    device.setReducedPeriod(1, 22, 0, 6, 0);
    delay(300);

    Serial.println("  -> Tarifs   : 25.0 cts/kWh (HP) | 15.0 cts/kWh (HC)");
    device.setPrices(25.0f, 15.0f);
    delay(300);

    // Relecture des paramètres pour vérification
    SEMSettings currentSettings;
    if (device.requestSettings(currentSettings) == SEM_OK) {
        Serial.println("  ✅ Paramètres validés par la prise :");
        Serial.printf("     Mode réduit actif : %s\n", currentSettings.reducedModeActive ? "OUI (1)" : "NON (0)");
        Serial.printf("     Prix normal (HP)  : %u cts\n", currentSettings.normalPriceCents);
        Serial.printf("     Prix réduit (HC)  : %u cts\n", currentSettings.reducedPriceCents);
        Serial.printf("     Créneau HC début  : %02d:%02d\n", (currentSettings.reducedPeriodStart >> 8), (currentSettings.reducedPeriodStart & 0xFF));
        Serial.printf("     Créneau HC fin    : %02d:%02d\n\n", (currentSettings.reducedPeriodStop >> 8), (currentSettings.reducedPeriodStop & 0xFF));
    }

    // =================================================================================
    // RELEVÉ INITIAL DES SLOTS
    // =================================================================================
    Serial.println("[ÉTAT INITIAL DES REGISTRES JOURNALIERS]");
    RawDailySlot initSlot29 = getRawDailySlot(29);
    RawDailySlot initSlot28 = getRawDailySlot(28);
    printRawSlot("Aujourd'hui (J-0)", 29, initSlot29);
    printRawSlot("Hier (J-1)",        28, initSlot28);
    Serial.println();

    // =================================================================================
    // PHASE DE CONSOMMATION SYMÉTRIQUE 50/50 (2 min HP + 2 min HC)
    // =================================================================================
    waitUserPrompt("2. LANCEMENT DU TEST DE CONSOMMATION 50/50 (4 MINUTES)",
                   "Nous allons caler l'horloge à 21:58:00 (le 21/11/2026) et allumer le relais.\n"
                   "  - De 21:58 à 22:00 (2 minutes) : Période HEURES PLEINES (HP)\n"
                   "  - De 22:00 à 22:02 (2 minutes) : Période HEURES CREUSES (HC)\n"
                   "  - À 22:02 : Extinction automatique du relais.\n"
                   "\n"
                   "L'énergie totale sera donc divisée à ~50%% en HP et ~50%% en HC !");

    Serial.println("[RTC] Calage de l'horloge à 21:58:00 (21/11/2026)...");
    device.setDateTime(0, 58, 21, 21, 11, 2026);
    delay(500);

    Serial.println("[RELAIS] Allumage du relais...");
    device.setRelayState(true);
    delay(500);

    Serial.println("\n🔥 DÉMARRAGE DE LA CONSOMMATION (4 MINUTES AU TOTAL) :");

    // Phase 1 : 2 minutes en HP (21:58:00 -> 22:00:00)
    Serial.println("\n--- [PHASE 1/2 : HEURES PLEINES (21:58 -> 22:00)] ---");
    for (int elapsed = 0; elapsed < 120; elapsed += 10) {
        int rem = 240 - elapsed;
        int simSec = elapsed % 60;
        int simMin = 58 + (elapsed / 60);
        SEMTelemetry tel;
        device.captureMeasurement(tel);
        Serial.printf("  [HP - Reste %3ds] Heure estimée : 21:%02d:%02d | P: %5.1f W | I: %.3f A\n",
                      rem, simMin, simSec, tel.powerWatts, tel.currentAmperes);
        delay(10000);
    }

    // Phase 2 : 2 minutes en HC (22:00:00 -> 22:02:00)
    Serial.println("\n--- [BASCULE À 22:00:00 -> ENTRÉE EN HEURES CREUSES !] ---");
    Serial.println("--- [PHASE 2/2 : HEURES CREUSES (22:00 -> 22:02)] ---");
    for (int elapsed = 120; elapsed < 240; elapsed += 10) {
        int rem = 240 - elapsed;
        int simSec = elapsed % 60;
        int simMin = (elapsed - 120) / 60;
        SEMTelemetry tel;
        device.captureMeasurement(tel);
        Serial.printf("  [HC - Reste %3ds] Heure estimée : 22:%02d:%02d | P: %5.1f W | I: %.3f A\n",
                      rem, simMin, simSec, tel.powerWatts, tel.currentAmperes);
        delay(10000);
    }

    // Extinction du relais
    Serial.println("\n[RELAIS] Coupure du relais (fin de la consommation) !");
    device.setRelayState(false);
    delay(500);

    // Relevé intermédiaire des slots horaires
    uint16_t hourly[24];
    device.requestDayHistory(hourly);
    Serial.println("\n[RELEVÉ INTERMÉDIAIRE DES TRANCHES HORAIRES]");
    Serial.printf("  Slot [22] (Tranche 21h-22h - HP) : %u Wh\n", hourly[22]);
    Serial.printf("  Slot [23] (Tranche 22h-23h - HC) : %u Wh\n", hourly[23]);
    float totalKwh = (float)(hourly[22] + hourly[23]);
    if (totalKwh > 0) {
        float ratioHcMeasured = ((float)hourly[23] / totalKwh) * 100.0f;
        Serial.printf("  ⚖️  Ratio mesuré réel : %.1f %% HP  /  %.1f %% HC\n\n",
                      100.0f - ratioHcMeasured, ratioHcMeasured);
    }

    // =================================================================================
    // SAUT TEMPOREL À 23h59:00 & BASCULE DE MINUIT
    // =================================================================================
    waitUserPrompt("3. BOND TEMPOREL À 23:59:00 & FRANCHISSEMENT DE MINUIT",
                   "La consommation est scellée.\n"
                   "Nous allons maintenant envoyer une trame pour caler l'horloge à 23:59:00 (même jour).\n"
                   "Puis nous allons attendre 70 secondes pour franchir Minuit (23:59:59 -> 00:00:10).\n"
                   "Minuit va sceller le jour dans Slot [28] et écrire l'octet B3 !");

    Serial.println("[RTC] Saut temporel à 23:59:00 (21/11/2026)...");
    device.setDateTime(0, 59, 23, 21, 11, 2026);
    delay(500);

    Serial.println("\n⏳ COMPTE À REBOURS DU FRANCHISSEMENT DE MINUIT (~70 secondes) :");
    for (int sec = 70; sec > 0; sec -= 5) {
        int simSec = (70 - sec) % 60;
        int simMin = (70 - sec >= 60) ? 0 : 59;
        int simHour = (70 - sec >= 60) ? 0 : 23;
        int simDay = (70 - sec >= 60) ? 22 : 21;
        Serial.printf("  [Reste %02ds] Heure estimée prise : %02d/11/2026 %02d:%02d:%02d\n",
                      sec, simDay, simHour, simMin, simSec);
        delay(5000);
    }

    // =================================================================================
    // VÉRIFICATION FINALE : L'OCTET B3
    // =================================================================================
    Serial.println("\n[MINUIT FRANCHI !] Relecture immédiate des slots journaliers avec B3...");
    RawDailySlot postSlot28 = getRawDailySlot(28);
    RawDailySlot postSlot29 = getRawDailySlot(29);

    Serial.println("\n--------------------------------------------------------------------------------");
    Serial.println("  📊 RÉSULTATS POST-MINUIT (HISTORIQUE SCIELLÉ DANS SLOT [28])");
    Serial.println("--------------------------------------------------------------------------------");
    printRawSlot("Journée scellée (J-1)", 28, postSlot28);
    printRawSlot("Nouveau jour (J-0)",    29, postSlot29);
    Serial.println("--------------------------------------------------------------------------------\n");

    // ANALYSE ET VERDICT
    Serial.println("================================================================================");
    Serial.println(" 🎯 ANALYSE & VERDICT SUR L'OCTET B3 :");
    Serial.printf("  Valeur observée sur B3 (Slot [28]) : 0x%02X (%u en décimal)\n\n",
                  postSlot28.b3, postSlot28.b3);

    if (postSlot28.b3 >= 40 && postSlot28.b3 <= 60) {
        Serial.println("  🎉 VICTOIRE TOTALE — HYPOTHÈSE DU POURCENTAGE DIRECT CONFIRMÉE !");
        Serial.printf("     -> B3 = %u %% ! (Correspond exactement aux ~50%% d'Heures Creuses consommées).\n", postSlot28.b3);
        Serial.println("     -> L'octet B3 code très exactement le POURCENTAGE (0 à 100%) d'énergie consommée en HC !");
    } else if (postSlot28.b3 >= 115 && postSlot28.b3 <= 140) {
        Serial.println("  🎉 VICTOIRE TOTALE — HYPOTHÈSE DU RATIO 8 BITS (0..255) CONFIRMÉE !");
        Serial.printf("     -> B3 = %u / 255 = %.1f %% ! (Correspond exactement aux ~50%% d'Heures Creuses consommées).\n",
                      postSlot28.b3, (postSlot28.b3 / 255.0f) * 100.0f);
        Serial.println("     -> L'octet B3 code un ratio normalisé sur un octet complet (0 à 255) !");
    } else if (postSlot28.b3 == 0) {
        Serial.println("  ℹ️  CONSTAT : B3 est resté à 0x00.");
        Serial.println("     -> Soit le firmware n'incrémente B3 que sous certaines conditions de durée,");
        Serial.println("     -> Soit B3 a un autre rôle (ex: code tarifaire ou réservé).");
    } else {
        Serial.printf("  ℹ️  VALEUR PARTICULIÈRE : B3 = 0x%02X (%u décimal). À analyser.\n", postSlot28.b3, postSlot28.b3);
    }
    Serial.println("================================================================================\n");

    // =================================================================================
    // RESTAURATION DE LA PRISE
    // =================================================================================
    waitUserPrompt("4. RESTAURATION & FIN DU TEST",
                   "Le test est terminé ! Nous allons remettre la prise à l'heure normale\n"
                   "(17:30:00 en plein milieu d'heure) et libérer la liaison BLE.\n"
                   "Appuyez sur BOOT pour effectuer la synchronisation finale.");

    Serial.println("[RESTORE] Remise à l'heure sur 21/11/2026 à 17:30:00...");
    device.setDateTime(0, 30, 17, 21, 11, 2026);
    delay(500);

    Serial.println("[BLE] Déconnexion propre de la prise...");
    device.disconnect();

    Serial.println("\n================================================================================");
    Serial.println("  🎉 EXPÉRIENCE 08 TERMINÉE : PRISE RESTAURÉE & DÉCONNECTÉE                     ");
    Serial.println("================================================================================\n");
}

void loop() {
    delay(1000);
}
