/*
 * =====================================================================================
 * EXPÉRIENCE 07 : Validation RTC, Bascule de Minuit & Impact du Recul d'Horloge
 * =====================================================================================
 * Cible : Prise Voltcraft SEM6000-FR (MAC: 94:a9:a8:1a:37:0c)
 *
 * Objectifs :
 *   1. PARTIE 1 : Vérifier le trigger de rotation journalière (Minuit).
 *      - Injection de 23:59:45 au jour J (21/11/2026).
 *      - Attente passive de ~35 secondes sous charge (> 60 W) pour franchir 00:00:00 (J+1).
 *      - Vérification du glissement de daily[29] (Aujourd'hui) -> daily[28] (Hier).
 *
 *   2. PARTIE 2 : Vérifier le comportement en cas de recul d'horloge (Double Rotation).
 *      - Cycle 1 : Injection de 01:58:30 -> Attente de 180s pour franchir 02:00:00 -> 02:01:30.
 *                  Relevé des slots horaires [21], [22], [23].
 *      - Recalage arrière : Réinjection forcée de 01:58:30.
 *      - Cycle 2 : Attente de 180s pour rejouer 02:00:00 -> 02:01:30.
 *                  Relevé des slots horaires pour constater si [22] est décalé dans [21].
 *
 * Ergonomie :
 *   - L'ESP32 guide l'utilisateur à chaque étape et attend l'appui sur le bouton BOOT (GPIO 0)
 *     ou la touche 'Entrée' dans le moniteur série.
 * =====================================================================================
 */

#include <Arduino.h>
#include "SEM6000Device.h"

static const char* TARGET_MAC = "94:a9:a8:1a:37:0c";
static const uint32_t TARGET_PIN = 0x00000000;
static const int BUTTON_BOOT_PIN = 0;

static SEM6000Device device(TARGET_MAC, TARGET_PIN);

// Structure de stockage des relevés
struct FullSnapshot {
    SEMTelemetry telemetry;
    uint16_t fff2_word0;
    uint8_t  fff2_byte2;
    float    fff2_currentA;
    uint16_t hourly[24];
    uint32_t daily[30];
    uint32_t monthly[12];
};

static FullSnapshot initialSnap;
static FullSnapshot postMidnightSnap;
static FullSnapshot cycle1Snap;
static FullSnapshot cycle2Snap;

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
        Serial.println("[BLE] ❌ ÉCHEC de connexion !");
        Serial.println("      (Vérifiez que l'application smartphone est bien fermée)");
        return false;
    }
    Serial.println("[BLE] ✅ Connecté !");

    SEM6000Status st = device.authenticate32((uint32_t)TARGET_PIN);
    if (st != SEM_OK) {
        Serial.printf("[BLE] ❌ ÉCHEC Authentification ! Code erreur: %d\n", st);
        device.disconnect();
        return false;
    }
    Serial.println("[BLE] ✅ Authentifié avec succès (PIN 32-bit: 0x00000000) !\n");
    return true;
}

bool captureFullSnapshot(FullSnapshot& s) {
    // 1. Diagnostic FFF2
    if (!device.readRawFFF2(s.fff2_word0, s.fff2_byte2, s.fff2_currentA)) {
        Serial.println("[WARN] Échec lecture FFF2");
        return false;
    }

    // 2. Télémesure 0x0400
    if (device.captureMeasurement(s.telemetry) != SEM_OK) {
        Serial.println("[WARN] Échec télémesure 0x0400");
        return false;
    }

    // 3. Historiques
    if (device.requestDayHistory(s.hourly) != SEM_OK) {
        Serial.println("[WARN] Échec lecture historique horaire");
        return false;
    }
    if (device.requestMonthHistory(s.daily) != SEM_OK) {
        Serial.println("[WARN] Échec lecture historique journalier");
        return false;
    }
    if (device.requestYearHistory(s.monthly) != SEM_OK) {
        Serial.println("[WARN] Échec lecture historique mensuel");
        return false;
    }

    return true;
}

void printComparisonTableDaily(const char* labelBefore, const FullSnapshot& before,
                               const char* labelAfter, const FullSnapshot& after) {
    Serial.println("\n--------------------------------------------------------------------------------");
    Serial.println("  📊 COMPARAISON DE L'HISTORIQUE JOURNALIER (Wh)");
    Serial.println("--------------------------------------------------------------------------------");
    Serial.printf("  Slot             | %-20s | %-20s | Différence\n", labelBefore, labelAfter);
    Serial.println("-------------------+----------------------+----------------------+--------------");

    for (int i = 27; i < 30; i++) {
        const char* name = (i == 29) ? "Slot [29] (J-0)" :
                           (i == 28) ? "Slot [28] (J-1)" : "Slot [27] (J-2)";
        int32_t diff = (int32_t)after.daily[i] - (int32_t)before.daily[i];
        char diffStr[30];
        if (diff != 0) {
            snprintf(diffStr, sizeof(diffStr), "%+ld Wh (CHANGÉ)", (long)diff);
        } else {
            snprintf(diffStr, sizeof(diffStr), "0 Wh (identique)");
        }
        Serial.printf("  %-16s | %10lu Wh        | %10lu Wh        | %s\n",
                      name,
                      (unsigned long)before.daily[i],
                      (unsigned long)after.daily[i],
                      diffStr);
    }
    Serial.println("--------------------------------------------------------------------------------\n");
}

void printComparisonTableHourly(const char* labelBefore, const FullSnapshot& before,
                                const char* labelAfter, const FullSnapshot& after) {
    Serial.println("\n--------------------------------------------------------------------------------");
    Serial.println("  📊 COMPARAISON DE L'HISTORIQUE HORAIRE (Wh)");
    Serial.println("--------------------------------------------------------------------------------");
    Serial.printf("  Slot             | %-20s | %-20s | Différence\n", labelBefore, labelAfter);
    Serial.println("-------------------+----------------------+----------------------+--------------");

    for (int i = 20; i < 24; i++) {
        const char* name = (i == 23) ? "Slot [23] (En cours)" :
                           (i == 22) ? "Slot [22] (H-1)" :
                           (i == 21) ? "Slot [21] (H-2)" : "Slot [20] (H-3)";
        int32_t diff = (int32_t)after.hourly[i] - (int32_t)before.hourly[i];
        char diffStr[30];
        if (diff != 0) {
            snprintf(diffStr, sizeof(diffStr), "%+ld Wh (CHANGÉ)", (long)diff);
        } else {
            snprintf(diffStr, sizeof(diffStr), "0 Wh (identique)");
        }
        Serial.printf("  %-16s | %10u Wh        | %10u Wh        | %s\n",
                      name,
                      before.hourly[i],
                      after.hourly[i],
                      diffStr);
    }
    Serial.println("--------------------------------------------------------------------------------\n");
}

void setup() {
    Serial.begin(115200);
    pinMode(BUTTON_BOOT_PIN, INPUT_PULLUP);

    // Attente active de la connexion du moniteur série USB CDC (max 4 secondes)
    uint32_t startWait = millis();
    while (!Serial && (millis() - startWait < 4000)) {
        delay(50);
    }
    delay(500);

    // Initialisation de la pile NimBLE (indispensable pour les mutex FreeRTOS)
    NimBLEDevice::init("Gateway_Exp07");

    Serial.println("\n\n");
    Serial.println("================================================================================");
    Serial.println("  🌟 EXPÉRIENCE 07 : VALIDATION DU COMPORTEMENT RTC & ROTATION DES BUFFERS      ");
    Serial.println("================================================================================");
    Serial.println("  Ce test valide deux comportements fondamentaux du SEM6000 :");
    Serial.println("    1. La bascule de minuit (déclenchement de la rotation journalière daily[29]->[28])");
    Serial.println("    2. Le recul d'horloge (vérification de la double rotation sur créneau rejoué)");
    Serial.println("================================================================================\n");

    waitUserPrompt("1. PRÉPARATION MATÉRIELLE",
                   "1. Branchez votre charge (ex. ampoule à incandescence 60-100W ou autre appareil).\n"
                   "2. Assurez-vous que la prise SEM6000 est bien sous tension.\n"
                   "3. Prenez tout votre temps ! Dès que vous êtes prêt :");

    // Connexion
    if (!connectAndAuth()) {
        Serial.println("[FATAL] Impossible de joindre la prise. Redémarrez l'ESP32 pour réessayer.");
        while (true) delay(1000);
    }

    // Assurer que le relais est allumé
    Serial.println("[SETUP] Vérification de l'état du relais...");
    device.setRelayState(true);
    delay(500);

    // Prise d'une référence initiale
    captureFullSnapshot(initialSnap);

    Serial.println("[ÉTAT INITIAL]");
    Serial.printf("  Tension : %d V | Fréquence : %d Hz\n", initialSnap.telemetry.voltageVolts, initialSnap.telemetry.frequencyHz);
    Serial.printf("  Puissance instantanée : %.1f W | Courant : %.3f A\n", initialSnap.telemetry.powerWatts, initialSnap.telemetry.currentAmperes);
    Serial.printf("  Slot horaire [23] (En cours) : %u Wh | Slot [22] (H-1) : %u Wh\n", initialSnap.hourly[23], initialSnap.hourly[22]);
    Serial.printf("  Slot journalier [29] (Aujourd'hui) : %lu Wh | Slot [28] (Hier) : %lu Wh\n",
                  (unsigned long)initialSnap.daily[29], (unsigned long)initialSnap.daily[28]);

    // =================================================================================
    // PARTIE 1 : LE VERDICT DE MINUIT
    // =================================================================================
    waitUserPrompt("2. PARTIE 1 : TEST DE LA BASCULE DE MINUIT",
                   "Nous allons programmer l'horloge de la prise à 23:59:45 (le 21/11/2026).\n"
                   "Puis nous allons attendre 35 secondes sous charge pour la laisser franchir\n"
                   "naturellement le cap de minuit (23:59:59 -> 00:00:00).\n"
                   "\n"
                   "Hypothèse testée : Le franchissement de minuit déclenche-t-il immédiatement\n"
                   "la rotation du buffer journalier (daily[29] -> daily[28]) ?");

    Serial.println("[RTC] Envoi de l'heure : 21/11/2026 à 23:59:45...");
    SEM6000Status st1 = device.setDateTime(45, 59, 23, 21, 11, 2026);
    if (st1 != SEM_OK) {
        Serial.printf("[RTC] ❌ Erreur setDateTime: %d\n", st1);
    } else {
        Serial.println("[RTC] ✅ Heure 23:59:45 acceptée par la prise !");
    }

    Serial.println("\n⏳ COMPTE À REBOURS DU FRANCHISSEMENT DE MINUIT (~35 secondes) :");
    for (int sec = 35; sec > 0; sec--) {
        int simulatedSec = 45 + (35 - sec);
        int simMin = 59;
        int simHour = 23;
        int simDay = 21;
        if (simulatedSec >= 60) {
            simulatedSec -= 60;
            simMin = 0;
            simHour = 0;
            simDay = 22;
        }

        SEMTelemetry liveTel;
        device.captureMeasurement(liveTel);

        Serial.printf("  [T-%02ds] Heure estimée prise : %02d/11/2026 %02d:%02d:%02d | P: %5.1f W | I: %.3f A\n",
                      sec, simDay, simHour, simMin, simulatedSec, liveTel.powerWatts, liveTel.currentAmperes);
        delay(1000);
    }

    Serial.println("\n[PARTIE 1] Minuit est franchi ! Lecture de l'historique post-minuit...");
    captureFullSnapshot(postMidnightSnap);

    printComparisonTableDaily("Avant Minuit", initialSnap, "Après Minuit", postMidnightSnap);

    // Analyse pédagogique du résultat
    Serial.println("================================================================================");
    Serial.println(" 🎯 ANALYSE & VERDICT PARTIE 1 :");
    if (postMidnightSnap.daily[28] == initialSnap.daily[29] && postMidnightSnap.daily[29] < initialSnap.daily[29]) {
        Serial.println("  ✅ SUCCÈS TOTAL : daily[29] a basculé dans daily[28] lors du passage de minuit !");
        Serial.println("     -> Preuve formelle que l'historique journalier est piloté par le front de MINUIT.");
    } else if (postMidnightSnap.daily[29] >= initialSnap.daily[29] && postMidnightSnap.daily[28] == initialSnap.daily[28]) {
        Serial.println("  ℹ️  CONSTAT : daily[29] n'a PAS basculé dans daily[28] (il a continué de cumuler).");
        Serial.println("     -> Cela indiquerait que le trigger journalier dépend d'un compteur horaire ou d'une autre condition.");
    } else {
        Serial.printf("  ℹ️  RÉSULTAT PARTICULIER : daily[29]=%lu Wh, daily[28]=%lu Wh.\n",
                      (unsigned long)postMidnightSnap.daily[29], (unsigned long)postMidnightSnap.daily[28]);
    }
    Serial.println("================================================================================\n");

    // =================================================================================
    // PARTIE 2 : RECUL D'HORLOGE & DOUBLE ROTATION
    // =================================================================================
    waitUserPrompt("3. PARTIE 2 : RECUL D'HORLOGE & DOUBLE ROTATION",
                   "Nous allons maintenant tester ce qui se passe lorsqu'on force un recul d'heure.\n"
                   "  - Cycle 1 : On injecte 01:58:30. On attend 3 minutes (180s) sous charge pour\n"
                   "              franchir 02:00:00 et observer la fermeture normale du slot [22].\n"
                   "  - Recalage arrière : On force à nouveau l'horloge à reculer à 01:58:30.\n"
                   "  - Cycle 2 : On attend à nouveau 3 minutes (180s) pour franchir une 2nde fois\n"
                   "              la même heure pile (02:00:00).\n"
                   "\n"
                   "Hypothèse testée : Le recul d'heure fait-il décaler à nouveau le buffer [22] dans [21]\n"
                   "(phénomène de double rotation et création de slot fantôme) ?");

    // CYCLE 1
    Serial.println("[CYCLE 1] Programmation de l'horloge à 01:58:30 (22/11/2026)...");
    device.setDateTime(30, 58, 1, 22, 11, 2026);
    Serial.println("[CYCLE 1] Horloge calée. Début de l'attente active de 180s (3 minutes)...");

    for (int elapsed = 0; elapsed < 180; elapsed += 10) {
        int rem = 180 - elapsed;
        int simSec = 30 + (elapsed % 60);
        int simMin = 58 + (elapsed / 60);
        int simHour = 1;
        if (simSec >= 60) { simSec -= 60; simMin++; }
        if (simMin >= 60) { simMin -= 60; simHour = 2; }

        SEMTelemetry tel;
        device.captureMeasurement(tel);
        Serial.printf("  [Cycle 1 - Reste %3ds] Heure estimée : %02d:%02d:%02d | P: %5.1f W | I: %.3f A\n",
                      rem, simHour, simMin, simSec, tel.powerWatts, tel.currentAmperes);
        delay(10000);
    }

    captureFullSnapshot(cycle1Snap);
    Serial.println("\n[CYCLE 1 TERMINÉ]");
    Serial.printf("  Slot [23] (En cours) : %u Wh\n", cycle1Snap.hourly[23]);
    Serial.printf("  Slot [22] (H-1)      : %u Wh\n", cycle1Snap.hourly[22]);
    Serial.printf("  Slot [21] (H-2)      : %u Wh\n", cycle1Snap.hourly[21]);

    // RECALAGE ARRIÈRE
    Serial.println("\n--------------------------------------------------------------------------------");
    Serial.println(" >>> ⏪ INJECTION DU RECUL D'HORLOGE : RETOUR FORCÉ À 01:58:30 !");
    Serial.println("--------------------------------------------------------------------------------");
    device.setDateTime(30, 58, 1, 22, 11, 2026);
    delay(500);

    // CYCLE 2
    Serial.println("[CYCLE 2] Début de la 2nde attente active de 180s (3 minutes) sous la même charge...");

    for (int elapsed = 0; elapsed < 180; elapsed += 10) {
        int rem = 180 - elapsed;
        int simSec = 30 + (elapsed % 60);
        int simMin = 58 + (elapsed / 60);
        int simHour = 1;
        if (simSec >= 60) { simSec -= 60; simMin++; }
        if (simMin >= 60) { simMin -= 60; simHour = 2; }

        SEMTelemetry tel;
        device.captureMeasurement(tel);
        Serial.printf("  [Cycle 2 - Reste %3ds] Heure estimée : %02d:%02d:%02d | P: %5.1f W | I: %.3f A\n",
                      rem, simHour, simMin, simSec, tel.powerWatts, tel.currentAmperes);
        delay(10000);
    }

    captureFullSnapshot(cycle2Snap);

    printComparisonTableHourly("Fin Cycle 1", cycle1Snap, "Fin Cycle 2", cycle2Snap);

    // Analyse pédagogique du résultat
    Serial.println("================================================================================");
    Serial.println(" 🎯 ANALYSE & VERDICT PARTIE 2 :");
    if (cycle2Snap.hourly[21] == cycle1Snap.hourly[22] && cycle2Snap.hourly[21] > 0) {
        Serial.println("  ⚠️ DOUBLE ROTATION CONFIRMÉE !");
        Serial.printf("     -> Le slot fermé au cycle 1 (%u Wh) a été poussé vers le slot [21] !\n", cycle2Snap.hourly[21]);
        Serial.println("     -> Cela prouve formellement que reculer l'horloge à travers une heure pile");
        Serial.println("        génère une rotation prématurée et un créneau horaire artificiel.");
        Serial.println("     -> RÈGLE D'OR VALIDÉE : Ne jamais reculer l'horloge à proximité d'une heure pile !");
    } else if (cycle2Snap.hourly[22] > cycle1Snap.hourly[22] && cycle2Snap.hourly[21] == cycle1Snap.hourly[21]) {
        Serial.println("  ℹ️  CUMUL SUR LE MÊME CRÉNEAU :");
        Serial.printf("     -> Le slot [22] a simplement accumulé l'énergie sans décaler vers [21] (%u Wh -> %u Wh).\n",
                      cycle1Snap.hourly[22], cycle2Snap.hourly[22]);
    } else {
        Serial.printf("  ℹ️  RÉSULTAT : [23]=%u Wh, [22]=%u Wh, [21]=%u Wh\n",
                      cycle2Snap.hourly[23], cycle2Snap.hourly[22], cycle2Snap.hourly[21]);
    }
    Serial.println("================================================================================\n");

    // =================================================================================
    // FIN DU TEST : REMISE À L'HEURE RÉELLE OU UTC+1
    // =================================================================================
    waitUserPrompt("4. RESTAURATION DE LA PRISE & FIN DU TEST",
                   "Les deux tests sont terminés avec succès !\n"
                   "Nous allons maintenant reprogrammer la prise sur la date/heure actuelle\n"
                   "en temps invariant (ex. 21/11/2026 à 16:30:00) pour qu'elle reprenne son cours normal.\n"
                   "Appuyez sur BOOT pour effectuer la synchronisation et libérer le BLE.");

    Serial.println("[RESTORE] Resynchronisation sur 21/11/2026 à 16:30:00 (en plein milieu d'heure)...");
    device.setDateTime(0, 30, 16, 21, 11, 2026);
    delay(500);

    Serial.println("[BLE] Déconnexion propre de la prise...");
    device.disconnect();

    Serial.println("\n================================================================================");
    Serial.println("  🎉 FIN DE L'EXPÉRIENCE 07 : PRISE DÉCONNECTÉE & DISPONIBLE                    ");
    Serial.println("================================================================================\n");
}

void loop() {
    // Rien à faire en boucle, le test est séquentiel et s'exécute dans setup()
    delay(1000);
}
