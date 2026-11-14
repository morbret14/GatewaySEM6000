# Directives et Règles du Projet GatewaySEM6000

## 1. Référence Technique Unique (SSOT)
La spécification technique complète (détail du protocole BLE SEM6000, structure des trames, registres d'historiques, logique de calcul d'énergie et intégration Domoticz) est consignée dans :
👉 `docs/architecture.md`

Consulter systématiquement ce document pour toute implémentation ou ajustement d'algorithme.

---

## 2. Environnement Matériel et Logiciel
* **Cible matérielle :** ESP32-S3 DevKitC-1 (Flash 16 Mo QIO, PSRAM OPI, USB CDC natif actif au démarrage).
* **Environnement de développement :** PlatformIO Core — Framework Arduino.
* **Fichier de configuration :** `platformio.ini`.
* **Pile Bluetooth :** `h2zero/NimBLE-Arduino` strictement verrouillée en version **`^2.5.2`**.
* **Vitesse moniteur série :** 115200 bauds.
* **Gestionnaire de versions :** Git / GitHub (`origin` -> `morbret14/GatewaySEM6000`).

---

## 3. Découpage en 3 Couches (Vue d'ensemble)
* **Couche 1 : Pilote BLE pur (`lib/SEM600Device/SEM6000Device.h/.cpp`)**
  * Encapsulation GATT/NimBLE bas niveau.
  * *Règle :* Pilote validé en v0.3.7 — préserver et ne pas altérer inutilement.
* **Couche 2 : Machine à états & Index Perpétuel (`src/ManagedSEM6000.h/.cpp`)**
  * Encapsule `SEM6000Device`. Gère la détection de coupure secteur, la remise à l'heure NTP et la continuité de l'index perpétuel en mémoire non volatile (`Preferences` / NVS).
* **Couche 3 : Ordonnanceur & Réseau (`src/SEM6000Manager.h/.cpp` et `src/main.cpp`)**
  * Gestionnaire de parc multi-prises (accès BLE séquentiels), synchronisation NTP, connexion Wi-Fi et client Domoticz.

---

## 4. Règles de Développement & Garde-Fous
* **Validation de compilation :** Toujours vérifier la non-régression via la commande `pio run`.
* **Typage strict C++ :** Pour `authenticate32()`, toujours typer explicitement le code PIN en `(uint32_t)0x00000000` pour éviter l'ambiguïté de surcharge avec `const uint8_t*`.
* **Séquencement radio BLE :** Ne jamais lancer d'opérations BLE concurrentes sur plusieurs prises simultanément ; toujours sérialiser les connexions.
* **Code non-bloquant :** Bannir les boucles d'attente bloquantes (`delay()`) pouvant perturber les tâches système ou déclencher le chien de garde (Watchdog ESP32).
