# Directives et Règles du Projet GatewaySEM6000

## 0. Posture Collaborative & Gouvernance de Travail (Règle Cardinale)
* **Travail en binôme & Accord préalable obligatoire :** Lorsque l'utilisateur formule une remarque, partage une réflexion ou demande un avis (*« Ton avis ? »*, *« Qu'en penses-tu ? »*, etc.), **ne jamais enclencher d'action** (création, modification, suppression, restructuration de fichiers ou lancement de compilation) avant d'avoir exposé l'analyse/proposition et obtenu son **accord explicite**.
* **Zéro fait accompli :** Ne jamais placer l'utilisateur devant le fait accompli. La démarche d'investigation, de dialogue et de compréhension partagée prime sur la vitesse d'exécution, sauf instruction explicite contraire de l'utilisateur (*« Vas-y, code et compile »*).

---

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
  * *Règle :* Pilote scellé et validé en **v0.4.0 ("La Grande Réunification : HW 2.0 & HW 3.0")** — support bi-génération transparent (`HW 2.0 / FW 1.24` et `HW 3.0 / FW 1.30`). Préserver et ne pas altérer.
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
* **Traçabilité & Archivage des Versions de `SEM6000Device` :**
  * **Mention d'en-tête obligatoire (`@requires`) :** Tout programme, test d'expérimentation (`experiments/exp_*.cpp`) ou couche logicielle (`ManagedSEM6000`, `SEM6000Manager`, `main.cpp`) s'appuyant sur la librairie `SEM6000Device` doit mentionner explicitement dans son cartouche d'en-tête le numéro de version de la librairie requis pour son fonctionnement optimal (ex. `@requires SEM6000Device >= v0.4.0`), appuyé par les constantes `SEM6000_DEVICE_VERSION_STR` / `SEM6000_DEVICE_VERSION_HEX`.
  * **Archivage systématique des versions :** Chaque version jalon de `SEM6000Device.h` et `SEM6000Device.cpp` doit être archivée à l'identique dans `archives/SEM6000Device/vX.Y.Z/` (hors des répertoires `lib/` et `src/` scrutés par PlatformIO) afin de permettre à tout moment la consultation, la comparaison ou la réexécution de travaux réalisés sur une version antérieure.

---

## 5. Directives Systèmes & Rigueur Embarquée (ESP32-S3 / C++)

### A. Gestion Stricte de la Mémoire & Anti-Fragmentation
* **Bannissement de la classe `String` :** L'usage de `String` (Arduino) est formellement interdit dans le code opérationnel nominal (Couches 2 et 3). Utiliser exclusivement des buffers fixes `char[]`, des pointeurs/longueurs ou `std::string_view` pour prévenir toute fragmentation du tas (*Heap*).
* **Zéro Allocation Dynamique Volatile :** Aucun `malloc()`, `calloc()` ou `new` dans les boucles de traitement (`loop()`, `update()`). Les instances de prises et buffers de travail doivent être alloués statiquement au démarrage (`setup()`).
* **SRAM Interne vs PSRAM (ESP32-S3) :** 
  * Tous les buffers DMA, descripteurs NimBLE, variables de synchronisation inter-cœurs et piles de tâches FreeRTOS doivent résider impérativement en **SRAM Interne**.
  * Réserver l'usage de la PSRAM externe (`MALLOC_CAP_SPIRAM`) uniquement aux structures volumineuses non soumises au DMA direct (ex. tables de diagnostic étendues, historiques passifs volumineux).

### B. Concurrence, Multi-Cœur & FreeRTOS
* **Cloisonnement Dual-Core :** Identifier explicitement le cœur d'exécution. Les opérations radio (stacks Wi-Fi et BLE NimBLE sur Core 0) et l'application métier (Core 1) ne doivent pas partager d'états mutables sans synchronisation explicite.
* **Synchronisation Sécurisée :** Protéger toute ressource partagée via `SemaphoreHandle_t` (Mutex FreeRTOS) ou sections critiques légères (`portMUX_TYPE`). Toujours privilégier le pattern RAII pour le verrouillage/déverrouillage.
* **Règles d'ISR (Interruptions) :** Toute routine d'interruption doit porter l'attribut `IRAM_ATTR`. Zéro allocation mémoire, zéro appel système bloquant dans une ISR ; communication exclusive vers les tâches via Queues FreeRTOS avec réveil différé (`xHigherPriorityTaskWoken`).

### C. Architecture C++ & Modélisation Objet
* **Pattern par Composition Forte :** Respecter la composition forte (has-a) plutôt que l'héritage (is-a) entre les couches (ex: `ManagedSEM6000` encapsule en membre privé un `SEM6000Device`).
* **Zéro Avertissement de Compilation :** Respecter le typage strict des formats `printf` (spécificateur `%lu` pour les types `uint32_t` sur toolchain Xtensa) pour garantir un build 100 % silencieux.
