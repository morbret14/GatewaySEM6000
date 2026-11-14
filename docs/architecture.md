# Architecture & Spécifications du Protocole — Passerelle Domoticz SEM6000

Ce document consigne l'état de l'art des connaissances acquises par rétro-ingénierie sur les prises connectées **Voltcraft SEM6000-FR** (Hardware v2.0 / Firmware v1.24) et définit l'architecture logicielle de la passerelle **ESP32-S3** sous **PlatformIO / Framework Arduino**.

---

## 1. Environnement Matériel & Logiciel

* **Microcontrôleur :** ESP32-S3 (Flash 16 Mo, PSRAM OPI, USB CDC actif au démarrage).
* **Environnement de développement :** PlatformIO Core (VS Code / Google Antigravity) — Framework Arduino.
* **Pile Bluetooth Low Energy :** `h2zero/NimBLE-Arduino` verrouillée en version **`2.5.2`**.
* **Cible Domotique :** Domoticz (hébergé sur Raspberry Pi 4) via API HTTP/JSON ou MQTT.

### Configuration `platformio.ini` de référence

    [env:esp32-s3-devkitc-1]
    platform = espressif32
    board = esp32-s3-devkitc-1
    framework = arduino

    board_build.arduino.memory_type = qio_opi
    board_build.flash_mode = qio
    board_build.partitions = app3M_fat9M_16MB.csv
    board_upload.flash_size = 16MB

    build_flags = 
        -DBOARD_HAS_PSRAM
        -DARDUINO_USB_MODE=1
        -DARDUINO_USB_CDC_ON_BOOT=1

    lib_deps = 
        h2zero/NimBLE-Arduino @ ^2.5.2

    monitor_speed = 115200

---

## 2. Rétro-ingénierie du Protocole BLE (Voltcraft SEM6000)

### 2.1 Authentification (`authenticate32`)
* L'accès aux commandes d'état, de télémétrie complète et d'historiques nécessite l'envoi d'un code PIN sur 32 bits (valeur par défaut : `(uint32_t)0x00000000`).
* **Note C++ :** Toujours typer explicitement le paramètre `(uint32_t)0x00000000` lors de l'appel à `authenticate32()` pour éviter toute ambiguïté de surcharge avec `const uint8_t*`.

### 2.2 Caractéristique `FFF2` (Diagnostic léger sans authentification)
Accessible en lecture directe (`READ`) **sans authentification préalable**, cette caractéristique retourne une trame brute de **4 octets** (`uint8_t[4]`) :

* **Octets `[0..1]` (`uint16_t` Big-Endian) — Seuil de puissance maximale / État horloge :** Indique la limite de coupure configurée (en W). **Valeur sentinelle `0xFFFF`** : signale une coupure secteur récente ayant entraîné la perte de l'heure interne (RTC non synchronisée).
* **Octets `[2..3]` (`uint16_t` Big-Endian) — Courant instantané :** Exprimé en milliampères (mA). L'octet `[3]` est corrélé à la puissance active et pilote la couleur de l'anneau LED de la prise.

### 2.3 Télémétrie Instantanée (`0x0400`)
* Fournit en temps réel : état du relais (`ON/OFF`), puissance active (W), tension (V), intensité (mA) et facteur de puissance.
* **Absence d'index perpétuel natif :** Sur Hardware v2.0, les octets 15 à 18 (`totalKwhWh`) ne fournissent pas un compteur cumulatif perpétuel fiable. La prise ne maintient aucun index global survivant aux cycles glissants ou coupures secteur.

### 2.4 Registres d'Historiques (`0x0A00`, `0x0B00`, `0x0C00`)
Validés par croisement entre captures BLE et exports CSV de l'application constructeur. **Les trois tableaux suivent strictement la même logique d'indexation chronologique :** le dernier index représente toujours la période en cours, et décrémenter l'index de N permet de remonter de N périodes dans le passé.

| Registre | Méthode `SEM6000Device` | Type & Taille | Période en cours (Présent) | Période précédente (N-1) | Cinématique de mise à jour |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Horaire (`0x0A00`)** | `requestDayHistory` | `uint16_t[24]` | **Slot `[23]`** | Slot `[22]` (H-1) | S'incrémente en continu (Wh). À chaque heure pile (`HH:00:00`), décale vers la gauche et repart à `0`. |
| **Journalier (`0x0B00`)** | `requestMonthHistory` | `uint32_t[30]` | **Slot `[29]`** | Slot `[28]` (Hier) | Mis à jour **à chaque heure pile (`HH:00:00`)** par addition de l'heure qui vient de se fermer. |
| **Mensuel (`0x0C00`)** | `requestYearHistory` | `uint32_t[12]` | **Slot `[11]`** | Slot `[10]` (M-1) | Mis à jour **à chaque heure pile (`HH:00:00`)** par addition de l'heure qui vient de se fermer. |

---

## 3. Architecture Logicielle Cible

L'architecture est découpée en trois couches distinctes pour isoler le protocole BLE bas niveau de la logique métier et du réseau :

    GatewaySEM6000/
    ├── docs/
    │   └── ARCHITECTURE.md          # Spécifications techniques (ce fichier)
    ├── lib/
    │   └── SEM6000Device/           # Couche 1 : Pilote BLE pur (NimBLE)
    │       ├── SEM6000Device.h
    │       └── SEM6000Device.cpp
    └── src/
        ├── ManagedSEM6000.h/.cpp    # Couche 2 : Machine à états, horloge & Index Perpétuel
        ├── SEM6000Manager.h/.cpp    # Couche 3 : Ordonnanceur multi-prises & Queue BLE
        └── main.cpp                 # Wi-Fi, NTP, Client Domoticz & Boucle principale

### Couche 1 : `SEM6000Device` (Validée — v0.3.7)
* Encapsule uniquement la communication GATT/NimBLE avec une prise (connexion, authentification 32 bits, commandes ON/OFF, synchronisation horloge, lecture `FFF2`, `0x0400` et historiques).
* Accesseurs standardisés sur les historiques :
  * `hourlyWh[23 - hoursAgo]`
  * `dailyWh[29 - daysAgo]`
  * `monthlyWh[11 - monthsAgo]`

### Couche 2 : `ManagedSEM6000` (Prochaine étape d'implémentation)
Encapsule une instance `SEM6000Device` pour garantir la continuité des données vers Domoticz :
1. **Détection de démarrage à froid :** Lecture de `FFF2` avant authentification. Si octets `[0..1] == 0xFFFF`, déclenchement immédiat d'une remise à l'heure NTP de la prise dès la connexion.
2. **Reconstruction de l'Index Perpétuel 32 bits (`uint32_t indexWh`) :**
   * S'appuie sur le différentiel d'énergie des registres internes de la prise (`hourlyWh[23]` et `hourlyWh[22]` lors d'un franchissement d'heure pile) afin de ne perdre aucun Watt-heure même en cas d'interruption temporaire du lien BLE.
   * Sauvegarde périodique en mémoire non volatile (**NVS / Preferences**) de `indexWh`, du timestamp du dernier commit et de la dernière valeur d'heure connue.

### Couche 3 : `SEM6000Manager`
* Gère un parc de plusieurs prises SEM6000 en séquentialisant les accès radio BLE (évite les collisions NimBLE), distribue le temps NTP et pousse les mises à jour vers Domoticz.

