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

### 2.2 Caractéristique `FFF2` (Diagnostic léger & Surcharge sans authentification)
Accessible en lecture directe (`READ`) **sans aucune authentification préalable**, cette caractéristique retourne une trame brute de **4 octets** (`uint8_t[4]`) d'une cohérence remarquable : les deux mots de 16 bits partagent la **même unité physique (le milliampère, mA)**.

* **Octets `[0..1]` (`uint16_t` Big-Endian) — Courant maximal admissible ($I_{\max}$) / Sentinelle Horloge :**
  * **En régime nominal :** Représente le seuil de courant de surcharge en **milliampères (mA)**. Ce seuil est recalculé dynamiquement par le microprogramme interne à partir de la puissance limite configurée par l'utilisateur ($P_{\max}$ en Watts, ex. 1234 W) et de la tension secteur instantanée ($U$ en Volts) :
    $$I_{\max}\text{ (mA)} = \frac{P_{\max}\text{ (W)}}{U\text{ (V)}} \times 1000$$
    *Preuve expérimentale formelle :* Pour $P_{\max} = 1234\text{ W}$ et une tension réseau mesurée de $U = 232\text{ V}$, la valeur brute lue dans `FFF2[0..1]` est très exactement **`0x14C6` = 5318 mA** ($\frac{1234}{232} \times 1000 = 5318,96\text{ mA}$).
  * **Valeur sentinelle `0xFFFF` :** En cas de coupure secteur récente ($\ge 30\text{ s}$), l'horloge interne (RTC) perd son repère. La prise force temporairement `[0..1]` à **`0xFFFF`** (65535) pour signaler son état d'amnésie jusqu'à la prochaine trame `setDateTime()`.

* **Octets `[2..3]` (`uint16_t` Big-Endian) — Courant instantané mesuré ($I_{\text{mesuré}}$) :**
  * Exprimé directement en **milliampères (mA)**.
  * Strictement identique au courant actif de la télémétrie complète `0x0400` (ex: $0x094C = 2380\text{ mA} = 2,380\text{ A}$ sous charge de 496 W).
  * L'octet `[3]` pilote également la couleur de l'anneau LED en façade de la prise.

#### Exploitation par la Passerelle (Zero-Cost BLE Monitoring)
Sans jamais négocier d'authentification ni consommer de bande passante, une lecture passive de 4 octets sur `FFF2` permet :
1. **La surveillance d'amnésie :** Si `Word0 == 0xFFFF`, déclencher immédiatement la resynchronisation NTP.
2. **La détection de dépassement / surcharge :** Si `Word1 > Word0` ($I_{\text{mesuré}} > I_{\max}$), la prise est en dépassement de puissance et va disjoncter.
3. **Une estimation de puissance temps réel :** $P \approx \frac{\text{Word1} \times 230}{1000}\text{ W}$.

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

### 2.5 Comportement du Firmware : Synchronisation, Heure Pile & Coupure Secteur
Les expérimentations menées sur banc de test réel (`experiments/exp_02` à `exp_05`) ont permis d'établir avec certitude la cinématique interne du microprogramme du SEM6000 :

#### 1. Changement d'heure (`setDateTime`)
* L'envoi d'une nouvelle heure via `setDateTime()` ne provoque **aucun recalcul ni décalage immédiat** des slots d'historiques.
* Le firmware ne fait pas de différentiel temporel sur la commande reçue : toute énergie consommée continue de s'additionner dans le slot courant `hourlyWh[23]` sans interruption.

#### 2. Cinématique atomique du franchissement d'Heure Pile (`XX:59:59 -> (XX+1):00:00`)
* Le décalage des historiques est asservi **exclusivement au passage matériel de la frontière `59m 59s -> 00m 00s`** du compteur RTC interne.
* À cet instant précis, de manière atomique :
  1. `hourlyWh[23]` (l'heure fermée) glisse vers la gauche dans `hourlyWh[22]` (`H-1`).
  2. `hourlyWh[23]` (la nouvelle heure qui démarre) est réinitialisé à `0 Wh`.
  3. L'énergie de l'heure qui vient de se fermer est ajoutée au cumul du jour en cours : `dailyWh[29] += hourlyWh[22]`.
  4. Cette même valeur est ajoutée au cumul du mois en cours : `monthlyWh[11] += hourlyWh[22]`.

#### 3. Coupure Secteur, Sentinelle `0xFFFF` & Persistance Flash vs Volatilité RAM
* **Détection sans authentification (`FFF2`) :** Lors d'une coupure secteur ($\ge 30\text{ s}$), la RTC interne perd l'heure. La prise active immédiatement la sentinelle `Word0 == 0xFFFF` sur la caractéristique `FFF2`.
* **Nature du stockage interne :**
  * Les historiques **journaliers (`dailyWh[30]`) et mensuels (`monthlyWh[12]`) sont persistés en mémoire Flash / EEPROM** de la prise avec un horodatage calendaire.
  * L'historique horaire (`hourlyWh[24]`) réside en **RAM volatile** et est remis à zéro lors d'un redémarrage.
* **Verrouillage des données en état amnésique :**
  * Tant que la prise est en état amnésique (`FFF2 == 0xFFFF`), le microprogramme **masque les historiques et renvoie 0 Wh** sur toutes les requêtes pour éviter de servir des index désynchronisés.
  * **Perte de comptage sous amnésie :** Toute énergie consommée tant que la prise est en état `0xFFFF` est **définitivement perdue** (le firmware suspend l'intégration en Flash tant que l'horloge n'est pas calée).
* **Démasquage automatique après `setDateTime()` :**
  * Dès réception d'une trame `setDateTime()` valide, `Word0` quitte la valeur `0xFFFF` pour réafficher sa valeur nominale de seuil (ex. `0x14C6` = 5318 W).
  * Le firmware recale ses pointeurs calendaires et **l'intégralité des historiques journaliers et mensuels réapparaît immédiatement et intacte**.

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
1. **Détection de démarrage à froid & Remise à l'heure prioritaire :**
   * Lecture de `FFF2` sans authentification. Si octets `[0..1] == 0xFFFF`, déclenchement immédiat dès la première seconde d'une commande `setDateTime()` avec l'heure NTP afin de minimiser la fenêtre pendant laquelle la prise ne compte pas l'énergie et de démasquer les historiques Flash.
2. **Reconstruction de l'Index Perpétuel 32 bits (`uint32_t indexWh`) :**
   * En régime nominal : calcul du différentiel continu sur `hourlyWh[23]` et prise en compte du basculement atomique vers `hourlyWh[22]` lors du franchissement de l'heure pile (`HH:00:00`).
   * En reprise après coupure secteur : le buffer horaire de 24h étant réinitialisé, la passerelle s'appuie sur la valeur réapparue du jour courant `dailyWh[29]` et sur son dernier index persisté en mémoire non volatile (**NVS / Preferences**) pour réconcilier le cumul absolu sans aucune perte de Watt-heure.
   * Sauvegarde périodique en NVS de `indexWh`, du timestamp de commit et de la dernière valeur journalière connue.

### Couche 3 : `SEM6000Manager`
* Gère un parc de plusieurs prises SEM6000 en séquentialisant les accès radio BLE (évite les collisions NimBLE), distribue le temps NTP et pousse les mises à jour vers Domoticz.

