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

### 2.5 Comportement du Firmware : Synchronisation, Heure Pile, Surcharge & Coupure Secteur
Les expérimentations menées sur banc de test réel (`experiments/exp_02` à `exp_07`) ont permis d'établir avec une certitude absolue la cinématique interne du microprogramme du SEM6000 :

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
  * **Perte de comptage sous amnésie :** Toute énergie consommée tant que la prise est en état `0xFFFF` est **définitivement perdue** (le firmware suspend l'intégration en Flash tant que l'horloge n'est pas calée). D'où l'impératif absolu pour la passerelle de réinjecter le temps NTP dès la première seconde de détection.
* **Démasquage automatique après `setDateTime()` :**
  * Dès réception d'une trame `setDateTime()` valide, `Word0` quitte la valeur `0xFFFF` pour réafficher sa valeur nominale de seuil (ex. `0x14C6` = 5318 mA).
  * Le firmware recale ses pointeurs calendaires et **l'intégralité des historiques journaliers et mensuels réapparaît immédiatement et intacte**.

#### 4. Disjonction par Surcharge vs Coupure Secteur (`exp_06`)
* **Comportement de `Word0` :** En cas de disjonction par dépassement de seuil ($I_{\text{mesuré}} > I_{\max}$), **`Word0` ne passe PAS à `0xFFFF`** mais reste figé à sa valeur nominale $I_{\max}$ (ex. `5318 mA`). L'alimentation 3.3V interne du microcontrôleur reste active : la RTC ne perd jamais l'heure.
* **Temporisation thermique interne :** Le microprogramme applique une temporisation de sécurité d'environ **1,5 seconde** avant d'ouvrir le relais, afin d'absorber les courants d'appel transitoires des moteurs.
* **Absence de flag persistant de surcharge :** La prise ne mémorise aucun code d'erreur dédié pour indiquer qu'elle a disjoncté. L'état `Relais == OFF` ne prouve donc pas une surcharge (l'utilisateur ou un planning a pu couper le relais manuellement).

#### 5. Trigger de Rotation Journalière : Le Verdict de Minuit (`exp_07`)
* **Démonstration empirique formelle :** Le basculement du buffer journalier `dailyWh[30]` n'est **pas** conditionné par un compteur modulo 24 heures, mais **strictement par le front montant calendaire de Minuit (`23:59:59 -> 00:00:00`)**.
* À minuit pile, de manière atomique :
  1. `dailyWh[29]` (énergie du jour J) est scellé et bascule dans `dailyWh[28]` (`J-1`).
  2. `dailyWh[29]` est réinitialisé à `0 Wh` pour entamer la nouvelle journée.
  3. L'historique des 30 jours glisse d'un cran vers la gauche (`[28] -> [27]`, etc.).

#### 6. Recul d'Horloge & Phénomène de Double Rotation (`exp_07`)
* **Démonstration de la cascade d'interruption :** Chaque franchissement de `HH:00:00` déclenche inconditionnellement la rotation du buffer horaire `hourlyWh[24]`.
* Si une commande `setDateTime()` recule l'horloge en arrière d'une heure pile déjà franchie (ex. passage de `02:01:30` à `01:58:30`) :
  * Le créneau horaire est rejoué.
  * Au second passage de `02:00:00`, la prise déclenche une **seconde rotation prématurée**.
  * Le slot fermé lors du premier passage (`hourlyWh[22]`) est violemment poussé dans `hourlyWh[21]`, créant un **slot fantôme artificiel** et faussant l'ensemble de l'historique horaire.

---

### 2.6 Stratégie Maîtresse de Synchronisation d'Horloge (Plus précise que le constructeur)

L'application mobile officielle du constructeur présente des lacunes de conception sévères :
1. Elle injecte brutalement l'heure locale du smartphone à chaque connexion, sans se soucier des frontières d'heures.
2. Lors du passage à l'heure d'hiver en octobre (`03:00 -> 02:00`), elle recule l'horloge de 1 heure, provoquant la corruption de l'historique horaire par double rotation.
3. Le protocole ne fournit **aucune commande de lecture d'horloge (`getDateTime`)** : la commande `0x0100` est un setter aveugle. De plus, à consommation nulle (relais OFF), 0 Wh glissant vers 0 Wh rend toute dérive invisible à l'observation passive.

Pour surpasser le constructeur et garantir une précision temporelle absolue sans aucun artefact d'historique, la passerelle applique les **3 Règles d'Or** suivantes :

#### Règle 1 : Temps Monotone Invariant (UTC ou UTC+1 Fixe)
* La prise est synchronisée exclusivement dans un référentiel horaire **invariant sans changement d'heure saisonnier (DST)** (ex. UTC+1 permanent, calé sur l'heure normale d'hiver française).
* Chaque journée compte strictement 24 heures et aucune heure n'est jamais rejouée ni sautée.

#### Règle 2 : Synchronisation en « Havre de Paix » (`HH:30:00`)
* Un quartz de montre standard (32,768 kHz) dérive naturellement de **2 à 4 secondes par 24 heures** (soit au pire ~1 minute par mois).
* La passerelle n'envoie de trame périodique de recalage NTP **qu'au milieu exact d'une heure**, idéalement à **`HH:30:00`**.
* À la 30ᵉ minute, la correction de quelques secondes s'opère à **30 minutes pleines de toute heure pile**. Il est ainsi **mathématiquement impossible** de déclencher une double rotation ou de sauter une heure pile.

#### Règle 3 : Remise à l'Heure Immédiate sur Détection Amnésique
* Si `FFF2 == 0xFFFF` (retour secteur après coupure), l'horloge est amnésique (an 2000) et le comptage est bloqué.
* Dans ce cas unique, la passerelle injecte immédiatement l'heure NTP à la première seconde de connexion sans attendre `HH:30`, afin de démasquer la Flash et de relancer le comptage au plus vite.

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

### Couche 2 : `ManagedSEM6000`
Encapsule une instance `SEM6000Device` pour garantir la continuité des données et l'intégrité de la base Domoticz :

1. **Détection d'amnésie & Remise à l'heure prioritaire :**
   * Lecture de `FFF2` sans authentification. Si octets `[0..1] == 0xFFFF`, déclenchement immédiat dès la première seconde d'une commande `setDateTime()` avec l'heure NTP afin de minimiser la fenêtre pendant laquelle la prise ne compte pas l'énergie et de démasquer les historiques Flash.

2. **Reconstruction de l'Index Perpétuel 32 bits (`uint32_t indexWh`) & Monotonie Stricte :**
   * **Règle absolue d'émission Domoticz :**
     $$\text{Index}_{\text{transmis}} = \max(\text{Index}_{\text{calculé}}, \text{Index}_{\text{Domoticz}})$$
     L'ESP32 ne transmet **jamais** un index inférieur au précédent afin d'éviter les débordements 32 bits dévastateurs dans la base SQLite de Domoticz.
   * **Initialisation par interrogation Domoticz :** Au démarrage de la passerelle (ou lors du remplacement à neuf d'un ESP32 sans NVS), interrogation de l'API HTTP/JSON (`/json.htm?type=command&param=getdevices&rid=IDX`) pour caler la ligne de base sur l'index officiel de Domoticz.

3. **Traitement Asynchrone du Franchissement d'Heure Pile :**
   * L'ESP32 n'a pas besoin d'être connecté à `HH:00:00` pile.
   * La condition `hourlyWh[23] < m_lastHourly23` suffit à certifier de manière asynchrone que l'heure a basculé côté prise : la fin de l'heure précédente est scellée dans `hourlyWh[22]` et le début de l'heure en cours dans `hourlyWh[23]`, sans aucun trou de comptage.
   * Si `hourlyWh[23] == m_lastHourly23 == 0` (charge nulle ou relais coupé), l'index reste parfaitement stable.

4. **Résilience et Rattrapage Multi-Échelles Post-Panne :**
   Au redémarrage après une absence prolongée (coupure Wi-Fi, crash ESP32, vacances), calcul de $\Delta T = T_{\text{NTP}} - T_{\text{NVS}}$ :
   * **Si $\Delta T \le 23\text{ h}$ :** Rattrapage intégral à la granularité horaire via `hourlyWh[24]`.
   * **Si $24\text{ h} < \Delta T \le 29\text{ jours}$ :** Rattrapage via la somme des cases non comptabilisées de `dailyWh[30]`.
   * **Si $\Delta T > 30\text{ jours}$ :** Rattrapage via l'historique mensuel `monthlyWh[12]`.

5. **Politique d'usure Flash NVS :**
   * Sauvegarde en NVS (`Preferences`) déclenchée uniquement si $\Delta \ge 10\text{ Wh}$ ou si plus de 30 minutes se sont écoulées, garantissant une longévité de plus de 50 ans de la Flash de l'ESP32.

### Couche 3 : `SEM6000Manager`
Ordonnance le parc de prises et assure la passerelle vers Domoticz :
* **Contrainte Mono-Connexion BLE :** La prise n'acceptant qu'un seul client BLE à la fois, les connexions sont strictement séquentialisées (une prise après l'autre, pas de concurrence radio).
* **Flotte de Passerelles Multi-Niveaux (Topologie) :**
  * Dans un logement sur plusieurs niveaux (ex. 10 prises réparties entre cave, garage, arrière-cuisine derrière murs béton, salon et étages), plusieurs ESP32 exécutant le même firmware générique sont déployés par zone.
  * **Attribution déclarative stricte (Master Controller) :** Chaque ESP32 détient sa propre table de prises assignées. Une prise n'est interrogée que par son ESP32 Maître exclusif pour éviter toute collision ou déconnexion sauvage.
* **Synchronisation Réseau :** Distribution du temps NTP et publication HTTP/JSON ou MQTT vers Domoticz.

