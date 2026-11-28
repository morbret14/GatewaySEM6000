# Architecture & Spécifications du Protocole — Passerelle Domoticz SEM6000

Ce document consigne l'état de l'art des connaissances acquises par rétro-ingénierie sur les prises connectées **Voltcraft SEM6000-FR** — couvrant les **deux générations matérielles coexistant dans le parc (`Hardware v2.0 / Firmware v1.24` et `Hardware v3.0 / Firmware v1.30`)** — et définit l'architecture logicielle de la passerelle **ESP32-S3** sous **PlatformIO / Framework Arduino**.

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
* **Invariance des octets 13 et 14 (Zone 1) :** Restent invariablement à `[0x00 0x00]` en toutes circonstances (à vide, sous forte charge, à l'approche de la coupure, en dépassement et après disjonction matérielle). Ils ne fournissent aucun préavis ni statut de surcharge (cf. Section 2.5 §10).
* **Absence d'index perpétuel natif :** Sur Hardware v2.0, les octets 15 à 18 (`totalKwhWh`) restent figés à `[0x00 0x00 0x00 0x00]`. La prise ne maintient aucun index global survivant aux cycles glissants ou coupures secteur.

### 2.4 Registres d'Historiques (`0x0A00`, `0x0B00`, `0x0C00`)
Validés par croisement entre captures BLE et exports CSV de l'application constructeur. **Les trois tableaux suivent strictement la même logique d'indexation chronologique :** le dernier index représente toujours la période en cours, et décrémenter l'index de N permet de remonter de N périodes dans le passé.

| Registre | Méthode `SEM6000Device` | Type & Taille | Période en cours (Présent) | Période précédente (N-1) | Cinématique de mise à jour |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Horaire (`0x0A00`)** | `requestDayHistory` | `uint16_t[24]` (2 octets/slot) | **Slot `[23]`** | Slot `[22]` (H-1) | S'incrémente en continu (Wh). À chaque heure pile (`HH:00:00`), décale vers la gauche et repart à `0`. |
| **Journalier (`0x0B00`)** | `requestMonthHistory` | `uint32_t[30]` (4 octets/slot) | **Slot `[29]`** | Slot `[28]` (Hier) | Mis à jour **à chaque heure pile (`HH:00:00`)** par addition de l'heure qui vient de se fermer. |
| **Mensuel (`0x0C00`)** | `requestYearHistory` | `uint32_t[12]` (4 octets/slot) | **Slot `[11]`** | Slot `[10]` (M-1) | Mis à jour **à chaque heure pile (`HH:00:00`)** par addition de l'heure qui vient de se fermer. |

#### Structure Quadri-Octets des Historiques Journaliers et Mensuels (`0x0B00` & `0x0C00`)
Chaque tranche historique (jour ou mois) est encodée sur **4 octets bruts `[B0, B1, B2, B3]`** :
* **Octets `[B0, B1, B2]` (24 bits Big-Endian) — Énergie Totale (Wh) :**
  $$E_{\text{total}} = (B_0 \ll 16) \mid (B_1 \ll 8) \mid B_2$$
  C'est cette valeur que la méthode `SEM6000Device::requestMonthHistory()` et `requestYearHistory()` extrait directement.
* **Octet `B3` (8 bits non signé) — Ratio d'Heures Creuses (0 à 100 %) :**
  * `B3` représente formellement le **pourcentage d'énergie consommé durant le créneau Heures Creuses (HC)** de la période considérée ($0 \le B_3 \le 100$).
  * Démontré empiriquement sur banc de test (`exp_08` & `exp_09`) à travers 42 points de mesures sur plusieurs prises (charge purement nocturne de véhicule électrique mesurée à 98-99%, consommations diurnes mixtes à 54-68%, charges exclusivement en heures pleines à 0%).

#### Algorithme de Calcul des Coûts de l'Application Mobile
La prise SEM6000 ne stocke **aucun centime d'euro en mémoire**. C'est l'application smartphone qui calcule dynamiquement à l'affichage les coûts de chaque jour et mois sur la base des tarifs configurés :
$$E_{\text{HC}} = E_{\text{total}} \times \frac{B_3}{100} \quad ; \quad E_{\text{HP}} = E_{\text{total}} \times \left(1 - \frac{B_3}{100}\right)$$
$$\text{Coût (€)} = \frac{E_{\text{HC}} \times P_{\text{HC}} + E_{\text{HP}} \times P_{\text{HP}}}{1000 \times 100}$$
*(où $P_{\text{HC}}$ et $P_{\text{HP}}$ sont les tarifs en centimes d'euro par kWh).*

Ce mécanisme explique pourquoi la modification des tarifs dans l'application mobile recalcule instantanément l'intégralité des coûts affichés sur 30 jours et 12 mois sans aucune écriture en mémoire sur la prise.

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

#### 7. Intégration Heures Creuses (HC) & Octet Ratio B3 (`exp_08` & `exp_09`)
* **Décompte des HC :** Lorsque le mode réduit est configuré (`setReducedPeriod`), le firmware comptabilise en interne la quote-part d'énergie consommée pendant ce créneau.
* **Scellement du Ratio :** À chaque franchissement d'heure pile et lors de la fermeture de la journée à minuit, le firmware calcule le ratio $B_3 = \text{round}\left(100 \times \frac{E_{\text{HC}}}{E_{\text{total}}}\right)$ et le stocke dans le 4ᵉ octet du slot historique journalier (`0x0B00`) et mensuel (`0x0C00`).
* **Conservation à l'échelle :** Ce ratio est strictement borné entre 0 et 100 %, sans aucune corruption possible de l'énergie totale qui reste isolée sur les 3 premiers octets `[B0, B1, B2]`.

#### 8. Encodage du Créneau Heures Creuses, Borne d'Exclusion & Bug UI Constructeur (`exp_10`)
* **Format mémoire interne de la prise :** Les heures de début et de fin du créneau Heures Creuses (`setReducedPeriod` / `0x0F0001` et `requestSettings` / `0x1000`) sont stockées sous la forme d'un **nombre entier de minutes écoulées depuis 00h00 en 16 bits Big-Endian** :
  $$T = (\text{Octet}_{\text{MSB}} \ll 8) \mid \text{Octet}_{\text{LSB}}$$
  Exemples :
  * $22\text{h}00 = 1320\text{ min} = \text{0x0528}$ (`[0x05, 0x28]`)
  * $06\text{h}00 = 360\text{ min} = \text{0x0168}$ (`[0x01, 0x68]`)
* **Cinématique de comptage (Intervalle semi-ouvert $[T_{\text{début}}, T_{\text{fin}}[$) :**
  Le firmware de la prise considère $T_{\text{fin}}$ comme une **borne d'exclusion stricte**. Tant que l'horloge interne $t < T_{\text{fin}}$, l'énergie est imputée au sous-total Heures Creuses. Dès la seconde exacte $t = T_{\text{fin}}$ (ex: $06\text{h}00\text{m}00\text{s}$), l'imputation bascule immédiatement en Heures Pleines.
* **Règle d'affichage de l'application mobile & Décalage d'une minute :**
  Pour afficher un intervalle fermé convivial à l'utilisateur, l'application smartphone soustrait une minute à la borne de fin stockée :
  $$\text{Fin}_{\text{affichée}} = T_{\text{fin}} - 1\text{ min}$$
  * Pour obtenir l'affichage contractuel français **`22:00 -> 05:59`**, la prise doit stocker la borne d'exclusion **`06h00`** (`360 min` = `0x0168`).
  * Si l'on injecte directement `05h59` (`359 min` = `0x0167`) comme borne de fin, l'application soustrait une tranche et affiche `22:00 -> 04:59`.
* **Élucidation du Bug de blocage de l'application mobile :**
  * Dans l'application, configurer une fin de créneau égale au début (ou bouclant sur 24h) enregistre en Flash $T_{\text{début}} = T_{\text{fin}} = 22\text{h}00$ (`[0x05, 0x28]`), ce qui affichait `22:00 -> 21:59`.
  * La prise accepte cette trame sans erreur, mais le composant graphique de l'application (slider circulaire / validation d'amplitude $\Delta > 0$) entre en conflit d'assertion : l'amplitude horaire étant nulle ($1320 - 1320 = 0$), l'UI refuse de sauvegarder ou de déplacer les curseurs.
  * **Solution :** Une écriture BLE directe via `setReducedPeriod(1, 0x05, 0x28, 0x01, 0x68)` (Début 22h00, Fin 06h00) restaure une amplitude saine en Flash et débloque instantanément l'interface graphique de l'application mobile.
* **Notification de réponse aux commandes 3 octets (`CMD_SET_3B`) :**
  Pour les commandes `0x0F00` comportant une sous-commande (`0x01` période HC, `0x04` prix, `0x05` LED), la notification de réponse retournée sur `0xFFF4` est `0F 05 0F 00 [SubCmd] [Status] [Checksum] FF FF`. L'octet de statut d'exécution (`0x00` = Succès) réside à l'index 5 (`m_rxBuffer[5]`), l'index 4 contenant l'écho de la sous-commande.

#### 9. Cinématique du Timer, Codage 24 bits en Secondes de `presetSeconds` & Sentinelle RAM Post-Coupure (`0x0900`, `exp_11`, `exp_20`, `exp_21`, `exp_22`)
* **Codage universel des octets `[B11..B13]` (Entier 24 bits Big-Endian en SECONDES) :**
  Contrairement à une hypothèse initiale issue d'un test sur 60 secondes (`[0x00, 0x00, 0x3C]`), les octets 11 à 13 (`m_rxBuffer[11..13]`) de la trame `0x0900` (`getTimerStatus`) ne sont **pas** codés en `[Heures, Minutes, Secondes]`, mais forment sur **`HW 2.0` comme sur `HW 3.0`** un **entier non signé 24 bits Big-Endian exprimé directement en SECONDES** ($0 \le \Delta t < 2^{24} = 16\,777\,216\text{ s} \approx 194,18\text{ jours}$) :
  $$\texttt{presetSeconds} = (\texttt{m\_rxBuffer[11]} \ll 16) \mid (\texttt{m\_rxBuffer[12]} \ll 8) \mid \texttt{m\_rxBuffer[13]}$$
  * *Preuves expérimentales :* `0x00003C` = $60\text{ s}$ (`exp_11`), `0x000E0E` = $3\,598\text{ s}$ (`exp_15`), `0x007E90` = $32\,400\text{ s} = 9\text{h}00\text{m}00\text{s}$ et `0x008CA0` = $36\,000\text{ s} = 10\text{h}00\text{m}00\text{s}$ (`exp_20`), `0x02A2FF` = $172\,799\text{ s} = 48\text{h} - 1\text{s}$ (`exp_22`).
* **Différence de comportement `HW 2.0` vs `HW 3.0` sur `[B11..B13]` :**
  * **Sur `Hardware 2.0 / FW 1.24` :** Valeur **statique** égale à la durée initiale $\Delta t = T_{\text{cible}} - T_{\text{interne}}$ calculée au moment de l'envoi de `setTimerAbs` (`0x0800`).
  * **Sur `Hardware 3.0 / FW 1.30` :** Tant que le timer est actif (`m_rxBuffer[4] != 0x00`), la prise retourne l'écart $\Delta t = (T_{\text{cible}} - T_{\text{interne}}) \pmod{2^{24}}$ à la seconde près. Dès que le timer est désarmé (`stopTimer()`, `m_rxBuffer[4] == 0x00`), `[B11..B13]` retombe à `0x000000` tandis que les octets `[B5..B10]` conservent en RAM l'écho de la dernière date cible programmée (`sec, min, hour, day, month, year`).
* **Bascule à l'échéance et fin de vie du Timer :**
  * Dès que $T_{\text{courant}} = T_{\text{cible}}$, le relais matériel commute immédiatement vers l'action programmée (ex: `ON`), et l'octet d'action `m_rxBuffer[4]` repasse à **`0x00` (INACTIF)**.
  * **Volatilité en RAM :** Contrairement aux plannings (`0x1400`) et au mode aléatoire (`0x1600`) stockés en Flash, un Timer unitaire actif (`0x0800`) réside uniquement en **RAM volatile** et est effacé lors d'une coupure secteur (`exp_22`).
  * **Signature RAM de coupure secteur sur `HW 3.0` (`00 00 00 00 00 30`) :** Au démarrage à froid après une coupure secteur, avant tout armement de timer, les octets `[B5..B10]` de `0x0900` valent invariablement **`00 00 00 00 00 30`** (`targetDay == 0x00`, `targetMonth == 0x00`, `targetYear == 0x30`), offrant un détecteur de coupure secteur en 1 seule requête `GET` sur `HW 3.0` (cf. Section 2.8 §6).

#### 10. Invariance des Octets 13-14 de Télémétrie (`0x0400`) & Surveillance Tactique de Surcharge par `FFF2` (`exp_12`)
* **Protocole expérimental formel (`exp_12`) :**
  Conduit sur la prise *Théoule* (`94:a9:a8:1a:37:0c`, HW v2.0, FW v1.0.8) sous charge résistive réelle de ~895 W ($I \approx 4,32\text{ A}$, $U \approx 227\text{ V}$). La puissance maximale admissible $P_{\max}$ (`setOverloadPower`) a été abaissée de manière dichotomique de 2000 W à 913 W jusqu'à franchissement du seuil limite de courant ($4326\text{ mA} > 4022\text{ mA}$) provoquant la disjonction matérielle.
* **Invariance stricte des Octets 13 et 14 de `0x0400` :**
  Tout au long du protocole, les octets 13 et 14 (`m_rxBuffer[13..14]`) sont demeurés strictement figés à **`[0x00 0x00]`** :
  * À vide ($P = 0\text{ W}$, relais ON ou OFF),
  * En charge intermédiaire ($P \approx 490\text{ W}$, marge $> 1500\text{ W}$),
  * En charge nominale ($P \approx 895\text{ W}$),
  * En approche critique immédiate du seuil de coupure (marge $\Delta P = 18\text{ W}$),
  * À l'instant du dépassement de seuil ($I_{\text{mesuré}} > I_{\max}$),
  * Après disjonction effective et ouverture du relais (`0x02` = OFF).
  * **Verdict certain :** Sur Hardware v2.0, les octets 13 et 14 ne constituent ni un préavis d'alarme, ni une jauge dynamique de proximité de coupure, ni un flag d'erreur de disjonction.
* **Invariance des Octets 15 à 18 (`totalKwhWh`) :**
  Les octets 15 à 18 sont restés figés à `[0x00 0x00 0x00 0x00]`, confirmant à 100% l'absence de compteur cumulatif dans la trame `0x0400` sur Hardware v2.0 et validant le rôle fondamental de la Couche 2 (`ManagedSEM6000`) pour reconstruire un index perpétuel fiable.
* **Cinématique de disjonction matérielle :**
  * Dès que $I_{\text{mesuré}} > I_{\max}$ (dans l'expérimentation : $4326\text{ mA} > 4022\text{ mA}$), la temporisation thermique interne d'environ **1,5 à 2 secondes** s'écoule avant ouverture mécanique du relais.
  * L'état du relais bascule de `0x01` (ON) à `0x02` (OFF).
  * `Word0` de `FFF2` conserve sa valeur limite ($4022\text{ mA}$) sans passer à `0xFFFF` (la logique 3.3V reste alimentée).
* **Exploitation tactique de `FFF2` pour la surveillance « à la culotte » :**
  * Alors que la télémétrie `0x0400` exige une session GATT authentifiée complète (lourde en bande passante et en temps de cycle), la caractéristique `FFF2` expose en **4 octets bruts sans authentification** :
    * `Word0` : Seuil de coupure calculé $I_{\max}$ (mA).
    * `Word1` : Courant actif instantané mesuré $I_{\text{mesuré}}$ (mA).
  * **Surveillance continue à coût zéro :** La passerelle peut scruter `FFF2` à haute fréquence sans verrouiller le protocole.
  * **Prévention et délestage préventif :** La comparaison temps réel $I_{\text{mesuré}} \ge I_{\max}$ (ou un seuil d'alerte, ex: $90\%$ de $I_{\max}$) permet à la passerelle de remonter une alerte proactive à Domoticz ou d'ordonner le délestage d'une charge secondaire sur une autre prise du parc **avant** l'ouverture irréversible du relais par la prise.

#### 11. Périmètres d'Action des Commandes de Réinitialisation : Reset Conso vs Factory Reset (`exp_13`)
L'expérimentation différentielle exhaustive menée sur banc réel (`exp_13`) a permis de cartographier avec une certitude absolue les effets des deux commandes de remise à zéro du protocole :

* **Reset des Données de Consommation (`resetConsumptionData` / `0x0F0002`) — Purge Sélective :**
  * **Périmètre d'effacement :** Réinitialise exclusivement et intégralement à zéro les trois registres d'historiques d'énergie :
    * `hourlyWh[24]` $\rightarrow$ $0\text{ Wh}$ sur les 24 slots.
    * `dailyWh[30]` $\rightarrow$ $0\text{ Wh}$ sur les 30 jours (cumuls et ratios HC remis à zéro).
    * `monthlyWh[12]` $\rightarrow$ $0\text{ Wh}$ sur les 12 mois.
  * **Préservation garantie :** Ne modifie absolument rien d'autre. Le nom GATT, le mot de passe PIN, le seuil de coupure $P_{\max}$, les tarifs HP/HC, les créneaux horaires HC, les programmations (Schedulers), l'anneau LED et le mode aléatoire restent **100 % intacts**.
  * **Cas d'usage :** Idéal pour remettre à zéro le sous-comptage d'un appareil sans devoir réappairer ni reconfigurer la prise.

* **Reset d'Usine Global (`factoryReset` / `0x0F0000`) — Restauration Complète :**
  * **Restauration de la configuration d'usine par défaut :**
    * **Nom GATT :** Rétabli au nom usine constructeur (`'Voltcraft'`).
    * **Relais :** Bascule immédiatement à **`ON`** (fermé par défaut à la livraison).
    * **Seuil de Surcharge :** Rétabli au standard physique maximal de **`3680 W`** (16 A sous 230 V).
    * **Tarifs :** Tarif Normal $\rightarrow$ **`200 centimes/kWh`** ($2,00\text{ €/kWh}$), Tarif Réduit $\rightarrow$ **`100 centimes/kWh`** ($1,00\text{ €/kWh}$).
    * **Créneau HC :** Désactivé (`NON`, $00\text{h}00 \rightarrow 00\text{h}00$).
    * **Anneau LED :** Réactivé (`0x01`).
    * **Schedulers :** Table entièrement purgée (**`0 programmation`**).
    * **Historiques :** Tous effacés à $0\text{ Wh}$.
    * **Code PIN :** Réinitialisé au code d'usine `0000` (`(uint32_t)0x00000000`).

#### 12. Architecture Bi-Génération des Plannings (`0x1300` / `0x1400`) & du Mode Aléatoire (`0x1500` / `0x1600`) (`exp_23`)
Le stress-test comparatif des plannings (création de 13 schedulers $\rightarrow$ 12 acceptés et 13ᵉ rejeté, modification en place de 2 slots, lecture multi-pages et purge complète) et du mode aléatoire sur *Théoule* (`HW 2.0 / FW 1.24`) et *Vélo* (`HW 3.0 / FW 1.30`) a établi les règles exactes suivantes :

* **Capacité maximale & Rejet du 13ᵉ planning (`0x1300`) :**
  * Les deux générations acceptent un maximum strict de **12 plannings** simultanés en mémoire Flash.
  * Toute tentative de créer un 13ᵉ planning est rejetée par la prise avec un acquittement négatif (`RX[4] = 0x01` $\rightarrow$ `SEM_ERR_RESPONSE_REJECTED`) sans altérer les 12 plannings existants.
* **Allocation des `SlotId` et règle universelle de création (`slotId = 0x00`) :**
  * **Sur `Hardware 2.0 / FW 1.24` :** Les `SlotId` sont numérotés de **`1` à `12` (`0x01..0x0C`)** et alloués en **ordre décroissant (LIFO : `12`, puis `11`, ..., jusqu'à `1`)**. Lors d'un appel `0x1300`, si l'octet `slotId` (`frame[5]`) correspond à un `SlotId` déjà existant dans la table, le `HW 2.0` **modifie ce slot en place** au lieu d'en créer un nouveau. Pour créer un nouveau slot sur `HW 2.0`, il est donc impératif de transmettre **`slotId = 0x00`** (avec `mode = 0x00`).
  * **Sur `Hardware 3.0 / FW 1.30` :** Les `SlotId` sont numérotés de **`0` à `11` (`0x00..0x0B`)** et alloués en **ordre croissant (FIFO : `0`, puis `1`, ..., jusqu'à `11`)**. L'octet `mode` (`frame[4]`) gouverne strictement l'opération : `0x00` crée un nouveau slot (quel que soit `slotId`), `0x01` modifie en place le `slotId` indiqué, et `0x02` supprime le `slotId` indiqué.
  * **Convention universelle bi-génération (`SEM6000Device::setScheduler`) :** Toujours passer **`mode = 0x00` (`SCHED_ONCE`) et `slotId = 0x00`** pour créer un planning (unitaire si `dayMask == 0x00`, hebdomadaire si `dayMask != 0x00`), et **`mode = 0x01` (`SCHED_REPEAT_WEEKLY`) avec le `slotId` réel** lu via `requestAllSchedulers()` pour modifier un planning existant.
* **Différence de représentation des plannings hebdomadaires et de la clef de tri (`SortKey`) dans `0x1400` :**
  * **Sur `Hardware 2.0` :** Conserve la date initiale (`year, month, day`) même lorsque `dayMask != 0x00`, et calcule un octet `SortKey` non nul (`block[11]`, recalculé à chaque modification).
  * **Sur `Hardware 3.0` :** Dès que `dayMask != 0x00` (planning hebdomadaire), le firmware force les octets de date à `00/00/00` (`2000/00/00`) et maintient `SortKey = 0x00` sur tous les slots.
* **Pagination 3 pages $\times$ 56 octets (`HW 2.0`) vs Trame monolithique 152 octets (`HW 3.0`) sur `0x1400` :**
  * L'octet `m_rxBuffer[4]` de `0x1400` indique toujours le **nombre total de plannings enregistrés dans la prise** (`0` à `12`), et non le nombre de slots dans la page courante.
  * **Sur `Hardware 2.0` (`MTU = 23`) :** La lecture est paginée par **4 slots maximum par page** (`56 octets` par page pleine : `0F 34 14 00 [Total] [4 x 12 o] [CS] FF FF`, assemblés en 3 fragments BLE) sur les pages `0`, `1` et `2`.
  * **Sur `Hardware 3.0` (`MTU = 160`) :** **Aucune pagination !** Dès la requête de la `Page 0`, la prise retourne **l'intégralité des plannings enregistrés (jusqu'à 12 slots) en 1 seule trame monolithique de 152 octets** (`0F 94 14 00 0C [12 x 12 o = 144 o] [CS] FF FF`, avec **`CS_Valid = OUI`** !). `requestAllSchedulers()` s'arrête donc dès que `list.size() >= totalSchedulers`.
* **Rafale de double notification `0x1300` + `0x1400` sur `HW 3.0` :**
  * Sur `HW 2.0`, une écriture `0x1300` renvoie un acquittement de **10 octets** (`0F 06 13 00 [Status] 00 00 [CS] FF FF`).
  * Sur `HW 3.0`, une écriture `0x1300` renvoie un acquittement de **8 octets** (`0F 04 13 00 [Status] [CS] FF FF`) suivi **~1 ms plus tard d'une notification spontanée `0x1400`** contenant la table complète des plannings mise à jour. Le verrouillage de `m_rxBuffer` par `m_expectedCmd` et `m_rxReady` dans `SEM6000Device::handleNotification()` garantit que cette notification spontanée `0x1400` n'écrase jamais l'acquittement `0x1300`.
* **Mode Aléatoire (`0x1500` / `0x1600`) :**
  * Format de requête et de réponse (15 octets, `CS_Valid = OUI`) **100 % identique** entre `HW 2.0` et `HW 3.0`.
  * Valeur d'usine au repos (`active = 0x00`) : `00:00 -> 00:00 (mask = 0x00)` sur `HW 2.0`, contre `20:00 -> 08:00 (mask = 0xFF)` préconfiguré par défaut sur `HW 3.0`.

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

#### Règle 4 : Sonde Temporelle Furtive — Lecture Indirecte d'Horloge via le Registre Timer (Couche 2)
Bien que le microprogramme du SEM6000 ne dispose d'aucune commande native de lecture d'horloge (`getDateTime`), la cinématique du Timer (`0x0800` / `0x0900`, cf. Section 2.5 §9) fournit un mécanisme détourné pour **mesurer l'horloge interne matérielle à la seconde près** sans modifier l'heure de la prise :
* **Principe arithmétique :**
  Lors d'un armement de timer (`setTimerAbs`), la prise calcule la différence en secondes entre la date cible absolue $T_{\text{cible}}$ fournie et son horloge interne instantanée $T_{\text{interne}}$ :
  $$\Delta t = T_{\text{cible}} - T_{\text{interne}}$$
  Cette durée $\Delta t$, encodée sous forme d'**entier 24 bits Big-Endian en secondes** dans `[B11..B13]` (`(B11 << 16) | (B12 << 8) | B13`), est immédiatement lisible via `getTimerStatus()` (`0x0900`). On en déduit directement la date et l'heure de la RTC interne à la seconde près (`deltaDays = deltaSec24 / 86400`, `secOfDay = secCible - (deltaSec24 % 86400)`) :
  $$T_{\text{interne}} = T_{\text{cible}} - \Delta t$$
* **Cinématique de la sonde furtive (< 150 ms) :**
  1. **Politesse d'accès :** Interrogation préalable de `getTimerStatus()`. Si `action != 0x00`, un timer physique utilisateur est en cours (ex. programmé via l'app mobile) $\rightarrow$ la passerelle n'y touche pas.
  2. **Armement sécurisé :** Si `action == 0x00` (inactif), armement d'un timer éphémère vers une cible lointaine ($T_{\text{NTP}} + 24\text{ h}$), en conservant l'action identique à l'état actuel du relais (`ON` si allumé, `OFF` si éteint) pour garantir un risque matériel strictement nul.
  3. **Capture & Déduction :** Lecture immédiate de `0x0900`, extraction de $\Delta t$ et calcul de l'écart de dérive par rapport à l'heure NTP référence :
     $$\text{Dérive} = T_{\text{NTP}} - T_{\text{interne}}$$
  4. **Désarmement immédiat :** Émission immédiate de `stopTimer()` pour libérer le registre.
* **Gouvernance d'architecture :**
  Cette mécanique d'auscultation requiert l'enchaînement de plusieurs commandes BLE et une comparaison avec le temps réseau NTP. Elle est **exclusivement implémentée dans la Couche 2 (`ManagedSEM6000`)**, préservant ainsi la Couche 1 (`SEM6000Device`) comme un pilote GATT bas niveau pur et autonome.

---

### 2.7 Taxonomie des Commandes SET/GET & Cartographie des Octets de Statut dans les Notifications

Lors de l'émission d'une commande de configuration ou d'action (`SET`) sur la caractéristique `0xFFF3`, la prise répond par une notification asynchrone sur la caractéristique `0xFFF4`.
Dans le microprogramme Voltcraft SEM6000, l'ordonnancement interne des octets au sein de ces notifications présente des **asymétries historiques selon les familles de commandes** qu'il est capital de formaliser.

#### Convention d'indexation
Toutes les positions ci-dessous sont exprimées en **index 0 (C-style)** dans le tampon de réception (`m_rxBuffer[i]`) :
* `m_rxBuffer[0]` = `0x0F` (Start byte statique)
* `m_rxBuffer[1]` = Longueur de la payload incluant la checksum
* `m_rxBuffer[2..3]` = Identifiant de la commande 16 bits (Big-Endian)
* `m_rxBuffer[4]` = **Position 4**
* `m_rxBuffer[5]` = **Position 5**

#### Les 3 Familles Universelles de Notifications de Réponses

1. **Famille PIN (`0x1700`) — Statut en Position 4 :**
   * Format : `0F 06 17 00 [STATUT] [ECHO_SOUS_CMD] 00 [Checksum] FF FF`
   * **`m_rxBuffer[4]` = STATUT D'EXÉCUTION** (`0x00` = Succès, `0x01` = Échec).
   * `m_rxBuffer[5]` = Écho de l'opération demandée (`0x00` = auth, `0x01` = changePin, `0x02` = resetPin).
   * ⚠️ **Garde-fou absolu :** Ne **jamais** chercher le statut en position 5 pour le PIN ! Si l'on vérifiait la position 5, `changePin` (`0x01`) et `resetPinToDefault` (`0x02`) seraient systématiquement rejetées avec une fausse erreur alors que la prise a réussi l'opération.

2. **Famille Commandes 3 octets (`0x0F00` — LED, Prix, Période HC, Reset Conso, Factory Reset) — Statut en Position 5 :**
   * Format : `0F 05 0F 00 [SOUS_CMD] [STATUT] [Checksum] FF FF`
   * `m_rxBuffer[4]` = Écho de la sous-commande (`0x00` = Factory Reset, `0x01` = Période HC, `0x02` = Reset Conso, `0x04` = Prix, `0x05` = LED Ring).
   * **`m_rxBuffer[5]` = STATUT D'EXÉCUTION** (`0x00` = Succès, `0x01` = Échec).
   * ℹ️ L'ensemble de la famille `0x0F00` est **100 % homogène sans exception** : le statut est toujours à l'index 5.

3. **Commandes Standard à Opcode 2 octets — Statut en Position 4 :**
   * Format : `0F 04/06 [CMD_16] [STATUT] ... [Checksum] FF FF`
   * **`m_rxBuffer[4]` = STATUT D'EXÉCUTION** (`0x00` = Succès, `0x01` = Échec).
   * Concerne : `setDateTime` (`0x0100`), `setDeviceName` (`0x0200`), `setRelayState` (`0x0300`), `setOverloadPower` (`0x0500`), `setTimer` / `stopTimer` (`0x0800`), `setScheduler` / `deleteScheduler` / `resetScheduler` (`0x1300`), `setRandomMode` (`0x1500`).

#### Tableau de Référence Exhaustif des Méthodes de l'Interface

| Méthode C++ (`SEM6000Device`) | Opcode émis | Nature | Index Statut | Contenu Index 4 | Contenu Index 5 |
| :--- | :---: | :---: | :---: | :--- | :--- |
| `authenticate` / `authenticate32` | `0x1700` (`sub 00`) | **SET** | **4** | **Statut (`0x00` = OK)** | Écho sous-cmd (`0x00`) |
| `changePin` / `changePin32` | `0x1700` (`sub 01`) | **SET** | **4** | **Statut (`0x00` = OK)** | Écho sous-cmd (`0x01`) |
| `resetPinToDefault` | `0x1700` (`sub 02`) | **SET** | **4** | **Statut (`0x00` = OK)** | Écho sous-cmd (`0x02`) |
| `setDateTime` | `0x0100` | **SET** | **4** | **Statut (`0x00` = OK)** | Checksum |
| `requestSettings` | `0x1000` | **GET** | - | Données de configuration | Données de configuration |
| `setLedRing` | `0x0F0005` | **SET** | **5** | Écho sous-cmd (`0x05`) | **Statut (`0x00` = OK)** |
| `setOverloadPower` | `0x0500` | **SET** | **4** | **Statut (`0x00` = OK)** | Checksum |
| `setPrices` | `0x0F0004` | **SET** | **5** | Écho sous-cmd (`0x04`) | **Statut (`0x00` = OK)** |
| `setReducedPeriod` | `0x0F0001` | **SET** | **5** | Écho sous-cmd (`0x01`) | **Statut (`0x00` = OK)** |
| `setRelayState` | `0x0300` | **SET** | **4** | **Statut (`0x00` = OK)** | Checksum |
| `getTimerStatus` | `0x0900` | **GET** | - | Données Timer | Données Timer |
| `setTimerAbs` / `setTimerRel` | `0x0800` | **SET** | **4** | **Statut (`0x00` = OK)** | Checksum |
| `stopTimer` | `0x0800` | **SET** | **4** | **Statut (`0x00` = OK)** | Checksum |
| `requestSchedulerPage` / `requestAllSchedulers` | `0x1400` | **GET** | - | Nombre et liste des schedulers | Liste schedulers |
| `setScheduler` | `0x1300` | **SET** | **4** | **Statut (`0x00` = OK)** | Padding `0x00` |
| `deleteScheduler` / `resetScheduler` | `0x1300` (`sub 02`) | **SET** | **4** | **Statut (`0x00` = OK)** | Padding `0x00` |
| `getRandomMode` | `0x1600` | **GET** | - | Configuration Aléatoire | Configuration Aléatoire |
| `setRandomMode` | `0x1500` | **SET** | **4** | **Statut (`0x00` = OK)** | Checksum |
| `captureMeasurement` | `0x0400` | **GET** | - | Télémétrie instantanée (W, V, mA, Hz) | Télémétrie |
| `requestDayHistory` (24h) | `0x0A00` | **GET** | - | Historique horaire `uint16_t[24]` | Historique |
| `requestMonthHistory` (30j) | `0x0B00` | **GET** | - | Historique journalier `uint32_t[30]` | Historique |
| `requestYearHistory` (12m) | `0x0C00` | **GET** | - | Historique mensuel `uint32_t[12]` | Historique |
| `factoryReset` | `0x0F0000` | **SET** | **5** | Écho sous-cmd (`0x00`) | **Statut (`0x00` = OK)** |
| `resetConsumptionData` | `0x0F0002` | **SET** | **5** | Écho sous-cmd (`0x02`) | **Statut (`0x00` = OK)** |
| `getDeviceVendorAndVersions` | `0xFFF1` | **GET** | - | Lecture directe ASCII constructeur & FW/HW | Données versions |
| `getDeviceNameGATT` | `0x2A00` | **GET** | - | Lecture directe nom GATT | Nom GATT |
| `setDeviceName` | `0x0200` | **SET** | **4** | **Statut (`0x00` = OK)** | Checksum |
| `getSerialNumber` | `0x1100` | **GET** | - | Numéro de série ASCII (16 octets, `HW 2.0` uniquement) | Série |

---

### 2.8 Dualité Matérielle du Parc : `Hardware 2.0 (FW 1.24)` vs `Hardware 3.0 (FW 1.30)` (`exp_14` à `exp_22`)

Les campagnes de qualification radio, protocolaires et temporelles (`exp_14` à `exp_22`) sur le parc réel de 10 prises ont révélé la coexistence de **deux générations matérielles et logicielles distinctes** sous la même référence commerciale Voltcraft SEM6000 :
* **Génération `HW 2.0 / FW 1.24` (7 prises du parc) :** Préfixes MAC `b0:b1:13:...` et `94:a9:a8:...` (*Bureau, Chambre Cédric, Salon, Smart EQ, Congélateur cuisine, Lave Vaisselle, Théoule*).
* **Génération `HW 3.0 / FW 1.30` (3 prises du parc) :** Préfixe MAC `a3:00:00:...` (*Lave linge, Sèche linge, Vélo / Wattmètre mobile*).

#### 1. Identification Passive en Advertising BLE & Signature Miroir `Manufacturer Data` (`exp_14` & `exp_15`)
Sans établir aucune connexion GATT, l'écoute passive des trames d'annonce BLE (*Advertising*) permet d'authentifier une prise SEM6000 et d'identifier instantanément sa génération matérielle grâce au champ `Manufacturer Data` de **8 octets** :
* **Octets `Mfg[2..7]` (6 octets) — Signature Miroir MAC (Little-Endian) :**
  Contiennent très exactement les **6 octets de l'adresse MAC BLE publique de la prise en ordre inversé**. Cette propriété offre un filtre d'authenticité infaillible à 100 % lors des scans passifs.
* **Octets `Mfg[0..1]` (2 octets) — Signature de Génération Matérielle & Cadence Radio :**
  * **`01 18` $\rightarrow$ `Hardware 2.0 / Firmware 1.24` :** Cadence d'émission des annonces BLE de **`60 ms`** (~16,6 paquets/s, soit ~1 000 trames par minute).
  * **`00 00` $\rightarrow$ `Hardware 3.0 / Firmware 1.30` :** Cadence d'émission des annonces BLE de **`120 ms`** (~8,3 paquets/s, soit ~500 trames par minute — deux fois plus sobre au repos).

#### 2. Comparaison des Tables GATT & Paramètres Radio Connectés (`exp_16`)
Le dump intégral comparatif des tables GATT entre *Chambre Cédric* (`HW 2.0`) et *Vélo* (`HW 3.0`) met en évidence une refonte profonde de la pile BLE sur le `HW 3.0` :

| Propriété / Service GATT | **`Hardware 2.0 / Firmware 1.24`** | **`Hardware 3.0 / Firmware 1.30`** | Impact Technique Passerelle |
| :--- | :--- | :--- | :--- |
| **MTU GATT négocié** | **`23 octets`** (fixe $\rightarrow$ fragmente toute trame $> 20\text{ o}$ en 3 ou 7 paquets) | **`160 octets`** (trames de 55 et 127 o reçues en **1 seul paquet BLE atomique** !) | Zéro fragmentation multi-paquets sur `HW 3.0` |
| **Paramètres GAP préférés (`0x2A04`)** | `50 00 A0 00 00 00 E8 03` (Intervalle **`100..200 ms`**, timeout `10 s`) | `08 00 0A 00 00 00 58 02` (Intervalle **`10..12,5 ms`**, timeout `6 s`) | Échanges GATT **~10x plus rapides** une fois connecté sur `HW 3.0` |
| **Nom GATT (`0x2A00`)** | `READ` seul | `READ \| WRITE` (supporte l'UTF-8, ex. `"Wattmètre mobile"`) | Compatible `getDeviceNameGATT()` |
| **Service `0x180A` (*Device Info*)** | **Présent** (9 caractéristiques `0x2A23..0x2A50`) | **Supprimé** | Sans impact (non utilisé) |
| **Caractéristique `0xFFF1` (Versions)** | `56 4F 4C 43 46 54 ... 01 18 02 00 13` (`FW 1.24`, `HW 2.0`, Byte16 `0x13`) | `56 4F 4C 43 46 54 ... 01 1E 03 00 0C` (`FW 1.30`, `HW 3.0`, Byte16 `0x0C`) | Permet l'identification formelle post-connexion |
| **Service `0xFFF0` (Voltcraft)** | **6 caractéristiques** : `0xFFF1`, **`0xFFF2`**, `0xFFF3`, `0xFFF4`, `0xFFF5`, `0xFFF6` | **3 caractéristiques** : `0xFFF1`, `0xFFF3`, `0xFFF4` (**`0xFFF2`, `0xFFF5`, `0xFFF6` supprimées !**) | **`m_pCharFFF2 == nullptr` sur `HW 3.0`** (`readRawFFF2()` renvoie `false`) |

#### 3. Différences Protocolaires sur `0xFFF3 / 0xFFF4` (`exp_15`, `exp_16`, `exp_18` à `exp_21`)
1. **Bourrage (Padding `00 00`) sur `requestSettings` (`0x1000`) :**
   * Sur `HW 3.0`, la réponse à `0x1000` est envoyée dans un paquet GATT de **20 octets** contenant 2 octets `00 00` après le marqueur `FF FF` :
     `0F 0E 10 00 01 12 0D 05 28 01 68 01 00 0E 60 36 FF FF 00 00`
   * **Loi universelle de longueur logique :** Pour toute trame standard terminée par `FF FF`, la longueur utile exacte de la trame est :
     $$L_{\text{logique}} = \texttt{m\_rxBuffer[1]} + 4$$
     *(Sur `0x1000` : `0x0E + 4 = 18 octets`, checksum `0x36` à l'index 15, `FF FF` aux index 16-17, octets de padding 18-19 ignorés).*
2. **Harmonisation de l'acquittement des commandes `0x0F00` (`CMD_SET_3B`) en 8 octets :**
   * Sur `HW 2.0`, toute commande `0x0F00` (`setLedRing`, `setPrices`, `setReducedPeriod`, `resetConsumptionData`, `factoryReset`) répond sur **9 octets** avec écho de la sous-commande en index 4 et statut en **index 5** (`0F 05 0F 00 [SubCmd] [Status] [CS] FF FF`).
   * Sur `HW 3.0`, la prise répond sur **8 octets** sans écho de sous-commande, plaçant le statut directement en **index 4** (`0F 04 0F 00 [Status=0x00] [CS=0x10] FF FF`), comme un `CMD_SET_2B`.
3. **Suppression de `getSerialNumber` (`0x1100`) :**
   * L'opcode `0x1100` est ignoré par le `HW 3.0` (aucune notification émise sur `0xFFF4`).
4. **Comportement des Checksums d'Historiques (`0x0A00`, `0x0B00`, `0x0C00`) sur `HW 3.0 / FW 1.30` (`exp_18` à `exp_22`) :**
   * **Alignement structurel 100 % identique :** Les trames `0x0A00` (`55 octets`, `Byte[1]=0x33`), `0x0B00` (`127 octets`, `Byte[1]=0x7B`) et `0x0C00` (`55 octets`, `Byte[1]=0x33`) ont exactement la même structure, les mêmes longueurs et le même marqueur final `FF FF` que sur `HW 2.0`. Les valeurs d'énergie en Wh et les ratios HC `B3` y sont strictement exacts.
   * **`0x0B00` (30 jours, 127 o) :** `CS_Got == CS_Exp` (`Écart = 0x00`) sur 100 % des captures (`exp_16` à `exp_22`) lorsque les jours récents `Slot[26..29]` sont actifs (`Slot[00..25] = 0 Wh`).
   * **`0x0A00` (24 heures, 55 o) — Omission des premiers slots de gauche dans `CS_Got` :**
     * Tous les slots récents de la moitié droite (`Slot[15..23]`) sont normalement inclus dans `CS_Got` (dès que `Slot[00..14] = 0`, `CS_Got == CS_Exp`, `Écart = 0x00`, démontré au Cliché #3 d'`exp_21`).
     * En revanche, lorsqu'une consommation glisse dans les premiers slots à gauche (`Slot[00..04]`), la boucle interne de calcul de checksum du `FW 1.30` omet de la sommer dans `CS_Got` : l'écart `CS_Exp - CS_Got` est alors très exactement égal à la somme des octets des slots situés à gauche (`45 -> 27 -> 9 -> 0` lors de la sortie de `[00..02]` dans `exp_21` ; `6 + 10 = 16 = 0x10` lorsque les 6 Wh et 10 Wh d'`exp_21` ont glissé en `Slot[00..01]` dans `exp_22`).
   * **`0x0C00` (12 mois, 55 o) — Biais calendaire dépendant de la date RTC dans `CS_Got` :**
     * Tout au long d'une même journée calendaire, toute variation de `Slot[11]` (Wh et ratio HC `B3`) incrémente `CS_Got` du même delta exact que `CS_Exp` ($\Delta\texttt{CS\_Got} = \Delta\texttt{CS\_Exp}$, écart `CS_Exp - CS_Got` strictement constant : `0x92` le 26/11, `0x9A` le 27/11, `0x62` le 28/11, `0x91` le 30/11, `0xE2` le 01/12 et au reset `01/01/2020`).
     * Toutefois, à payload de 52 octets `buf[0..51]` strictement constante (`CS_Exp = 0x08` invariant dans `exp_19` Clichés #0 à #4), le simple changement de la date RTC via `setDateTime()` modifie `CS_Got`, prouvant que le firmware `1.30` calcule la checksum de `0x0C00` sur sa structure interne en mémoire (incluant la date) et non sur le tampon BLE `tx_buf[2..51]`.
   * **Stratégie de validation en Couche 1 (`SEM6000Device::handleNotification`) :**
     Sur `HW 3.0`, les trames `0x0A00` et `0x0C00` étant transmises chacune en **1 seul paquet BLE atomique (`MTU = 160`)** protégé au niveau radio par le **CRC matériel 24 bits du Link Layer BLE** (aucun risque de perte ou de mélange de fragments contrairement au `HW 2.0`), `handleNotification()` valide ces trames sur leur intégrité structurelle stricte (`m_rxBuffer[0] == 0x0F`, `expectedLen == (size_t)m_rxBuffer[1] + 4 == 55`, et `m_rxBuffer[53..54] == 0xFF 0xFF`).

> [!IMPORTANT]
> **Points Ouverts & Limites Connues (TODO avant publication publique de la librairie `SEM6000Device`) :**
> Dans le cadre d'une diffusion open-source future de `SEM6000Device` auprès d'utilisateurs tiers susceptibles d'exploiter l'ensemble des méthodes d'historiques :
> 1. **`0x0A00` (`HW 3.0 / FW 1.30`) :** Déterminer l'index frontière exact (situé entre `Slot[04]` exclu et `Slot[15]` inclus, ex. boucle bornée à tort sur 12 slots `[12..23]`) à partir duquel le firmware `1.30` commence à sommer les slots dans `CS_Got`.
> 2. **`0x0B00` (`HW 3.0 / FW 1.30`) :** Vérifier le comportement de `CS_Got` lorsque des consommations non nulles glissent dans les premiers index `Slot[00..25]` (au-delà de `J-4`).
> 3. **`0x0C00` (`HW 3.0 / FW 1.30`) :** Établir la formule exacte du biais calendaire injecté par la date RTC dans `CS_Got`, ou documenter explicitement cette anomalie du firmware `1.30` dans les notes de version publiques de la bibliothèque.

#### 4. Cinématique de Comptage d'Énergie & Rotations Calendaires sur `HW 3.0 / FW 1.30` (`exp_17` & `exp_21`)
Le suivi haute cadence (~1 Hz) sous charge réelle (`exp_17`) et les tests de rotations Horaire, Journalière et Mensuelle (`exp_21`) ont établi les lois de comptage du `HW 3.0` :

* **Loi n°1 (`HW 3.0`) — Intégration événementielle en RAM au quantum `+1 Wh` (*« Pas de conso, pas de calcul »*) :**
  Sur `HW 3.0`, le report vers le jour courant `dailyWh[29]` et le mois courant `monthlyWh[11]` n'attend **pas l'heure pile (`HH:00:00`)**. À l'instant précis où l'intégrateur franchit `+1 Wh`, les trois registres `hourlyWh[23]`, `dailyWh[29]` et `monthlyWh[11]` sont mis à jour simultanément en temps réel :
  $$\begin{aligned}
  \texttt{hourlyWh[23]} &\mathrel{+}= 1\text{ Wh} \quad (\text{accumulateur de l'heure en cours en RAM volatile}) \\
  \texttt{dailyWh[29]} &= \sum (\text{heures révolues du jour en Flash}) + \texttt{hourlyWh[23]} \\
  \texttt{monthlyWh[11]} &= \sum (\text{jours révolus du mois en Flash}) + \texttt{dailyWh[29]}
  \end{aligned}$$
  * ⚠️ **Corollaire critique (Anti-Double Comptage) :** Sur `HW 3.0`, `dailyWh[29]` et `monthlyWh[11]` **incluent déjà l'heure en cours `hourlyWh[23]`** à chaque instant.
* **Loi n°2 (`HW 3.0`) — Cinématique atomique de `totalKwhWh` (`0x0400[14..17]`) à l'heure pile (`HH:00:00`) :**
  Le registre 32 bits `totalKwhWh` de la trame `0x0400` représente le **cumul en mémoire Flash de toutes les heures révolues** :
  * Pendant toute la durée d'une heure en cours, `totalKwhWh` reste **strictement constant** (aucune écriture Flash à chaque Wh).
  * Au franchissement d'une heure pile (`HH:00:00`), le firmware ajoute atomiquement l'heure qui vient de se fermer dans `totalKwhWh` (`totalKwhWh += H_fermée`), décale `hourlyWh[23]` dans `hourlyWh[22]` (sauvegardé en Flash) et remet `hourlyWh[23]` à `0 Wh`.
* **Loi n°3 (`HW 3.0`) — Pré-condition de déclenchement des rotations en test accéléré (`exp_19` vs `exp_21`) :**
  Lors d'un saut temporel par `setDateTime()`, injecter une heure à seulement 2 secondes de la frontière (`XX:59:58`) ne laisse pas à la tâche périodique du firmware `1.30` le temps d'échantillonner l'état pré-bascule. Une marge d'environ **15 secondes avant l'échéance (`XX:59:45`)** suivie d'un écoulement naturel jusqu'à `(XX+1):00:10` déclenche à 100 % les rotations Horaire (`0x0A00` + `0x0400`), Journalière (`0x0B00`) et Mensuelle (`0x0C00`).

#### 5. Micro-Coupure / Coupure Secteur sur `HW 3.0` (`exp_22`) & Critique Architecturale de `totalKwhWh`
L'expérience de coupure secteur complète avec charge active avant coupure, à froid après coupure, puis après remise à l'heure + 30 s (`exp_22` v1 & v2) a révélé le comportement exact du `HW 3.0` :

1. **Reset et Gel de la RTC interne au `01/01/2020 00:00:00` :**
   * Après une coupure secteur (~10 s), la RTC interne du `HW 3.0` est réinitialisée à la date d'usine **`01/01/2020 à 00:00:00`** et **reste totalement figée à `00:00:00` (les secondes n'avancent pas)** tant que la prise n'a pas reçu une trame `setDateTime` (`0x0100`).
   * *Preuves arithmétiques directes par la Sonde Timer (`exp_22`) :*
     * Cible `02/01/2020 23:59:59` $\rightarrow$ $\Delta t = \texttt{0x02A2FF} = 172\,799\text{ s} = 2 \times 86\,400 - 1\text{ s}$ (soit très exactement `01/01/2020 00:00:00` à la seconde près !).
     * Cible `31/12/2026 23:59:59` $\rightarrow$ $\Delta t_{24\text{b}} = \texttt{0x2B0B7F} = (2\,557 \times 86\,400 - 1) \ \& \ \texttt{0xFFFFFF}$ (`0x0D2B0B7F` tronqué sur 24 bits).
2. **Arrêt total du comptage tant que la prise n'est pas remise à l'heure :**
   * Tant que la RTC est figée au `01/01/2020 00:00:00`, **aucun Wh n'est comptabilisé** (même sous une charge de `895 W`) : `hourlyWh[23]` reste bloqué à `0 Wh`, et comme l'horloge ne franchit jamais `HH:00:00`, **`totalKwhWh` (`0x0400`) reste lui aussi indéfiniment figé**.
   * Une prise `HW 3.0` subissant une micro-coupure secteur sans qu'une passerelle ou l'application mobile ne se reconnecte pour lui envoyer `setDateTime()` cesse donc définitivement de compter l'énergie.
3. **Persistance Flash vs Perte RAM & Phénomène de Masquage par la Date :**
   * **Conservés à 100 % en Flash :** `totalKwhWh` (`0x0400`), les heures révolues (`0x0A00`), les jours révolus (`0x0B00`), les mois révolus (`0x0C00`), les paramètres (`0x1000`), les plannings (`0x1400`) et le mode aléatoire (`0x1600`), ainsi que l'état du relais (`ON`).
   * **Perdus en RAM volatile :** L'énergie de **l'heure entamée `hourlyWh[23]`** (non encore versée dans `totalKwhWh` à `HH:00:00`) et tout **Timer unitaire actif (`0x0800`)** sont effacés à `0`.
   * **Masquage temporaire de `M[11]` et `J[29]` sous amnésie :** Tant que la RTC pointe sur `01/01/2020`, `M[11]` et `J[29]` affichent `0 Wh`. Dès que `setDateTime()` restaure la date courante et que le 1ᵉʳ Wh est consommé, les jours révolus du mois en cours (stockés en Flash dans `0x0B00`) et les heures révolues du jour en cours **réapparaissent immédiatement** dans `M[11]` et `J[29]` (*dans `exp_22` v2 : `M[11]` passe de `130 Wh (116 Flash + 14 RAM)` avant coupure $\rightarrow$ `0 Wh` masqué après coupure $\rightarrow$ `117 Wh (116 Flash + 1 RAM)` dès `[+5s]` après remise à l'heure, puis `123 Wh (116 Flash + 7 RAM)` à `[+30s]`*).
4. **Détecteur rapide de coupure secteur en 1 seule requête `GET` (`0x0900` $\rightarrow$ `00 00 00 00 00 30`) :**
   * Alors que `0xFFF1`, `0x0400[12..13]` (`0x0100`), `0x1000`, `0x1400` et `0x1600` restent strictement inchangés après une coupure secteur, la lecture à froid de `getTimerStatus()` (`0x0900`) retourne invariablement la signature d'initialisation RAM du boot :
     $$\texttt{0F 0E 09 00 00 } \mathbf{\texttt{00 00 00 00 00 30}} \texttt{ 00 00 00 00 3A FF FF}$$
     où **`targetDay == 0x00 && targetMonth == 0x00 && targetYear == 0x30`** (`00/00/2048 00:00:00`).
   * En s'assurant après chaque `setDateTime()` que les octets `targetDay/targetMonth` de `0x0900` en RAM contiennent une date valide non nulle (via un cycle furtif `setTimerAbs` + `stopTimer` si aucun timer n'était déjà mémorisé), une simple lecture passive `getTimerStatus()` (`0x0900`) détecte à 100 % toute micro-coupure secteur en **1 seule commande GET**, sans jamais perturber un éventuel timer actif (`action != 0x00`).
5. **Critique Architecturale du Compteur `totalKwhWh` (« traité à l'économie ») :**
   * Au lieu de faire de `totalKwhWh` le compteur maître incrémenté à chaque `+1 Wh` indépendamment de la RTC, le constructeur a subordonné `totalKwhWh` à l'accumulateur horaire volatile `hourlyWh[23]` (versé en Flash une seule fois par heure à `HH:00:00`).
   * **Piège de régression d'index lors d'une coupure secteur :** Si la passerelle envoyait naïvement `totalKwhWh + hourlyWh[23]` à Domoticz, alors dans le cas d'`exp_22 (v2)` :
     * Avant coupure : $34\,075 + 14 = \mathbf{34\,089\text{ Wh}}$ transmis à Domoticz.
     * Après coupure + remise à l'heure + 30 s (`14 Wh` perdus en RAM côté prise, `7 Wh` nouveaux) : la prise affiche $34\,075 + 7 = \mathbf{34\,082\text{ Wh}}$ ($< 34\,089\text{ Wh}$ !).
   * **Conclusion :** L'ESP32-S3 (`ManagedSEM6000`, Couche 2) doit obligatoirement maintenir son propre **« Vrai Index Perpétuel Monotone » en NVS (`m_perpetualIndexWh`)** pour chaque prise `HW 3.0` comme `HW 2.0`, en intégrant les deltas positifs de `hourlyWh[23]` au fil de l'eau et en utilisant `totalKwhWh` comme **ancre de rattrapage longue durée** lorsque c'est l'ESP32-S3 qui a été interrompu pendant plusieurs heures ou jours.

#### 6. Tableau Récapitulatif Exhaustif des Différences Protocolaires et Fonctionnelles (`HW 2.0` vs `HW 3.0`)

| Domaine / Fonction | **`Hardware 2.0 / Firmware 1.24`** | **`Hardware 3.0 / Firmware 1.30`** |
| :--- | :--- | :--- |
| **Préfixes MAC observés** | `b0:b1:13:...`, `94:a9:a8:...` | `a3:00:00:...` |
| **Advertising `Mfg[0..1]` & Cadence** | `01 18` — Annonce toutes les **`60 ms`** | `00 00` — Annonce toutes les **`120 ms`** |
| **Advertising `Mfg[2..7]`** | Adresse MAC inversée (Little-Endian) | Adresse MAC inversée (Little-Endian) |
| **MTU GATT & Fragmentation** | **`23 octets`** (trames 55/127 o fragmentées en 3 ou 7 paquets) | **`160 octets`** (toutes les trames reçues en **1 seul paquet**) |
| **Intervalle de connexion GAP (`0x2A04`)** | `100..200 ms` | **`10..12,5 ms`** (~10x plus rapide) |
| **Service `0x180A` (*Device Info*)** | Présent | **Supprimé** |
| **Caractéristiques du service `0xFFF0`** | `0xFFF1`, **`0xFFF2`**, `0xFFF3`, `0xFFF4`, `0xFFF5`, `0xFFF6` | `0xFFF1`, `0xFFF3`, `0xFFF4` (**`0xFFF2`, `0xFFF5`, `0xFFF6` absentes**) |
| **Identification `0xFFF1` (`[11..15]`)** | `01 18 02 00 13` (`FW 1.24`, `HW 2.0`) | `01 1E 03 00 0C` (`FW 1.30`, `HW 3.0`) |
| **Numéro de série `0x1100`** | Supporté (25 octets, 16 caractères ASCII) | **Supprimé** (aucune notification) |
| **Réponse `requestSettings` (`0x1000`)** | **18 octets** (`0F 0E 10 00 ... CS FF FF`) | **20 octets** avec padding `00 00` (`0F 0E 10 00 ... CS FF FF 00 00`) |
| **Acquittement `CMD_SET_3B` (`0x0F00`)** | **9 octets** : `0F 05 0F 00 [SubCmd] [Status] [CS] FF FF` (**Statut index 5**) | **8 octets** : `0F 04 0F 00 [Status] [CS] FF FF` (**Statut index 4**) |
| **Durée Timer `0x0900[11..13]`** | Entier **24 bits Big-Endian en secondes** (statique après SET) | Entier **24 bits Big-Endian en secondes** ($\Delta t$ recalculé si actif, `0` si inactif) |
| **Lecture Plannings (`0x1400`)** | **Paginée en 3 pages $\times$ 56 o** (4 slots max/page, `CS_Valid = OUI`) | **Non paginée : 1 trame monolithique de 152 o** dès Page 0 (12 slots, `CS_Valid = OUI`) |
| **Allocation `SlotId` Plannings (`0x1300`)** | **`12..1` (LIFO décroissant)**, calcule un `SortKey` non nul, conserve la date en hebdo | **`0..11` (FIFO croissant)**, `SortKey = 0`, force la date à `00/00/00` en hebdo |
| **Acquittement `0x1300` (`setScheduler`)** | **10 octets** (`0F 06 13 00 [Status] 00 00 [CS] FF FF`) | **8 octets** (`0F 04 13 00 [Status] [CS] FF FF`) **+ notif spontanée `0x1400` ~1 ms après** |
| **Mode Aléatoire (`0x1500` / `0x1600`)** | 15 octets (`CS_Valid = OUI`), défaut usine `00:00->00:00 (mask=0x00)` | 15 octets (`CS_Valid = OUI`), défaut usine `20:00->08:00 (mask=0xFF)` |
| **Octets `[12..13]` de `0x0400`** | Toujours `0x0000` | Toujours `0x0100` |
| **Compteur `totalKwhWh` (`0x0400[14..17]`)** | Inactif (`0 Wh` permanent) | **Actif en Flash**, mis à jour **1 fois/heure à `HH:00:00`** (`+= H_fermée`) |
| **Mise à jour de `H[23]` (`0x0A00`)** | Temps réel à chaque `+1 Wh` (en RAM) | Temps réel à chaque `+1 Wh` (en RAM) |
| **Mise à jour de `J[29]` & `M[11]`** | **À l'heure pile (`HH:00:00`)** uniquement | **Temps réel à chaque `+1 Wh`** ($J[29]$ et $M[11]$ incluent déjà $H[23]$) |
| **Checksum `0x0A00` (24h, 55 o)** | Standard `(1 + sum(buf[2..51])) & 0xFF` sur les 24 slots | **Anomalie `FW 1.30`** : omet les premiers slots à gauche (`Slot[00..04]`) |
| **Checksum `0x0B00` (30j, 127 o)** | Standard `(1 + sum(buf[2..123])) & 0xFF` | Standard vérifiée sur `[26..29]` (à confirmer sur `[00..25]`) |
| **Checksum `0x0C00` (12m, 55 o)** | Standard `(1 + sum(buf[2..51])) & 0xFF` | **Anomalie `FW 1.30`** : inclut un biais interne lié à la date RTC |
| **État RTC après coupure secteur** | Amnésique (`Word0 == 0xFFFF` sur `0xFFF2`), comptage bloqué | Figée au **`01/01/2020 00:00:00`**, comptage `H[23]` et `totalKwhWh` bloqué |
| **Détection rapide de coupure secteur** | Lecture `0xFFF2` sans auth : **`Word0 == 0xFFFF`** | Lecture `0x0900` (`getTimerStatus`) : **`targetDay == 0 && targetMonth == 0` (`00 00 30`)** |
| **Impact coupure secteur sur les Wh** | `H[23]` perdu (RAM) ; `J` et `M` (Flash) masqués à `0` jusqu'à `setDateTime` | `H[23]` perdu (RAM) ; `totalKwhWh` intact ; `J[29]` et `M[11]` masqués à `0` jusqu'à `setDateTime` |

---

## 3. Architecture Logicielle Cible & Stratégie Bi-Génération (`HW 2.0` + `HW 3.0`)

L'architecture est découpée en trois couches distinctes pour isoler le protocole BLE bas niveau de la logique métier et du réseau, permettant **au même binaire ESP32-S3 de piloter de manière transparente un parc mixte de prises `HW 2.0` et `HW 3.0`** :

    GatewaySEM6000/
    ├── docs/
    │   └── architecture.md          # Spécifications techniques (ce fichier - SSOT)
    ├── lib/
    │   └── SEM600Device/            # Couche 1 : Pilote BLE pur bi-génération (NimBLE)
    │       ├── SEM6000Device.h
    │       └── SEM6000Device.cpp
    └── src/
        ├── ManagedSEM6000.h/.cpp    # Couche 2 : Machine à états, horloge & Index Perpétuel
        ├── SEM6000Manager.h/.cpp    # Couche 3 : Ordonnanceur multi-prises & Queue BLE
        └── main.cpp                 # Wi-Fi, NTP, Client Domoticz/MQTT & Boucle principale

### Couche 1 : `SEM6000Device` (Pilote BLE Unifié `HW 2.0` / `HW 3.0` — Version `v0.4.0` « La Grande Réunification »)
* Encapsule uniquement la communication GATT/NimBLE avec une prise (connexion, authentification 32 bits, commandes ON/OFF, synchronisation horloge, lecture `FFF2` si présente, `0x0400`, `0x0900` et historiques).
* **Adaptations bi-génération transparentes (`v0.4.0`) :**
  1. **Auto-détection de la génération matérielle dès `connect()` :** L'absence de la caractéristique `0xFFF2` (`m_pCharFFF2 == nullptr`) identifie immédiatement une prise `HW 3.0` (`isHardwareV3()`).
  2. **Décodage par longueur logique dans `handleNotification()` :** Utilisation de `expectedLen = (size_t)m_rxBuffer[1] + 4` pour localiser le marqueur `FF FF` et la checksum, éliminant automatiquement le padding `00 00` des paquets de 20 octets du `HW 3.0` (`0x1000`).
  3. **Acquittement polymorphe `CMD_SET_3B` (`0x0F00`) :** Validation de `m_rxBuffer[5] == 0x00` lorsque la réponse fait 9 octets (`HW 2.0`) ou de `m_rxBuffer[4] == 0x00` lorsqu'elle fait 8 octets (`HW 3.0`).
  4. **Décodage 24 bits Big-Endian en secondes dans `getTimerStatus()` (`0x0900`) :** `presetSeconds = (B11 << 16) | (B12 << 8) | B13`, et extraction systématique des octets cibles `targetSec..targetYear` même lorsque `active == false` (`action == 0x00`) pour permettre l'inspection de la sentinelle RAM post-coupure.
  5. **Validation structurelle des historiques `0x0A00` / `0x0C00` sur `HW 3.0` :** Acceptation des trames `0x0A00` et `0x0C00` sur `HW 3.0` dès lors que `m_rxBuffer[0] == 0x0F`, `m_rxLength == expectedLen` (`55`) et `m_rxBuffer[53..54] == 0xFF 0xFF`, avec mémorisation d'un indicateur de diagnostic `m_lastChecksumValid`.
  6. **Détection et correction de perte d'horloge bi-génération (`checkAndFixClockLoss`) :**
     * Sur `HW 2.0` : via `readRawFFF2()` (`word0 == 0xFFFF`).
     * Sur `HW 3.0` : via `getTimerStatus()` (`0x0900` : `targetDay == 0 && targetMonth == 0`), suivie de `setDateTime()` et de l'armement/désarmement immédiat de la sentinelle RAM (`setTimerAbs` + `stopTimer`).
  7. **Filtrage des notifications spontanées par `m_expectedCmd` & Lecture bi-génération des plannings (`0x1300` / `0x1400`) :**
     * Verrouillage de `m_rxBuffer` dans `handleNotification()` (`if (length == 0 || m_rxReady || m_expectedCmd == 0) return;` et vérification `incomingCmd == m_expectedCmd`) pour ignorer la notification spontanée `0x1400` émise par le `HW 3.0` ~1 ms après chaque acquittement `0x1300`.
     * Arrêt automatique de pagination dans `requestAllSchedulers()` dès que `list.size() >= totalSchedulers` (traitant aussi bien la trame monolithique de 152 octets du `HW 3.0` que les 3 pages de 56 octets du `HW 2.0`).
* Accesseurs standardisés sur les historiques :
  * `hourlyWh[23 - hoursAgo]`
  * `dailyWh[29 - daysAgo]`
  * `monthlyWh[11 - monthsAgo]`

### Couche 2 : `ManagedSEM6000` (Machine à États & Index Perpétuel Monotone Bi-Génération)
Encapsule par **composition forte** une instance `SEM6000Device` et maintient en **NVS (`Preferences`)** le **Vrai Index Perpétuel Monotone (`uint32_t m_perpetualIndexWh`)** de chaque prise :

1. **Calcul Différentiel de l'Index Perpétuel 32 bits (`m_perpetualIndexWh`) & Immunité aux Coupures Secteur :**
   * **Pourquoi un accumulateur différentiel NVS est indispensable même sur `HW 3.0` :**
     Lors d'une coupure secteur en milieu d'heure (ex. après `14 Wh` accumulés dans `H[23]`), la prise `HW 3.0` perd `H[23]` en RAM sans l'avoir versé dans `totalKwhWh`. Un calcul naïf `totalKwhWh + H[23]` reculerait de `14 Wh` et perdrait l'énergie consommée avant la coupure.
   * **Algorithme sur `Hardware 3.0` (couple d'état `(m_lastTotalKwh0400, m_lastH23)`) :**
     * *En cours d'heure (`totalKwhWh == m_lastTotalKwh0400` et `H[23] >= m_lastH23`)* :
       $$\Delta\text{Wh} = H[23] - m\_\text{lastH23} \quad ; \quad m\_\text{perpetualIndexWh} \mathrel{+}= \Delta\text{Wh} \quad ; \quad m\_\text{lastH23} = H[23]$$
     * *Sur détection de coupure secteur en cours d'heure (`0x0900` = `00 00 30`, ou `H[23] < m_lastH23` à `totalKwhWh` constant)* :
       Les Wh accumulés avant la coupure étant déjà acquis dans `m_perpetualIndexWh`, on réinitialise la référence `m_lastH23 = 0` avant d'intégrer la nouvelle valeur `H[23]` post-resynchronisation ($\Delta\text{Wh} = H[23]$).
     * *Au franchissement d'une ou plusieurs heures piles (`totalKwhWh > m_lastTotalKwh0400`, y compris après une longue coupure de l'ESP32-S3)* :
       $$\Delta\text{Wh} = \max\Big(0,\ (\texttt{totalKwhWh} - m\_\texttt{lastTotalKwh0400}) + H[23] - m\_\text{lastH23}\Big)$$
       $$m\_\text{perpetualIndexWh} \mathrel{+}= \Delta\text{Wh} \quad ; \quad m\_\text{lastTotalKwh0400} = \texttt{totalKwhWh} \quad ; \quad m\_\text{lastH23} = H[23]$$
   * **Algorithme sur `Hardware 2.0` (`totalKwhWh == 0` permanent) :**
     Intégration différentielle de `H[23]`, récupération du solde de fin d'heure dans `H[22]` lors du franchissement `HH:00:00` ($\Delta\text{Wh} = (H[22] - m\_\text{lastH23}) + H[23]$), remise à zéro de `m_lastH23` sur coupure secteur (`0xFFF2 == 0xFFFF`), et rattrapage multi-échelles (`hourlyWh[24]`, `dailyWh[30]`, `monthlyWh[12]`) après une absence prolongée de l'ESP32-S3.
   * **Garde-fou universel d'émission Domoticz (commun `HW 2.0` & `HW 3.0`) :**
     L'ESP32-S3 ne transmet **jamais** un index inférieur au dernier index publié afin de prévenir tout rejet ou débordement 32 bits dans la base SQLite de Domoticz.

2. **Détection d'Amnésie Horloge & Maintien à l'Heure NTP :**
   * **Sur `Hardware 2.0` :** Lecture de `0xFFF2` sans PIN. Si `Word0 == 0xFFFF`, injection immédiate de `setDateTime()` pour démasquer la Flash et relancer le comptage.
   * **Sur `Hardware 3.0` (`0xFFF2` absente) :** Lecture de `0x0900` (`getTimerStatus`). Si `targetDay == 0 && targetMonth == 0` (`00 00 30`), la RTC est figée au `01/01/2020 00:00:00` suite à une micro-coupure $\rightarrow$ injection immédiate de `setDateTime()` pour relancer le comptage et démasquer `M[11]`/`J[29]`, suivie du réarmement de la sentinelle RAM (`setTimerAbs` + `stopTimer`). En régime nominal, recalage périodique en « Havre de Paix » (`HH:30:00`) si la dérive mesurée par la Sonde Timer dépasse 5 secondes.

3. **Surveillance de Surcharge :**
   * **Sur `Hardware 2.0` :** Surveillance légère via `0xFFF2` (`Word1` vs `Word0`).
   * **Sur `Hardware 3.0` :** Comparaison directe lors du poll `0x0400` entre la puissance active mesurée $P_{\text{mesurée}}$ (ou $I_{\text{mesuré}}$) et la consigne $P_{\max}$ mise en cache lors de la lecture initiale de `0x1000`.

4. **Résilience et Rattrapage Multi-Échelles Post-Panne de l'ESP32-S3 :**
   Au redémarrage de l'ESP32-S3 après une absence prolongée (coupure Wi-Fi, maintenance ESP32, vacances) alors que les prises sont restées alimentées :
   * **Sur `Hardware 3.0` :** Rattrapage direct et exact quelle que soit la durée d'absence grâce au différentiel Flash $\texttt{totalKwhWh} - m\_\texttt{lastTotalKwh0400}$.
   * **Sur `Hardware 2.0` :** Calcul de $\Delta T = T_{\text{NTP}} - T_{\text{NVS}}$ et rattrapage via `hourlyWh[24]` ($\Delta T \le 23\text{ h}$), `dailyWh[30]` ($24\text{ h} < \Delta T \le 29\text{ j}$) ou `monthlyWh[12]` ($\Delta T > 30\text{ j}$).

5. **Politique d'usure Flash NVS :**
   * Sauvegarde en NVS (`Preferences`) déclenchée uniquement si $\Delta \ge 10\text{ Wh}$ ou si plus de 30 minutes se sont écoulées, garantissant une longévité de plus de 50 ans de la Flash de l'ESP32-S3.

### Couche 3 : `SEM6000Manager`
Ordonnance le parc de prises et assure la passerelle vers Domoticz :
* **Contrainte Mono-Connexion BLE :** La prise n'acceptant qu'un seul client BLE à la fois, les connexions sont strictement séquentialisées (une prise après l'autre, pas de concurrence radio).
* **Flotte de Passerelles Multi-Niveaux (Topologie) :**
  * Dans un logement sur plusieurs niveaux (ex. 10 prises réparties entre cave, garage, arrière-cuisine derrière murs béton, salon et étages), plusieurs ESP32 exécutant le même firmware générique sont déployés par zone.
  * **Attribution déclarative stricte (Master Controller) :** Chaque ESP32 détient sa propre table de prises assignées. Une prise n'est interrogée que par son ESP32 Maître exclusif pour éviter toute collision ou déconnexion sauvage.
* **Synchronisation Réseau :** Distribution du temps NTP et publication HTTP/JSON ou MQTT vers Domoticz.


