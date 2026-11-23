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

#### 9. Cinématique du Timer & Durée d'Armement Statique (`0x0900`, `exp_11`)
* **Nature des octets `[B11..B13]` :**
  Dans la trame de réponse `0x0900` (Timer Status), les octets 11 à 13 (`m_rxBuffer[11..13]`, codés en `Heures, Minutes, Secondes`) représentent formellement la **durée d'exécution initiale calculée par le SEM6000 au moment du SET**.
  * Ce sont des valeurs **strictement statiques** (preset de consigne) qui ne décroissent pas au fil du temps (démontré par `exp_11` : reste figé à `[0x00 0x00 0x3C]` = 60s tout au long de la minute d'attente).
  * La prise ne maintient aucun compte à rebours décrémental en mémoire vive dans ce registre.
* **Calcul du temps restant côté client / passerelle :**
  Pour afficher un compte à rebours dynamique en temps réel, l'application cliente ou la passerelle doit calculer la différence entre la date cible absolue (`Octets 5 à 10 : Seconde, Minute, Heure, Jour, Mois, Année`) et l'heure courante :
  $$\Delta t_{\text{restant}} = T_{\text{cible}} - T_{\text{courant}}$$
* **Bascule à l'échéance et fin de vie du Timer :**
  * Dès que $T_{\text{courant}} = T_{\text{cible}}$, le relais matériel commute immédiatement vers l'action programmée (ex: `ON`).
  * De manière atomique, l'octet d'action `m_rxBuffer[4]` repasse immédiatement de `0x01` (ou `0x02`) à **`0x00` (INACTIF)**.
  * La date cible et la durée statique initiale restent conservées dans le tampon comme historique de la dernière consigne exécutée.
  * **Test d'activité univoque :** La condition `m_rxBuffer[4] != 0x00` certifie à 100% si un timer est en cours d'exécution.

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
  Lors d'un armement de timer (`setTimerAbs`), la prise calcule la différence entre la date cible absolue $T_{\text{cible}}$ fournie et son horloge interne instantanée $T_{\text{interne}}$ :
  $$\Delta t = T_{\text{cible}} - T_{\text{interne}}$$
  Cette durée statique $\Delta t$ (`[B11..B13] = Heures, Minutes, Secondes`) est immédiatement lisible via `getTimerStatus()` (`0x0900`). On en déduit directement :
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
| `getSerialNumber` | `0x1100` | **GET** | - | Numéro de série ASCII (16 octets) | Série |

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

### Couche 1 : `SEM6000Device` (Validée — v0.3.8)
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

6. **Auscultation Furtive de l'Horloge Interne (Lecture Indirecte via Timer) :**
   * Orchestre la séquence non-destructive d'armement/lecture/désarmement rapide sur le registre Timer de Couche 1 (`SEM6000Device`) afin d'extraire la dérive réelle du quartz ($\Delta t$) par rapport au temps NTP (cf. Section 2.6 Règle 4).
   * Permet d'inhiber tout recalage NTP superflu lorsque la dérive mesurée est inférieure à un seuil tolérable (ex: $|\text{Dérive}| < 5\text{ s}$), réduisant encore l'activité d'écriture sur la prise.

### Couche 3 : `SEM6000Manager`
Ordonnance le parc de prises et assure la passerelle vers Domoticz :
* **Contrainte Mono-Connexion BLE :** La prise n'acceptant qu'un seul client BLE à la fois, les connexions sont strictement séquentialisées (une prise après l'autre, pas de concurrence radio).
* **Flotte de Passerelles Multi-Niveaux (Topologie) :**
  * Dans un logement sur plusieurs niveaux (ex. 10 prises réparties entre cave, garage, arrière-cuisine derrière murs béton, salon et étages), plusieurs ESP32 exécutant le même firmware générique sont déployés par zone.
  * **Attribution déclarative stricte (Master Controller) :** Chaque ESP32 détient sa propre table de prises assignées. Une prise n'est interrogée que par son ESP32 Maître exclusif pour éviter toute collision ou déconnexion sauvage.
* **Synchronisation Réseau :** Distribution du temps NTP et publication HTTP/JSON ou MQTT vers Domoticz.

