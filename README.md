# BauBau – robot Micro Sumo (JSumo XS1 / XMotion Micro) pentru RoboChallenge

Acest proiect conține codul complet al robotului și ghidul de montaj, testare și reglaj.

| Fișier | Ce este |
|---|---|
| `BauBau_MicroSumo/BauBau_MicroSumo.ino` | codul robotului (se deschide în Arduino IDE) |
| `README.md` | acest ghid |

Codul a fost compilat cu succes pentru placa **Arduino Leonardo** (ATmega32u4, placa XMotion Micro):
10,9 KB din 28 KB flash, 221 bytes RAM.

---

## 1. Ce știe codul

- **Caută și atacă** adversarul cu cei 3 senzori JS40F (stânga / față / dreapta): drept înainte cu viteză maximă,
  arc când adversarul e în față + lateral, rotire pe loc spre el când e doar lateral.
- **Marginea are prioritate absolută**: la linia albă dă înapoi, se întoarce și reia căutarea.
  Linia se citește de ~3000 de ori pe secundă, fără `delay()` – robotul nu rămâne „orb” nicio clipă.
- **Memorie**: ține minte pe ce parte a fost văzut ultima dată adversarul și se rotește spre el.
- **Căutare**: rotire, apoi o bucată înainte, apoi rotire… nu stă pe loc niciodată.
- **3 tactici de start** alese din butoane: caută spre STÂNGA, DREPT înainte, caută spre DREAPTA.
- **Pornire de la modulul IR de start/stop** (obligatoriu la RoboChallenge) sau de la butoane (cu 5 s numărătoare, pentru antrenament).
- **STOP** instant de la orice buton / de la telecomanda arbitrului.
- **Mod TEST** (senzori + motoare) și **calibrare automată a senzorului de linie** (salvată în EEPROM).
- Filtre anti-zgomot pe butoane, senzorul de linie și semnalul modulului, ca să nu se oprească/pornească singur din cauza motoarelor.

---

## 2. Hardware – ce verifici înainte de orice

### 2.1 Conexiunile pe XMotion Micro

Codul folosește exact pinii din codul oficial JSumo pentru XS1. Verifică pe placă (etichetele de lângă conectori) că senzorii sunt în porturile corecte:

| Componentă | Port / pin | Logică |
|---|---|---|
| JS40F **dreapta** | A0 | HIGH când vede adversarul (LED-ul roșu de pe senzor se aprinde) |
| JS40F **față** | A1 | idem |
| JS40F **stânga** | A2 | idem |
| ML1 senzor de linie (sub robot, în față) | A3 | valoare mică (~0) pe alb, mare pe negru |
| Buton dreapta | D8 | LOW când e apăsat |
| Buton stânga | D12 | LOW când e apăsat |
| LED1 / LED2 | D4 / D6 | HIGH = aprins |
| Motor (roata stângă în cod) | D10 direcție, D9 PWM | eticheta de pe placă poate fi „Right” – este așa și în codul oficial |
| Motor (roata dreaptă în cod) | D13 direcție, D5 PWM | eticheta de pe placă poate fi „Left” |

Dacă în testul de motoare (secțiunea 4.3) roțile ies invers, **nu rescrii nimic** – schimbi doar
`MOTOR_SWAP`, `MOTOR_LEFT_INVERT` sau `MOTOR_RIGHT_INVERT` din cod. La fel, dacă senzorul din stânga reacționează
când pui mâna în dreapta, inversează cablurile celor doi senzori laterali (sau schimbă `PIN_OPP_LEFT` / `PIN_OPP_RIGHT`).

Cabluri JS40F: la senzorii vechi maro = 5V, albastru = GND, negru = semnal; la cei din 2025 roșu = 5V, negru = GND, galben = semnal.

### 2.2 Alimentarea

- XMotion Micro merge la **7–12 V** (tabelul oficial JSumo). Folosește bateria LiPo 2S (7,4 V) din kit
  sau una echivalentă (2S, 7,4 V, 200–300 mAh). Nu depăși 12 V.
- **Placa NU are protecție la polaritate inversă.** Verifică polaritatea conectorului bateriei înainte de prima conectare.
- Bateria plină ține 5–10 minute de funcționare continuă: încarcă-o înainte de fiecare rundă de teste și la concurs.
- Când încarci cod pe USB, placa e alimentată de la USB, dar motoarele merg doar cu bateria pornită.

### 2.3 Modulul IR de start/stop – OBLIGATORIU la RoboChallenge

Regulamentul Micro Sumo RoboChallenge spune clar: robotul **trebuie** să aibă modul IR de start/stop funcțional
(secțiunea 2.2). Fără el **nu ești lăsat să concurezi** (7.3B). Runda pornește **imediat** la semnalul arbitrului,
**fără** cele 5 secunde clasice (4.2A). Dacă robotul se mișcă fără semnalul arbitrului este abatere majoră (7.5.1B).

**De unde îl iei:** magazinul partener Robomaniacs, produsul „Modul Start-Stop Nanolabs”
(https://robomaniacs.ro/product/modul-start-stop-nanolabs/). Se poate cumpăra și la concurs, la check-in (plata doar cu cardul).
Standardul folosit este cel de pe startmodule.com (protocol RC-5, cu identificator unic per dohyo programat de arbitru).
Modulul **MicroStart de la JSumo lucrează cu telecomenzi Sony** și nu este documentat ca fiind compatibil cu acest standard –
nu-l lua pentru RoboChallenge decât cu confirmare de la organizatori.

**Cum se conectează** (modulul are 4 pini, funcționează la 5 V sau 3,3 V):

| Pin modul | Unde pe XMotion Micro | Comportament |
|---|---|---|
| VCC | 5V | – |
| GND | GND | – |
| Start | un terminal liber (ex. D2) | 0 la pornire, **1 când arbitrul dă START**, 0 la STOP |
| Kill (opțional) | un terminal liber (ex. D3) | 1 la pornire, 0 la STOP |

Apoi în cod, în secțiunea de configurare:

```c
#define USE_START_MODULE  1     // ⇐ pune 1 când modulul este conectat
#define PIN_START_MODULE  2     // ⇐ numărul terminalului folosit (D2 → 2, D7 → 7, A4 → A4)
#define USE_KILL_PIN      0     // 1 dacă ai legat și pinul Kill
#define PIN_KILL_MODULE   3
```

Reguli de care ține cont:
- Receptorul IR trebuie să „vadă” telecomanda arbitrului: montează-l **deasupra**, neacoperit de lamă sau cabluri.
- La pornirea robotului LED-ul verde al modulului trebuie să fie **stins**; dacă e aprins, cere arbitrului să dea STOP și repornește robotul.
- Recântărește robotul după montaj: XS1 are ~94 g, limita e **sub 100 g** (la omologare cântarul acceptă până la 101 g).
- Fără telecomanda de test (se vinde separat pe p1r.se) nu poți simula arbitrul acasă. Codul permite pornirea de antrenament
  ținând apăsat un buton 1,5 s (vezi secțiunea 5).

### 2.4 Mecanică

- Dimensiuni maxime 5 × 5 × 5 cm, masă sub 100 g (regulament 2.1).
- Lama: la teste de acasă poate lipsi (siguranță), la concurs se montează cu bandă dublu adezivă; unghiul de 20° ajută să intri sub adversar.
- Roțile de silicon se curăță cu alcool izopropilic / șervețele cu alcool **înainte de fiecare meci** – diferența de aderență e mare.
- Șuruburile M3 ale roților și cele M2 ale corpului: verifică-le strânse înainte de concurs; vibrațiile le slăbesc.

---

## 3. Încărcarea codului (Arduino IDE)

1. Instalează Arduino IDE (2.x) de pe arduino.cc. Acceptă instalarea driverelor dacă ți se cere.
2. Deschide `BauBau_MicroSumo/BauBau_MicroSumo.ino` (File → Open). IDE-ul cere ca fișierul să fie în folderul cu același nume – așa este deja.
3. **Tools → Board → Arduino AVR Boards → Arduino Leonardo.**
4. Conectează cablul micro-USB. **Tools → Port** → alege portul care apare (pe Mac: `/dev/cu.usbmodemXXXX`).
5. **Pune robotul pe un suport, cu roțile în aer**, dacă bateria este pornită.
6. Apasă **Upload** (săgeata). Când scrie „Done uploading”, gata.

Dacă încărcarea eșuează (nu găsește portul): deconectează și reconectează USB-ul, reselectează portul; placa Leonardo
schimbă portul pentru câteva secunde în timpul încărcării – este normal. Nu folosi butonul de reset ca „pornire”:
după reset, bootloader-ul așteaptă ~8 s înainte să ruleze codul; pornește robotul de la întrerupător.

---

## 4. Testare pas cu pas (în ordinea asta)

### 4.1 Senzorii, fără calculator (1 minut)

Pornește robotul normal și lasă-l în așteptare. LED-urile arată senzorii în timp real:
- **LED1 aprins** = cel puțin un senzor vede ceva. Pune mâna în față, apoi stânga, apoi dreapta (la 10–30 cm): LED1 trebuie să se aprindă de fiecare dată, iar LED-ul roșu de pe senzorul respectiv la fel.
- **LED2 aprins** = senzorul de linie vede alb. Pune robotul pe dohyo cu senzorul pe negru (LED2 stins), apoi cu senzorul pe linia albă (LED2 aprins).

Dacă LED1 stă aprins permanent cu nimic în față: un senzor e deconectat sau vede un obiect apropiat (masa, un perete). Testează cu robotul în mână, în aer, departe de obiecte.

### 4.2 Telemetrie pe Serial Monitor (ca să vezi exact ce citește)

1. **Ține apăsat butonul STÂNGA** în timp ce pornești robotul de la întrerupător – cele două LED-uri clipesc de 3 ori = mod TEST.
2. Conectează USB-ul, deschide **Tools → Serial Monitor**, viteza **115200**.
3. Vezi de 10 ori pe secundă: `ADV L:0 F:1 R:0 | LINIE raw:812 prag:45 negru | BTN L:0 R:0 | ASTEPTARE tactica:DREPT`.
   - `ADV` – senzorii de adversar (1 = vede).
   - `LINIE raw` – citirea 0–1023 (pe negru valoare mare, pe alb aproape 0); `prag` – valoarea sub care considerăm alb.
4. Comenzi din Serial Monitor (scrie litera și Enter): `t` telemetrie, `m` test motoare, `c` calibrare linie, `d` prag implicit,
   `1`/`2`/`3` tactică, `s` start imediat (robotul în aer!), `x` stop, `?` ajutor.

### 4.3 Motoarele (robotul ÎN AER, pe un suport)

În modul TEST apasă **butonul STÂNGA** (sau trimite `m`). După 5 clipiri rulează 8 pași de câte 0,7 s, cu LED1 aprins când e comandată roata stângă și LED2 pentru dreapta:

1. roata **STÂNGĂ înainte** · 2. stânga înapoi · 3. roata **DREAPTĂ înainte** · 4. dreapta înapoi ·
5. ambele înainte · 6. ambele înapoi · 7. rotire stânga (stânga înapoi + dreapta înainte) · 8. rotire dreapta.

„Înainte” înseamnă că roata împinge robotul spre lamă. Dacă nu vezi exact asta, în cod:
- se învârte cealaltă roată → `MOTOR_SWAP 1`;
- o roată merge invers → `MOTOR_LEFT_INVERT 1` sau `MOTOR_RIGHT_INVERT 1`.
Reîncarcă și repetă testul până e perfect. Nu trece mai departe până nu e corect: cu roțile inversate robotul fuge de adversar.

### 4.4 Calibrarea senzorului de linie (pe dohyo-ul pe care joci)

Valorile implicite sunt cele din codul JSumo (alb = sub 45), dar fiecare dohyo e altfel vopsit. Calibrează:

1. **Ține apăsat butonul DREAPTA** când pornești robotul (sau, în mod TEST, apasă DREAPTA / trimite `c`). Ambele LED-uri stau aprinse.
2. Pune robotul pe dohyo, cu senzorul de linie pe **negru**, și apasă orice buton. LED1 începe să clipească.
3. Mută robotul cu senzorul pe **linia albă** și apasă orice buton.
4. 3 clipiri lente = salvat în EEPROM (rămâne și după oprire). 10 clipiri rapide = eroare (diferență prea mică între alb și negru:
   senzor murdar, prea sus, cablu desprins) – se păstrează valorile vechi.

Pragul se pune la 35 % din distanța alb→negru, ca să prindă linia și la viteză mare, dar să ignore zgârieturile.
Repetă calibrarea **la concurs, pe dohyo-ul oficial**, la omologare sau în pauza de dinaintea primului meci.

### 4.5 Primul meci de test (cu butoane)

Cu `USE_START_MODULE 0` (așa este livrat codul):
1. Pune robotul pe dohyo, pornește-l.
2. Apasă **STÂNGA** (va căuta rotindu-se spre stânga), **DREAPTA** (spre dreapta) sau **ambele** (pleacă drept).
3. LED-urile clipesc alternativ 5 s (rapid în ultima secundă), apoi pornește. O nouă apăsare în timpul numărătorii o anulează.
4. **Orice buton în timpul luptei = STOP.**

Pune-i în față o cutie neagră sau alt robot și urmărește: atacă ce vede, la linie dă înapoi și se întoarce, apoi caută.
Pentru teste pe birou fără dohyo: linia nu există, deci va merge până cade – ține-l pe dohyo sau în aer.

### 4.6 Testul cu modulul de start

După ce ai montat modulul (2.3) și ai pus `USE_START_MODULE 1`: la omologare arbitrul programează modulul (LED-ul verde clipește de 2 ori),
apoi START trebuie să pornească robotul instant și STOP să-l oprească. În telemetrie vezi `START:0/1`.

---

## 5. Cum se folosește la meci

| Acțiune | Fără modul (`USE_START_MODULE 0`) | Cu modul (`USE_START_MODULE 1`) |
|---|---|---|
| apăsare scurtă STÂNGA | tactica „stânga” + numărătoare 5 s | doar alege tactica „stânga” (LED1 aprins 1 s) |
| apăsare scurtă DREAPTA | tactica „dreapta” + numărătoare | doar alege tactica „dreapta” (LED2 aprins 1 s) |
| ambele butoane | tactica „drept” + numărătoare | doar alege tactica „drept” (ambele LED-uri 1 s) |
| ținut apăsat 1,5 s | – | pornire de antrenament cu numărătoare (fără telecomandă) |
| semnal START de la arbitru | – | **pornește imediat** cu tactica aleasă |
| semnal STOP / orice buton în luptă | STOP | STOP |

Tactica implicită la pornirea robotului este „DREPT”. Alege tactica în funcție de cum sunt așezați roboții:
dacă adversarul e în fața ta → drept; dacă e în stânga/spate → stânga; dacă e în dreapta → dreapta.

Procedura la masă: pornești robotul (LED verde al modulului stins) → arbitrul programează modulul → așezi robotul în spatele
liniei de start → apeși scurt tactica → te retragi → arbitrul dă START. Nu atinge robotul și nu ține mâinile lângă dohyo în
timpul rundei: senzorii văd până la 60–80 cm și vor „ataca” mâinile.

---

## 6. Reglaje

Toți parametrii sunt în secțiunea „1. CONFIGURARE” din cod. Cei mai importanți:

| Simptom | Ce modifici |
|---|---|
| cade de pe dohyo când dă înapoi (dohyo-ul are doar 38,5 cm) | scade `EDGE_BACK_MS` (150 → 100) |
| după retragere nu se întoarce destul (tinta: ~120–150°) / se întoarce prea mult | crește / scade `EDGE_TURN_MS` |
| se rotește și nu „prinde” adversarul (trece de el) | scade `SPEED_SEARCH_SPIN` (150 → 120) și `SPEED_TURN` |
| oscilează stânga-dreapta când adversarul e în față | crește `SPEED_ATTACK_ARC` (150 → 190) |
| detectează linia aiurea pe zgârieturi | calibrează; sau `LINE_CONFIRM_SAMPLES 3` |
| nu vede linia la viteză mare | calibrează (pragul urcă); `LINE_CONFIRM_SAMPLES 1` |
| motoarele „fluieră” | `PWM_HIGH_FREQ 1` (testează că driverul nu se încălzește) |
| motoarele se încălzesc tare la teste lungi | `SPEED_ATTACK` 255 → 220 |
| vrei start imediat la butoane (fără 5 s) | `START_DELAY_MS 0` |

`EDGE_PUSH_OVERRIDE_MS` lasă-l 0: la margine senzorii văd obiecte din afara dohyo-ului (oameni, masa) și robotul s-ar arunca după ele.

Procedura de reglaj: schimbă un singur parametru o dată, reîncarcă, testează de 3–4 ori pe dohyo.

---

## 7. Checklist ziua concursului

- [ ] Baterie încărcată (plus una de rezervă, încărcată).
- [ ] Modul de start montat, `USE_START_MODULE 1`, pinul corect; LED-ul verde stins la pornire.
- [ ] Cântar: sub 100 g cu lamă și modul; cub 5 × 5 × 5 cm.
- [ ] Test senzori din mână (LED1) și linie pe dohyo-ul oficial (LED2), apoi **calibrare pe dohyo-ul oficial**.
- [ ] Roți curățate cu alcool înainte de fiecare meci.
- [ ] Șuruburi strânse, cabluri fixate (nu stau în calea roților, nu se pot smulge la impact).
- [ ] Laptop + cablu micro-USB la tine, cu Arduino IDE și acest cod, pentru reglaje rapide.
- [ ] Știi tacticile: STÂNGA / DREAPTA / ambele = DREPT.

---

## 8. Depanare

| Problemă | Cauze probabile |
|---|---|
| nu apare portul în Arduino IDE | cablu doar de încărcare (fără date); alt cablu; reconectează; vezi 3. |
| robotul nu pornește la butoane | ești în mod TEST (ai ținut STÂNGA la pornire) – repornește; sau `USE_START_MODULE 1` fără modul (atunci ține apăsat 1,5 s) |
| pornește singur / se oprește singur | `USE_START_MODULE 1` cu pinul Start neconectat (plutește) – pune 0 sau conectează modulul; conectori slăbiți la butoane |
| merge invers / fuge de adversar | testul de motoare (4.3) și `MOTOR_*`; senzorii laterali inversați |
| se rotește la nesfârșit | nu vede adversarul: senzori deconectați (LED1 stins când pui mâna) – verifică cablurile |
| „vede” tot timpul ceva | obiecte la < 80 cm, un senzor mort (stă pe 1); testează în aer, departe de obiecte |
| cade de pe dohyo | senzorul de linie prea sus / murdar / necalibrat; LED2 nu se aprinde pe alb |
| LED-urile par inversate față de ghid | la unele plăci LED1/LED2 sunt desenate invers; nu afectează funcționarea |

---

## 9. Surse

- Regulament Micro Sumo RoboChallenge: https://robochallenge.ro/regulation/micro-sumo
- FAQ RoboChallenge (modul de start, magazin partener): https://robochallenge.ro/faq
- Standardul modulului de start: https://p1r.se/startmodule (implementare: https://p1r.se/startmodule/implement-yourself/)
- Codul oficial JSumo XS1 și explicații: https://blog.jsumo.com/xs1-micro-sumo-arduino-code-explanation-of-the-robot-code/
- Kit XS1: https://jsumo.com/xs1-micro-sumo-robot-kit · XMotion Micro: https://www.jsumo.com/xmotion-micro-all-in-one-controller
- Senzor JS40F: https://www.jsumo.com/js40f-digital-distance-sensor-min-40-cm-range
