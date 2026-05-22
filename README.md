# Úloha 5
---

## Prehľad

Cieľom úlohy bolo implementovať globálnu lokalizáciu robota na vopred vytvorenej mape pomocou algoritmu Monte Carlo (MCL). Robot sa inicializuje bez znalosti svojej polohy, pomocou MCL konverguje k odhadu polohy a následne sa autonómne naviguje domov. Navigačná logika z Úlohy 4 bola refaktorovaná do samostatnej triedy `Map`.

---

## Architektúra

Riešenie je rozdelené do troch tried:

- `Map` - načítanie/ukladanie mapy, bufferovanie, extrakcia cesty (z Úlohy 4)
- `MCL` - logika MCL
- `robot` - odometria, riadenie, prepínanie medzi fázami MCL a navigácie

---

## Použité algoritmy

### 1. Monte Carlo Lokalizácia

MCL je implementácia lokalizácie pomocou reprezentácie pravdepodobnostného rozdelenia. Každá častica reprezentuje jednu predpokladanu polohu robota.

**Inicializácia:**  
N = 300 častíc sa rozptýli náhodne po všetkých `FREE` bunkách mapy s náhodným uhlom `fi:(-pi, pi)`.

---

#### 1a. Pohybový model (Motion Update)

Každá častica sa posunie podľa `odometrie robota + gaussovský šum`. Šum je úmerný veľkosti pohybu aj otočenia:

```
noisy_fi   = delta_fi   + (A1*|delta_fi| + A2*|step_dist|) * gauss(0,1)
noisy_dist = step_dist  + (A3*|step_dist| + A4*|delta_fi|) * gauss(0,1)

p.fi += noisy_fi
p.x  += noisy_dist * cos(p.fi)
p.y  += noisy_dist * sin(p.fi)
```

Častica, ktorá sa po pohybe ocitne mimo mapy alebo v `OCCUPIED`/`BUFFER` bunke, dostane váhu `1e-9`.

---

#### 1b. Senzorový model

Pre hodnotenie zhody každej častice s meraním LIDARu sa používa predpočítané pole vzdialeností od prekážok (`dist_field`).

**Predpočet distance field:**  
Záplavový algoritmus z každej `OCCUPIED` bunky postupuje do všetkých susedov. Výsledkom je mapa, kde hodnota bunky udáva vzdialenosť (v bunkách) k najbližšej prekážke.

**Váhovanie:**  
Pre každú 10. vzorku LIDARu sa vypočíta, kde by bol zásah keby robot stál v polohe danej časticou:

```
angle = p.fi - (scanAngle * pi / 180)
hit_x = p.x + dist_m * cos(angle)
hit_y = p.y + dist_m * sin(angle)
error += dist_field[row][col] * CELL_SIZE
```

Priemerná chyba sa prevedie na váhu: `weight = 1 / (1 + mean_error)`

Váhy sa následne normalizujú.

---

#### 1c. Prevzorkovanie

Používa sa systematické prevzorkovanie (nižší rozptyl ako multinomické):

1. Vypočíta sa distribúcia váh
2. Náhodne sa zvolí štartovací bod `u: [0, 1/N]`
3. N vzoriek sa vyberie v pravidelných intervaloch `u, u+1/N, u+2/N, ...`

**Adaptívne vloženie náhodných častíc:**  
Podiel náhodne injektovaných častíc závisí od rozptylu najlepšej polovice populácie:

```
inject_ratio = clamp(variance / VAR_HIGH, 0.01, 0.05)
n_random = max(1, N * inject_ratio)
n_resample = N - n_random
```

Tým sa zabezpečuje diverzita pri globálnom hľadaní a súčasne efektívne sústredenie pri konvergencii.

---

#### 1d. Detekcia konvergencie a odhad polohy

Rozptyl sa vypočítava len z najlepšej polovice populácie (robustnosť voči outlierom):

```
Zoraď časticice podľa váhy zostupne
Vypočítaj strednú polohu top 50%
Variance = priemer kvadratických odchýlok od strednej polohy
```

| Prahová hodnota | Konštanta |
|-----------------|-----------|
| Konvergovaný | `VAR_CONVERGED = 0.01` |

Odhad polohy je poloha najvyššie váženej časticice (nie vážená stredná hodnota), čo je robustnejšie pri multimodálnom rozdelení.

Správnosť odhadu je potvrdená ak má najlepšia častica aspoň 5 susedov do vzdialenosti 0.5 m.

---

### 2. Fázové prepínanie: MCL vs. Navigácia

Robot prechádza dvomi fázami:

**Fáza 1 - MCL aktívny:**  
Robot sa pohybuje reaktívne (bez plánovanej cesty) na základe LIDARu:
- Ak `min_front < 0.7 m`: zastavenie a otočenie smerom k voľnejšiemu priestoru
- Inak: pomalý pohyb dopredu s koreláciou uhla podľa rozdielov vzdialeností vľavo/vpravo

**Fáza 2 - MCL konvergoval:**  
Po dosiahnutí `MCL_CONVERGE_REQUIRED` po sebe idúcich tickov s `variance < VAR_CONVERGED`:
- MCL sa deaktivuje
- Poloha robota sa nastaví na odhad MCL
- Spustí sa navigácia domov cez Dijkstrov algoritmus + waypoint navigáciu (identické s Úlohou 4)

**Detekcia kolízie pri navigácii:**  
Ak je odometricky odhadovaná poloha blízko steny (`distField < 4`) a zároveň LIDAR detekuje viac ako 5 bodov bližšie ako 25 cm vpredu, MCL sa reštartuje (reinit) kvôli odhadu polohy po kolízii.

---
