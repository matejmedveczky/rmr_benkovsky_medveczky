# Úloha 3

## Prehľad

Cieľom úlohy bolo rozšíriť riešenie z Úlohy 1 o schopnosť mapovania prostredia. Robot pri pohybe buduje 2D mapu okolia pomocou LIDARu a odometricky odhadovanej polohy. Výsledná mapa sa ukladá do súboru.

---

## Použité algoritmy

### 1. Odometria (prevzatá z Úlohy 1)

Poloha robota je odhadovaná rovnakým spôsobom ako v Úlohe 1 - fúziou enkodérov kolies a gyroskopu. Navigácia k žiadanej polohe je zabezpečená P-regulátorom. Oproti Úlohe 1 je pohyb spomalený (obmedzenie rate limitera) kvôli kvalitnejšiemu mapovaniu. Navyše sa ukladá história pol;h do fronty `poseHistory` (200 záznamov) pre potreby časovej synchronizácie s LIDARom.

---

### 2. Occupancy Grid (binárna mriežková mapa)

Mapa prostredia je reprezentovaná dvojrozmerným poľom `grid[GRID_SIZE][GRID_SIZE]`. Každá bunka nadobúda jednu z troch hodnôt:

| Hodnota | Stav |
|---------|------|
| `UNKNOWN (0)` | Neprebádaná bunka |
| `FREE (1)` | Bunka je voľná (lúč LIDARu cez ňu prešiel)
| `OCCUPIED (2)` | Bunka je obsadená (zásah LIDARu) |

Veľkosť bunky je `CELL_SIZE` metrov. Počiatok mapy je definovaný offsetmi `gridOriginX`, `gridOriginY`.

**Konverzia súradníc:**  
Svetové súradnice `(wx, wy)` sa prevádzajú na indexy mriežky:

```
col = floor((wx - gridOriginX) / CELL_SIZE)
row = floor((wy - gridOriginY) / CELL_SIZE)
```

---

### 3. Časová interpolácia (Pose Interpolation)

LIDAR a odometria bežia asynchrónne na rôznych frekvenciách. Každý LIDAR bod má vlastný `timestamp`. Aby sa lúč premietal zo správnej polohy robota, používa sa lineárna interpolácia v histórii póz:

```
ratio = (ts - a.timestamp) / (b.timestamp - a.timestamp)
x_interp = a.x + ratio * (b.x - a.x)
y_interp = a.y + ratio * (b.y - a.y)
fi_interp = a.fi + ratio * delta_fi    // delta_fi normalizovaný do (-pi, pi)
```

Ak je timestamp mimo rozsahu, použije sa najbližší krajný záznam.

---

### 4. Aktualizácia mapy - Bresenhamov algoritmus

Pre každý platný bod LIDARu sa aktualizuje mapa v dvoch krokoch:

**a) Výpočet súradníc zásahu:**  
Z interpolovanej pózy robota a meraného uhla/vzdialenosti lúča sa vypočíta globálna poloha odrazu:

```
global_angle = pose.fi - (scanAngle * pi / 180)
x_hit = pose.x + dist_m * cos(global_angle)
y_hit = pose.y + dist_m * sin(global_angle)
```

**b) Bresenhamov algoritmus lúča:**  
Medzi polohou robota a bodom zásahu sa všetky bunky pozdĺž lúča označia ako `FREE`. Používa sa klasická implementácia Bresenhamovho algoritmu:

```
Vstup: (col_robot, row_robot) -> (col_hit, row_hit)
Výstup: všetky bunky na trase -> FREE (ak nie sú OCCUPIED)
```

Algoritmus krokuje v 8-rozmernom priestore (počíta s dx, dy) bez použitia delenia.

**c) Označenie bodu zásahu:**  
Po trasovaní lúča sa bunka `(col_hit, row_hit)` nastaví na `OCCUPIED`.

**Filtrovanie meraní:**  
Pred spracovaním sa filtrujú nevhodné hodnoty:
- `dist < 0.05 m` - príliš blízko (nepresné meranie)
- `dist > 2.5 m` - nad spolohlivým dosahom senzora
- `0.5 m < dist < 0.7 m` - pásmo šumu LIDARu)

---

### 5. Ukladanie a vizualizácia mapy

Mapa sa ukladá do textového súboru ako matica čísel oddelených medzerami (jeden riadok = jeden riadok mriežky). Python skript `vismap.py` slúži na vizualizáciu uloženej mapy.

Po každej aktualizácii sa mapa emituje cez Qt signál `publishMap` na zobrazenie v GUI.

---

## Poznámky k implementácii

- Filtrované pásmo `(0.5, 0.7) m` je špecifické pre konkrétny LIDAR
- História polôh je obmedzená na 200 záznamov - pri nižšej rýchlosti robota pokrýva dlhší časový úsek
- Mapa sa neukladá automaticky - uloženie sa spúšťa z GUI
