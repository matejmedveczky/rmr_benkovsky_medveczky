# Úloha 4

## Prehľad

Cieľom úlohy bolo implementovať autonómnu navigáciu robota z ľubovoľnej polohy do žiadanej cieľovej polohy s využitím vopred vytvorenej mapy z Úlohy 3.

---

## Použité algoritmy

### 1. Načítanie a bufferovanie mapy

**Načítanie:**  
Mapa uložená v Úlohe 3 sa načíta zo súboru. Bunky s hodnotou `> 3` sú pri načítaní nastavené na `FREE` 

**Bufferovanie prekážok:**  
Pre každú bunku označenú ako `OCCUPIED` sa rozšíri nebezpečná zóna o `BUFFER_SIZE` buniek vo všetkých 8 smeroch. Bunky v tejto zóne dostanú stav `BUFFER`. Cieľom je zaistiť bezpečnú vzdialenosť robota od prekážok pri plánovaní cesty.

```
Pre každú OCCUPIED bunku (r, c):
    Pre každý z 8 smerov, kroky 1..BUFFER_SIZE:
        Ak bunka je FREE -> nastaviť na BUFFER
        Ak bunka je OCCUPIED -> zastaviť rozširovanie v tomto smere
```

---

### 2. Generácia nákladovej mapy

Z cieľovej polohy sa propaguje mapa do všetkých dosiahnuteľných buniek pomocou Dijkstrovho algoritmu s prioritnou frontou (`priority_queue`).

**Postup:**

1. Cieľová bunka dostane počiatočnú hodnotu `4` (rezerva nad stavovými hodnotami `0-3`)
2. Zo štartovacej bunky sa expandujú susedia vo všetkých 8 smeroch
3. Cena pohybu: `10` pre kardinálne smery (sever, juh, východ, západ), `14` pre diagonály (aproximácia √2 × 10)
4. Susedná bunka sa zaradí do fronty len ak má stav `FREE`
5. Algoritmus sa zastaví hneď po dosiahnutí štartovacej bunky

Výsledkom je mapa, kde hodnota každej bunky reprezentuje najkratšiu cenu cesty z tejto bunky do cieľa.

---

### 3. Extrakcia cesty

Z nákladovej mapy sa extrahuje cesta greedy algoritmom - v každom kroku sa vyberie sused s najnižšou efektívnou cenou.

**Penalizácia zatáčania:**  
Aby sa uprednostnila priama cesta pred cik-cak pohybom, pridáva sa penalizácia pri zmene smeru pohybu:

```
is_turn = (dx[i] != cur_dx || dy[i] != cur_dy)
turn_penalty = is_turn ? 3 : 0
effective_cost = grid[ny][nx] + turn_penalty
```

**Zjednodušenie cesty:**  
Z detailnej mriežkovej cesty sa extrahujú len body, kde sa mení smer pohybu (zlomové body). Tým sa znižuje počet waypointov a eliminujú sa zbytočné medzizastávky.

```
Pre každý bod i v grid_path:
    ak smer(i-1 -> i) ≠ smer(i -> i+1):
        pridaj bod i do world_path
```

Prvý a posledný bod sú vždy zahrnuté.

---

### 4. Navigácia

Robot postupuje po sérii waypointov generovaných v kroku 3. Pre každý waypoint sa používa rovnaký P-regulátor ako v Úlohe 1.

**Variabilná tolerancia dosaženia:**

- Medzibody cesty: tolerancia `0.15 m` (väčšia, robustnejší prechod)
- Posledný bod (cieľ): tolerancia `0.02 m`

Po dosiahnutí medzibodu sa automaticky nastaví nasledujúci waypoint.

---

## Poznámky k implementácii

- Wavefront hodnoty začínajú od `4` aby nekolidovali so stavovými hodnotami `0` (UNKNOWN), `1` (FREE), `2` (OCCUPIED), `3` (BUFFER)
- Pred každým novým plánovaním cesty sa wavefront hodnoty z mapy vymažú (`> BUFFER = FREE`) a mapa sa znova zabuferuje
- Maximálna dĺžka hľadania cesty je obmedzená na `GRID_SIZE*GRID_SIZE` krokov pred vyhlásením chyby