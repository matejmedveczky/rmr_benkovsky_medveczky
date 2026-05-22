# RMR – navigácia robota pomocou VFH+

Tento projekt rieši autonómnu navigáciu mobilného robota z aktuálnej polohy do zadaného cieľa so súčasným vyhýbaním sa prekážkam. Robot využíva údaje z odometrie, gyroskopu a laserového diaľkomeru. Základom riešenia je metóda **VFH+ (Vector Field Histogram Plus)**, ktorá rozdeľuje okolie robota na uhlové sektory, vyhodnocuje ich obsadenosť a vyberá najvhodnejší smer pohybu.

## Princíp riešenia

Robot si priebežne udržiava svoju približnú polohu `x`, `y` a natočenie `fi`. Poloha sa aktualizuje vo funkcii `processThisRobot()` z údajov enkóderov kolies a gyroskopu. Na základe rozdielu medzi aktuálnou polohou a cieľom sa vypočíta vzdialenosť k cieľu a smer k cieľu v lokálnych súradniciach robota.

Údaje z lidaru sa spracúvajú vo funkcii `processThisLidar()`. Keďže laserový diaľkomer pracuje v opačnej orientácii, uhly sa najskôr prepočítajú pomocou funkcie `normalizeLaserAngle()`. Vzdialenosti zo senzora sa zároveň prevádzajú z milimetrov na metre. Ak je cesta k cieľu voľná, robot ide priamo na cieľ. Ak sa v smere jazdy nachádza prekážka, použije sa VFH+ a vytvorí sa dočasný waypoint.

## Histogram prekážok

Okolie robota je rozdelené na `36` sektorov, teda každý sektor predstavuje `10°`. Vo funkcii `updatePrimaryHistogram()` sa pre každý bod z lidaru vypočíta, do ktorých sektorov môže prekážka zasahovať. Zohľadňuje sa pritom bezpečnostný polomer robota `robotSafetyRadius`, aby robot neprechádzal príliš blízko popri prekážkach. Blízke prekážky majú väčšiu váhu ako vzdialenejšie prekážky.

Následne sa primárny histogram prevedie na binárny histogram vo funkcii `updateBinaryHistogram()`. Ak je hodnota sektora väčšia alebo rovná prahu `histogramThreshold`, sektor sa označí ako obsadený. V opačnom prípade je považovaný za voľný. V aktuálnej implementácii je maskovaný histogram rovnaký ako binárny histogram, pretože funkcia `updateMaskedHistogram()` iba kopíruje hodnoty z binárneho histogramu.

## Výber smeru pohybu

Po vytvorení histogramu sa vo funkcii `findFreeGaps()` hľadajú súvislé voľné oblasti sektorov. Z týchto oblastí sa následne vytvoria kandidátske smery. Pri úzkych priechodoch sa vyberá stred voľnej oblasti, pri širších priechodoch sa vyberajú smery bližšie ku krajom priechodu. Ak cieľ leží vo voľnom sektore, pridá sa ako ďalší kandidát.

Najlepší sektor sa vyberá vo funkcii `chooseBestSector()`. Každý kandidát je ohodnotený cenovou funkciou `candidateCost()`, ktorá berie do úvahy tri kritériá:

* blízkosť kandidáta k cieľovému smeru,
* blízkosť kandidáta k priamemu smeru robota,
* podobnosť s predchádzajúcim vybraným sektorom.

Tým sa robot snaží ísť čo najviac smerom k cieľu, ale zároveň sa obmedzuje prudké prepínanie smerov. Po výbere sektora sa vo funkcii `updateWaypointFromSector()` vytvorí dočasný cieľ vo vzdialenosti `0.25 m` pred robotom v smere vybraného sektora.

## Priama jazda na cieľ

Ak je cieľový sektor voľný a laser v smere cieľa nevidí prekážku pred cieľom, robot nepoužije náhradný waypoint, ale ide priamo na cieľ. Táto logika je vo funkcii `updateVFHNavigation()`. Pre prípad, že je robot už blízko cieľa, sa používa aj kontrola `canIgnoreVFHNearGoal()`, ktorá umožní dokončiť pohyb priamo k cieľu, ak je medzi robotom a cieľom ešte dostatočne voľný priestor.

## Riadenie robota

Samotný pohyb na aktuálny waypoint alebo cieľ rieši regulátor vo funkcii `processThisRobot()`. Najskôr sa vypočíta lineárna a uhlová chyba. Ak je robot natočený príliš mimo smeru jazdy, najskôr sa otáča na mieste. Ak je natočenie prijateľné, začne sa pohybovať dopredu. Rýchlosť sa obmedzuje, aby sa robot nerozbiehal skokovo a pohyb bol plynulejší.

## Grafické rozhranie

V `MainWindow` sa zobrazujú dáta z lidaru a histogram. Sektory sú vykreslené farebne:

* červená farba označuje obsadené sektory,
* žltá farba označuje maskované sektory,
* zelená farba označuje voľné sektory.

Cieľ sa zadáva cez textové polia `x\_des` a `y\_des`. Po stlačení tlačidla pokračovania sa zavolá `setGoal()`, nastaví sa nový cieľ a robot začne navigovať podľa aktuálnych dát zo senzorov.

## Zhodnotenie

Implementácia umožňuje robotovi pohybovať sa k zadanému cieľu a pri výskyte prekážky vybrať náhradný smer na základe histogramu. Výhodou riešenia je jednoduché rozdelenie priestoru na sektory a priebežné prepočítavanie smeru podľa aktuálnych dát z lidaru. Obmedzením aktuálnej verzie je jednoduché maskovanie, ktoré zatiaľ nekontroluje dynamické možnosti robota detailnejšie, ale iba preberá binárny histogram. Napriek tomu riešenie pokrýva základnú funkcionalitu VFH+ navigácie a umožňuje vizuálne sledovať rozhodovanie robota v GUI.

