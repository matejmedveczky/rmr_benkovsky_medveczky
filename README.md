# Úloha 1
---

## Prehľad

Cieľom úlohy bolo implementovať odhad polohy mobilného robota Kobuki pomocou odometrie a zabezpečiť schopnosť dosiahúť žiadanú polohu

---

## Použité algoritmy

### 1. Odometria - odhad polohy

Poloha robota je odhadovaná fúziou dvoch zdrojov: enkodérov kolies a gyroskopu. Každý príchod dát z robota spustí prepočet.

**Spracovanie enkodérov:**  
Z rozdielov hodnôt ľavého a pravého enkodéra sa vypočíta prejdená vzdialenosť každého kolesa. Keďže enkodéry sú 16-bitové s pretečením, implementuje sa preto korekcia:

```
delta_left  > +32767  →  delta_left  -= 65536
delta_left  < -32767  →  delta_left  += 65536
```

Prejdená vzdialenosť sa vypočíta ako: `distance = tick_to_meter * delta_encoder`

**Odhad uhla natočenia:**  
Uhol natočenia `fi` sa aktualizuje z gyroskopu. Nameraný prírastok uhla `delta_deg` sa konvertuje na radiány a pripočíta k uhlu. Výsledok sa normalizuje do rozsahu `(-pi, pi)`.

**Aktualizácia polohy:**  
Poloha `(x, y)` sa aktualizuje kinematickým modelom diferenciálneho podvozku:

```
step_dist = (left_distance + right_distance) / 2
x += step_dist * cos(fi)
y += step_dist * sin(fi)
```

---

### 2. Proporcionálny regulátor navigácie (P-regulátor)

Pre navigáciu k požadovanej polohe `(x_des, y_des)` je implementovaný P-regulátor.

**Výpočet chýb:**

```
err_lin = sqrt((x_des - x)^2 + (y_des - y)^2)    // lineárna odchylka [m]
err_ang = atan2(y_des - y, x_des - x) - fi         // uhlová odchylka [rad]
```

Uhlová chyba sa normalizuje do `(-pi, pi)`.

**Riadiaca logika:**

Regulátor pracuje v dvoch režimoch podľa uhlovej odchýlky:

- Ak `|err_ang| > 0.9 rad`: robot sa otáča na mieste (`v = 0`, `w = Kp_ang * err_ang`)
- Inak robot kombinuje uhlovú a lineárnu rýchlosŤ (`v = Kp_lin * err_lin * 1000`, `w = Kp_ang * err_ang`)

Ak `err_lin < 0.02 m`: robot sa zastaví.

**Parametre:**

| Parameter | Hodnota |
|-----------|---------|
| `Kp_lin` | 0.3 |
| `Kp_ang` | 1.2 |

---

### 3. Obmedzenie rýchlosti zmeny

Aby sa predišlo skokovým zmenám príkazov rýchlosti, je implementovaný jednoduchý rate limiter:

```
if (v - prev_v) > 5:     v = prev_v + 5      // max prírastok 5 mm/s za tick
if |w - prev_w| > 0.3:   w = prev_w ± 0.3    // max prírastok 0.3 rad/s za tick
```

Finálne hodnoty sú saturované: `v: [-400, 400] mm/s`, `w: [-2.0, 2.0] rad/s`.
