# Sledovanie trajektórie II
Porovnanie Stanleyho algoritmu (kinematický a dynamický model) a Pure Pursuit pri sledovaní trajektórie.

## Výsledky

### Vplyv zisku ($k$) na kinematický model Stanley
| $k$ | $\text{RMSE}$ [m] | $\text{Max}$ [m] |
| :--- | :--- | :--- |
| 2.5 | 0.5926 | 1.3574 |
| 4 | 0.51785 | 1.1009 |
| 6 | 0.43447 | 0.93881 |
| 8 | 0.39166 | 0.8533 |
| 12 | 0.34776 | 0.76874 |
| 16 | 0.32514 | 0.72757 |
| 18 | 0.31737 | 0.71346 |
| 20 | 0.31111 | 0.70285 |

**Najlepšie $k = 20$**

---

### Porovnanie jednotlivých algoritmov
| Algoritmus | $\text{RMSE}$ [m] | $\text{Max}$ [m] |
| :--- | :--- | :--- |
| **Stanley kin** | 0.65926 | 1.3574 |
| **Stanley dyn** | 0.40181 | 0.91435 |
| **Pure Pursuit** | 0.054887 | 0.13688 |
| **Stanley kin (vylepšený)** | 0.31111 | 0.70285 |

> **Zistenie:** Vylepšením zisku kleslo RMSE kinematického Stanleyho o **52,8 %**. Podrobnosti sú uvedené v dokumentácii `Zadanie03_AMS_Kyrylova.pdf`[cite: 1].

---

### Porovnanie trajektórií
<p align="center">
  <img src="https://github.com/user-attachments/assets/49f332cb-1c55-468f-8f49-f9fe7ddbfb74" alt="Porovnanie Stanley a Pure Pursuit" width="700">
</p>

### Maximálne odchýlky v bodoch:
* **Stanley kin:** $\text{max} = 1.357\text{ m}$ v bode $x = 63.8$, $y = -45.8$[cite: 1]
* **Stanley dyn:** $\text{max} = 0.914\text{ m}$ v bode $x = 64.0$, $y = -45.4$[cite: 1]
* **Pure Pursuit:** $\text{max} = 0.137\text{ m}$ v bode $x = 58.5$, $y = -43.7$[cite: 1]
* **Stanley kin (vylepšený):** $\text{max} = 0.703\text{ m}$ v bode $x = 61.1$, $y = -44.9$[cite: 1]

---

## Súbory v repozitári

- `Zadanie03_AMS_Kyrylova.pdf`: Kompletná dokumentácia[cite: 1]
- `stanleySimple_kin2.slx`: Stanley – kinematický model[cite: 1]
- `stanleySimple2.slx`: Stanley – dynamický model[cite: 1]
- `zadanie2.slx`: Pure Pursuit[cite: 1]
- `setUpModel.mlx`: Príprava trate a referencie[cite: 1]
- `cvico3_zlepsenie.mlx`: Simulácie, vyhodnotenie a ladenie zisku[cite: 1]


