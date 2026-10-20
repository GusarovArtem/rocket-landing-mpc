# Założenia modelu fizycznego

Ten plik opisuje decyzje modelowe przyjęte na początku projektu, zgodnie z
`thesis/plan_original.md` (sekcja 2). Każda zmiana tych założeń w trakcie projektu
powinna być tutaj odnotowana, żeby rozdział metodologii w pracy dyplomowej był
zgodny z tym, co faktycznie zaimplementowano.

## Układ współrzędnych

Lokalny układ inercjalny, lądowisko w początku układu:

- `x` — wschód
- `y` — północ
- `z` — pionowo w górę
- grawitacja działa wzdłuż ujemnej osi `z`

## Wektor stanu

```
state = [x, y, z, vx, vy, vz, m]
```

7 liczb: pozycja (3), prędkość (3), aktualna masa (1).

## Wektor sterowania

```
control = [thrust, alpha, beta]
```

- `thrust` — wartość ciągu silnika [N]
- `alpha`, `beta` — kąty określające kierunek wektora ciągu względem osi `z`

Jednostkowy wektor kierunku ciągu:

```
d(alpha, beta) = [sin(alpha) * cos(beta), sin(beta), cos(alpha) * cos(beta)]
```

(ma długość 1 dla każdych `alpha`, `beta` — to jest matematyczna właściwość tej
parametryzacji, warto to sprawdzić testem).

## Co jest UWZGLĘDNIONE w pierwszej wersji modelu

- Grawitacja, stała, wzdłuż `-z`.
- Zmienna masa (zużycie paliwa proporcjonalne do ciągu, przez impuls właściwy).
- Ograniczony ciąg (`Tmin <= T <= Tmax`).
- Ograniczony kąt odchylenia wektora ciągu od pionu.
- Trójwymiarowy ruch środka masy.

## Co jest JAWNIE WYŁĄCZONE / UPROSZCZONE w pierwszej wersji

- **Brak oporu aerodynamicznego** — `F_D = 0`. Zostanie dodany później jako
  rozszerzenie (opcjonalne), jeśli zostanie czas.
- **Brak dynamiki orientacji (6-DoF)** — korpus rakiety jest umownie "ustawiony",
  nie obraca się; kierunek ciągu jest sterowany bezpośrednio jako wektor jednostkowy,
  bez modelowania silników sterujących, momentów bezwładności czy kwaternionów.
- **Brak dynamiki cieczy w zbiornikach, elastyczności korpusu, turbomaszyn silnika.**

Te uproszczenia są świadome i opisane w `thesis/plan_original.md` (sekcja 1,
"Czego nie trzeba uwzględniać w pierwszej wersji").

## Parametry pojazdu

Na początku parametry edukacyjnego modelu rakiety będą zwykłymi polami w klasie
Python (`src/rocket_landing/params.py`), nie plikiem YAML — to prostsze dla kogoś,
kto uczy się Pythona. Migracja do `configs/*.yaml` jest zaplanowana w Fazie E
(eksperymenty), gdy potrzebne będzie łatwe przełączanie scenariuszy.
