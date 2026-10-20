# Oryginalny opis pracy dyplomowej (zachowany do odniesienia)

> Ten plik zawiera pełny, pierwotny opis pracy dyplomowej dostarczony na początku
> projektu. Jest trzymany tutaj jako punkt odniesienia przy pisaniu rozdziałów pracy
> (metodologia, model matematyczny, MPC, eksperymenty). Faktyczna sekwencja realizacji
> projektu — bez podziału na tygodnie, jako płaska lista kroków — znajduje się w
> `PROGRESS.md` w katalogu głównym repozytorium.

## Temat pracy dyplomowej

Autonomiczne lądowanie rakiety i sterowanie trajektorią

Opracowanie algorytmu sterującego trajektorią rakiety podczas lądowania, z
uwzględnieniem ograniczeń dotyczących ciągu silnika, prędkości i położenia.

Czego się nauczę: sterowania predykcyjnego (Model Predictive Control, MPC),
optymalizacji trajektorii, dynamiki lotu oraz symulacji numerycznych.

## Plan pracy dyplomowej na 4 miesiące

### Autonomiczne lądowanie rakiety z wykorzystaniem sterowania MPC

Celem jest stworzenie nie tylko animacji lądowania rakiety, lecz także projektu
inżynierskiego z dziedziny naprowadzania, nawigacji i sterowania (Guidance,
Navigation and Control, GNC). Projekt obejmuje trójwymiarowy model powracającego
pierwszego stopnia rakiety, autonomiczny regulator MPC, ograniczenia dotyczące
ciągu i dynamiki, powtarzalne eksperymenty oraz porównanie z regulatorem bazowym.

Jest to dobry kierunek przygotowania do pracy w zespołach zajmujących się
sterowaniem lotem, nawigacją, optymalizacją trajektorii i lądowaniem statków
kosmicznych. Aby projekt można było ukończyć w ciągu czterech miesięcy, zaleca się
najpierw opracować uproszczony, ale fizycznie spójny model 3D z ustaloną orientacją
korpusu. Następnie, jeśli pozostanie czas, można dodać sterowanie orientacją.

### Główne zadanie

Modelowanie 3D · MPC · Ograniczenia · Eksperymenty numeryczne

Opracowanie regulatora, który kieruje powracającym stopniem rakiety w stronę
lądowiska, zmniejsza prędkość pionową i poziomą przed przyziemieniem oraz
przestrzega ograniczeń układu wykonawczego.

Czas realizacji: 16 tygodni, czyli cztery etapy po miesiącu.
Język programowania: Python.
Biblioteki i narzędzia: NumPy, SciPy, CasADi, IPOPT.
Model 3D: położenie, prędkość, masa i ciąg.
Porównanie: PID kontra MPC oraz testy odporności.

## 1. Określenie zakresu projektu

Proponowany tytuł pracy dyplomowej:

Autonomiczne lądowanie rakiety z wykorzystaniem sterowania predykcyjnego:
trójwymiarowa optymalizacja trajektorii przy ograniczeniach stanu i elementów
wykonawczych

(Autonomous Rocket Landing Using Model Predictive Control: Three-Dimensional
Trajectory Optimization Under State and Actuator Constraints)

### Główne pytanie badawcze

W jakim stopniu MPC poprawia dokładność i jakość autonomicznego lądowania
powracającej rakiety w porównaniu z bazowym regulatorem PID, przy ograniczonym
ciągu, niepewności modelu i występowaniu zakłóceń?

### Zakres projektu

- Trójwymiarowa dynamika translacyjna rakiety.
- Grawitacja, zmienna masa i ograniczony ciąg silnika.
- Regulator MPC z ograniczonym horyzontem predykcji i ograniczeniami.
- Regulator PID jako algorytm bazowy do porównania.
- Modelowanie wiatru, błędu początkowej prędkości, zmian masy i błędów modelu.
- Metryki lądowania, analiza niepowodzeń i powtarzalne wykresy.

### Czego nie trzeba uwzględniać w pierwszej wersji

Nie należy od razu rozpoczynać od pełnego modelowania aerodynamiki, elastycznego
korpusu, turbomaszyn, silnika, dynamiki cieczy w zbiornikach ani kompletnego
sterowania orientacją w sześciu stopniach swobody (6-DoF). Są to odrębne, złożone
zagadnienia.

W pierwszej wersji „model 3D” oznacza trójwymienny ruch środka masy. Korpus rakiety
może być umownie ustawiony pionowo, a kierunek ciągu może być sterowany w granicach
dopuszczalnego stożka odchylenia. Pozwala to badać lądowanie przestrzenne bez
konieczności jednoczesnego rozwiązywania pełnego problemu orientacji.

Orientację i dynamikę kątową można dodać później jako rozszerzenie projektu.

## 2. Sformułowanie matematyczne

### 2.1. Układ współrzędnych i stan rakiety

Należy wybrać lokalny inercjalny układ współrzędnych:

- oś x — wschód;
- oś y — północ;
- oś z — kierunek pionowo w górę;
- lądowisko znajduje się w początku układu współrzędnych;
- grawitacja działa wzdłuż ujemnej osi z.

Stan rakiety: `[x, y, z, vx, vy, vz, m]^T`, gdzie `r` oznacza położenie, `v` —
prędkość, a `m` — aktualną masę.

Sterowanie: `u = [T, alpha, beta]^T`, gdzie `T` oznacza ciąg silnika, natomiast
`alpha` i `beta` są parametrami określającymi kierunek ciągu. Można ich użyć do
zdefiniowania jednostkowego wektora kierunku działania silnika:

```
d(alpha, beta) = [ sin(alpha)*cos(beta), sin(beta), cos(alpha)*cos(beta) ]
```

Przy takiej parametryzacji wektor ma długość jednostkową. Dla niewielkich kątów
odchylenia jest to wygodny model. Przed implementacją należy sprawdzić, czy wybrane
kąty odpowiadają fizycznemu rozmieszczeniu osi.

### 2.2. Równania ruchu

W pierwszej wersji należy zastosować dynamikę translacyjną ze zmienną masą:

```
dr/dt = v
dv/dt = (T/m) * d(alpha, beta) + g + F_D / m
dm/dt = -T / (Isp * g0)
```

gdzie:

- `T` — wartość ciągu silnika, w niutonach;
- `Isp` — impuls właściwy silnika, w sekundach;
- `g0` — standardowe przyspieszenie grawitacyjne;
- `g` — wektor przyspieszenia grawitacyjnego;
- `F_D` — siła oporu aerodynamicznego.

Na pierwszym etapie można przyjąć zerowy opór aerodynamiczny, ale założenie to
należy wyraźnie opisać w pracy dyplomowej. W drugim etapie można dodać uproszczony
model oporu:

```
F_D = -0.5 * rho * C_D * A * |v_rel| * v_rel
```

gdzie `rho` oznacza gęstość powietrza, `C_D` — współczynnik oporu, a `A` —
efektywną powierzchnię.

Ważne ograniczenie modelu: korpus nie obraca się w tych równaniach, a kierunek
wektora ciągu jest sterowany bezpośrednio. Jest to zatem trójwymiarowy model
trajektorii ze sterowanym wektorem ciągu, ale nie pełny model rakiety 6-DoF.

Pełny model wymaga zastosowania kwaternionów lub macierzy orientacji, prędkości
kątowych, momentów bezwładności oraz dynamiki siłowników sterujących wektorem
ciągu.

### 2.3. Ograniczenia

Regulator powinien wyznaczać nie tylko trajektorię prowadzącą do lądowiska, ale
również trajektorię dopuszczalną z punktu widzenia ograniczeń fizycznych.

Ograniczenia ciągu: `Tmin <= T_k <= Tmax`.

Jeśli w rozpatrywanej fazie lotu silnik nie może zostać wyłączony, należy
uwzględnić odpowiednie ograniczenie minimalnego ciągu. Jeśli wyłączenie silnika
jest dopuszczalne, trzeba modelować je osobno — nie wolno automatycznie zakładać,
że silnik może generować dowolny ciąg od zera do maksimum.

Ograniczenie odchylenia kierunku ciągu: `sqrt(dx^2 + dy^2) <= sin(theta_max)`,
gdzie `theta_max` oznacza maksymalny kąt odchylenia od pionu.

Ograniczenia stanu: `z_k >= 0`, `m_k >= m_dry`, gdzie `m_dry` oznacza masę własną
stopnia rakiety bez paliwa.

Warunki końcowe miękkiego lądowania: `|r_N - r_pad| <= epsilon_r`,
`|v_N| <= epsilon_v`. Wysokość powinna być bliska zeru, a odchylenie poziome musi
mieścić się w granicach promienia lądowiska.

Na początku należy wybrać parametry edukacyjnego modelu rakiety, a nie rzeczywistej
rakiety. Na przykład masa początkowa, masa własna, maksymalny ciąg i impuls
właściwy powinny być określone w pliku konfiguracyjnym. Ich wartości trzeba
sprawdzić pod kątem wiarygodności fizycznej. Model nie powinien wymagać
przyspieszenia, którego silnik nie jest w stanie zapewnić.

## 3. Sformułowanie zadania MPC

Model Predictive Control, czyli sterowanie predykcyjne, polega na tym, że na
każdym kroku algorytm:

1. Otrzymuje aktualne oszacowanie stanu.
2. Rozwiązuje zadanie optymalizacyjne na skończonym horyzoncie predykcji.
3. Stosuje wyłącznie pierwsze sterowanie z wyznaczonej sekwencji.
4. Po kolejnym kroku ponownie mierzy stan i oblicza optymalne sterowanie.

To właśnie ponowna optymalizacja odróżnia MPC od trajektorii obliczonej wcześniej
i realizowanej bez bieżącej korekty.

### 3.1. Funkcja celu

Załóżmy, że horyzont obejmuje `N` kroków, a czas trwania jednego kroku wynosi
`dt`. Zadanie optymalizacyjne ma postać `min_u J`.

Przykładowa funkcja kosztu:

```
J = sum_{k=0}^{N-1} [ ||r_k - r_ref,k||^2_Qr
                     + ||v_k - v_ref,k||^2_Qv
                     + ||u_k - u_ref,k||^2_R
                     + ||du_k||^2_S ]
  + ||r_N - r_pad||^2_Qf
  + ||v_N||^2_Pf
  + w_m * (m_0 - m_N)
```

Interpretacja składników:

- `Qr` — waga kary za odchylenie od pożądanej trajektorii;
- `Qv` — waga kary za nadmierną prędkość;
- `R` — waga określająca znaczenie wielkości sygnału sterującego;
- `S` — waga kary za gwałtowne zmiany sterowania;
- `Qf` i `Pf` — wagi określające znaczenie dokładności lądowania i małej
  prędkości przyziemienia;
- ostatni składnik zachęca do oszczędzania paliwa poprzez ograniczenie zużycia
  masy.

Jest to przykładowa funkcja kosztu, a nie jedyne poprawne rozwiązanie. Wagi należy
dobierać eksperymentalnie i dokumentować ich wartości.

Zbyt duża kara za zużycie paliwa może prowadzić do lądowania z dużą prędkością,
jeśli warunki końcowe są określone wyłącznie przez miękkie kary. Dlatego w zadaniu
lądowania należy stosować twarde ograniczenia lub weryfikowalne warunki końcowe,
zamiast polegać wyłącznie na wagach funkcji kosztu.

### 3.2. Dyskretyzacja dynamiki

Optymalizator wymaga dyskretnego modelu: `x_{k+1} = f_d(x_k, u_k)`.

W pracy dyplomowej zaleca się całkowanie metodą Rungego–Kutty czwartego rzędu
(RK4). Zazwyczaj zapewnia ona większą dokładność niż prosta jawna metoda Eulera
przy porównywalnym kroku całkowania.

Ważna kwestia inżynierska: należy używać tych samych równań fizycznych, ale
niezależnych implementacji integratora dla predykcji MPC i symulatora. Pozwala to
uniknąć sztucznie idealnych wyników wynikających z identycznych błędów numerycznych
w obu modelach.

### 3.3. Ustawienia pierwszego prototypu

| Parametr | Zalecana wartość początkowa |
| --- | --- |
| Krok symulacji | 0,02–0,05 s |
| Okres sterowania | 0,1–0,2 s |
| Horyzont predykcji | 20–40 kroków |
| Optymalizator | CasADi + IPOPT |

Są to wartości początkowe do profilowania wydajności, a nie gwarancja zbieżności
lub działania w czasie rzeczywistym.

Przy okresie sterowania wynoszącym 0,1 s i horyzoncie 30 kroków optymalizator
przewiduje ruch na 3 sekundy w przód. Później należy przeprowadzić analizę wpływu
długości horyzontu, częstotliwości sterowania i czasu rozwiązywania zadania.

W pierwszej implementacji warto wykorzystać CasADi do symbolicznego opisu dynamiki
i obliczania pochodnych. Jeśli optymalizacja nieliniowa okaże się zbyt wolna, można
rozważyć uproszczony model liniowy, sekwencyjną optymalizację kwadratową lub
specjalizowane sformułowanie problemu sterowania optymalnego.

## 4. Architektura projektu w Pythonie

Projekt należy podzielić na niezależne moduły. Dynamika nie powinna zależeć od
konkretnego regulatora, a regulator nie powinien zależeć od sposobu wizualizacji.
Dzięki temu można zastąpić PID przez MPC bez przepisywania symulatora.

```
rocket-landing-mpc/
├── pyproject.toml
├── README.md
├── configs/
│   ├── vehicle.yaml
│   ├── simulation.yaml
│   └── mpc.yaml
├── src/
│   └── rocket_landing/
│       ├── dynamics.py
│       ├── integrators.py
│       ├── environment.py
│       ├── actuators.py
│       ├── reference.py
│       ├── controllers/
│       │   ├── base.py
│       │   ├── pid.py
│       │   └── mpc.py
│       ├── simulation.py
│       ├── metrics.py
│       └── visualization.py
├── experiments/
│   ├── nominal.py
│   ├── robustness.py
│   ├── horizon_sweep.py
│   └── monte_carlo.py
├── tests/
│   ├── test_dynamics.py
│   ├── test_integrators.py
│   ├── test_constraints.py
│   └── test_metrics.py
├── results/
└── thesis/
```

### Odpowiedzialność poszczególnych modułów

| Moduł | Zakres implementacji |
| --- | --- |
| `dynamics.py` | Równania ruchu, zużycie masy, aerodynamika |
| `integrators.py` | RK4 i numeryczne propagowanie stanu |
| `environment.py` | Grawitacja, wiatr, parametry atmosfery |
| `actuators.py` | Ograniczenia ciągu, opóźnienia, szybkość zmian sterowania |
| `reference.py` | Trajektoria referencyjna lub cel końcowy |
| `controllers/pid.py` | Regulator bazowy |
| `controllers/mpc.py` | Zadanie optymalizacyjne, ograniczenia, inicjalizacja ciepłym startem |
| `simulation.py` | Zamknięta pętla: stan → sterowanie → ruch |
| `metrics.py` | Błędy lądowania, zużycie masy, naruszenia ograniczeń |
| `visualization.py` | Trajektorie 3D oraz wykresy stanu i sterowania |

### Zalecany zestaw narzędzi

- **NumPy i SciPy** — obliczenia numeryczne, całkowanie oraz algorytmy pomocnicze.
- **CasADi + IPOPT** — budowanie nieliniowego zadania optymalizacyjnego i
  obliczanie pochodnych.
- **Matplotlib** — wykresy trajektorii, prędkości, ciągu, masy i porównania
  regulatorów.
- **pytest** — automatyczne testy modelu fizycznego, ograniczeń i metryk.

Na początku nie warto poświęcać czasu na skomplikowaną grafikę 3D. Wykres
trójwymiarowy w Matplotlib i oddzielne wykresy przebiegów czasowych w zupełności
wystarczą do pracy dyplomowej.

### Przykładowy interfejs modelu dynamiki

```python
import numpy as np


def dynamics(state, control, params, wind=None):
    # state = [x, y, z, vx, vy, vz, mass]
    # control = [thrust, alpha, beta]

    position = state[:3]
    velocity = state[3:6]
    mass = state[6]

    thrust, alpha, beta = control

    direction = np.array([
        np.sin(alpha) * np.cos(beta),
        np.sin(beta),
        np.cos(alpha) * np.cos(beta),
    ])

    gravity = np.array([0.0, 0.0, -params.g])
    acceleration = thrust / mass * direction + gravity

    # Opór powietrza zostanie dodany w osobnym module.
    mass_rate = -thrust / (params.isp * params.g0)

    return np.concatenate([
        velocity,
        acceleration,
        [mass_rate],
    ])
```

Jest to jedynie początkowa wersja funkcji. Nie uwzględnia jeszcze oporu powietrza,
kontroli dopuszczalnych stanów, modelu siłowników ani ograniczeń. Przed
wykorzystaniem w MPC należy dodać testy wymiarów, kierunku grawitacji, zachowania
masy i poprawności jednostkowego wektora ciągu.

## 5. Szczegółowy plan na 16 tygodni (oryginalny, zastąpiony płaską listą kroków)

> Uwaga: ten harmonogram tygodniowy jest zachowany tylko jako punkt odniesienia.
> Faktyczna realizacja projektu idzie według płaskiej listy kroków w
> `PROGRESS.md`, bez sztywnych terminów tygodniowych — użytkownik pracuje
> codziennie, w swoim tempie, aż do ukończenia wszystkich kroków.

### Miesiąc 1 — dynamika i model fizyczny

Rezultat: zweryfikowany symulator 3D bez MPC.

- Tydzień 1: poznanie podstawowych pojęć związanych ze sterowaniem, określenie
  założeń, zdefiniowanie układu współrzędnych i parametrów rakiety.
- Tydzień 2: implementacja równań ruchu i integratora RK4; napisanie testów
  grawitacji, ciągu i zużycia masy.
- Tydzień 3: dodanie lądowiska, warunków początkowych, ograniczeń ciągu i
  wykrywania kontaktu z podłożem.
- Tydzień 4: utworzenie trójwymiarowej trajektorii i wykresów prędkości oraz
  masy; ręczne sprawdzenie kilku scenariuszy.

Punkt kontrolny: model porusza się w przewidywalny sposób przy zadanym sterowaniu,
a testy przechodzą poprawnie.

### Miesiąc 2 — regulator bazowy i MPC

Rezultat: dwa działające algorytmy.

- Tydzień 5: implementacja PID do sterowania odchyleniem poziomym i prędkością
  pionową. Dobór parametrów dla nominalnego scenariusza.
- Tydzień 6: poznanie sformułowania MPC; implementacja modelu dyskretnego i
  funkcji kosztu.
- Tydzień 7: utworzenie zadania optymalizacyjnego w CasADi i dodanie ograniczeń
  stanu oraz sterowania.
- Tydzień 8: implementacja sterowania z przesuwanym horyzontem, ciepłego startu
  i obsługi nieudanego rozwiązania optymalizatora.

Punkt kontrolny: MPC potrafi przeprowadzić nominalne lądowanie w symulacji, a
przyczyny braku zbieżności są zapisywane w logach.

### Miesiąc 3 — eksperymenty i odporność

Rezultat: obiektywne porównanie algorytmów.

- Tydzień 9: standaryzacja warunków początkowych i metryk dla PID oraz MPC.
- Tydzień 10: dodanie wiatru, błędu oszacowania prędkości początkowej i
  rozbieżności między modelem regulatora a modelem symulatora.
- Tydzień 11: przeprowadzenie serii testów z różnymi horyzontami, wagami i
  ograniczeniami.
- Tydzień 12: uruchomienie eksperymentów Monte Carlo i przygotowanie tabel oraz
  wykresów.

Punkt kontrolny: istnieje powtarzalny zestaw eksperymentów, uwzględniający
nieudane lądowania i przypadki braku zbieżności.

### Miesiąc 4 — analiza, praca dyplomowa i obrona

Rezultat: ukończona praca naukowo-techniczna.

- Tydzień 13: weryfikacja dokładności numerycznej, poprawności ograniczeń i
  wrażliwości wyników.
- Tydzień 14: opis metodologii, modelu matematycznego, MPC i konfiguracji
  eksperymentów.
- Tydzień 15: opracowanie wyników, wniosków i ograniczeń modelu, a także pliku
  README i instrukcji uruchomienia.
- Tydzień 16: przygotowanie prezentacji, demonstracji i odpowiedzi na pytania
  komisji.

Punkt kontrolny: każdy główny wynik przedstawiony w pracy można odtworzyć na
podstawie kodu i opisanych ustawień.

## 6. Plan eksperymentów

Eksperymenty powinny odpowiadać na pytania badawcze, a nie służyć wyłącznie
prezentowaniu atrakcyjnych trajektorii.

### Eksperyment A — lądowanie nominalne

Uruchom PID i MPC przy identycznych parametrach rakiety, warunkach początkowych i
ograniczeniach. Porównaj: odchylenie poziome od środka lądowiska, prędkość
pionową i całkowitą w chwili przyziemienia, zużycie paliwa, maksymalne odchylenie
ciągu, liczbę naruszeń ograniczeń, czas obliczania sterowania.

### Eksperyment B — zmiana stanu początkowego

Zmieniaj początkowe położenie i prędkość w ustalonym zakresie. Sprawdź, dla jakich
stanów początkowych algorytm nadal potrafi zapewnić miękkie lądowanie.

### Eksperyment C — zakłócenia i błędy modelu

Wprowadzaj kolejno pojedyncze czynniki, a następnie kombinacje: wiatr boczny, błąd
początkowej prędkości, błąd początkowej masy, różnica między rzeczywistym a
oczekiwanym ciągiem, zmiana impulsu właściwego, opóźnienie sterowania lub
ograniczona szybkość zmian ciągu. Szczególnie ważne jest rozdzielenie modelu
wykorzystywanego przez MPC od rzeczywistego modelu symulatora.

### Eksperyment D — dobór parametrów MPC

Zmieniaj pojedynczo: długość horyzontu predykcji, wagę błędu końcowego, karę za
zmianę sterowania, częstotliwość ponownego obliczania sterowania, dopuszczalne
odchylenie wektora ciągu. Mierz jakość lądowania i czas obliczeń.

### Eksperyment E — symulacje Monte Carlo

Losowo generuj stany początkowe i parametry modelu w uprzednio ustalonych
zakresach. W pierwszej serii warto wykonać 100–300 przebiegów, zapisując ziarno
generatora liczb losowych.

## 7. Metryki i kryteria powodzenia

| Metryka | Sposób pomiaru |
| --- | --- |
| Błąd lądowania | Pozioma odległość punktu przyziemienia od środka lądowiska |
| Prędkość przyziemienia | Prędkość pionowa oraz całkowita |
| Skuteczność | Odsetek scenariuszy spełniających wszystkie warunki miękkiego lądowania |
| Zużycie paliwa | Zmiana masy w trakcie lotu |
| Naruszenia ograniczeń | Liczba i wielkość naruszeń ograniczeń ciągu, masy, wysokości i kątów |
| Czas obliczeń | Mediana, 95. percentyl i maksymalny czas optymalizacji |
| Odporność | Zmiana skuteczności w obecności zakłóceń |

Proponowane kryteria edukacyjne: poprawność fizyczna, lądowanie nominalne,
porównanie PID i MPC, odporność, powtarzalność. Jako wstępne wartości liczbowe
można przyjąć poziomy błąd lądowania nie większy niż 2 m oraz prędkość
przyziemienia nie większą niż 1 m/s — do zweryfikowania, czy są osiągalne przy
wybranym ciągu i warunkach początkowych.

## 8. Literatura i materiały NASA

- G-FOLD — Fuel Optimal Large Divert Guidance (NASA TechPort / NTRS).
- Entry, Descent & Landing Subsystems (NASA Johnson Space Center).
- Guidance, Navigation & Control Subsystems (NASA Johnson Space Center).
- NESC Academy — Guidance, Navigation, and Control (materiały szkoleniowe NASA).
- JPL/Masten: precision landing guidance (NASA / Jet Propulsion Laboratory).
- Ping Lu, *Propellant-Optimal Powered Descent Guidance*, Journal of Guidance,
  Control, and Dynamics.
- Opis stanowiska SpaceX Starfall (jako kontekst zawodowy, nie wymóg akademicki).

## 9. Jak przekształcić pracę dyplomową w mocny projekt do portfolio

- Publiczne repozytorium kodu: przejrzysta struktura, automatyczne testy,
  ustalone zależności, pliki konfiguracyjne i instrukcja odtwarzania
  eksperymentów.
- Krótkie wideo techniczne: trójwymiarowa trajektoria lądowania, wykresy
  prędkości i ciągu, porównanie PID i MPC oraz przykład scenariusza, w którym
  jeden z regulatorów sobie nie radzi.
- Krótki raport techniczny po angielsku: problem, model, ograniczenia, algorytm,
  wyniki, złożoność obliczeniowa i ograniczenia przyjętych założeń.

## 10. Co zrobić w ciągu pierwszych siedmiu dni (oryginalna lista kontrolna)

- Utworzyć repozytorium i wirtualne środowisko Python.
- Zapisać założenia modelu i definicje układu współrzędnych.
- Wybrać wiarygodne parametry edukacyjnego modelu rakiety i zapisać je w
  konfiguracji.
- Zaimplementować stan `(x, y, z, vx, vy, vz, m)` i równania ruchu.
- Napisać test swobodnego spadku bez ciągu.
- Napisać test lotu pionowego przy stałym ciągu.
- Wygenerować pierwszy wykres wysokości i prędkości w funkcji czasu.

Główna zasada: najpierw opracuj wiarygodny model fizyczny, następnie uruchom
działający MPC, a dopiero później udowadniaj jego przewagę za pomocą
eksperymentów.
