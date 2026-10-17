# rocket-landing-mpc

Praca dyplomowa: autonomiczne lądowanie powracającego pierwszego stopnia rakiety
sterowane regulatorem predykcyjnym (Model Predictive Control, MPC), porównane z
bazowym regulatorem PID.

## Gdzie szukać czego

- **`PROGRESS.md`** — płaska lista wszystkich kroków projektu (bez podziału na
  tygodnie) z checkboxami. To jest aktualny stan pracy — zawsze zaczynaj tutaj.
- **`thesis/plan_original.md`** — pełny, oryginalny opis zadania (matematyka,
  architektura, eksperymenty, literatura). Punkt odniesienia przy pisaniu pracy.
- **`thesis/archive_pycharm_1d_tutorial.md`** — archiwalna, nigdy niezrealizowana
  pierwsza wersja README (prosty tutorial 1D bez MPC). Zachowana tylko historycznie.
- **`docs/assumptions.md`** — założenia modelu fizycznego (układ współrzędnych,
  wektor stanu, wektor sterowania).
- **`src/rocket_landing/`** — kod źródłowy pakietu.
- **`tests/`** — testy pytest.
- **`experiments/`** — skrypty eksperymentów (porównanie PID vs MPC, Monte Carlo,
  itd.).
- **`configs/`** — pliki konfiguracyjne (dodane w późniejszej fazie projektu).

## Jak uruchomić

```
source .venv/bin/activate
pytest tests/ -v
```

## Status

Projekt w trakcie realizacji — zobacz `PROGRESS.md` dla aktualnego kroku.
