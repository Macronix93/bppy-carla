# Integration von BPpy mit CARLA

- Eine Implementation von BPpy mit dem 3D Simulator CARLA für realistische Verkehrsszenarien
- Darüberhinaus wird die einfache Austauschbarkeit über Gymnasium mit anderen Simulatoren gewährleistet (wie z.B. SUMO).

## Installation
Damit alles reibungslos abläuft und funktioniert müssen folgende Schritte durchgeführt werden:

!!! Bitte auch die CARLA Systemanforderungen beachten auf https://carla.readthedocs.io/en/latest/start_quickstart/ !!!

1. Download und Installation einer Python Version >=3.10 und <=3.12 (https://www.python.org/downloads/).
Warum ab 3.10? Weil bestimmte Packages, die benutzt werden, ältere Versionen nicht unterstützen.
2. Download der aktuellsten CARLA Server Version auf https://github.com/carla-simulator/carla/releases und
Extraktion der Dateien in einen neuen Ordner
3. Download für SUMO: https://sumo.dlr.de/docs/Downloads.php. Anschließend der Installationsanleitung folgen.
4. Setzen der Umgebungsvariable `CARLA_SERVER` auf den Ordner, in dem sich die ausführbare Serverdatei (Windows: CarlaUE4.exe, Linux: CarlaUE4.sh) befindet
mit folgendem Befehl:

#### Windows:
```bash
setx CARLA_SERVER "Laufwerksbuchstabe:\Pfad\zum\CARLA\Server"
```

#### Linux:
Öffne das Terminal und füge die folgende Zeile zu deiner `.bashrc`- oder `.zshrc`-Datei hinzu: \
```bash
export CARLA_SERVER="/Pfad/zum/CARLA/Server"
```
Führe anschließend `source ~/.bashrc` oder `source ~/.zshrc` aus, um die Änderungen zu übernehmen.

Nach der Installation kann ein Szenario aus dem Ordner `scenario` mit einer Python-IDE oder alternativ mit folgenden Terminal-Befehlen
(von dem Projektverzeichnis aus) gestartet werden:

```bash
python -m pip install -r requirements.txt
python -m src.scenario.follow_behind

(oder alternativ ein anderes Szenario wie z.B. overtake)
```

Der CARLA Server sollte automatisch starten, ansonsten einmal manuell starten.
Konfiguriert werden können die bereits vorhandenen Szenarien über die jeweiligen JSON Dateien im Ordner ``scenario``
und globale Einstellungen befinden sich in der datei ``config.py``. Möchte man z.B. einen anderen Simulator, dann muss in der `config.py`
``CURRENT_SIM`` auf beispielsweise `sumo` gesetzt werden.