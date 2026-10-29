# ODrive (MKS XDRIVE Mini) su Ubuntu 26.04 — guida di installazione

Come far funzionare `odrivetool` 0.5.1 con la scheda **MKS ODrive Mini v1** su **Ubuntu 26.04**,
e quali problemi abbiamo incontrato. Servirà anche come base per il Docker di fine progetto.

## Configurazione funzionante

| Componente | Versione |
|---|---|
| Sistema operativo | Ubuntu 26.04.1 LTS (Python di sistema 3.14 — **non usato**) |
| Python dell'ambiente | **3.13.7** (installato con `uv`) |
| `odrive` (pip) | **0.5.1.post0** |
| `ipython` | **< 9** (testato 8.x) ← punto chiave |
| Firmware scheda | community "MKS XDRIVE Mini" basato su v0.5.1 — **NON aggiornare** |
| Scheda | MKS ODrive Mini v1 (8-56V), encoder magnetico SPI montato sulla scheda |
| Motore | C6374 170kv, alimentazione da banco 24V |

Le dipendenze bloccate sono in [`requirements-odrive.txt`](requirements-odrive.txt).

## Installazione passo per passo

```bash
cd ~/Documents/triple_pendulum_balancer/single_pendulum_test

# 1) uv: gestisce versioni di Python separate da quella di sistema
curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="$HOME/.local/bin:$PATH"          # per renderlo permanente: aggiungerlo a ~/.bashrc

# 2) Python 3.13.7 + ambiente virtuale
uv python install 3.13.7
uv venv --python 3.13.7 --seed odrive-legacy
source odrive-legacy/bin/activate

# 3) Librerie (ODrive 0.5.1 + IPython 8)
python -m pip install -r ../hw_specifications/driver/requirements-odrive.txt

# 4) Regole udev: permettono l'accesso USB alla scheda senza sudo (da fare una volta sola)
sudo "$(which odrivetool)" udev-setup
#    poi scollegare e ricollegare il cavo USB

# 5) Avvio
odrivetool
```

Test di sola lettura, al prompt `In [1]:`:

```python
odrv0.vbus_voltage        # deve rispondere circa 24.0
```

La configurazione della scheda è in [`my_config.txt`](my_config.txt).

## Problemi incontrati e soluzioni

### 1. Manca `python3.14-venv`
Ubuntu 26.04 non installa il modulo `venv`, quindi `python3 -m venv` fallisce.
In un primo momento l'abbiamo risolto con `sudo apt install python3.14-venv`. Poi siamo passati a `uv` con Python 3.13.7,
che non dipende dal Python di sistema.

### 2. `AttributeError: module 'IPython' has no attribute 'terminal'`
`odrivetool` va in crash appena parte.
- **Causa:** il codice di Fibre, la libreria USB di ODrive 0.5.1, usa `IPython.terminal.embed` senza importarlo.
  Con le versioni vecchie di IPython funzionava lo stesso, perché `import IPython` caricava anche quel modulo.
  IPython 9 invece carica i sottomoduli solo se vengono importati esplicitamente.
- Passare da Python 3.14 a 3.13.7 **non** l'ha risolto: il problema è IPython, non Python.
- Una soluzione temporanea era importare il modulo prima di avviare `odrivetool`:
  `python -c 'import IPython.terminal.embed, runpy, shutil; runpy.run_path(shutil.which("odrivetool"), run_name="__main__")'`.

### 3. `NameError: name 'odrv0' is not defined`
Con la soluzione temporanea la console si avviava e compariva "Connected to ODrive ... as odrv0",
ma la variabile `odrv0` non esisteva.
- **Causa:** in IPython 9 la console integrata gestisce le variabili in modo diverso. Fibre invece mette
  `odrv0` nelle variabili globali del suo modulo, e la console non le vede.
  Lo abbiamo riprodotto con una scheda simulata; non abbiamo analizzato il codice di IPython riga per riga.
- **Soluzione definitiva per i problemi 2 e 3:** `pip install "ipython<9"`. Con IPython 8 `odrivetool` parte normalmente
  e `odrv0` funziona, senza bisogno di soluzioni temporanee.
- Il comando temporaneo `~/.local/bin/odrv`, se l'hai creato, non serve più: `rm ~/.local/bin/odrv`.

### 4. `usbi_mutex_destroy: Assertion ... failed / Aborted (core dumped)`
Compare **solo dopo** un crash di Python: libusb si chiude in modo anomalo.
Non indica un guasto USB o elettrico.

### 5. Disconnessioni USB intermittenti (ancora da verificare)
Le regole udev sono applicate. Resta da capire se le cadute di connessione dipendono da disturbi elettrici
(anelli di massa, nel caso serve un isolatore USB) o dalla stabilità dell'alimentatore.
Da tenere d'occhio quando il motore gira.

## Regole di sicurezza
- **Non aggiornare il firmware** della scheda: restare sulla versione community basata su v0.5.1.
- `odrv0.erase_configuration()` cancella tutto e riavvia la scheda. "Oh no odrv0 disappeared" seguito dalla riconnessione è normale.
- La prima calibrazione va fatta con l'albero libero (senza pendolo) e un comando alla volta,
  controllando `dump_errors(odrv0)` dopo ogni passo.

## Appunti per il Docker di fine progetto
- Immagine base: `python:3.13.7-slim` + pacchetto `libusb-1.0-0` + `pip install -r requirements-odrive.txt`.
- Le regole udev vanno installate sull'**host** (Ubuntu), non nel container.
- Passare la USB al container: `--device /dev/bus/usb` (oppure `-v /dev/bus/usb:/dev/bus/usb --device-cgroup-rule='c 189:* rmw'`
  per gestire le riconnessioni dopo `reboot()`/`erase_configuration()`).
- La console interattiva richiede `docker run -it`.
