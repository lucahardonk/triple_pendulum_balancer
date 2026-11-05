# ODrive USB ROS 2 Driver

Driver Python ROS 2 a nodo singolo per controllare ODrive tramite USB, con comandi in unità SI e telemetria selezionabile tramite JSON.

## Interfaccia e architettura

- Un solo `USBWorker` possiede la connessione ed esegue tutte le operazioni USB.
- `ODriveDriver` gestisce le interfacce ROS.
- Callback group distinti per ricezione comandi, servizi e pubblicazione; `MultiThreadedExecutor` con tre thread. I gruppi consentono concorrenza, ma non sono associati rigidamente a specifici thread.
- Comandi: target predefinito 500 Hz. Il worker conserva e riscrive l'ultimo valore valido anche senza nuovi messaggi ROS.
- Telemetria: target predefinito 100 Hz. Una proprietà viene letta per volta, controllando le scadenze dei comandi tra le letture.
- Pubblicazione attivata da una guard condition quando una raccolta è completa, senza timer di ripubblicazione. Un buffer conserva solo la raccolta più recente.
- Nessun logging periodico, statistica di debug, topic amministrativo o servizio `get_status`.

Le frequenze sono obiettivi, non garanzie real-time. Una chiamata USB sincrona già iniziata non può essere interrotta; letture e servizi possono ritardare i comandi. Le scadenze perse vengono saltate, senza invii di recupero a raffica. Un campionamento troppo lento può ridurre la frequenza effettiva di telemetria.

## Struttura del pacchetto

```text
odrive_driver/
  LICENSE
  README.md
  setup.py
  setup.cfg
  package.xml
  resource/odrive_driver
  odrive_driver/__init__.py
  odrive_driver/driver_node.py
  odrive_driver/py.typed
  test/
```

`resource/odrive_driver` è un marker vuoto. In `setup.py`, il console entry point deve essere:

```python
entry_points={
    'console_scripts': [
        'driver_node = odrive_driver.driver_node:main',
    ],
}
```

I `data_files` devono installare il marker in `share/ament_index/resource_index/packages` e `package.xml` in `share/odrive_driver`. Installare anche questo README è facoltativo; se è elencato in `data_files`, il file deve esistere nella radice del pacchetto.

`setup.cfg`:

```ini
[develop]
script_dir=$base/lib/odrive_driver

[install]
install_scripts=$base/lib/odrive_driver
```

In `package.xml` dichiarare `ament_python` come buildtool e `rclpy`, `std_msgs`, `std_srvs` come dipendenze. Sostituire l'email segnaposto del maintainer con un indirizzo reale.

## Ambiente Python

Nella configurazione verificata in questa conversazione:

- Workspace: `~/Documents/triple_pendulum_balancer/ros2_ws`
- Venv: `/home/nuc/Documents/triple_pendulum_balancer/.venv-odrive-ros`
- Seriale ODrive: `316B315D3335`

Mantenere la versione della libreria ODrive già funzionante. Il Python del venv deve poter importare sia ROS sia ODrive:

```bash
/home/nuc/Documents/triple_pendulum_balancer/.venv-odrive-ros/bin/python   -c "import rclpy, odrive; print('Dipendenze OK')"
```

Caricare prima l'ambiente della propria installazione ROS 2. Il percorso dipende dalla distribuzione installata. Non installare `rclpy` con pip per aggirare un'incompatibilità dell'interprete: il venv deve essere compatibile con l'installazione ROS.

## Build

Usare esplicitamente il Python del venv, senza bisogno di attivarlo:

```bash
cd ~/Documents/triple_pendulum_balancer/ros2_ws

/home/nuc/Documents/triple_pendulum_balancer/.venv-odrive-ros/bin/python   -m colcon build --packages-select odrive_driver

source install/setup.bash
```

Il modulo `colcon` deve essere disponibile a quell'interprete.

**Non usare `--symlink-install` per questa configurazione:** nel tentativo precedente il launcher era stato installato nel venv anziché nel prefisso ROS atteso. La build senza symlink ha prodotto correttamente il marker ament e il launcher nel workspace.

Verifica:

```bash
ls -l install/odrive_driver/share/ament_index/resource_index/packages/odrive_driver
head -n 1 install/odrive_driver/lib/odrive_driver/driver_node
ros2 pkg prefix odrive_driver
ros2 pkg executables odrive_driver
```

Lo shebang verificato è:

```text
#!/home/nuc/Documents/triple_pendulum_balancer/.venv-odrive-ros/bin/python
```

Questo permette a `ros2 run` di usare il venv senza attivazione manuale. Non spostare o cancellare il venv: il launcher dipende dal percorso assoluto. Se il progetto viene trasferito su un'altra macchina, usare il percorso corretto dell'ambiente e ricostruire il pacchetto.

Non serve cancellare `build/odrive_driver` e `install/odrive_driver` a ogni build. La cancellazione è stata usata soltanto per ripulire l'installazione precedente.

## Avvio normale

In un terminale con l'ambiente ROS di base già caricato:

```bash
source ~/Documents/triple_pendulum_balancer/ros2_ws/install/setup.bash

ros2 run odrive_driver driver_node --ros-args   -p serial_number:="'316B315D3335'"   -p axis:=0   -p rate_hz:=500.0   -p telemetry_hz:=100.0
```

Non serve attivare il venv. Nello stesso terminale non è necessario ripetere il source a ogni riavvio del nodo.

La connessione legge la modalità di controllo esistente e prepara la telemetria. Non seleziona automaticamente una modalità e non richiede il closed-loop. La modalità di controllo iniziale deve essere una tra posizione, velocità e coppia supportate dal driver.

`ODrive connected` conferma la connessione, non il raggiungimento delle frequenze richieste. L'installazione del pacchetto non configura l'avvio automatico al boot.

## Modifiche successive

### Modifiche al codice o ai metadati del pacchetto

Fermare il nodo, modificare i file in `src/odrive_driver`, ripetere la build sopra e riavviare. Senza symlink il codice viene copiato nell'installazione: modificare il sorgente non aggiorna automaticamente il processo avviato con `ros2 run`.

### Modifiche ai parametri

Per cambiare seriale, asse, frequenze o selezione della telemetria basta fermare il nodo e riavviarlo con i nuovi parametri. Non serve ricompilare.

| Operazione | Build necessaria | Attivazione manuale venv |
|---|---|---|
| Avvio normale | No | No |
| Modifica a driver_node.py | Sì | No, usando il Python esplicito |
| Modifica a setup.py, setup.cfg o package.xml | Sì | No |
| Cambio parametri di avvio | No | No |
| Chiamata servizi o pubblicazione comandi | No | No |

## Parametri di avvio

| Parametro | Default | Significato |
|---|---|---|
| `serial_number` | stringa vuota | Vuoto: qualsiasi dispositivo trovato |
| `axis` | `0` | Asse 0 o 1 |
| `connect_timeout_s` | `20.0` | Timeout connessione in secondi |
| `rate_hz` | `500.0` | Frequenza obiettivo di riscrittura del comando |
| `telemetry_hz` | `100.0` | Frequenza obiettivo di raccolta telemetria |
| `telemetry_json` | posizione e velocità | Proprietà da leggere e pubblicare |
| `unit_rules_json` | `{}` | Regole di conversione aggiuntive |

I parametri sono letti all'avvio. Le frequenze e il timeout devono essere positivi. `log_hz` non esiste più.

## Topic

I nomi seguenti assumono il nome predefinito del nodo, `odrive_driver`.

| Topic | Tipo | Funzione |
|---|---|---|
| `/odrive_driver/command` | `std_msgs/msg/Float64` | Setpoint in SI |
| `/odrive_driver/feedback` | `std_msgs/msg/String` | Telemetria selezionata in JSON |

Entrambi usano QoS best-effort, profondità 1. I topic `request` e `response` sono stati eliminati.

### Comandi

| Modalità | Significato Float64 | Unità |
|---|---|---|
| Posizione | Target assoluto | rad |
| Velocità | Target velocità | rad/s |
| Coppia | Target coppia | N*m |

L'ultimo valore valido viene ripetuto al target `rate_hz`. I valori intermedi possono essere sovrascritti da comandi più recenti. I comandi non finiti vengono ignorati; quelli ricevuti prima che il worker sia pronto o durante una transizione vengono ignorati.

Ogni richiesta di transizione tramite servizio invalida l'ultimo comando. Dopo la transizione serve un comando nuovo. Un publisher ancora attivo può fornirlo immediatamente, anche mentre l'asse è IDLE.

Non è presente un timeout host: fermare un publisher non ferma il motore. Ripetere un comando non significa ricevere dati freschi dal controller ROS.

## Servizi

Tutti usano `std_srvs/srv/Trigger` sotto `/odrive_driver/`.

| Servizio | Azione |
|---|---|
| `set_idle` | Richiede IDLE |
| `set_closed_loop` | Richiede closed-loop, partendo da IDLE |
| `set_vel_mode` | Velocità con passthrough |
| `set_position_mode` | Posizione con passthrough |
| `set_torque_mode` | Coppia con passthrough |
| `set_passthrough` | Passthrough per la modalità corrente |
| `set_ramped` | Rampa velocità, traiettoria posizione o rampa coppia |
| `set_position_filter` | Filtro posizione; richiede modalità posizione |

I cambi di modalità e shaping richiedono IDLE. Inizializzano il target posizione dall'encoder e azzerano velocità e coppia di feedforward. Non configurano pendenze delle rampe, banda del filtro o limiti delle traiettorie.

Prima del closed-loop il driver verifica IDLE, azzera velocità e coppia e inizializza la posizione corrente. Il successo di un servizio di stato significa che la richiesta è stata scritta, non che lo stato sia stato raggiunto. In particolare, un IDLE appena richiesto potrebbe non essere ancora attivo quando si chiama il servizio successivo: controllare i risultati e lo stato selezionato nella telemetria.

`get_status` è stato eliminato. Per osservare stato ed errori selezionarli in `telemetry_json`.

## Esempio velocità

Mettere in sicurezza il meccanismo, fermare altri publisher di comando e tenere disponibile un arresto indipendente. Controllare il risultato di ciascun servizio prima di continuare.

```bash
ros2 service call /odrive_driver/set_idle std_srvs/srv/Trigger '{}'
ros2 service call /odrive_driver/set_vel_mode std_srvs/srv/Trigger '{}'
ros2 service call /odrive_driver/set_closed_loop std_srvs/srv/Trigger '{}'

ros2 topic pub --once /odrive_driver/command std_msgs/msg/Float64   '{data: 0.2}'
```

Il valore è 0.2 rad/s. Per pubblicare da CLI a 100 Hz:

```bash
ros2 topic pub --rate 100 /odrive_driver/command std_msgs/msg/Float64   '{data: 0.2}'
```

Il worker continuerà a riscrivere l'ultimo setpoint al proprio target di 500 Hz anche se il publisher è più lento. È possibile pubblicare valori variabili dallo stesso topic tramite un controller ROS.

## Arresto

Fermare prima i publisher attivi. In modalità velocità, inviare zero:

```bash
ros2 topic pub --once /odrive_driver/command std_msgs/msg/Float64   '{data: 0.0}'
```

Poi richiedere IDLE:

```bash
ros2 service call /odrive_driver/set_idle std_srvs/srv/Trigger '{}'
```

IDLE disabilita il controllo attivo: non equivale a una frenatura controllata. In posizione, zero è un target assoluto di zero radianti, non un comando di stop.

## Posizione e traiettorie

Con publisher fermi e asse IDLE, selezionare `set_position_mode`, poi `set_ramped`, quindi `set_closed_loop`, verificando ogni risultato. Il topic comando rappresenta il target assoluto in radianti.

Il driver usa i limiti di traiettoria già configurati nel dispositivo. La versione semplificata non offre get/set generici, configurazione numerica delle rampe o movimenti relativi. Configurare preventivamente limiti adatti con uno strumento compatibile, senza accesso concorrente alla connessione del driver. Le unità di strumenti esterni non sono gestite da questo nodo.

## Telemetria

```bash
ros2 topic echo /odrive_driver/feedback   --qos-reliability best_effort --full-length
```

Il payload contiene soltanto gli alias richiesti, ciascuno con `value` e `unit`. Esempio illustrativo del formato, non misura reale:

```json
{
  "position_rad": {"value": 0.0, "unit": "rad"},
  "velocity_rad_s": {"value": 0.0, "unit": "rad/s"}
}
```

Non vengono aggiunti automaticamente stato, errori, modalità, timestamp o statistiche. Le proprietà vengono lette sequenzialmente, non simultaneamente. Una pubblicazione rappresenta una raccolta completa; non viene ripubblicata periodicamente una raccolta vecchia. Se il publisher ROS non tiene il passo, il buffer può sostituire raccolte non ancora pubblicate con quella più recente.

### Selezione personalizzata

```bash
ros2 run odrive_driver driver_node --ros-args   -p serial_number:="'316B315D3335'"   -p axis:=0   -p rate_hz:=500.0   -p telemetry_hz:=100.0   -p telemetry_json:='{"position_rad":{"path":"encoder.pos_estimate"},"velocity_rad_s":{"path":"encoder.vel_estimate"},"state":{"path":"current_state"},"axis_error":{"path":"error"},"bus_voltage":{"scope":"device","path":"vbus_voltage"}}'
```

Ogni alias definisce:

- `path`: percorso esatto della proprietà.
- `scope`: `axis` per percorsi relativi all'asse selezionato, oppure `device` per percorsi relativi al dispositivo. Default: `axis`.

Aggiungere proprietà aumenta il lavoro USB. Per mantenere le massime prestazioni selezionare soltanto quelle necessarie. Con `{}` non viene pubblicata telemetria.

## Conversioni SI

Il driver converte posizione e velocità native in radianti e rad/s. La coppia usa N*m. La conversione è basata su regole esatte, non su deduzioni dai nomi delle proprietà.

Per proprietà numeriche non incluse nelle regole integrate, aggiungere una voce a `unit_rules_json` con questi campi:

- Chiave: percorso esatto della proprietà, coerente con `path`.
- `native_to_si`: fattore positivo che moltiplica il valore nativo per ottenere il valore SI.
- `unit`: etichetta non vuota dell'unità SI.

Il fattore va determinato dalle unità documentate della proprietà e dalla versione firmware: non assumere che sia 1. Le regole integrate non possono essere sovrascritte. Le proprietà numeriche non registrate vengono rifiutate durante la configurazione della telemetria. Booleani e stringhe non richiedono una regola numerica. Valori numerici non finiti letti dal dispositivo vengono rappresentati come stringhe per mantenere il JSON valido.

I percorsi devono esistere sul firmware collegato. Non esiste più un'interfaccia ROS get/set generica.

## Sicurezza e limiti

- Nessuna calibrazione, cancellazione errori, scrittura flash o riconnessione automatica.
- Nessun watchdog feeding implementato, timeout host dei comandi o limite di ampiezza nel nodo.
- L'ultimo comando continua a essere ripetuto dopo la disconnessione del publisher.
- Transizioni di stato/modalità invalidano il comando conservato; fermare comunque i publisher prima delle transizioni.
- Il firmware può rifiutare il closed-loop per errori o prerequisiti mancanti.
- Un'eccezione non gestita nel percorso USB arresta il worker e genera un errore; il nodo ROS può restare attivo senza feedback nuovo.
- In chiusura il driver tenta IDLE. Interruzioni USB, terminazione forzata o chiamate bloccate possono impedire o ritardare l'arresto; la chiusura attende il worker.
- Nessun accesso concorrente da altri programmi alla connessione ODrive. Cambiare modalità esternamente rende incoerente la modalità memorizzata dal driver.
- Usare misure di sicurezza hardware indipendenti. Il nodo non è un sistema di arresto di emergenza.

## Troubleshooting

### No module named odrive

Verificare gli import con il Python del venv e controllare lo shebang del launcher. Ricostruire con il percorso assoluto del Python corretto. Non basta che `python3` di sistema trovi ROS.

### Package odrive_driver not found / eseguibile assente

Caricare ROS e l'overlay, verificare marker ament, entry point e `setup.cfg`. Nella configurazione descritta ricostruire senza `--symlink-install`. Il solo messaggio di build riuscita non conferma il percorso di installazione: controllare i file e il log di build.

### Warning AMENT_PREFIX_PATH dopo una pulizia

Se il prefisso è appena stato cancellato, il terminale può conservare un riferimento alla vecchia installazione. Ricostruire e caricare l'overlay aggiornato; usare un terminale nuovo con ROS e workspace caricati per verificare se persiste.

### Axis must be IDLE

Richiedere IDLE, osservare `current_state` nella telemetria e riprovare soltanto quando la transizione è avvenuta. Il successo della richiesta non certifica lo stato raggiunto.

### Nessun movimento o risposta lenta

Controllare il comando ROS, la modalità selezionata, `current_state`, gli errori e `encoder.vel_estimate` tramite telemetria. Se necessario includere `controller.input_vel` per confrontare setpoint e velocità misurata. Aumentare il comando non corregge ritardi USB, limiti configurati o problemi di controllo del motore.

### Telemetria assente / USB stopped

Controllare eccezioni, validità dei percorsi e regole unità. Le proprietà devono essere compatibili con firmware e libreria installati. Correggere la causa prima di riavviare; non è prevista riconnessione automatica.

## Test del pacchetto

I test di lint del pacchetto non servono all'avvio e non validano il comportamento del motore o le prestazioni USB. Possono essere eseguiti con:

```bash
cd ~/Documents/triple_pendulum_balancer/ros2_ws
colcon test --packages-select odrive_driver
colcon test-result --verbose
```

## Verifiche effettuate

L'utente ha riportato una build riuscita senza symlink, il marker ament presente, il launcher con interprete del venv, il pacchetto riconosciuto da ROS e l'avvio tramite `ros2 run` con messaggio `ODrive connected`. Le frequenze effettive 500/100 Hz e il comportamento hardware della versione semplificata non sono stati verificati in questa sessione.