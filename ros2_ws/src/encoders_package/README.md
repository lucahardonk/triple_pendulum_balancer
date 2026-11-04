# encoders_package

Package ROS 2 per leggere tre encoder in quadratura da un ESP32-C6,
pubblicare posizione e velocità e richiedere l'azzeramento tramite servizio.

## Ambiente verificato

- ROS 2 Lyrical.
- ESP32-C6 con frequenza CPU verificata a 160 MHz.
- Collegamento seriale a 921600 baud, 8N1, senza controllo di flusso.
- Firmware configurato per inviare un campione ogni millisecondo.

## Collegamenti encoder

| Encoder | Canale A | Canale B |
|---------|----------|----------|
| bottom  | GPIO18   | GPIO19   |
| middle  | GPIO20   | GPIO21   |
| top     | GPIO22   | GPIO23   |

Decodifica in quadratura tramite periferica hardware PCNT.
Massa comune tra encoder ed ESP32; pull-up esterni da 3,3 kΩ a 3,3 V
per ciascun ingresso, secondo il cablaggio previsto dal firmware.

## Protocollo seriale

Ogni campione contiene sei valori separati da virgole e termina con `\n`:

    bottom_rad,bottom_rad_s,middle_rad,middle_rad_s,top_rad,top_rad_s

Esempio:

    0.4455,-0.504,0.0000,0.000,0.0000,0.000

- Posizione: radianti, nell'intervallo [0, 2π).
- Velocità: radianti al secondo.
- Comando `Z` o `z`: azzeramento della posizione dei tre encoder.

Durante l'utilizzo del driver, disabilitare le stampe diagnostiche del
firmware. Una stampa inserita tra due frammenti di un campione può
corrompere la riga.

Non aprire contemporaneamente altri lettori sulla stessa porta seriale.

## Interfacce ROS 2

### Nodo

`encoders_driver`

### Topic

`/encoders_state`

Tipo: `encoders_package/msg/EncoderState`

Campi:

- `stamp`: timestamp assegnato sul computer quando il parser riceve
  una riga completa; non è il timestamp di acquisizione sull'ESP32.
- `bottom.position`, `bottom.velocity`
- `middle.position`, `middle.velocity`
- `top.position`, `top.velocity`

Le coppie posizione/velocità utilizzano il messaggio custom
`encoders_package/msg/Encoder`.

Il driver pubblica un messaggio per ogni riga valida, senza timer ROS.
La frequenza segue quindi il flusso seriale e non è imposta dal nodo.

QoS del publisher: SensorDataQoS, best effort, volatile, profondità 1.
I subscriber devono utilizzare una configurazione compatibile.

Il parser mantiene i frammenti tra letture successive, ricompone le
righe terminate da newline e scarta quelle malformate. All'avvio
scarta i byte fino al primo newline per riallinearsi.

### Servizio di reset

Nome: `/reset_encoders`

Tipo: `std_srvs/srv/Trigger`

Il servizio invia un singolo carattere `Z` sulla seriale.

`success: true` conferma che il driver seriale del computer ha accettato
il byte. Il protocollo non prevede una conferma dell'avvenuto
azzeramento da parte del firmware.

Le callback ROS vengono eseguite su un thread separato dal ciclo di
lettura seriale.

## Compilazione

Dalla root del workspace:

    cd ~/Documents/triple_pendulum_balancer/ros2_ws
    source /opt/ros/lyrical/setup.bash

    colcon build \
      --packages-select encoders_package \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DPython3_EXECUTABLE=/usr/bin/python3

Dopo la compilazione:

    source install/setup.bash

Dipendenze principali:

- ament_cmake
- rclcpp
- std_srvs
- builtin_interfaces
- rosidl_default_generators
- rosidl_default_runtime
- Threads, individuato tramite CMake

## Avvio del driver

    source /opt/ros/lyrical/setup.bash
    source ~/Documents/triple_pendulum_balancer/ros2_ws/install/setup.bash

    ros2 run encoders_package encoders_driver

Porta predefinita:

    /dev/serial/by-id/usb-1a86_USB_Single_Serial_5970065826-if00

Per specificare un'altra porta:

    ros2 run encoders_package encoders_driver \
      --ros-args -p port:=/dev/ttyUSB0

Sostituire `/dev/ttyUSB0` con la porta effettiva.

Il driver attuale termina in caso di errore o disconnessione della
seriale; non implementa la riconnessione automatica.

## Verifica dei dati

Eseguire i comandi in un secondo terminale con l'ambiente ROS e il
workspace caricati.

Leggere un singolo messaggio:

    ros2 topic echo /encoders_state \
      encoders_package/msg/EncoderState \
      --qos-reliability best_effort \
      --once

Misurare la frequenza ricevuta:

    ros2 topic hz /encoders_state

Nella prova effettuata, la frequenza osservata da questo comando era
circa 986–988 Hz. La misura riguarda il subscriber di diagnostica:
non permette, da sola, di distinguere la frequenza del firmware da
eventuali perdite o ritardi lungo il percorso.

Non rappresenta una misura del jitter di acquisizione sull'ESP32.

## Azzeramento

Con il driver attivo:

    ros2 service call /reset_encoders std_srvs/srv/Trigger "{}"

Per verificare l'effetto, mantenere gli encoder fermi in posizioni
diverse da zero, inviare il comando e osservare le posizioni pubblicate.

L'azzeramento cambia il riferimento delle posizioni: non muove
fisicamente i pendoli.

## Visualizzazione con PlotJuggler e Foxglove Bridge

Nell'ambiente verificato, PlotJuggler 3.17.2 include il plugin
Foxglove Bridge e il parser ROS 2, mentre il pacchetto
`ros-lyrical-plotjuggler-ros` non era disponibile nei repository configurati.

Installazione:

    sudo apt install \
      ros-lyrical-plotjuggler \
      ros-lyrical-foxglove-bridge

Lasciare il driver in esecuzione.

In un altro terminale:

    source /opt/ros/lyrical/setup.bash
    source ~/Documents/triple_pendulum_balancer/ros2_ws/install/setup.bash

    ros2 run foxglove_bridge foxglove_bridge

Il bridge, nella configurazione provata, ascolta sulla porta 8765.
Verificare nel log che venga annunciato `/encoders_state`.

In un ulteriore terminale, con lo stesso ambiente caricato:

    ros2 run plotjuggler plotjuggler

In PlotJuggler:

1. Selezionare Foxglove Bridge nella sezione Streaming.
2. Premere Start.
3. Inserire l'indirizzo `ws://127.0.0.1:8765`.
4. Connettersi e selezionare `/encoders_state`.
5. Trascinare i campi di posizione e velocità nei grafici.

Usare grafici separati per posizione (rad) e velocità (rad/s).

Attenzione: nel test il bridge ascoltava su `0.0.0.0`, quindi anche
sulle interfacce di rete del computer, non soltanto su localhost.
