# ODrive via CAN in ROS 2 (triple_pendulum_balancer)

Driver ufficiale ODrive [`ros_odrive`](https://github.com/odriverobotics/ros_odrive) (pacchetto `odrive_can`)
con una piccola patch locale per il firmware **0.5.1** (ODrive 3.6 non ufficiale).
Sostituisce il vecchio pacchetto `odrive_driver` (USB), che è stato rimosso.

| Cosa | Valore |
|---|---|
| ROS 2 | Lyrical (Ubuntu 26, Python 3.14) |
| Adattatore | Jhoinrch RH-02 (CANable, firmware candleLight, driver `gs_usb`) → `can0` |
| Bitrate | 250 kbit/s (`odrv0.can.config.baud_rate = 250000`) |
| Motore | asse 0, `can_node_id = 0` |
| Meccanica | motore → cinghia → carrello lineare |

---

## ⚠️ Sicurezza e ordine di accensione (IMPORTANTE)

Nel mio setup **la massa comune (GND) tra PC e ODrive passa dalla USB dell'ODrive**:
sul connettore CAN dell'ODrive ci sono solo H e L, il filo GND dell'RH-02 non è collegato.

Ordine obbligatorio:

1. **Collegare la USB dell'ODrive al PC** (fornisce il GND comune).
2. **Collegare l'adattatore CAN RH-02** allo stesso PC.
3. **Solo dopo, accendere l'alimentazione del motore/ODrive.**

Spegnimento: ordine inverso (prima l'alimentazione del motore, poi le USB).

- Se si vuole togliere la USB dell'ODrive, **prima** collegare il GND dell'RH-02 a un GND di segnale dell'ODrive.
- Non collegare mai il pin 5 V.
- L'asse 0 è configurato per entrare in **closed loop all'accensione**: tenere il carrello libero di muoversi e la mano vicino all'alimentazione.
- Switch RH-02: **BOOT su** (lavoro), **R120 giù** (terminazione 120 Ω attiva). Tra H e L, a sistema spento, si misurano ~60 Ω.

---

## Avvio

```bash
# 1. Interfaccia CAN ("Device or resource busy" = già attiva, va bene)
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up

# 2. ROS
cd ~/Documents/triple_pendulum_balancer/ros2_ws
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 launch pendulum_bringup hardware.launch.py
```

Il launch avvia `odrive_can_node` (namespace `odrive_axis0`, `node_id: 0`, `interface: can0`,
`legacy_fw05: true`, `poll_rate_hz: 200.0`) e `joy_node`.

### Build (solo la prima volta o dopo modifiche)

`~/.local/bin/python3.13` è nel PATH prima di quello di sistema: va forzato il Python di ROS.

```bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

---

## Interfaccia ROS

Tutto passa dal bus CAN: `tuo nodo → topic ROS → odrive_can_node → can0 → ODrive`.
**Unità: giri e giri/s (non radianti). 1 giro = 2π rad.**

### Telemetria — `/odrive_axis0/controller_status` (`odrive_can/msg/ControllerStatus`)

| Campo | Contenuto con FW 0.5.1 |
|---|---|
| `pos_estimate` | posizione [giri], ~200 Hz |
| `vel_estimate` | velocità [giri/s], ~200 Hz |
| `axis_state` | stato asse (8 = closed loop, 1 = idle), dall'heartbeat a 10 Hz |
| `active_errors` | errore asse, dall'heartbeat |
| `iq_*`, `torque_*` | sempre 0 (non richiesti) |
| `procedure_result`, `trajectory_done_flag` | sempre 0 |

`/odrive_axis0/odrive_status` (tensione, temperatura) **non viene pubblicato** con FW 0.5.

```bash
ros2 topic hz   /odrive_axis0/controller_status
ros2 topic echo /odrive_axis0/controller_status --once
```

### Stato dell'asse — servizio `/odrive_axis0/request_axis_state` (`odrive_can/srv/AxisState`)

| Valore | Stato |
|---|---|
| 1 | IDLE (motore libero) |
| 3 | calibrazione completa |
| 4 | calibrazione motore |
| 7 | calibrazione offset encoder |
| 8 | CLOSED_LOOP_CONTROL |

Chiedendo uno stato diverso da IDLE il nodo cancella prima gli errori.
C'è anche `/odrive_axis0/clear_errors` (`std_srvs/srv/Empty`).

```bash
ros2 service call /odrive_axis0/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 8}"
ros2 service call /odrive_axis0/request_axis_state odrive_can/srv/AxisState "{axis_requested_state: 1}"
```

### Comandi — topic `/odrive_axis0/control_message` (`odrive_can/msg/ControlMessage`)

Campi: `control_mode`, `input_mode`, `input_pos`, `input_vel`, `input_torque`.
Per ogni messaggio il nodo invia 2 frame CAN (imposta modalità + setpoint).

| `control_mode` | Controllo | Campi usati |
|---|---|---|
| 1 | coppia [Nm] | `input_torque` (richiede `motor.config.torque_constant` corretto) |
| 2 | velocità [giri/s] | `input_vel` + `input_torque` come feedforward |
| 3 | posizione [giri] | `input_pos` + `input_vel`/`input_torque` come feedforward (max ±32.7) |
| 0 | tensione | non supportato |

| `input_mode` | Comportamento |
|---|---|
| **1** | **PASSTHROUGH**: applicato subito — da usare per il controllore del pendolo |
| 2 | VEL_RAMP |
| 3 | POS_FILTER |
| 5 | TRAP_TRAJ |
| 6 | TORQUE_RAMP |

I parametri di rampe/traiettorie si configurano via USB con `odrivetool`.

```bash
# muovi a 0.1 giri/s, poi ferma
ros2 topic pub --once /odrive_axis0/control_message odrive_can/msg/ControlMessage "{control_mode: 2, input_mode: 1, input_vel: 0.1}"
ros2 topic pub --once /odrive_axis0/control_message odrive_can/msg/ControlMessage "{control_mode: 2, input_mode: 1, input_vel: 0.0}"
```

Esempio Python (rclpy):

```python
from odrive_can.msg import ControlMessage, ControllerStatus

pub = node.create_publisher(ControlMessage, "/odrive_axis0/control_message", 10)
node.create_subscription(ControllerStatus, "/odrive_axis0/controller_status", on_status, 10)

pub.publish(ControlMessage(control_mode=2, input_mode=1, input_vel=0.5))  # 0.5 giri/s
```

### USB
La USB resta disponibile per `odrivetool` (configurazione, calibrazione, errori).
**Non comandare il motore da USB e da CAN contemporaneamente.**

---

## Patch locale `legacy_fw05`

Il nodo ufficiale è scritto per FW 0.6: pubblica `controller_status` solo dopo aver ricevuto
heartbeat `0x001`, encoder `0x009`, Iq `0x014` e coppie `0x01C`. Il FW 0.5.1 trasmette da solo
**solo l'heartbeat**, quindi senza patch non arriva telemetria.

Con `legacy_fw05: true` il nodo:
1. invia ogni `1/poll_rate_hz` s una richiesta RTR `0x009` al nodo 0; l'ODrive risponde con posizione e velocità;
2. pubblica `controller_status` a ogni risposta `0x009`;
3. azzera `procedure_result`/`trajectory_done_flag` (sulla 0.5 quei byte hanno un altro significato).

Con `legacy_fw05: false` il comportamento è identico all'originale.

`src/ros_odrive` è una **copia vendorizzata** dell'upstream (commit `182e040`) con la patch **già applicata**:
- `odrive_node/CMakeLists.txt`: `ament_target_dependencies` (rimosso in Lyrical) → `target_link_libraries(... rclcpp::rclcpp ${std_srvs_TARGETS})`
- `odrive_node/include/odrive_can_node.hpp`, `odrive_node/src/odrive_can_node.cpp`: patch `legacy_fw05`
- `odrive_ros2_control/COLCON_IGNORE`, `odrive_botwheel_explorer/COLCON_IGNORE`: pacchetti esclusi dal build

`odrive_lyrical_fw05.patch` documenta le differenze rispetto all'upstream. Per aggiornare a una nuova versione upstream:

```bash
cd ~/Documents/triple_pendulum_balancer/ros2_ws/src
rm -rf ros_odrive
git clone --depth 1 https://github.com/odriverobotics/ros_odrive.git
cd ros_odrive
git apply ../../odrive_lyrical_fw05.patch      # risolvere a mano eventuali conflitti
touch odrive_ros2_control/COLCON_IGNORE odrive_botwheel_explorer/COLCON_IGNORE
rm -rf .git                                    # deve restare una cartella normale, non un submodule
```

---

## Nodi del progetto portati su CAN

Entrambi lavorano in radianti: la conversione in giri (÷ 2π) è fatta dentro il nodo.

### `pendulum_safety_package` → `overrun_node`

Trova i finecorsa del carrello e lo riporta al centro.
Sequenza: IDLE → closed loop (8) → sweep in velocità (`control_mode: 2`, `input_mode: 1`) → centro → IDLE.

| Interfaccia | Tipo |
|---|---|
| pubblica `/odrive_axis0/control_message` | `odrive_can/msg/ControlMessage` |
| ascolta `/odrive_axis0/controller_status` | `odrive_can/msg/ControllerStatus` (best effort) |
| client `/odrive_axis0/request_axis_state` | `odrive_can/srv/AxisState` |
| pubblica `~/position_rad` | `std_msgs/msg/Float64` |
| servizio `~/stop` | `std_srvs/srv/Trigger` (porta l'asse in IDLE) |

```bash
ros2 run pendulum_safety_package overrun_node
```

### `single_pendulum_package` → `joystick_first_test_node`

Muove il motore con il joystick (`/joy`, avviato dal launch) in velocità o coppia.
Parametri: `max_velocity` [rad/s] (10.0), `max_torque` [Nm] (10.0), `deadzone` (0.05).

```bash
ros2 run single_pendulum_package joystick_first_test_node
```

---

## Banda del bus (stima)

A 250 kbit/s un frame da 8 byte dura ~0.5 ms: polling a 200 Hz ≈ 15 % del bus, comandi a 500 Hz
(2 frame ciascuno) ≈ 50 %. Per controllo più veloce alzare il bitrate a 500 kbit/s o 1 Mbit/s
(stesso valore su `odrv0.can.config.baud_rate` + salvataggio/riavvio e su `ip link`).
