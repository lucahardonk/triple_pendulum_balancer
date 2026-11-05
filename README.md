# triple_pendulum_balancer
a robot to balance a homemade triple pendulum for testing control theory algorithms

## Struttura

| Cartella | Contenuto |
|---|---|
| `ros2_ws/` | workspace ROS 2 Lyrical — **istruzioni ODrive via CAN in [`ros2_ws/README.md`](ros2_ws/README.md)** |
| `ros2_ws/src/ros_odrive` | driver ufficiale ODrive (`odrive_can`) con patch per Lyrical e firmware 0.5 |
| `ros2_ws/src/pendulum_bringup` | launch dell'hardware (`hardware.launch.py`) |
| `ros2_ws/src/pendulum_safety_package` | `overrun_node`: ricerca finecorsa e centratura del carrello |
| `ros2_ws/src/single_pendulum_package` | `joystick_first_test_node`: test del motore con joystick |
| `ros2_ws/src/encoders_package` | lettura dei tre encoder del pendolo da ESP32-C6 |
| `hw_specifications/` | specifiche hardware (driver, motore, ...) |
| `single_pendulum_test/` | primi test del pendolo singolo (venv `odrive-legacy`) |


> ⚠️ Prima di accendere il motore: **USB dell'ODrive collegata (porta il GND comune) → adattatore CAN → alimentazione motore.**
> Dettagli in [`ros2_ws/README.md`](ros2_ws/README.md).
