# SwarmBot micro-ROS

Enjambre de robots móviles diferenciales (`robot_01` … `robot_12`) basados en
**ESP32-S3** con **ESP-IDF** y **micro-ROS**, comunicados por Wi-Fi con
**ROS 2 Humble** en el PC. Cada robot es un nodo ROS 2 propio que publica sus
sensores y recibe consignas para sus ruedas.

> 🚧 **En desarrollo.** El primer robot (`robot_01`) ya tiene comunicación
> bidireccional estable, IMU y el lazo PI de velocidad de una rueda. Ver
> [Estado](#estado).

```text
 ┌──────────────── robot_01 (ESP32-S3) ────────────────┐          ┌──────────── PC (ROS 2 Humble) ────────────┐
 │ MPU6050 (I2C) ──► /robot_01/imu            20 Hz    │          │ micro_ros_agent (UDP :8888)               │
 │ encoder + L298N ◄─► PI de velocidad        100 Hz   │  Wi-Fi   │ robot_state_publisher + URDF              │
 │   /robot_01/wheel_left/{target_velocity,velocity}   │◄────────►│ imu_orientation_tf  (yaw desde la IMU)    │
 │ LED WS2812 ◄── /robot_01/led_cmd                    │ UDP/XRCE │ RViz2                                     │
 │ /robot_01/status (latido)                           │          │ scripts de sintonía del PI                │
 └─────────────────────────────────────────────────────┘          └───────────────────────────────────────────┘
```

## Qué incluye

### Firmware (`firmware/`): ESP32-S3 + ESP-IDF v5.5

Proyectos ESP-IDF en orden de complejidad, cada uno probado en la placa
ESP32-S3-DevKitC-1:

| Proyecto | Qué hace |
|---|---|
| `hello_world/` | Verificación del toolchain y de la placa |
| `blink/` | LED RGB WS2812 integrado (GPIO48) con `led_strip` |
| `freertos_blink/` | El mismo blink en una tarea FreeRTOS dedicada |
| `wifi_test/` | Conexión Wi-Fi en modo estación con reconexión indefinida |
| `i2c_scan/` | Escaneo del bus I2C (detección del MPU6050) |
| `gpio_scan/` | Lectura cruda de GPIO para diagnosticar cableado (p. ej. encoders) |
| `microros_publisher/` | Primer nodo micro-ROS: publica un contador en `/robot_01/status` |
| `microros_subscriber/` | **Firmware principal del robot**: publisher + subscriber simultáneos, IMU a 20 Hz, LED por tópico, control PI de velocidad de rueda a 100 Hz (PWM LEDC + encoder por PCNT) y recuperación automática si se cae el agente |

El lazo de velocidad se portó desde una versión en Arduino, que está en
[`Extra/03_control_PI_velocidad.ino`](Extra/03_control_PI_velocidad.ino) como referencia.

### Simulación y lado ROS 2 (`microros_ws/`)

| Paquete | Qué hace |
|---|---|
| `swarmbot_description` | Modelo URDF/xacro del robot (chasis, 2 ruedas motrices, rueda loca, IMU y 3 sensores de distancia; dimensiones provisionales hasta tener el CAD) y configuraciones de RViz |
| `swarmbot_bringup` | `sim.launch.py`: simulación del modelo en RViz2, sin hardware, moviendo las articulaciones con `joint_state_publisher_gui`. `real_robot.launch.py`: agente micro-ROS + modelo + orientación en vivo desde la IMU del robot real. Scripts para sintonizar el PI (`tune_wheel_pi.py`) y medir el seguimiento (`test_wheel_ramp.py`) |

Todos los nodos van bajo el namespace del robot (`robot_01`) con
`frame_prefix`, preparados para varios robots a la vez.

## Hardware de `robot_01`

| Componente | Detalle |
|---|---|
| Microcontrolador | ESP32-S3-DevKitC-1 (16 MB flash, 8 MB PSRAM) |
| IMU | MPU6050 por I2C (SDA = GPIO8, SCL = GPIO9) |
| Motores | Pololu 25D 98.78:1 con encoder de 48 CPR (4741.44 cuentas por vuelta) |
| Driver | L298N (izquierdo: IN1/IN2/PWM = GPIO4/5/6, encoder = GPIO7/15) |
| LED | WS2812 integrado (GPIO48) |

## Estado

| Hito | Estado |
|---|---|
| Toolchain, Wi-Fi y micro-ROS por UDP | ✅ |
| Comunicación bidireccional ESP32 ↔ ROS 2 | ✅ |
| Robustez: reinicio del ESP32, del agente y pérdida de Wi-Fi, con recuperación automática | ✅ |
| IMU en `/robot_01/imu` y orientación (yaw) del modelo en RViz2 | ✅ |
| PI de velocidad del motor izquierdo (escalón a 2 rad/s sin sobrepaso) | ✅ |
| Motor derecho | ⏸️ pausado por un problema de conexiones del hardware |
| Odometría con ambos encoders y `cmd_vel` | 📋 pendiente |
| Sensores de distancia ToF, chasis definitivo (CAD) y lanzamiento multi-robot | 📋 pendiente |

## Puesta en marcha

**Firmware** (necesita ESP-IDF v5.5 y el componente micro-ROS):

```bash
git clone -b v5.5 --recursive https://github.com/espressif/esp-idf.git esp/esp-idf
./esp/esp-idf/install.sh esp32s3
. esp/esp-idf/export.sh
pip3 install catkin_pkg colcon-common-extensions lark "empy<4"

cd firmware/microros_subscriber
git clone -b humble https://github.com/micro-ROS/micro_ros_espidf_component.git \
    components/micro_ros_espidf_component
# Red Wi-Fi e IP del PC (este archivo no se sube a git)
cat > sdkconfig.defaults.local <<'EOF'
CONFIG_ESP_WIFI_SSID="mi_red"
CONFIG_ESP_WIFI_PASSWORD="mi_password"
CONFIG_MICRO_ROS_AGENT_IP="192.168.1.100"
EOF
idf.py build flash monitor
```

El ESP32-S3 solo admite Wi-Fi de 2.4 GHz.

**ROS 2** (PC en la misma red que el robot):

```bash
cd microros_ws
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build && source install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh && ros2 run micro_ros_setup build_agent.sh
source install/setup.bash

ros2 launch swarmbot_bringup sim.launch.py          # simulación, sin hardware
ros2 launch swarmbot_bringup real_robot.launch.py   # con el robot real
```

## Documentación

| Documento | Contenido |
|---|---|
| [`firmware/README.md`](firmware/README.md) | Configuración paso a paso, cada proyecto de firmware, pruebas de robustez y decisiones tomadas |
| [`microros_ws/README.md`](microros_ws/README.md) | URDF, launch files, IMU, motores, bugs corregidos y próximos pasos |
| [`microros_ws/GUIA_OFFLINE.md`](microros_ws/GUIA_OFFLINE.md) | Cómo correr todo sin internet |
| [`microros_ws/src/swarmbot_bringup/README.md`](microros_ws/src/swarmbot_bringup/README.md) | Convención de nombres y namespaces para el enjambre |

## Autor

Miguel Olortegui — UTEC
