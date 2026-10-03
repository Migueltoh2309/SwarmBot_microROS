# Guía offline — correr SwarmBot sin internet

Todo lo de este workspace corre **100% local**: el `micro_ros_agent` vive en
tu propia PC, ROS 2/RViz no llaman a ningún servidor externo, y el toolchain
de ESP-IDF ya está instalado en `~/swarmbot/esp/esp-idf` (no descarga nada al
compilar). El único requisito de red es que **el PC y el ESP32-S3 estén en
la misma red Wi-Fi** — esa red no necesita salida a internet para nada de
esto. Sirve un router de casa, un hotspot de celular, o un router portátil
sin cable WAN conectado; lo único que no sirve es una red que el ESP32-S3 no
soporte (6GHz — el chip solo hace 2.4GHz y 5GHz).

Si vas a usar una red distinta a la que ya tiene compilada el firmware, hay
que reflashear (paso 1). Si vas a usar la misma red de siempre, saltate al
paso 2.

## 0. Config actual grabada en el firmware (referencia)

```
CONFIG_ESP_WIFI_SSID="TU_RED_WIFI"
CONFIG_ESP_WIFI_PASSWORD="TU_PASSWORD"
CONFIG_MICRO_ROS_AGENT_IP="<IP_DEL_PC>"
CONFIG_MICRO_ROS_AGENT_PORT="8888"
```

(`microros_subscriber/sdkconfig` y `sdkconfig.defaults.local` — si cambias de
red, edita ambos archivos: `sdkconfig` para el build actual,
`sdkconfig.defaults.local` para que el cambio sobreviva a un
`idf.py fullclean`. Ese `.local` no se sube a git; ver la sección 6 de
`firmware/README.md`.)

## 1. Cambiar de red (solo si el router/hotspot es distinto al de arriba)

1. Conecta el PC a la red nueva y anota su IP:
   ```bash
   nmcli -t -f active,ssid,device dev wifi 2>/dev/null | grep '^yes'  # confirma que estas conectado
   ip -4 addr show | grep inet                                        # anota la IP del PC en esa red
   ```
2. Edita `~/swarmbot/firmware/microros_subscriber/sdkconfig` (y
   `sdkconfig.defaults`), reemplazando las 4 líneas de la sección 0 con la
   nueva SSID, password, y la IP del PC que acabas de anotar. El puerto
   (`8888`) normalmente no hace falta cambiarlo.
3. Compila y flashea:
   ```bash
   source ~/swarmbot/esp/esp-idf/export.sh
   cd ~/swarmbot/firmware/microros_subscriber
   idf.py -p /dev/ttyACM0 build flash
   ```
   Si `/dev/ttyACM0` da "Permission denied", ver la sección de problemas
   comunes más abajo (grupo `dialout`).

## 2. Conectar todo a la misma red

1. Conecta el PC (WiFi del sistema operativo) a la red que tiene compilada
   el firmware (sección 0, o la que hayas puesto en el paso 1).
2. Enciende el ESP32-S3 (USB o batería) — se conecta solo a esa red al
   bootear, no requiere ninguna acción más.
3. Verifica que la IP del PC en esa red coincide con
   `CONFIG_MICRO_ROS_AGENT_IP` del firmware:
   ```bash
   ip -4 addr show | grep inet
   ```
   Si no coincide (p. ej. el router te dio otra IP), o edita el firmware
   (paso 1) o fuerza una IP fija para el PC en la config del router —
   cualquiera de las dos funciona, no hace falta reflashear si solo cambia
   la IP y editas el sdkconfig.

## 3. Compilar el workspace ROS 2 (una sola vez, o si tocaste código)

```bash
source /opt/ros/humble/setup.bash
cd ~/swarmbot/microros_ws
colcon build --packages-select swarmbot_description swarmbot_bringup
source install/setup.bash
```

## 4. Levantar todo

```bash
source /opt/ros/humble/setup.bash
source ~/swarmbot/microros_ws/install/setup.bash

# Simulación pura — sin ESP32, sin red, sin agente. Para probar el modelo.
ros2 launch swarmbot_bringup sim.launch.py

# Robot real — agente + robot_state_publisher + imu_orientation_tf + RViz.
# Requiere el PC en la misma red que el firmware (secciones 1-2).
ros2 launch swarmbot_bringup real_robot.launch.py
```

Para cerrar todo: `Ctrl+C` en la terminal donde corre `ros2 launch` (mata
todos los procesos hijos).

## 5. Verificar que está vivo (sin RViz, solo terminal)

```bash
source /opt/ros/humble/setup.bash
source ~/swarmbot/microros_ws/install/setup.bash

ros2 topic list                                   # deberia listar /robot_01/status, /robot_01/imu, /robot_01/led_cmd
ros2 topic echo /robot_01/status --once            # contador subiendo
ros2 topic echo /robot_01/imu --once               # datos crudos del MPU6050
ros2 topic pub /robot_01/led_cmd std_msgs/msg/Int32 "{data: 1}"  # prende el LED
ros2 run tf2_ros tf2_echo robot_01/odom robot_01/base_link       # orientacion (yaw) en vivo
```

## 6. Reflashear el firmware (cambios de código, no solo de red)

```bash
source ~/swarmbot/esp/esp-idf/export.sh
cd ~/swarmbot/firmware/microros_subscriber
idf.py -p /dev/ttyACM0 build flash
```

## Problemas comunes

- **`micro_ros_agent` nunca dice "session established"**: el PC y el ESP32
  están en redes distintas, o el PC está en una red de 6GHz (el ESP32-S3 no
  la soporta). Revisa `ip -4 addr show` en el PC contra
  `CONFIG_MICRO_ROS_AGENT_IP` del firmware, y confirma que ambos dispositivos
  están en el mismo SSID.
- **`idf.py -p /dev/ttyACM0 flash` da "Permission denied"**: tu usuario no
  está en el grupo `dialout`.
  ```bash
  groups                        # revisa si aparece "dialout"
  sudo usermod -aG dialout $USER
  ```
  Hace falta cerrar sesión (o reiniciar) para que el cambio de grupo tome
  efecto.
- **RViz se queja de que falta el plugin `rviz_default_plugins/Imu`**: no
  importa, no se usa — la orientación se ve por TF (`imu_orientation_tf.py`),
  no por ese display. Ver sección 6 del README principal.
- **El robot "gira solo" en RViz apenas arranca, sin tocar el sensor**:
  normal durante el primer segundo (`imu_orientation_tf` está calibrando el
  bias del gyro, robot debe estar quieto en ese momento). Si sigue
  girando después de esos ~2-3 segundos, revisar el bias medido en el log
  del nodo (`Calibracion lista: bias gyro Z = ...`).
- **Orphan processes** (`rviz2`/`robot_state_publisher`/etc. quedan
  corriendo tras un `Ctrl+C` que no cerró todo):
  ```bash
  pkill -9 -f "rviz2|robot_state_publisher|joint_state_publisher|micro_ros_agent|imu_orientation_tf"
  ```
