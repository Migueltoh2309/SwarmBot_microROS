# microros_ws — lado ROS 2 / RViz de SwarmBot

Workspace ROS 2 Humble para `robot_01` (primer nodo del swarm). Contiene el
agente micro-ROS y, desde el 2026-08-31, la visualización en RViz del robot:
simulación pura (sin hardware) y bringup con el ESP32-S3 real. La
configuración del firmware (ESP-IDF, Wi-Fi, micro-ROS embebido) está
documentada aparte en `~/swarmbot/firmware/README.md`.

Estado: **RViz vinculado y verificado contra el ESP32-S3 físico**, incluyendo
un MPU6050 real publicando `/robot_01/imu` y rotando el modelo en RViz en
vivo (solo orientación en yaw, sin traslación — la posición sigue pendiente
de encoders). Ver sección 6 (2026-09-04) para el detalle completo.

Motor izquierdo con PI de velocidad funcionando bien (sin overshoot, ver
sección 7, 2026-09-12). **Motor derecho pausado por un problema de hardware
sin resolver** (conexión inestable, no un problema de software/tuning) —
continuar cuando haya mejor hardware/conexiones para revisarlo.

---

## 1. Estructura

| Paquete | Qué es |
|---|---|
| `micro_ros_setup`, `uros/micro-ROS-Agent`, `uros/micro_ros_msgs` | Toolchain y agente micro-ROS (ya existían, ver `~/swarmbot/firmware/README.md`) |
| `swarmbot_description` | URDF/xacro del robot + config de RViz (`swarmbot.rviz`) |
| `swarmbot_bringup` | Launch files: simulación pura y bringup con el robot real |

## 2. `swarmbot_description`

`urdf/swarmbot.urdf.xacro` — 8 links/frames en total:

- `base_link` — chasis (caja), origen del árbol de TF
- `wheel_left_link`, `wheel_right_link` — ruedas motorizadas (joint `continuous`, futuro origen de la odometría por encoder)
- `caster_link` — rueda loca pasiva (joint `fixed`)
- `imu_link` — punto de montaje del IMU (joint `fixed`) — **sensor confirmado**
- `range_front_link`, `range_left_link`, `range_right_link` — punto de montaje de 3 sensores ToF (joint `fixed`) — **sensor aún no confirmado**; están aislados en un solo bloque de macro (líneas 94-114 del xacro) para poder borrarse sin tocar el resto si al final no se usan

Todas las dimensiones (`chassis_length`, `wheel_radius`, etc., declaradas como
`<xacro:property>` al inicio del archivo) son **placeholders**. Se
reemplazarán cuando llegue el archivo CAD del chasis real — ese cambio solo
toca este paquete, no `swarmbot_bringup`.

Dos configs de RViz, mismas displays, distinto Fixed Frame:

- `rviz/swarmbot.rviz` — usada por `sim.launch.py`. Fixed Frame:
  `robot_01/base_link` (en modo simulación no hay ningún nodo publicando
  `odom → base_link`).
- `rviz/swarmbot_real.rviz` — usada por `real_robot.launch.py`. Fixed Frame:
  `robot_01/odom`, porque ahí sí hay algo publicando esa transformada (ver
  sección 6): `imu_orientation_tf.py` ancla la cámara a un frame que no
  rota con el robot, para poder ver la rotación.

En ambas, el display `Grid` tiene `Offset Z: -0.04` (mitad del alto del
chasis + radio de rueda, con las dimensiones placeholder actuales) para que
el piso quede a la altura de las ruedas en vez de atravesar el centro del
chasis. **Si cambian las dimensiones (CAD real), este offset hay que
recalcularlo en ambos archivos.**

Displays de Odometry y 3× Range presentes pero desactivados (apuntan a
`/robot_01/odom` y `/robot_01/range_{front,left,right}`, que aún no
existen).

## 3. `swarmbot_bringup`

Ver también `swarmbot_bringup/README.md` (convención de nombres/namespaces
para cuando existan `robot_02..robot_12`).

```bash
# Simulación pura — sin ESP32, sin agente. Para probar el modelo/URDF.
ros2 launch swarmbot_bringup sim.launch.py

# Robot real — levanta micro_ros_agent + robot_state_publisher +
# imu_orientation_tf (orientacion en yaw desde el MPU6050) + RViz.
# Requiere que este PC esté en la MISMA red que el firmware (ver sección 5).
ros2 launch swarmbot_bringup real_robot.launch.py
```

Ambos namespacean los nodos bajo `robot_01` (argumento `robot_name`) y usan
`frame_prefix` para las transformadas, dejando el terreno listo para
múltiples robots más adelante sin tener que rediseñar nada.

`scripts/imu_orientation_tf.py` (solo en `real_robot.launch.py`, fuera del
namespace a propósito — ver sección 6) se suscribe a `/robot_01/imu` y
publica el TF `robot_01/odom → robot_01/base_link`: solo rotación en yaw
(eje Z), sin traslación. Calibra el bias del gyro al arrancar (primer
segundo de lecturas, robot quieto).

## 4. Compilar

```bash
source /opt/ros/humble/setup.bash
cd ~/swarmbot/microros_ws
colcon build --packages-select swarmbot_description swarmbot_bringup
source install/setup.bash
```

## 5. Verificación con el ESP32-S3 real (hecha el 2026-08-31)

Requisito clave: **la red Wi-Fi del PC debe coincidir con la que tiene
compilada el firmware** (`CONFIG_ESP_WIFI_SSID` / `CONFIG_MICRO_ROS_AGENT_IP`
en `~/swarmbot/firmware/microros_subscriber/sdkconfig`). Si el PC está en
otra red (p. ej. la del campus), el agente nunca va a recibir sesión aunque
el ESP32 esté encendido y en rango.

Con el PC en la red correcta (`TU_RED_WIFI`, `<IP_DEL_PC>`) se
confirmó, corriendo `real_robot.launch.py`:

- `micro_ros_agent` reporta `session established` con la IP del ESP32.
- `ros2 topic echo /robot_01/status` muestra el contador `Int32` subiendo en vivo.
- `ros2 topic pub /robot_01/led_cmd std_msgs/msg/Int32 "{data: 1}"` /
  `"{data: 0}"` prende/apaga el LED físico.
- RViz: `Global Status: Ok`, sin errores en `RobotModel` ni `TF`.

**Por qué no se mueve nada en el visor 3D todavía:** el firmware actual solo
publica `/robot_01/status` (un contador, sin significado espacial) y escucha
`/robot_01/led_cmd`. No hay odometría, IMU ni joint states reales — el nodo
`joint_state_publisher` que corre en `real_robot.launch.py` solo mantiene las
ruedas en una pose de reposo mientras no exista un encoder real. Esto es
esperado, no un bug: llegará cuando se implementen motor+encoder y
odometría (ver roadmap en
`src/Configuración inicial ESP32-S3 + ESP-IDF + micro-ROS para SwarmBot.md`,
sección 16).

## 6. IMU (MPU6050) y orientación en RViz (hecho el 2026-09-04)

**Cableado** (mismo ESP32-S3-DevKitC-1 de la sección 5): VCC→3V3, GND→GND,
SDA→GPIO8, SCL→GPIO9, AD0→GND (dirección I2C 0x68). GPIO48 (LED) y los pines
USB (19/20) ya estaban ocupados; el resto del bus I2C estaba libre.
Verificado con un escaneo I2C standalone (`~/swarmbot/firmware/i2c_scan/`)
antes de tocar el firmware real: el MPU6050 respondía en 0x68 de forma
consistente.

**Firmware** (`~/swarmbot/firmware/microros_subscriber/main/main.c`): nuevo
publisher `sensor_msgs/Imu` en `/robot_01/imu` a 20Hz. Solo lectura cruda,
sin fusión a bordo — `orientation_covariance[0] = -1` indica "sin
estimación de orientación" (convención estándar del mensaje). Con el
sensor quieto sobre la mesa se verificó `linear_acceleration.z ≈ 9.86 m/s²`
(gravedad) y `angular_velocity` cerca de cero — prueba de que ejes y
cableado están bien, no solo que el I2C respondió.

**Orientación en RViz** (`swarmbot_bringup/scripts/imu_orientation_tf.py`,
nodo `imu_orientation_tf`): se suscribe a `/robot_01/imu` y publica el TF
`robot_01/odom → robot_01/base_link`. Dos decisiones deliberadas, pedidas
explícitamente:

- **Solo yaw** (rotación en Z): al ser un diferencial con rueda loca, el
  chasis siempre está plano sobre el piso — roll/pitch se fijan en 0 y no
  se usan, aunque el MPU6050 sí los reporta.
- **Sin traslación**: la transformada siempre tiene posición `(0,0,0)`. No
  hay encoders todavía, así que no hay de dónde sacar una posición real —
  moverla sería inventar datos. Es un adelanto visual de la futura
  odometría real (motor + encoder), no odometría en sí.

**Calibración del bias del gyro**: sin corrección, el yaw integrado deriva
solo con el sensor quieto (bias de fábrica del MPU6050 medido en este chip:
~-0.014 rad/s, equivalente a ~47°/min de giro fantasma). El nodo promedia
el primer segundo de lecturas al arrancar (asumiendo el robot quieto en ese
momento) y resta ese offset de ahí en adelante — el residuo bajó a
~1.2°/min en la prueba. No elimina la deriva de largo plazo (el bias real
varía con temperatura/tiempo) ni corrige el yaw contra una referencia
absoluta (no hay magnetómetro) — para eso hacen falta encoders u otra
fuente de verdad independiente.

**RViz**: el piso (`Grid`) se bajó a la altura de las ruedas en vez del
centro del chasis (ver sección 2), y `real_robot.launch.py` ahora usa
`swarmbot_real.rviz` con `Fixed Frame: robot_01/odom` en vez de
`robot_01/base_link`, para que la cámara quede anclada a un frame que no
rota con el robot y se pueda ver la rotación.

## 7. Motores (Pololu 25D + L298N + PI de velocidad) — 2026-09-12

**Estado: izquierdo funcionando bien, derecho con problema de hardware sin resolver (ver más abajo). Continuar cuando haya mejor hardware/conexiones para el motor derecho.**

### Pines

Motor izquierdo y derecho: Pololu 25D 98.78:1 con encoder 48 CPR (4741.44
cuentas x4 por vuelta del eje de salida), puente H L298N (jumper `ENA`
quitado para controlar velocidad por PWM).

| Señal | Motor izquierdo | Motor derecho |
|---|---|---|
| `IN1` | GPIO4 | GPIO16 |
| `IN2` | GPIO5 | GPIO17 |
| `ENA` (PWM) | GPIO6 | GPIO18 |
| Encoder A | GPIO7 | GPIO21 |
| Encoder B | GPIO15 | GPIO38 |
| Encoder Vcc | 3.3V del ESP32 (por debajo del mínimo 3.5V del datasheet, pero funciona en la práctica) | igual |
| GND | común con ESP32/L298N | igual |

Evitados a propósito: I2C (8/9), LED (48), USB (19/20), strapping pins
(0/3/45/46), pines de PSRAM (33-37).

### Arquitectura del lazo de control (`microros_subscriber/main/motor_control.c/h`)

Un solo driver reusable (`motor_t`) instanciado dos veces (`left_motor`,
`right_motor`), cada uno con su propio puente H, PCNT (encoder por
hardware, decodificación x4) y canal/timer LEDC independiente (canal 0 +
timer 0 el izquierdo, canal 1 + timer 1 el derecho).

Portado desde `Extra/03_control_PI_velocidad.ino` (PI de velocidad ya
validado por el usuario en Arduino), con las mismas dos piezas clave:

- **Lazo a 100Hz** (`MOTOR_CONTROL_PERIOD_MS = 10`), no 20Hz — a 20Hz la
  cuenta de encoder por ciclo es demasiado ruidosa para que cualquier
  ganancia funcione bien.
- **Filtro de media móvil** (`MOTOR_VELOCITY_FILTER_LEN = 7`) sobre la
  velocidad medida antes de metérsela al PI — sin esto, el ruido de
  cuantización domina la medición a cualquier tasa de refresco.

**Bug real encontrado al subir a 100Hz**: `motor_control_update()` usaba
el período *nominal* del timer (10ms) para convertir cuentas de encoder a
velocidad, en vez del tiempo *real* transcurrido. A 20Hz el jitter del
WiFi/ejecutor era una fracción chica del período y no se notaba; a 100Hz
medí jitter real de 7-77ms (nominal 10ms) y arruinaba el cálculo por
completo. Se arregló midiendo el dt real con `esp_timer_get_time()` en
cada llamada (`motor_t.last_update_us`).

**Ganancias por defecto** (`MOTOR_LEFT_KP/KI` en `main.c`): `Kp=0.045,
Ki=0.19`, conversión de unidades directa del PI de Arduino (`Kp=1.2,
Ki=5.0` en RPM→PWM 0-255; factor rad/s→RPM = 60/(2π) ≈ 9.5493, factor
PWM→[-1,1] = 1/255). Con el lazo a 100Hz + filtro, estas ganancias
funcionaron casi de inmediato — la parte difícil no era encontrar
ganancias nuevas, era portar completo el diseño que ya funcionaba.

Kp/Ki también son ajustables en caliente por topic (`/robot_01/wheel_<lado>/kp`
y `/kp`/`ki`, `std_msgs/Float32`) para poder iterar sin reflashear — cambiar
una ganancia resetea el integrador y el filtro (`motor_reset_control_state()`).

### Motor izquierdo — validado

Con los defaults (`kp=0.045, ki=0.19`), un escalón a 2.0 rad/s se asienta
en **1.97-2.03 rad/s, sin overshoot visible** — calidad equivalente a la
versión de Arduino. Verificado con `scripts/tune_wheel_pi.py` (barrido de
25 combinaciones Kp/Ki, 2 repeticiones c/u, contra el motor real) y a mano.

### Motor derecho — problema de hardware sin resolver

Mismo modelo de motor, mismo código, pines distintos — pero:

1. **Encoder mal cableado inicialmente**: canal B en un pin equivocado
   (leía 0 fijo, como forzado a GND). Diagnosticado con un firmware
   standalone nuevo, `~/swarmbot/firmware/gpio_scan/` (lee GPIO21/GPIO38
   crudos, sin pasar por PCNT, imprime nivel + conteo de transiciones cada
   100ms) — mucho más rápido que adivinar por software cuando ya se
   sospecha del cableado. Arreglado tras 2 rondas de revisión.
2. **El puente H dejó de responder en algún punto** (con todo cableado
   "bien" según revisión manual) y volvió a funcionar solo tras un
   reflash — no hay explicación firme, huele a conexión marginal/intermitente.
3. **Auto-tuning no converge**: 3 barridos completos (`tune_wheel_pi.py`),
   cada uno peor que el anterior (el último con casi todas las
   combinaciones "stalled"). Esto ya no es un problema de ganancias — si
   NINGUNA combinación funciona bien y además empeora entre corridas, es
   la firma de una conexión inestable, no de un lazo mal afinado.

**Antes de seguir con el motor derecho**: revisar continuidad con
multímetro (no solo a simple vista) en los cables de potencia y lógica, y
verificar si el canal del L298N se calentó de más (posible daño del
incidente del punto 2) — probar ese motor en un canal/L298N distinto para
descartar el chip.

### Herramientas creadas (reusables para cualquier rueda futura)

- `swarmbot_bringup/scripts/tune_wheel_pi.py` — barrido de Kp/Ki contra el
  motor real, parametrizado por `wheel:=wheel_left|wheel_right`.
- `swarmbot_bringup/scripts/test_wheel_ramp.py` — rampa 0→target_max→0,
  mide RMSE de tracking y overshoot.
- `~/swarmbot/firmware/gpio_scan/` — firmware standalone para leer
  cualquier par de GPIO crudos (útil para diagnosticar cualquier señal
  digital sin pasar por drivers de alto nivel).

## 8. Bugs encontrados y corregidos durante la integración

Todos ya están arreglados en `swarmbot_description/rviz/swarmbot.rviz`,
verificados abriendo RViz de verdad (no solo por logs):

1. **Topic del `RobotModel` sin namespacear**: apuntaba a `/robot_description`
   en vez de `/robot_01/robot_description`.
2. **QoS incompatible**: el topic de `robot_description` es *Transient Local*
   (mensaje "latched") en `robot_state_publisher`; el display lo pedía como
   *Volatile* y nunca lo recibía si RViz se conectaba después del publish.
3. **`TF Prefix` vacío en el display `RobotModel`**: sin `TF Prefix: robot_01`,
   RViz buscaba frames sin prefijo (`base_link`) en vez de `robot_01/base_link`,
   y fallaba con "No transform from [base_link]" aunque TF mostrara `Status: Ok`.
4. **Plugin `rviz_default_plugins/Imu` ausente** en la versión de RViz2
   instalada en esta máquina (`ros-humble-rviz-default-plugins` 11.2.19) — no
   es un error de nombre, la clase no existe en este build. Se quitó el
   display Imu de la config; el TF de orientación de la sección 6 lo
   esquiva por completo (no depende de ese plugin), así que ya no bloquea
   nada — solo revisar si una actualización del paquete la trae de vuelta,
   por si se quiere el display nativo más adelante.
5. **`CMakeLists.txt` con `PRIV_REQUIRES` incompleto** (2026-09-04): al
   agregar `PRIV_REQUIRES driver` en
   `microros_subscriber/main/CMakeLists.txt` para el I2C, ESP-IDF dejó de
   autodetectar dependencias del componente `main` y rompió includes que
   antes "aparecían solos" (`esp_timer.h`, `uros_network_interfaces.h`,
   `led_strip.h`). Hubo que listar explícitamente `driver esp_timer
   micro_ros_espidf_component espressif__led_strip`.
6. **Deriva del yaw por bias de gyro sin calibrar** (2026-09-04, ver sección
   6): sin calibración, el robot "giraba solo" en RViz con el sensor
   perfectamente quieto. No era un bug de TF ni de RViz — es el offset de
   fábrica del MPU6050 sin corregir.
7. **`motor_control_update()` asumía el período nominal del timer como dt
   real** (2026-09-12, ver sección 7): funcionaba "bien" a 20Hz porque el
   jitter era una fracción chica del período, pero al subir a 100Hz para
   el PI de velocidad el jitter real (medido: 7-77ms contra un nominal de
   10ms) arruinaba el cálculo de velocidad por completo. Se arregló
   midiendo el dt real con `esp_timer_get_time()` en cada llamada.
8. **Encoder del motor derecho mal cableado** (2026-09-12): canal B
   conectado a un pin equivocado, se leía fijo en 0 (como forzado a GND).
   Diagnosticado con un firmware standalone (`gpio_scan`) que lee los pines
   crudos sin pasar por PCNT — más rápido que adivinar por software una
   vez que se sospecha del cableado.

## 9. Próximos pasos (no implementados todavía)

- **Motor derecho**: resolver el problema de hardware (ver sección 7) antes
  de retomar el auto-tuning — probablemente cambiar de canal L298N o
  revisar continuidad con multímetro. El izquierdo ya está listo.
- Odometría real (traslación) combinando ambos encoders → que el robot se
  mueva de verdad en RViz, no solo rote sobre sí mismo. Requiere el motor
  derecho funcionando de forma confiable primero.
- `cmd_vel` (differential drive Twist) → convertir a target de cada rueda,
  una vez que ambos motores esten tuneados y confiables.
- ToF (si finalmente se usan) → activar los 3 displays `Range` ya preparados.
- Reemplazar geometría placeholder del xacro por el CAD real del chasis (y
  recalcular el offset del Grid en ambos `.rviz`, ver sección 2).
- Launch multi-robot cuando existan `robot_02..robot_12` (namespace/frame_prefix
  ya soportan esto, falta el archivo de orquestación).
- Opcional: mejorar la calibración del gyro (p. ej. recalibrar cuando el
  robot lleve varios segundos detenido) si el residuo de deriva molesta en
  uso prolongado.
- Si se necesita yaw absoluto (sin deriva) más adelante: el MPU6050 no
  tiene magnetómetro. Opciones evaluadas (2026-09-04, no compradas
  todavía): **BNO055** (fusión sensorial a bordo, da orientación ya
  calculada, más caro/grande) o **ICM-20948** (9-DoF crudo, sucesor en
  producción del MPU9250 ya descontinuado, más barato pero hay que escribir
  el driver del magnetómetro y la fusión a mano). Cualquiera de los dos hay
  que montarlo lejos de los motores N20 (interferencia magnética).

## 10. Cómo correr todo sin internet

Ver [`GUIA_OFFLINE.md`](./GUIA_OFFLINE.md) — guía paso a paso para levantar
todo el sistema (firmware + agente + RViz) usando solo la red local
(router/hotspot con Wi-Fi, sin salida a internet), incluyendo cómo cambiar
la red que trae compilada el firmware si cambias de router.
