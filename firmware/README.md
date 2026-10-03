# SwarmBot — ESP32-S3 + ESP-IDF + micro-ROS

Configuración inicial para el robot móvil `robot_01`, primer nodo de un swarm de
hasta 12 robots. Este README documenta paso a paso la configuración de una
máquina de desarrollo Ubuntu + ROS 2 Humble, para que el entorno sea
reproducible en otra computadora. Se basa en las indicaciones de
`~/swarmbot/microros_ws/src/Configuración inicial ESP32-S3 + ESP-IDF + micro-ROS para SwarmBot.md`.

Estado: **Fases 1-3 completas**, incluyendo el criterio de éxito de la sección 12 (comunicación bidireccional ESP32↔ROS2 simultánea por Wi-Fi/UDP) y **los 3 tests de robustez de la sección 13** (reinicio del ESP32, reinicio del Agente, pérdida de Wi-Fi — los tres con recuperación automática, sin intervención física). Queda documentada una salvedad operativa (sesiones zombie en el Agente tras reboots repetidos, sección 4.7) a revisar cuando haya varios robots simultáneos. Siguiente: hardware (IMU, motores+encoder — sección 16).

---

## 1. Versiones utilizadas

| Componente | Versión / branch |
|---|---|
| Sistema operativo | Ubuntu (host actual) |
| ROS 2 | Humble |
| ESP-IDF | `v5.5` (rama estable, clonada con `--recursive`) |
| micro-ROS (agente) | `micro_ros_setup`, branch `humble` |
| micro-ROS (firmware, componente ESP-IDF) | `micro_ros_espidf_component`, branch `humble` (pendiente de integrar — Fase 3) |
| Middleware | Micro XRCE-DDS (default) |
| Transporte | UDP/IPv4 sobre Wi-Fi |

## 2. Hardware detectado

Identificado con `esptool.py` (Fase 1, sección 3.1 del documento):

| Campo | Valor |
|---|---|
| Placa | ESP32-S3-DevKitC-1 (oficial Espressif, confirmada por serigrafía) |
| Chip | ESP32-S3 (QFN56), revisión v0.2 |
| Features | WiFi, BLE, PSRAM embebida 8MB (AP_3v3) |
| Flash | 16MB, quad (4 líneas de datos) |
| MAC | (omitida) |
| Puerto USB | `/dev/ttyACM0` — **USB nativo JTAG/Serial** del chip (`303a:1001`, no adaptador CP210x/CH340 externo) |
| LED integrado | RGB direccionable (WS2812), confirmado físicamente en **GPIO48** (se probó primero GPIO38, que es el default de ESP-IDF para revisiones v1.1 de la DevKitC-1, pero no respondió; GPIO48 sí) |

## 3. Configuración del sistema (host de desarrollo)

### 3.1 Paquetes del sistema (requeridos por ESP-IDF)

```bash
sudo apt update && sudo apt install -y git wget flex bison gperf python3 python3-pip \
  python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0-dev
```

### 3.2 Permisos de puerto serie

El usuario debe pertenecer al grupo `dialout` para acceder a `/dev/ttyACM0` sin `sudo`:

```bash
sudo usermod -aG dialout $USER
# cerrar sesión y volver a entrar (o usar `sg dialout -c "<comando>"` para no reiniciar sesión)
```

### 3.3 ESP-IDF

Instalado en `~/swarmbot/esp/esp-idf`, target `esp32s3` únicamente (para instalar más targets, volver a correr `install.sh` con la lista deseada):

```bash
mkdir -p ~/swarmbot/esp && cd ~/swarmbot/esp
git clone -b v5.5 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32s3
```

Para activar el entorno en cada terminal nueva:

```bash
. ~/swarmbot/esp/esp-idf/export.sh
```

### 3.4 Workspace del agente micro-ROS

Compilado desde código fuente (no se usó Docker) en `~/swarmbot/microros_ws`:

```bash
pip3 install --user vcstool   # rosdep no pudo instalar python3-vcstool por falta de sudo interactivo

source /opt/ros/humble/setup.bash
mkdir -p ~/swarmbot/microros_ws/src && cd ~/swarmbot/microros_ws
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build

source install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

Binario resultante: `~/swarmbot/microros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent`

Para correr el agente (UDP, puerto 8888 — ver sección 6):

```bash
source ~/swarmbot/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

## 4. Proyectos de firmware (este directorio)

| Carpeta | Fase del documento | Estado |
|---|---|---|
| `hello_world/` | 4 — Hello World | ✅ hecho |
| `blink/` | 5 — Blink (LED RGB WS2812 vía `led_strip`, ver nota abajo) | ✅ hecho |
| `freertos_blink/` | 6 — Blink con tarea FreeRTOS dedicada | ✅ hecho |
| `wifi_test/` | 7 — Conexión Wi-Fi | ✅ hecho |
| `microros_publisher/` | 10 — Publisher `/robot_01/status` | ✅ hecho |
| `microros_subscriber/` | 11-12 — Subscriber `/robot_01/led_cmd` + pub/sub simultáneo | ✅ hecho |

Cada subcarpeta tendrá sus propios comandos de compilar/flashear documentados
abajo a medida que se completen.

### 4.1 Hello World (`hello_world/`)

Copiado sin modificaciones desde `$IDF_PATH/examples/get-started/hello_world`.

```bash
. ~/swarmbot/esp/esp-idf/export.sh
cp -r ~/swarmbot/esp/esp-idf/examples/get-started/hello_world ~/swarmbot/firmware/hello_world
cd ~/swarmbot/firmware/hello_world
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash
idf.py -p /dev/ttyACM0 monitor
```

Resultado verificado: boot limpio, sin resets inesperados, imprime `Hello world!`
y la info del chip por el puerto serie.

Nota menor: aparece un warning `Detected size(16384k) larger than the size in
the binary image header(2048k)` porque el sdkconfig por defecto del ejemplo
asume 2MB de flash. No afecta esta prueba; a definir el tamaño real (16MB) en
sdkconfig cuando se arme el proyecto final.

### 4.2 Blink (`blink/`)

Copiado desde `$IDF_PATH/examples/get-started/blink`.

**Desviación respecto al documento de indicaciones:** la sección 5 del
documento asume un LED simple controlable con `gpio_set_direction()` /
`gpio_set_level()`. La ESP32-S3-DevKitC-1 en realidad trae un **LED RGB
direccionable WS2812** (protocolo serie de un solo hilo, no un GPIO on/off
convencional), así que se usó el driver oficial `led_strip` (basado en RMT)
que ya trae el ejemplo de ESP-IDF. El comportamiento pedido (ON 500ms / OFF
500ms vía `vTaskDelay`) se mantiene igual, solo cambia el mecanismo de bajo
nivel para "prender" el LED.

El GPIO exacto tampoco se puede asumir: el default de ESP-IDF para esp32s3
(`sdkconfig.defaults.esp32s3`) es GPIO38 (DevKitC-1 rev v1.1), pero **se probó
físicamente en la placa y no respondió**; **GPIO48** (rev v1.0) sí funcionó.

```bash
. ~/swarmbot/esp/esp-idf/export.sh
cp -r ~/swarmbot/esp/esp-idf/examples/get-started/blink ~/swarmbot/firmware/blink
cd ~/swarmbot/firmware/blink
sed -i 's/CONFIG_BLINK_GPIO=38/CONFIG_BLINK_GPIO=48/' sdkconfig.defaults.esp32s3
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash
```

Color modificado a verde (solo por preferencia visual) editando
`main/blink_example_main.c`, función `blink_led()`:

```c
led_strip_set_pixel(led_strip, 0, 0, 16, 0);  // R, G, B — verde tenue
```

Resultado verificado: LED parpadea en verde, ON/OFF cada 500ms.

### 4.3 FreeRTOS — tarea dedicada (`freertos_blink/`)

Copiado desde `blink/`, con el archivo principal renombrado a
`freertos_blink_main.c` (y actualizado `main/CMakeLists.txt` +
`project()` en el `CMakeLists.txt` raíz).

El parpadeo se movió del loop de `app_main()` a una tarea FreeRTOS dedicada
`blink_task`, creada con `xTaskCreate()`:

```c
xTaskCreate(blink_task, "blink_task", BLINK_TASK_STACK_SIZE, NULL,
            BLINK_TASK_PRIORITY, NULL);
```

`blink_task` usa `vTaskDelayUntil()` (no `vTaskDelay()`) para una ejecución
periódica exacta, que no acumula drift por el tiempo que toma
`led_strip_refresh()`. `app_main()` retorna después de crear la tarea; el
scheduler sigue corriendo `blink_task` de forma independiente — así se
comprueba creación de tareas, prioridades y funcionamiento del scheduler,
sin necesidad todavía de fijar afinidad de núcleo.

Se agregó `CONFIG_BLINK_PERIOD=500` a `sdkconfig.defaults.esp32s3` (el
default de ESP-IDF es 1000ms; el documento pide 500ms ON / 500ms OFF).

```bash
. ~/swarmbot/esp/esp-idf/export.sh
cp -r ~/swarmbot/firmware/blink ~/swarmbot/firmware/freertos_blink
cd ~/swarmbot/firmware/freertos_blink
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash
```

Resultado verificado por log serie: transiciones `LED ON` / `LED OFF`
exactamente cada 500ms, tarea corriendo de forma estable.

### 4.4 Wi-Fi (`wifi_test/`)

Copiado desde `$IDF_PATH/examples/wifi/getting_started/station`, archivo
principal renombrado a `wifi_test_main.c`.

**Modificación respecto al ejemplo oficial:** el ejemplo original se rinde
(`WIFI_FAIL_BIT`) tras `CONFIG_ESP_MAXIMUM_RETRY` (5) intentos fallidos y
deja de reconectar. Un robot del swarm necesita reconexión **indefinida**
(sección 13 del documento — robustez ante Wi-Fi intermitente), así que se
simplificó `event_handler()` para llamar `esp_wifi_connect()` en cada
`WIFI_EVENT_STA_DISCONNECTED` sin límite de reintentos, y se agregaron los
logs exactos que pide la sección 7 del documento: `WiFi connected` / `IP: ...`.

SSID y password se configuraron en `sdkconfig.defaults` (`CONFIG_ESP_WIFI_SSID`,
`CONFIG_ESP_WIFI_PASSWORD` — **ojo: `sdkconfig` contiene la contraseña en texto
plano, no debe subirse a un repositorio público**).

```bash
. ~/swarmbot/esp/esp-idf/export.sh
cp -r ~/swarmbot/esp/esp-idf/examples/wifi/getting_started/station ~/swarmbot/firmware/wifi_test
cd ~/swarmbot/firmware/wifi_test
# editar sdkconfig.defaults con SSID/password, o usar idf.py menuconfig
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash
```

Resultado verificado por log serie:

```
I (500) wifi:mode : sta (xx:xx:xx:xx:xx:xx)
I (540) wifi_test: WiFi disconnected, reintentando...   # primeros intentos de asociación
I (3000) wifi:connected with TU_RED_WIFI, aid = 2, channel 1, BW20, ...
I (3000) wifi:security: WPA2-PSK, phy: bgn, rssi: -42
I (4060) wifi_test: WiFi connected
I (4060) wifi_test: IP: <IP_DEL_ESP32>
```

Y `ping <IP_DEL_ESP32>` desde la laptop: 0% packet loss, confirmando
conectividad IP end-to-end sobre Wi-Fi (sección 7 del documento).

Pendiente para la fase de robustez (sección 13): probar reinicio del ESP32
ya conectado, y pérdida/recuperación de la señal Wi-Fi en caliente — se
dejará para antes de cerrar la Fase 3 (micro-ROS), como indica el documento.

### 4.5 micro-ROS — Publisher (`microros_publisher/`)

Proyecto nuevo (no un ejemplo copiado 1:1) que integra
`micro_ros_espidf_component` (rama `humble`, clonado dentro de
`components/micro_ros_espidf_component`, sin `.git` para no duplicar
historial) y adapta el ejemplo `examples/int32_publisher` del propio
componente: nodo **`robot_01`**, publica `std_msgs/msg/Int32` incremental en
**`/robot_01/status`** cada 1000ms.

Dependencias Python necesarias dentro del entorno virtual de ESP-IDF (una
sola vez, no dependen del proyecto):

```bash
. ~/swarmbot/esp/esp-idf/export.sh
pip3 install catkin_pkg colcon-common-extensions lark "empy<4"
```

Configuración de WiFi + Agente en `sdkconfig.defaults`:

```
CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE=y
CONFIG_MICRO_ROS_ESP_NETIF_WLAN=y
CONFIG_ESP_WIFI_SSID="TU_RED_WIFI"
CONFIG_ESP_WIFI_PASSWORD="TU_PASSWORD"
CONFIG_MICRO_ROS_AGENT_IP="<IP_DEL_PC>"
CONFIG_MICRO_ROS_AGENT_PORT="8888"
```

**Nota de seguridad:** el propio componente (`micro_ros_espidf_component`,
capa `wifi_station_netif`) imprime el password del WiFi en texto plano por
el log serie al conectar (`connected to ap SSID:... password:...`). Es un
comportamiento de la librería, no de nuestro código — tenerlo en cuenta si
se comparten logs de consola públicamente.

```bash
. ~/swarmbot/esp/esp-idf/export.sh
cd ~/swarmbot/firmware/microros_publisher
idf.py set-target esp32s3   # la primera vez compila libmicroros.a via colcon, tarda varios minutos
idf.py build
idf.py -p /dev/ttyACM0 flash
```

Levantar el Agente en la laptop:

```bash
source /opt/ros/humble/setup.bash
source ~/swarmbot/microros_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Resultado verificado (sección 10 del documento):

```bash
$ ros2 topic list
/parameter_events
/robot_01/status
/rosout

$ ros2 topic echo /robot_01/status
data: 23
---
data: 24
---
```

ESP32-S3 → ROS 2 confirmado.

### 4.6 micro-ROS — Publisher + Subscriber combinados (`microros_subscriber/`)

Copia de `microros_publisher/` extendida para cumplir simultáneamente las
secciones 11 y 12 del documento: mismo nodo `robot_01`, mismo publisher
`/robot_01/status`, y ahora además un **subscriber** a
**`/robot_01/led_cmd`** (`std_msgs/msg/Int32`) que controla el LED físico
(WS2812 en GPIO48, mismo driver `led_strip` de `blink/`): `0` → LED OFF,
`1` → LED ON. Basado en el ejemplo oficial
`micro_ros_espidf_component/examples/int32_sub_pub`.

Dependencia extra en `main/idf_component.yml` (además del componente
micro-ROS ya clonado en `components/`):

```yaml
dependencies:
  espressif/led_strip: "^3.0.0"
```

Mismo `sdkconfig.defaults` (SSID/password/Agent IP/puerto) que
`microros_publisher/`.

```bash
. ~/swarmbot/esp/esp-idf/export.sh
cd ~/swarmbot/firmware/microros_subscriber
idf.py set-target esp32s3   # vuelve a compilar libmicroros.a (proyecto nuevo = cache nueva)
idf.py build
idf.py -p /dev/ttyACM0 flash
```

Con el Agente corriendo (ver 4.5), resultado verificado — **criterio de
éxito de la sección 12, cumplido**:

```bash
$ ros2 topic list
/parameter_events
/robot_01/led_cmd
/robot_01/status
/rosout

$ ros2 topic echo /robot_01/status     # sigue publicando, en paralelo
data: 17
---
data: 18
---

$ ros2 topic pub --once /robot_01/led_cmd std_msgs/msg/Int32 "{data: 1}"
# LED fisico se enciende (verde)

$ ros2 topic pub --once /robot_01/led_cmd std_msgs/msg/Int32 "{data: 0}"
# LED fisico se apaga
```

ESP32-S3 → ROS2 (status) y ROS2 → ESP32-S3 (led_cmd) confirmados
funcionando **al mismo tiempo**, por Wi-Fi/UDP, sin USB para los mensajes
(USB solo se usó para flashear).

### 4.7 Pruebas de robustez (sección 13)

#### Test 1 — Reinicio del ESP32 ya conectado

Reset físico (vía RTS, sin reflashear) con el Agente corriendo. Resultado:
reconecta solo (boot → Wi-Fi → Agente → ROS2), publisher retoma desde 0,
`led_cmd` sigue funcionando. **Pasado.**

#### Test 2 — Reinicio del Agente (sin tocar el ESP32)

**Primer intento — falló.** Con el firmware original de 4.6 (crea las
entidades una sola vez en `app_main`, sin ningún chequeo de vida del
Agente), al matar y volver a levantar el Agente el ESP32 quedó sordo:
`/robot_01/status` desapareció de `ros2 topic list` y nunca volvió, porque
el firmware no tiene forma de detectar que la sesión anterior murió.

**Segundo intento — máquina de estados clásica de micro-ROS (descartado).**
Se implementó el patrón estándar `WAITING_AGENT → AGENT_AVAILABLE →
AGENT_CONNECTED → AGENT_DISCONNECTED`, destruyendo y recreando las
entidades RCL en caliente según el estado de un ping periódico
(`rmw_uros_ping_agent`). Se encontraron dos bugs reales al probarlo:

1. En `WAITING_AGENT`, `rmw_uros_ping_agent(100, 1)` a secas nunca
   funcionó — esa función necesita un transporte ya configurado con una
   IP, y en ese estado todavía no existe ninguna sesión. El ESP32 se
   quedaba pegado imprimiendo "Waiting for Agent" para siempre aunque el
   Agente estuviera arriba. Fix: usar `rmw_uros_ping_agent_options()` con
   unas `rmw_init_options_t` locales con la IP/puerto puestos a mano.
2. Al reusar esa misma función con IP explícita dentro de `AGENT_CONNECTED`
   (para chequear que la sesión activa seguía viva), cada ping abría un
   **transporte UDP nuevo en paralelo** al de la sesión ya activa. Eso
   chocaba con la sesión existente y el ESP32 entraba en un bucle de
   crear/destruir todas las entidades cada 200ms (visible clarísimo en el
   log del Agente: `create_client` → ... → `delete_client` → `create_client`
   → ... una y otra vez). Fix a medias: usar el ping "simple" (sin
   opciones) ahí, que si reutiliza el transporte activo — pero esto ya
   apuntaba a que reconstruir el contexto RCL en caliente es frágil en
   este componente.

**Solución final — adoptada.** Se simplificó: las entidades (`node`,
`publisher`, `subscriber`, `timer`, `executor`) se crean **una sola vez**,
al arrancar, después de esperar bloqueado a que el Agente responda
(`wait_for_agent()`, con `rmw_uros_ping_agent_options()` — sin sesión activa
todavía, sin riesgo de choque). Ya en marcha, cada 3 segundos se hace un
ping simple (`rmw_uros_ping_agent(200, 1)`, reutiliza el transporte de la
sesión activa). Si el ping falla, el ESP32 se reinicia solo por software
(`esp_restart()`) en vez de intentar reconstruir el contexto en caliente —
un boot completo siempre reconecta bien (ya confirmado en el Test 1), y
es mucho más simple y confiable que la máquina de estados con
destroy/recreate.

Resultado verificado: se mató el proceso del Agente sin tocar el ESP32; el
ESP32 detectó la pérdida (~3s después) y se reinició solo; al volver a
levantar el Agente, reconectó limpio (una sola sesión en el log, sin
bucles) y tanto `/robot_01/status` como `/robot_01/led_cmd` volvieron a
funcionar sin reflashear ni tocar el hardware. **Pasado.**

**Trade-off asumido:** la recuperación no es "en caliente" — implica un
reboot completo del ESP32 (unos ~4s). Para un swarm de robots esto es
aceptable: nadie necesita intervenir físicamente, y el robot vuelve a un
estado limpio conocido. Si más adelante se necesita una recuperación sin
interrumpir controladores locales (PID de motores, etc. — ver sección 15),
habría que revisar por qué `rclc_support_init_with_options()` fallaba al
recrearse en caliente después de un ping con transporte propio, y separar
mejor los ciclos de vida de "sesión micro-ROS" vs. "control local del
robot" (que en la arquitectura de dos núcleos prevista corre en el otro
core de todos modos, y no se vería afectado por un reboot rápido del
núcleo que lleva Wi-Fi/micro-ROS — a confirmar cuando se implemente).

#### Test 3 — Pérdida temporal de Wi-Fi

**Primer intento — falló.** Se apagó el hotspot del celular. El ESP32 nunca
se recuperó solo: `micro_ros_espidf_component` trae su **propia** lógica de
conexión Wi-Fi interna (componente `wifi_station_netif`, separado de
nuestro `wifi_test/` de la Fase 2), y esa versión sí tiene el límite de
reintentos original de ESP-IDF (5) — el fix de reconexión indefinida de la
Fase 2 nunca se portó a este proyecto porque usan código distinto.
Confirmado con un ping directo al ESP32: `Destination Host Unreachable`.
Además, `wait_for_agent()` esperaba para siempre sin ningún timeout, así
que aunque el Wi-Fi hubiera vuelto por su cuenta, no había ningún mecanismo
que forzara un nuevo intento de conexión Wi-Fi.

**Fix aplicado.** Se agregó un timeout (`WAIT_FOR_AGENT_TIMEOUT_MS`,
30s) a `wait_for_agent()`: si pasa ese tiempo sin encontrar el Agente
(sea por Wi-Fi caído o por el Agente mismo caído), el ESP32 se reinicia
solo (`esp_restart()`). Un boot completo reintenta la conexión Wi-Fi desde
cero, reseteando el contador de reintentos interno del componente. Con
esto el ciclo completo cuando no hay Wi-Fi es: boot → Wi-Fi falla 5 veces
(~15s) → `wait_for_agent()` espera hasta agotar el timeout (30s) →
reinicio → repetir. Confirmado por log serie: el ESP32 quedó reintentando
en bucle sin colgarse mientras el hotspot estuvo apagado, y al volver a
prenderlo reconectó en el primer ciclo siguiente.

**Segundo hallazgo — sesiones "zombie" en el Agente.** Con el fix de
arriba ya aplicado, `/robot_01/status` y `ping` al ESP32 funcionaban bien
tras recuperar el Wi-Fi, pero `/robot_01/led_cmd` dejó de responder. Causa:
cada uno de los varios reboots forzados durante la prueba (por el timeout)
crea una sesión **nueva** en el Agente (nuevo `client_key`, nuevo
participante, nuevo topic/subscriber) sin que la sesión **anterior** se
cierre limpiamente primero (un reboot abrupto no alcanza a mandar
`delete_client`). El log del Agente mostraba 3 sesiones distintas creadas
en ~40s sin ningún `destroy_session` entre medio — el Agente quedó con
participantes/subscribers "fantasma" del mismo tópico, y eso interfirió
con la entrega de mensajes al subscriber real (el publisher, en cambio,
siguió funcionando bien — asimetría real observada, no asumida). Se
confirmó la causa reiniciando el Agente por completo (proceso nuevo, sin
estado acumulado): con un Agente limpio, `led_cmd` volvió a funcionar de
inmediato.

**Implicancia para el swarm (a tener en cuenta, no resuelto aún):** con
varios robots reconectándose/reiniciándose de forma independiente a lo
largo de una sesión larga, el Agente puede ir acumulando sesiones zombie
si no hay un mecanismo que las expire. Pendiente para más adelante:
investigar la config de timeout de sesión del Agente
(`micro_ros_agent --help` / flags de `middleware-session-timeout` o
similar) o reiniciar el Agente periódicamente en despliegues largos.
**Pasado con esta salvedad documentada** (no bloqueante para seguir a
hardware, pero sí relevante cuando haya 3+ robots simultáneos).

## 5. Comandos generales de compilación/flasheo

```bash
. ~/swarmbot/esp/esp-idf/export.sh
cd ~/swarmbot/firmware/<proyecto>
idf.py set-target esp32s3
idf.py build
idf.py flash    # requiere pertenecer al grupo dialout, o usar `sg dialout -c "idf.py flash"`
idf.py monitor  # salir con Ctrl+]
```

## 6. Configuración Wi-Fi / Agente micro-ROS (Fase 2-3)

> **Credenciales fuera de git.** En `wifi_test/`, `microros_publisher/` y
> `microros_subscriber/`, el `sdkconfig.defaults` versionado trae valores de
> ejemplo (`TU_RED_WIFI`, `TU_PASSWORD`, `192.168.1.100`). Los reales van en
> `sdkconfig.defaults.local` (ignorado por git), que el `CMakeLists.txt` de
> cada proyecto carga encima si existe:
>
> ```
> CONFIG_ESP_WIFI_SSID="mi_red"
> CONFIG_ESP_WIFI_PASSWORD="mi_password"
> CONFIG_MICRO_ROS_AGENT_IP="IP del PC donde corre el agente"
> ```
>
> Los `sdkconfig` generados de esos tres proyectos tampoco se versionan.


- Red de pruebas: hotspot de celular (no hay router con Wi-Fi disponible por ahora), SSID `TU_RED_WIFI`.
  - **Importante:** el ESP32-S3 solo soporta Wi-Fi de **2.4GHz** (802.11 b/g/n), no 5GHz. El hotspot del celular por defecto se conectó en 5GHz (5745 MHz); hubo que forzar la banda a 2.4GHz manualmente en los ajustes del punto de acceso móvil del celular (Ajustes > Punto de acceso móvil > banda AP) antes de que el ESP32 pudiera asociarse.
- IP de la laptop (PC, IP del Agente) en esa red: **`<IP_DEL_PC>` — fijada como estática** (ver abajo), para que no cambie si el celular se reinicia.
- IP obtenida por el ESP32-S3 en la última prueba: `<IP_DEL_ESP32>` (dinámica vía DHCP del hotspot — sí puede cambiar entre reinicios; no es un problema porque el ESP32 es quien inicia la conexión hacia el Agente, no al revés)
- Puerto UDP del Agente: `8888` (por defecto)
- Latencia de ping observada hacia el ESP32 sobre el hotspot: ~30-310ms, bastante variable (típico de hotspots de celular con ahorro de energía Wi-Fi). A tener en cuenta si más adelante se necesita telemetría de alta frecuencia; con un router dedicado debería mejorar.

### 6.1 IP estática de la laptop (persistencia entre reinicios del hotspot)

Tanto la laptop como el ESP32-S3 reconectan solos si el hotspot se apaga y
se vuelve a prender: NetworkManager tiene `autoconnect: yes` para el perfil
`TU_RED_WIFI`, y el firmware de `wifi_test/` reintenta
`esp_wifi_connect()` indefinidamente. Pero ambas IPs eran asignadas por
DHCP, así que podían cambiar en cada reconexión — un problema para la Fase 3,
donde la IP del Agente (la laptop) queda hardcodeada en el firmware del
ESP32.

Se fijó la IP de la laptop como estática para esa red:

```bash
nmcli connection modify "TU_RED_WIFI" ipv4.method manual \
  ipv4.addresses <IP_DEL_PC>/24 \
  ipv4.gateway <IP_DEL_ESP32> \
  ipv4.dns <IP_DEL_ESP32>
nmcli connection up "TU_RED_WIFI"
```

Con esto la laptop siempre tomará `<IP_DEL_PC>` en esta red, sin importar
cuántas veces se reinicie el hotspot. La IP del ESP32 sigue siendo dinámica
(vía DHCP), pero no importa: es el ESP32 quien inicia la conexión hacia el
Agente, nunca al revés.

**Nota:** si en algún momento se cambia de red (otro hotspot, un router de
laboratorio con otro rango de IPs), esta IP estática hay que reconfigurarla
para el rango correspondiente, o volver a `ipv4.method auto`.

## 7. Comandos de prueba ROS 2 (Fase 10-11)

*Pendiente — se documentarán al llegar a esa fase.*

```bash
ros2 topic list
ros2 topic echo /robot_01/status
ros2 topic pub /robot_01/led_cmd std_msgs/msg/Int32 "{data: 1}"
ros2 topic pub /robot_01/led_cmd std_msgs/msg/Int32 "{data: 0}"
```

## 8. Notas / decisiones tomadas

- Se eligió **ESP-IDF nativo** (no Arduino), y transporte **WiFi/UDP** (no serial), acorde al documento de indicaciones.
- Se eligió **compilar el agente micro-ROS desde código fuente** vía `micro_ros_setup` en vez de usar Docker, porque Docker no estaba instalado en el host y se prefirió no agregar esa dependencia.
- `rosdep install` no pudo instalar `python3-vcstool` vía apt (sin sudo interactivo disponible); se resolvió instalando `vcstool` con `pip3 install --user vcstool`, lo cual fue suficiente para que `create_agent_ws.sh` funcionara.
