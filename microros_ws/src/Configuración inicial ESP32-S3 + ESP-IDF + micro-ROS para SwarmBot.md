# Configuración inicial ESP32-S3 + ESP-IDF + micro-ROS para SwarmBot

## 1. Objetivo general

Se quiere desarrollar un robot móvil diferencial pequeño basado en **ESP32-S3**, que posteriormente formará parte de un swarm de hasta 12 robots.

La arquitectura futura será aproximadamente:

```text
PC Ubuntu + ROS 2 Humble
        │
        │ Ethernet / Wi-Fi
        ▼
      Router
        │
        │ Wi-Fi 2.4 GHz
        ▼
     ESP32-S3
        │
     micro-ROS
        │
 ┌──────┼─────────┐
 │      │         │
IMU   Motores   Sensores
      +Encoder    ToF
 │
LED Matrix RGB
```

Por ahora **NO implementar motores, encoders, IMU ni sensores**.

El objetivo inicial es validar correctamente:

```text
ESP-IDF
   ↓
ESP32-S3
   ↓
Wi-Fi
   ↓
micro-ROS
   ↓
ROS 2 Humble
```

---

# 2. Framework a utilizar

Utilizar:

- **ESP-IDF**
- FreeRTOS incluido en ESP-IDF
- ESP32-S3
- ROS 2 Humble
- micro-ROS
- Micro XRCE-DDS
- comunicación **UDP sobre Wi-Fi**

No utilizar Arduino IDE como framework principal.

Se puede trabajar por terminal o mediante VS Code + extensión oficial de ESP-IDF.

---

# 3. Fase 1 — Verificar completamente el ESP32-S3

El ESP32-S3 ya está conectado físicamente al computador.

Antes de instalar micro-ROS, verificar que el SDK de Espressif funciona correctamente.

## 3.1 Identificar la placa

Determinar:

- modelo exacto del ESP32-S3;
- puerto USB utilizado;
- dispositivo `/dev/tty...`;
- GPIO correspondiente al LED integrado, si existe;
- tipo de conexión USB disponible.

No asumir el GPIO del LED: verificarlo según la placa.

---

## 3.2 Configurar ESP-IDF

Utilizar una versión estable compatible con `micro_ros_espidf_component`.

Preferencia inicial:

```text
ESP-IDF 5.5.x
```

Configurar el target como:

```bash
idf.py set-target esp32s3
```

---

# 4. Prueba básica 1 — Hello World

Compilar y ejecutar primero un proyecto oficial sencillo de ESP-IDF.

Verificar correctamente:

```bash
idf.py build
idf.py flash
idf.py monitor
```

Debe observarse por consola que:

- el ESP32-S3 inicia;
- el firmware se carga correctamente;
- el puerto serial funciona;
- no hay resets inesperados.

---

# 5. Prueba básica 2 — Blink

Realizar una prueba de **Blink usando ESP-IDF**, no Arduino.

Objetivo:

```text
ESP32-S3
   │
   ▼
 GPIO
   │
   ▼
 LED

ON
500 ms

OFF
500 ms
```

Verificar:

- configuración GPIO;
- `gpio_set_direction()`;
- `gpio_set_level()`;
- retardos mediante FreeRTOS;
- funcionamiento estable.

Utilizar preferiblemente:

```c
vTaskDelay(...)
```

y no retardos bloqueantes propios de Arduino.

Esta prueba servirá también como primera introducción a FreeRTOS.

---

# 6. Prueba básica 3 — FreeRTOS

Crear una tarea sencilla, por ejemplo:

```text
blink_task
```

que sea responsable exclusivamente del LED.

Objetivo conceptual:

```text
app_main()
   │
   ▼
xTaskCreate(...)
   │
   ▼
blink_task()
```

No es necesario todavía trabajar con ambos núcleos.

Solo comprobar:

- creación de tareas;
- prioridades;
- `vTaskDelayUntil()` o equivalente para ejecución periódica;
- funcionamiento correcto del scheduler.

---

# 7. Fase 2 — Wi-Fi

Después del Blink, conectar el ESP32-S3 al router del laboratorio.

Arquitectura:

```text
          ROUTER
        SWARM_LAB
        /       \
       /         \
 ESP32-S3        PC
   Wi-Fi       Ethernet
```

Por ahora puede utilizarse cualquier router disponible para pruebas.

El ESP32 debe:

1. conectarse al SSID;
2. obtener dirección IP;
3. mostrar por consola la IP;
4. detectar desconexiones;
5. reconectarse automáticamente.

Ejemplo esperado:

```text
WiFi connected
IP: 192.168.50.101
```

Verificar desde la PC que existe conectividad IP.

---

# 8. Fase 3 — Integración micro-ROS

Una vez validado ESP-IDF + Wi-Fi, integrar:

```text
micro_ros_espidf_component
```

para **ROS 2 Humble**.

Utilizar:

```text
Micro XRCE-DDS
```

como middleware.

Utilizar inicialmente:

```text
UDP / IPv4
```

como transporte.

---

# 9. Arquitectura micro-ROS inicial

```text
             PC Ubuntu
             ROS 2 Humble
                  │
                  ▼
          micro-ROS Agent
             UDP :8888
                  │
                  ▼
               Router
                  │
                  ▼
             ESP32-S3
                  │
          micro-ROS Client
```

La dirección IP del Agent debe ser configurable.

Ejemplo:

```text
AGENT_IP   = 192.168.50.21
AGENT_PORT = 8888
```

---

# 10. Prueba micro-ROS 1 — Publisher

Crear un nodo:

```text
robot_01
```

que publique periódicamente:

```text
/robot_01/status
```

Inicialmente puede utilizar:

```text
std_msgs/msg/Int32
```

y publicar:

```text
0
1
2
3
4
...
```

Desde la PC debe ser posible ejecutar:

```bash
ros2 topic list
```

y observar:

```text
/robot_01/status
```

Después:

```bash
ros2 topic echo /robot_01/status
```

Debe mostrar correctamente los valores enviados por el ESP32-S3.

Con esto se valida:

```text
ESP32-S3 → ROS 2
```

---

# 11. Prueba micro-ROS 2 — Subscriber

Crear:

```text
/robot_01/led_cmd
```

El ESP32-S3 debe suscribirse al tópico.

Por ejemplo:

```text
0 → LED OFF
1 → LED ON
```

Desde la PC:

```bash
ros2 topic pub /robot_01/led_cmd std_msgs/msg/Int32 "{data: 1}"
```

debe encender físicamente el LED.

Y:

```bash
ros2 topic pub /robot_01/led_cmd std_msgs/msg/Int32 "{data: 0}"
```

debe apagarlo.

Con esto se valida:

```text
ROS 2 → ESP32-S3
```

---

# 12. Criterio principal de éxito

La configuración micro-ROS se considerará validada únicamente cuando funcione de manera simultánea:

```text
ESP32-S3
   │
   ├──────► /robot_01/status
   │
   ◄─────── /robot_01/led_cmd
   │
   ▼
 LED físico
```

Es decir:

```text
ESP32 → ROS 2
```

y:

```text
ROS 2 → ESP32
```

por **Wi-Fi + UDP**, sin utilizar USB para transportar los mensajes ROS.

USB puede mantenerse únicamente para:

- flashing;
- debugging;
- monitor serial.

---

# 13. Pruebas de robustez

Antes de avanzar verificar:

### Reinicio del ESP32

Después de reiniciar debe:

```text
boot
 ↓
Wi-Fi
 ↓
micro-ROS Agent
 ↓
ROS 2
```

reconectarse correctamente.

### Reinicio del Agent

Si se detiene y vuelve a ejecutar el micro-ROS Agent, el ESP32 debe recuperar la comunicación.

### Wi-Fi

Probar pérdida temporal de Wi-Fi y recuperación.

---

# 14. Organización prevista del robot

Utilizar desde el inicio:

```text
/robot_01/...
```

porque posteriormente existirán:

```text
/robot_01
/robot_02
/robot_03
...
/robot_12
```

Los tópicos futuros probablemente serán:

```text
/robot_01/cmd_vel
/robot_01/odom
/robot_01/imu
/robot_01/status
/robot_01/battery
/robot_01/range_left
/robot_01/range_front
/robot_01/range_right
/robot_01/led_pattern
```

No es necesario implementarlos todavía.

---

# 15. Arquitectura FreeRTOS futura

Tener en cuenta desde el diseño que posteriormente se utilizarán los dos núcleos del ESP32-S3.

La arquitectura prevista es:

```text
ESP32-S3
│
├── CORE 0
│     ├── Wi-Fi
│     ├── micro-ROS
│     ├── publishers/subscribers
│     └── matriz LED
│
└── CORE 1
      ├── control motor izquierdo
      ├── control motor derecho
      ├── encoders
      ├── odometría
      ├── IMU
      └── seguridad
```

Los dos motores tendrán posteriormente control PD/PID local, probablemente a:

```text
200–500 Hz
```

ROS 2 únicamente enviará referencias como:

```text
/cmd_vel
```

a una frecuencia mucho menor.

No implementar todavía esta arquitectura, pero evitar decisiones de software que impidan posteriormente utilizar tareas FreeRTOS independientes.

---

# 16. Hardware que se añadirá posteriormente

Después de validar micro-ROS se incorporarán progresivamente:

```text
1. IMU
2. matriz RGB
3. un motor N20 + encoder
4. control PD/PID
5. segundo motor + encoder
6. cinemática diferencial
7. /cmd_vel
8. odometría
9. sensores ToF
10. monitor de batería
11. AprilTag
12. swarm
```

---

# 17. Entregables de esta primera etapa

Al terminar, dejar:

### Código

Proyecto ESP-IDF reproducible con:

```text
ESP32-S3
+ Wi-Fi
+ micro-ROS publisher
+ micro-ROS subscriber
+ LED
```

### README

Documentar:

- versión de ESP-IDF;
- versión/branch/componente de micro-ROS;
- versión de ROS 2;
- modelo exacto de ESP32-S3;
- GPIO del LED;
- puerto USB;
- configuración Wi-Fi;
- IP del Agent;
- puerto UDP;
- comandos para compilar;
- comandos para flashear;
- comandos para monitor;
- comando utilizado para iniciar micro-ROS Agent;
- comandos ROS 2 utilizados para probar publisher/subscriber.

### Resultado final esperado

```text
ROS 2 Humble
     ↕
micro-ROS Agent
     ↕
UDP / Wi-Fi
     ↕
ESP32-S3
     ↕
LED
```

**No avanzar hacia motores, IMU o demás sensores hasta que esta comunicación bidireccional sea estable y reproducible.**