/* micro-ROS Publisher + Subscriber con recuperacion automatica del Agente
   — SwarmBot Fase 8-13, mas IMU (Fase 14)

   Nodo "robot_01":
     - Publica std_msgs/Int32 incremental en "/robot_01/status" cada 1000ms.
     - Publica sensor_msgs/Imu en "/robot_01/imu" cada 50ms (20Hz), leyendo
       un MPU6050 por I2C (SDA=GPIO8, SCL=GPIO9, direccion 0x68 con AD0 a
       GND). Sin magnetometro ni fusion sensorial todavia, asi que solo se
       llenan angular_velocity y linear_acceleration; orientation_covariance
       se deja en -1 en el indice 0 (convencion de sensor_msgs/Imu para
       "este campo no esta disponible", ver comentarios del propio .msg).
     - Se suscribe a std_msgs/Int32 en "/robot_01/led_cmd": 0 -> LED OFF,
       1 -> LED ON (LED RGB WS2812 en GPIO48, via el driver led_strip, igual
       que en blink/ y freertos_blink/).
     - Motor izquierdo (Pololu 25D, 98.78:1, encoder 48 CPR -> 4741.44
       cuentas x4 por vuelta del eje de salida) con puente H L298N:
       IN1=GPIO4, IN2=GPIO5, ENA(PWM)=GPIO6, encoder A=GPIO7, B=GPIO15.
       Lazo PI de velocidad a 100Hz con filtro de media movil sobre la
       velocidad medida (motor_control.c/h, ver Extra/03_control_PI_velocidad.ino
       de referencia). Se suscribe a
       std_msgs/Float32 en "/robot_01/wheel_left/target_velocity" (rad/s,
       default 0 = detenido al bootear) y publica std_msgs/Float32 en
       "/robot_01/wheel_left/velocity" (rad/s medidos). El segundo motor
       (derecho) se agrega despues, reusando el mismo motor_t.

   Recuperacion ante caida del Agente (seccion 13 del documento):
     - Primer intento fue una maquina de estados clasica de micro-ROS
       (WAITING_AGENT/AGENT_AVAILABLE/AGENT_CONNECTED/AGENT_DISCONNECTED)
       que destruye y recrea las entidades RCL en caliente. Se descarto:
       usar rmw_uros_ping_agent_options() (transporte propio) mientras ya
       hay una sesion activa abre un segundo transporte UDP que choca con
       el primero, y ademas rclc_support_init_with_options() fallaba de
       forma consistente al recrearse justo despues de un ping con
       transporte propio (ver seccion 4.7 del README para el detalle).
     - Solucion adoptada, mas simple y probada: las entidades se crean UNA
       sola vez, cuando el Agente responde por primera vez. Despues, cada
       pocos segundos se hace un ping "simple" (reutiliza el transporte de
       la sesion ya activa, sin abrir uno nuevo). Si el ping falla, el
       ESP32 se reinicia solo por software (esp_restart()) — un boot
       completo SIEMPRE reconecta bien (confirmado en el Test 1 de
       robustez), y evita la fragilidad de reconstruir el contexto RCL en
       caliente.

   La reconexion Wi-Fi (capa por debajo) ya es indefinida desde la Fase 2
   (wifi_test/); esta capa cubre la sesion micro-ROS, que es independiente.
*/
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "led_strip.h"
#include "driver/i2c_master.h"
#include "motor_control.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/imu.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define LED_GPIO 48
#define AGENT_PING_PERIOD_MS 3000

/* MPU6050 por I2C: ver seccion "Conexion MPU6050" del README para el
   cableado. AD0 a GND -> direccion 0x68. */
#define I2C_SDA_GPIO      8
#define I2C_SCL_GPIO      9
#define MPU6050_ADDR      0x68
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
/* Sensibilidad para full-scale +-2g / +-250 deg/s (config por defecto). */
#define MPU6050_ACCEL_LSB_PER_G   16384.0
#define MPU6050_GYRO_LSB_PER_DPS  131.0
#define STANDARD_GRAVITY          9.80665
#define DEG_TO_RAD                0.017453292519943295

/* Motor izquierdo: Pololu 25D 98.78:1 con encoder 48 CPR. Puente H L298N,
   ENA sin jumper (PWM controla velocidad), pines elegidos evitando I2C
   (8/9), LED (48), USB (19/20) y los strapping pins (0/3/45/46). */
#define MOTOR_LEFT_IN1_GPIO       4
#define MOTOR_LEFT_IN2_GPIO       5
#define MOTOR_LEFT_PWM_GPIO       6
#define MOTOR_LEFT_ENCODER_A_GPIO 7
#define MOTOR_LEFT_ENCODER_B_GPIO 15
#define MOTOR_COUNTS_PER_REV      4741.44f /* 48 CPR x4 * 98.78:1, eje de salida */
/* Ganancias del PI (2026-09-12). Equivalentes convertidas del PI de
   referencia validado en Arduino (Extra/03_control_PI_velocidad.ino:
   Kp=1.2, Ki=5.0 en unidades RPM->PWM 0-255; conversion de unidades:
   factor rad/s->RPM = 60/(2*pi) ~= 9.5493, factor PWM 0-255 -> [-1,1] =
   1/255): Kp_aqui = Kp_arduino*9.5493/255 ~= 0.045, Ki_aqui =
   Ki_arduino*9.5493/255 ~= 0.19.
   Solo funcionan bien junto con las otras dos piezas portadas del mismo
   diseno: lazo a 100Hz (MOTOR_CONTROL_PERIOD_MS) y filtro de media movil
   sobre la velocidad medida (MOTOR_VELOCITY_FILTER_LEN en motor_control.h)
   — sin esas dos, cualquier ganancia se ve mal porque la "medicion" que
   recibe el PI es puro ruido de cuantizacion del encoder.
   Verificado en vivo: velocidad se asienta en 2.0 rad/s +-0.05, sin
   overshoot visible, muy por encima del barrido anterior (rmse~0.41,
   overshoot~33%) que corria a 20Hz sin filtro y con un bug de dt fijo
   (ver motor_control.c, motor_control_update). */
#define MOTOR_LEFT_KP             0.045f
#define MOTOR_LEFT_KI             0.19f
#define MOTOR_CONTROL_PERIOD_MS   10 /* 100Hz, mismo dt usado en el PI —
   igual que Extra/03_control_PI_velocidad.ino. A 20Hz la lectura cruda del
   encoder era demasiado ruidosa por ciclo para cualquier ganancia (ver
   MOTOR_VELOCITY_FILTER_LEN en motor_control.h). */

/* Motor derecho: mismo modelo Pololu 25D y mismo driver L298N, pines
   distintos (2026-09-12). Arranca con las mismas ganancias que el
   izquierdo (mismo motor/reductora) — reajustar con tune_wheel_pi.py si
   la mecanica de este lado difiere lo suficiente. */
#define MOTOR_RIGHT_IN1_GPIO       16
#define MOTOR_RIGHT_IN2_GPIO       17
#define MOTOR_RIGHT_PWM_GPIO       18
#define MOTOR_RIGHT_ENCODER_A_GPIO 21
#define MOTOR_RIGHT_ENCODER_B_GPIO 38
#define MOTOR_RIGHT_KP             MOTOR_LEFT_KP
#define MOTOR_RIGHT_KI             MOTOR_LEFT_KI

static rcl_publisher_t publisher;
static rcl_publisher_t imu_publisher;
static rcl_publisher_t wheel_left_velocity_publisher;
static rcl_publisher_t wheel_right_velocity_publisher;
static rcl_subscription_t subscriber;
static rcl_subscription_t wheel_left_target_subscriber;
static rcl_subscription_t wheel_left_kp_subscriber;
static rcl_subscription_t wheel_left_ki_subscriber;
static rcl_subscription_t wheel_right_target_subscriber;
static rcl_subscription_t wheel_right_kp_subscriber;
static rcl_subscription_t wheel_right_ki_subscriber;
static std_msgs__msg__Int32 status_msg;
static std_msgs__msg__Int32 led_cmd_msg;
static std_msgs__msg__Float32 wheel_left_velocity_msg;
static std_msgs__msg__Float32 wheel_left_target_msg;
static std_msgs__msg__Float32 wheel_left_kp_msg;
static std_msgs__msg__Float32 wheel_left_ki_msg;
static std_msgs__msg__Float32 wheel_right_velocity_msg;
static std_msgs__msg__Float32 wheel_right_target_msg;
static std_msgs__msg__Float32 wheel_right_kp_msg;
static std_msgs__msg__Float32 wheel_right_ki_msg;
static sensor_msgs__msg__Imu imu_msg;
static led_strip_handle_t led_strip;
static i2c_master_dev_handle_t mpu6050_dev;
static char imu_frame_id[] = "robot_01/imu_link";
static motor_t left_motor;
static motor_t right_motor;

static void configure_led(void)
{
	led_strip_config_t strip_config = {
		.strip_gpio_num = LED_GPIO,
		.max_leds = 1,
	};
	led_strip_rmt_config_t rmt_config = {
		.resolution_hz = 10 * 1000 * 1000,
		.flags.with_dma = false,
	};
	ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
	led_strip_clear(led_strip);
}

static void set_led(bool on)
{
	if (on) {
		led_strip_set_pixel(led_strip, 0, 0, 16, 0); /* verde tenue */
		led_strip_refresh(led_strip);
	} else {
		led_strip_clear(led_strip);
	}
}

/* Deja el bus I2C y el MPU6050 listos para leer. Se llama una sola vez en
   app_main(), antes de que arranque la tarea de micro-ROS: leer el sensor
   no depende de que haya Agente ni sesion, asi que no tiene sentido
   esperar a eso. */
static void configure_mpu6050(void)
{
	i2c_master_bus_config_t bus_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port = I2C_NUM_0,
		.scl_io_num = I2C_SCL_GPIO,
		.sda_io_num = I2C_SDA_GPIO,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};
	i2c_master_bus_handle_t bus_handle;
	ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

	i2c_device_config_t dev_config = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = MPU6050_ADDR,
		.scl_speed_hz = 400000,
	};
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &mpu6050_dev));

	/* Despertar el sensor (sale de sleep mode al bootear) y fijar el
	   full-scale range explicitamente aunque coincida con el default,
	   para no depender de que el reset previo lo haya dejado asi. */
	uint8_t wake[2] = { MPU6050_REG_PWR_MGMT_1, 0x00 };
	ESP_ERROR_CHECK(i2c_master_transmit(mpu6050_dev, wake, sizeof(wake), -1));
	uint8_t gyro_fs[2] = { MPU6050_REG_GYRO_CONFIG, 0x00 };  /* +-250 dps */
	ESP_ERROR_CHECK(i2c_master_transmit(mpu6050_dev, gyro_fs, sizeof(gyro_fs), -1));
	uint8_t accel_fs[2] = { MPU6050_REG_ACCEL_CONFIG, 0x00 };  /* +-2g */
	ESP_ERROR_CHECK(i2c_master_transmit(mpu6050_dev, accel_fs, sizeof(accel_fs), -1));

	printf("MPU6050 listo en 0x%02X (SDA=GPIO%d, SCL=GPIO%d)\n",
	       MPU6050_ADDR, I2C_SDA_GPIO, I2C_SCL_GPIO);
}

static void configure_left_motor(void)
{
	left_motor.in1_gpio = MOTOR_LEFT_IN1_GPIO;
	left_motor.in2_gpio = MOTOR_LEFT_IN2_GPIO;
	left_motor.pwm_gpio = MOTOR_LEFT_PWM_GPIO;
	left_motor.encoder_a_gpio = MOTOR_LEFT_ENCODER_A_GPIO;
	left_motor.encoder_b_gpio = MOTOR_LEFT_ENCODER_B_GPIO;
	left_motor.ledc_channel = LEDC_CHANNEL_0;
	left_motor.ledc_timer = LEDC_TIMER_0;
	left_motor.counts_per_rev = MOTOR_COUNTS_PER_REV;
	left_motor.kp = MOTOR_LEFT_KP;
	left_motor.ki = MOTOR_LEFT_KI;
	motor_init(&left_motor);
}

static void configure_right_motor(void)
{
	right_motor.in1_gpio = MOTOR_RIGHT_IN1_GPIO;
	right_motor.in2_gpio = MOTOR_RIGHT_IN2_GPIO;
	right_motor.pwm_gpio = MOTOR_RIGHT_PWM_GPIO;
	right_motor.encoder_a_gpio = MOTOR_RIGHT_ENCODER_A_GPIO;
	right_motor.encoder_b_gpio = MOTOR_RIGHT_ENCODER_B_GPIO;
	right_motor.ledc_channel = LEDC_CHANNEL_1; /* timer/canal propio, no compartir con el izquierdo */
	right_motor.ledc_timer = LEDC_TIMER_1;
	right_motor.counts_per_rev = MOTOR_COUNTS_PER_REV;
	right_motor.kp = MOTOR_RIGHT_KP;
	right_motor.ki = MOTOR_RIGHT_KI;
	motor_init(&right_motor);
}

/* Lee accel[3]/gyro[3] crudos (14 bytes desde ACCEL_XOUT_H: accel, temp,
   gyro) y los convierte a m/s^2 y rad/s. Devuelve false si fallo el I2C. */
static bool read_mpu6050(float accel_mps2[3], float gyro_rads[3])
{
	uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
	uint8_t raw[14];
	esp_err_t err = i2c_master_transmit_receive(mpu6050_dev, &reg, 1, raw, sizeof(raw), -1);
	if (err != ESP_OK) {
		return false;
	}

	int16_t ax = (raw[0] << 8) | raw[1];
	int16_t ay = (raw[2] << 8) | raw[3];
	int16_t az = (raw[4] << 8) | raw[5];
	/* raw[6]/raw[7] = temperatura, sin usar por ahora */
	int16_t gx = (raw[8] << 8) | raw[9];
	int16_t gy = (raw[10] << 8) | raw[11];
	int16_t gz = (raw[12] << 8) | raw[13];

	accel_mps2[0] = (ax / MPU6050_ACCEL_LSB_PER_G) * STANDARD_GRAVITY;
	accel_mps2[1] = (ay / MPU6050_ACCEL_LSB_PER_G) * STANDARD_GRAVITY;
	accel_mps2[2] = (az / MPU6050_ACCEL_LSB_PER_G) * STANDARD_GRAVITY;

	gyro_rads[0] = (gx / MPU6050_GYRO_LSB_PER_DPS) * DEG_TO_RAD;
	gyro_rads[1] = (gy / MPU6050_GYRO_LSB_PER_DPS) * DEG_TO_RAD;
	gyro_rads[2] = (gz / MPU6050_GYRO_LSB_PER_DPS) * DEG_TO_RAD;
	return true;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		printf("Publishing /robot_01/status: %d\n", (int) status_msg.data);
		RCSOFTCHECK(rcl_publish(&publisher, &status_msg, NULL));
		status_msg.data++;
	}
}

void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer == NULL) {
		return;
	}

	float accel[3];
	float gyro[3];
	if (!read_mpu6050(accel, gyro)) {
		printf("MPU6050 read failed, skipping this /robot_01/imu publish\n");
		return;
	}

	imu_msg.linear_acceleration.x = accel[0];
	imu_msg.linear_acceleration.y = accel[1];
	imu_msg.linear_acceleration.z = accel[2];
	imu_msg.angular_velocity.x = gyro[0];
	imu_msg.angular_velocity.y = gyro[1];
	imu_msg.angular_velocity.z = gyro[2];

	RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
}

void led_cmd_callback(const void * msgin)
{
	const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *) msgin;
	printf("Received /robot_01/led_cmd: %d\n", (int) msg->data);
	set_led(msg->data != 0);
}

/* Corre el PI de velocidad de las dos ruedas y publica lo medido. El dt
   real se mide dentro de motor_control_update() con esp_timer_get_time()
   — MOTOR_CONTROL_PERIOD_MS es solo el periodo NOMINAL del rcl_timer, no
   se puede asumir que cada disparo cae exacto a esa distancia (bug real
   encontrado el 2026-09-12 al subir a 100Hz: el jitter del executor/WiFi
   ya no era despreciable frente al periodo). */
void motor_control_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer == NULL) {
		return;
	}

	motor_control_update(&left_motor);
	wheel_left_velocity_msg.data = left_motor.measured_velocity_rad_s;
	RCSOFTCHECK(rcl_publish(&wheel_left_velocity_publisher, &wheel_left_velocity_msg, NULL));

	motor_control_update(&right_motor);
	wheel_right_velocity_msg.data = right_motor.measured_velocity_rad_s;
	RCSOFTCHECK(rcl_publish(&wheel_right_velocity_publisher, &wheel_right_velocity_msg, NULL));
}

void wheel_left_target_callback(const void * msgin)
{
	const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *) msgin;
	printf("Received /robot_01/wheel_left/target_velocity: %.3f rad/s\n", (double) msg->data);
	left_motor.target_velocity_rad_s = msg->data;
}

void wheel_right_target_callback(const void * msgin)
{
	const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *) msgin;
	printf("Received /robot_01/wheel_right/target_velocity: %.3f rad/s\n", (double) msg->data);
	right_motor.target_velocity_rad_s = msg->data;
}

/* Ganancias ajustables en caliente (para el autotuning por topics, sin
   reflashear en cada iteracion). Cambiar de ganancia resetea el integrador:
   el acumulado bajo la ganancia vieja no significa nada bajo la nueva. */
void wheel_left_kp_callback(const void * msgin)
{
	const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *) msgin;
	left_motor.kp = msg->data;
	motor_reset_control_state(&left_motor);
	printf("wheel_left kp=%.4f\n", (double) msg->data);
}

void wheel_left_ki_callback(const void * msgin)
{
	const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *) msgin;
	left_motor.ki = msg->data;
	motor_reset_control_state(&left_motor);
	printf("wheel_left ki=%.4f\n", (double) msg->data);
}

void wheel_right_kp_callback(const void * msgin)
{
	const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *) msgin;
	right_motor.kp = msg->data;
	motor_reset_control_state(&right_motor);
	printf("wheel_right kp=%.4f\n", (double) msg->data);
}

void wheel_right_ki_callback(const void * msgin)
{
	const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *) msgin;
	right_motor.ki = msg->data;
	motor_reset_control_state(&right_motor);
	printf("wheel_right ki=%.4f\n", (double) msg->data);
}

/* Espera bloqueante (con reintentos) a que el Agente responda, usando un
   transporte propio con la IP/puerto explicitos — todavia no existe
   ninguna sesion activa en este punto, asi que no hay riesgo de choque.

   Si la espera se alarga demasiado (WAIT_FOR_AGENT_TIMEOUT_MS), el ESP32
   se reinicia solo. Esto cubre tanto una caida prolongada del Agente como
   una caida de Wi-Fi: el componente micro_ros_espidf_component trae su
   propia logica de conexion Wi-Fi interna (distinta del fix de
   reconexion indefinida que se hizo a mano en wifi_test/, Fase 2) que SI
   tiene limite de reintentos (5, el default de ESP-IDF) y se rinde -
   confirmado con un ping directo al ESP32 quedando "Destination Host
   Unreachable" tras perder el Wi-Fi. Sin este timeout, wait_for_agent()
   quedaria esperando para siempre sin Wi-Fi y sin forma de recuperarse
   sola (bug real encontrado en el Test 3 de robustez, ver seccion 4.7 del
   README). Un reboot completo reintenta la conexion Wi-Fi desde cero. */
#define WAIT_FOR_AGENT_TIMEOUT_MS 30000

static void wait_for_agent(void)
{
	rcl_allocator_t alloc = rcl_get_default_allocator();
	int64_t start_ms = esp_timer_get_time() / 1000;

	while (1) {
		rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
		if (rcl_init_options_init(&init_options, alloc) == RCL_RET_OK) {
			rmw_init_options_t * rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
			rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options);

			if (RMW_RET_OK == rmw_uros_ping_agent_options(200, 1, rmw_options)) {
				rcl_init_options_fini(&init_options);
				return;
			}
			rcl_init_options_fini(&init_options);
		}

		if ((esp_timer_get_time() / 1000) - start_ms > WAIT_FOR_AGENT_TIMEOUT_MS) {
			printf("Agent/WiFi unreachable for too long, restarting...\n");
			vTaskDelay(pdMS_TO_TICKS(200));
			esp_restart();
		}

		printf("Waiting for micro-ROS Agent (%s:%s)...\n",
		       CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

void micro_ros_task(void * arg)
{
	wait_for_agent();

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
	rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
	RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "robot_01", "", &support));

	RCCHECK(rclc_publisher_init_default(
		&publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"robot_01/status"));

	RCCHECK(rclc_publisher_init_default(
		&imu_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"robot_01/imu"));

	RCCHECK(rclc_publisher_init_default(
		&wheel_left_velocity_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"robot_01/wheel_left/velocity"));

	RCCHECK(rclc_publisher_init_default(
		&wheel_right_velocity_publisher, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"robot_01/wheel_right/velocity"));

	RCCHECK(rclc_subscription_init_default(
		&subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"robot_01/led_cmd"));

	RCCHECK(rclc_subscription_init_default(
		&wheel_left_target_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"robot_01/wheel_left/target_velocity"));

	RCCHECK(rclc_subscription_init_default(
		&wheel_left_kp_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"robot_01/wheel_left/kp"));

	RCCHECK(rclc_subscription_init_default(
		&wheel_left_ki_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"robot_01/wheel_left/ki"));

	RCCHECK(rclc_subscription_init_default(
		&wheel_right_target_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"robot_01/wheel_right/target_velocity"));

	RCCHECK(rclc_subscription_init_default(
		&wheel_right_kp_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"robot_01/wheel_right/kp"));

	RCCHECK(rclc_subscription_init_default(
		&wheel_right_ki_subscriber, &node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
		"robot_01/wheel_right/ki"));

	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

	rcl_timer_t imu_timer;
	const unsigned int imu_timer_timeout = 50; /* 20Hz */
	RCCHECK(rclc_timer_init_default(&imu_timer, &support, RCL_MS_TO_NS(imu_timer_timeout), imu_timer_callback));

	rcl_timer_t motor_timer;
	RCCHECK(rclc_timer_init_default(&motor_timer, &support, RCL_MS_TO_NS(MOTOR_CONTROL_PERIOD_MS),
	                                 motor_control_timer_callback));

	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 10, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
	RCCHECK(rclc_executor_add_timer(&executor, &motor_timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &led_cmd_msg,
	                                        &led_cmd_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &wheel_left_target_subscriber, &wheel_left_target_msg,
	                                        &wheel_left_target_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &wheel_left_kp_subscriber, &wheel_left_kp_msg,
	                                        &wheel_left_kp_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &wheel_left_ki_subscriber, &wheel_left_ki_msg,
	                                        &wheel_left_ki_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &wheel_right_target_subscriber, &wheel_right_target_msg,
	                                        &wheel_right_target_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &wheel_right_kp_subscriber, &wheel_right_kp_msg,
	                                        &wheel_right_kp_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &wheel_right_ki_subscriber, &wheel_right_ki_msg,
	                                        &wheel_right_ki_callback, ON_NEW_DATA));

	status_msg.data = 0;

	/* frame_id se deja apuntando a un buffer estatico (sin malloc) — el
	   mismo string para siempre, no hace falta rosidl String__assign. */
	imu_msg.header.frame_id.data = imu_frame_id;
	imu_msg.header.frame_id.size = strlen(imu_frame_id);
	imu_msg.header.frame_id.capacity = sizeof(imu_frame_id);
	/* Sin magnetometro/fusion: no hay estimacion de orientacion. -1 en el
	   indice 0 es la convencion de sensor_msgs/Imu para "campo no
	   disponible, ignorar orientation por completo". */
	imu_msg.orientation_covariance[0] = -1.0;
	/* Covarianzas de placeholder (no calibradas) para angular_velocity y
	   linear_acceleration: solo la diagonal, con un valor razonable para
	   un MPU6050 sin calibrar. Ajustar cuando se calibre el sensor. */
	imu_msg.angular_velocity_covariance[0] = 0.02;
	imu_msg.angular_velocity_covariance[4] = 0.02;
	imu_msg.angular_velocity_covariance[8] = 0.02;
	imu_msg.linear_acceleration_covariance[0] = 0.04;
	imu_msg.linear_acceleration_covariance[4] = 0.04;
	imu_msg.linear_acceleration_covariance[8] = 0.04;

	printf("Agent connected, session established (node robot_01)\n");

	int64_t last_ping_ms = esp_timer_get_time() / 1000;

	while (1) {
		/* Timeout de espera bajado de 100ms a 10ms (y el usleep de cola
		   tambien) para no meterle latencia extra al timer del motor,
		   que ahora corre a 100Hz (MOTOR_CONTROL_PERIOD_MS). */
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

		int64_t now_ms = esp_timer_get_time() / 1000;
		if (now_ms - last_ping_ms > AGENT_PING_PERIOD_MS) {
			last_ping_ms = now_ms;
			/* Ping "simple": reutiliza el transporte de la sesion ya
			   activa, no abre uno nuevo (ver nota al inicio del archivo). */
			if (RMW_RET_OK != rmw_uros_ping_agent(200, 1)) {
				printf("Agent lost, restarting to recover session...\n");
				vTaskDelay(pdMS_TO_TICKS(200));
				esp_restart();
			}
		}

		usleep(1000);
	}

	vTaskDelete(NULL);
}

void app_main(void)
{
	configure_led();
	configure_mpu6050();
	configure_left_motor();
	configure_right_motor();

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
	ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif

	xTaskCreate(micro_ros_task,
	            "uros_task",
	            CONFIG_MICRO_ROS_APP_STACK,
	            NULL,
	            CONFIG_MICRO_ROS_APP_TASK_PRIO,
	            NULL);
}
