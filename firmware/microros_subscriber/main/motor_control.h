#pragma once

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"

/* Ventana del filtro de media movil sobre la velocidad medida (igual que
   el PI de referencia en Arduino, Extra/03_control_PI_velocidad.ino):
   suaviza el ruido de cuantizacion de contar pulsos en una ventana corta,
   a cambio de (VENTANA-1)/2 muestras de retardo. */
#define MOTOR_VELOCITY_FILTER_LEN 7

/* Un motor DC + puente H (L298N) + encoder de cuadratura, con lazo PI de
   velocidad. Pensado para instanciarse una vez por rueda (izquierda/derecha)
   sin duplicar el driver — cada rueda es un motor_t independiente. */
typedef struct {
    /* --- Configurar antes de motor_init() --- */
    gpio_num_t in1_gpio;
    gpio_num_t in2_gpio;
    gpio_num_t pwm_gpio;
    gpio_num_t encoder_a_gpio;
    gpio_num_t encoder_b_gpio;
    ledc_channel_t ledc_channel;
    ledc_timer_t ledc_timer;

    float counts_per_rev;   /* cuentas x4 por revolucion del eje de SALIDA (con caja reductora) */
    float kp;
    float ki;

    /* --- Estado interno, no tocar a mano --- */
    pcnt_unit_handle_t pcnt_unit;
    float pi_integral;
    float measured_velocity_rad_s;
    float target_velocity_rad_s;
    float last_command; /* ultima salida del PI aplicada, [-1,1] — solo diagnostico */
    int64_t last_update_us; /* 0 = todavia no se llamo a motor_control_update() */

    float velocity_filter_buf[MOTOR_VELOCITY_FILTER_LEN];
    int velocity_filter_idx;
    int velocity_filter_count;
    float velocity_filter_sum;
} motor_t;

/* Inicializa GPIOs, PWM (LEDC) y encoder (PCNT) para este motor. El motor
   arranca detenido (PWM en 0) hasta la primera llamada a
   motor_control_update() con un target distinto de 0. */
void motor_init(motor_t *m);

/* Llamar periodicamente (ej. cada 10ms via un rcl_timer). El dt real se
   mide con esp_timer_get_time() en vez de asumir el periodo nominal del
   timer: a 100Hz el jitter del executor/WiFi es una fraccion grande del
   periodo, y usar un dt fijo ahi metia un error serio en la velocidad
   calculada (bug real encontrado el 2026-09-12, ver README). Lee el
   encoder, promedia con las ultimas MOTOR_VELOCITY_FILTER_LEN muestras,
   corre el PI contra m->target_velocity_rad_s, y aplica la salida al
   puente H. Actualiza m->measured_velocity_rad_s (ya filtrada). */
void motor_control_update(motor_t *m);

/* Detiene el motor inmediatamente y resetea el integrador del PI y el
   filtro de velocidad (evita que arranque de golpe con un error/medicion
   acumulados de antes). */
void motor_stop(motor_t *m);

/* Resetea integrador del PI y filtro de velocidad sin tocar target/PWM.
   Llamar cuando cambian kp/ki en caliente (autotuning): lo acumulado bajo
   ganancias/mediciones viejas no es valido para las nuevas. */
void motor_reset_control_state(motor_t *m);
