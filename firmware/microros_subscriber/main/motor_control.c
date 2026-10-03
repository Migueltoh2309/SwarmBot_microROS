#include "motor_control.h"

#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "motor_control";

#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT
#define LEDC_FREQ_HZ    20000
#define PI_OUTPUT_LIMIT 1.0f  /* comando al puente H en [-1, 1] */

/* Rango de conteo del PCNT: se limpia cada ciclo de control (ver
   motor_control_update), asi que nunca deberia acercarse a este limite en
   uso normal — no hace falta manejar overflow con watch points/ISR. */
#define PCNT_LIMIT 30000

static void configure_direction_pins(const motor_t *m)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << m->in1_gpio) | (1ULL << m->in2_gpio),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);
    gpio_set_level(m->in1_gpio, 0);
    gpio_set_level(m->in2_gpio, 0);
}

static void configure_pwm(const motor_t *m)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = m->ledc_timer,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t channel_conf = {
        .gpio_num = m->pwm_gpio,
        .speed_mode = LEDC_MODE,
        .channel = m->ledc_channel,
        .timer_sel = m->ledc_timer,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
}

static void configure_encoder(motor_t *m)
{
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_LIMIT,
        .low_limit = -PCNT_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &m->pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(m->pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = m->encoder_a_gpio,
        .level_gpio_num = m->encoder_b_gpio,
    };
    pcnt_channel_handle_t chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(m->pcnt_unit, &chan_a_config, &chan_a));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = m->encoder_b_gpio,
        .level_gpio_num = m->encoder_a_gpio,
    };
    pcnt_channel_handle_t chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(m->pcnt_unit, &chan_b_config, &chan_b));

    /* Decodificacion x4 estandar: cada flanco de A o de B suma una cuenta,
       la fase del otro canal decide si suma o resta (sentido de giro). */
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(m->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(m->pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(m->pcnt_unit));
}

void motor_init(motor_t *m)
{
    m->pi_integral = 0.0f;
    m->measured_velocity_rad_s = 0.0f;
    m->target_velocity_rad_s = 0.0f;
    m->last_command = 0.0f;
    m->last_update_us = 0;
    m->velocity_filter_idx = 0;
    m->velocity_filter_count = 0;
    m->velocity_filter_sum = 0.0f;
    for (int i = 0; i < MOTOR_VELOCITY_FILTER_LEN; i++) {
        m->velocity_filter_buf[i] = 0.0f;
    }

    configure_direction_pins(m);
    configure_pwm(m);
    configure_encoder(m);

    ESP_LOGI(TAG, "Motor listo: IN1=%d IN2=%d PWM=%d ENC_A=%d ENC_B=%d",
             m->in1_gpio, m->in2_gpio, m->pwm_gpio, m->encoder_a_gpio, m->encoder_b_gpio);
}

static void motor_set_pwm(const motor_t *m, float command)
{
    if (command > 1.0f) {
        command = 1.0f;
    } else if (command < -1.0f) {
        command = -1.0f;
    }

    if (command > 0.0f) {
        gpio_set_level(m->in1_gpio, 1);
        gpio_set_level(m->in2_gpio, 0);
    } else if (command < 0.0f) {
        gpio_set_level(m->in1_gpio, 0);
        gpio_set_level(m->in2_gpio, 1);
    } else {
        gpio_set_level(m->in1_gpio, 0);
        gpio_set_level(m->in2_gpio, 0);
    }

    uint32_t max_duty = (1u << LEDC_DUTY_RES) - 1u;
    uint32_t duty = (uint32_t)(fabsf(command) * (float)max_duty);
    ledc_set_duty(LEDC_MODE, m->ledc_channel, duty);
    ledc_update_duty(LEDC_MODE, m->ledc_channel);
}

void motor_reset_control_state(motor_t *m)
{
    m->pi_integral = 0.0f;
    m->velocity_filter_idx = 0;
    m->velocity_filter_count = 0;
    m->velocity_filter_sum = 0.0f;
    for (int i = 0; i < MOTOR_VELOCITY_FILTER_LEN; i++) {
        m->velocity_filter_buf[i] = 0.0f;
    }
}

void motor_stop(motor_t *m)
{
    m->target_velocity_rad_s = 0.0f;
    motor_reset_control_state(m);
    motor_set_pwm(m, 0.0f);
}

void motor_control_update(motor_t *m)
{
    int64_t now_us = esp_timer_get_time();
    if (m->last_update_us == 0) {
        /* Primera llamada: solo establece el reloj. Sin un dt real todavia
           no hay con que calcular velocidad, mejor no inventar un valor. */
        m->last_update_us = now_us;
        pcnt_unit_clear_count(m->pcnt_unit);
        return;
    }
    float dt_s = (float) (now_us - m->last_update_us) / 1000000.0f;
    m->last_update_us = now_us;
    if (dt_s <= 0.0f) {
        return;
    }

    int pulse_count = 0;
    pcnt_unit_get_count(m->pcnt_unit, &pulse_count);
    pcnt_unit_clear_count(m->pcnt_unit);

    float delta_rev = (float)pulse_count / m->counts_per_rev;
    float raw_velocity_rad_s = (delta_rev * 2.0f * (float)M_PI) / dt_s;

    /* Media movil sobre las ultimas MOTOR_VELOCITY_FILTER_LEN muestras:
       una sola lectura cruda a este periodo es muy ruidosa (pocas cuentas
       de encoder por ciclo), promediarla es lo que de verdad estabiliza
       el lazo — ver motor_control.h y Extra/03_control_PI_velocidad.ino. */
    float oldest = m->velocity_filter_buf[m->velocity_filter_idx];
    m->velocity_filter_buf[m->velocity_filter_idx] = raw_velocity_rad_s;
    m->velocity_filter_idx = (m->velocity_filter_idx + 1) % MOTOR_VELOCITY_FILTER_LEN;
    m->velocity_filter_sum += raw_velocity_rad_s - oldest;
    if (m->velocity_filter_count < MOTOR_VELOCITY_FILTER_LEN) {
        m->velocity_filter_count++;
    }
    m->measured_velocity_rad_s = m->velocity_filter_sum / (float) m->velocity_filter_count;

    float error = m->target_velocity_rad_s - m->measured_velocity_rad_s;
    m->pi_integral += error * dt_s;

    float output = m->kp * error + m->ki * m->pi_integral;

    /* Anti-windup por back-calculation: si ya esta saturado, no sigas
       acumulando error en la integral (si no, tarda en "des-saturar"
       cuando el error cambia de signo). */
    if (output > PI_OUTPUT_LIMIT) {
        output = PI_OUTPUT_LIMIT;
        m->pi_integral -= error * dt_s;
    } else if (output < -PI_OUTPUT_LIMIT) {
        output = -PI_OUTPUT_LIMIT;
        m->pi_integral -= error * dt_s;
    }

    m->last_command = output;
    motor_set_pwm(m, output);
}
