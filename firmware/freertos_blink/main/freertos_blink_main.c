/* FreeRTOS Blink — SwarmBot Fase 6

   Blink del LED movido a una tarea FreeRTOS dedicada (blink_task), creada
   desde app_main() con xTaskCreate(). Usa vTaskDelayUntil() para una
   ejecucion periodica precisa (no acumula drift como vTaskDelay()).

   Basado en el ejemplo oficial ESP-IDF get-started/blink.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = "freertos_blink";

#define BLINK_GPIO CONFIG_BLINK_GPIO
#define BLINK_TASK_STACK_SIZE 4096
#define BLINK_TASK_PRIORITY 5

static uint8_t s_led_state = 0;

#ifdef CONFIG_BLINK_LED_STRIP

static led_strip_handle_t led_strip;

static void blink_led(void)
{
    if (s_led_state) {
        /* Verde tenue, ver README seccion 4.2 */
        led_strip_set_pixel(led_strip, 0, 0, 16, 0);
        led_strip_refresh(led_strip);
    } else {
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Configurando LED direccionable (WS2812) en GPIO%d", BLINK_GPIO);
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1,
    };
#if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
#elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    led_strip_clear(led_strip);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Configurando LED simple en GPIO%d", BLINK_GPIO);
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

#else
#error "unsupported LED type"
#endif

/* Tarea FreeRTOS responsable exclusivamente del LED. */
static void blink_task(void *pvParameters)
{
    const TickType_t period_ticks = CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS;
    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;) {
        ESP_LOGI(TAG, "LED %s", s_led_state ? "ON" : "OFF");
        blink_led();
        s_led_state = !s_led_state;

        /* Ejecucion periodica exacta: no acumula drift como vTaskDelay(). */
        vTaskDelayUntil(&last_wake_time, period_ticks);
    }
}

void app_main(void)
{
    configure_led();

    xTaskCreate(blink_task, "blink_task", BLINK_TASK_STACK_SIZE, NULL,
                BLINK_TASK_PRIORITY, NULL);

    /* app_main() puede terminar: el scheduler sigue corriendo blink_task
       de forma independiente. */
}
