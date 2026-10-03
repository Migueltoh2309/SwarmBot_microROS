#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define PIN_A 21
#define PIN_B 38

void app_main(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_A) | (1ULL << PIN_B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);

    printf("Leyendo GPIO%d (A) y GPIO%d (B) crudo, cada 100ms. Gira el eje a mano.\n", PIN_A, PIN_B);

    int last_a = -1, last_b = -1;
    int transitions_a = 0, transitions_b = 0;

    while (1) {
        int a = gpio_get_level(PIN_A);
        int b = gpio_get_level(PIN_B);
        if (a != last_a) { transitions_a++; last_a = a; }
        if (b != last_b) { transitions_b++; last_b = b; }
        printf("A=%d B=%d | transiciones A=%d B=%d\n", a, b, transitions_a, transitions_b);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
