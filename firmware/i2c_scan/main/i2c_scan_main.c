#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#define I2C_SDA_GPIO 8
#define I2C_SCL_GPIO 9
#define I2C_PORT     I2C_NUM_0

static const char *TAG = "i2c_scan";

void app_main(void)
{
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_PORT,
        .scl_io_num = I2C_SCL_GPIO,
        .sda_io_num = I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar el bus I2C (SDA=%d, SCL=%d): %s",
                 I2C_SDA_GPIO, I2C_SCL_GPIO, esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Bus I2C listo (SDA=GPIO%d, SCL=GPIO%d). Escaneando cada 3s...",
              I2C_SDA_GPIO, I2C_SCL_GPIO);

    while (1) {
        int found = 0;
        printf("\n     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
        for (int row = 0; row < 128; row += 16) {
            printf("%02x: ", row);
            for (int col = 0; col < 16; col++) {
                int addr = row + col;
                if (addr < 3 || addr > 119) {
                    printf("   ");
                    continue;
                }
                esp_err_t probe = i2c_master_probe(bus_handle, addr, 50);
                if (probe == ESP_OK) {
                    printf("%02x ", addr);
                    found++;
                } else {
                    printf("-- ");
                }
            }
            printf("\n");
        }

        if (found == 0) {
            ESP_LOGW(TAG, "Ningun dispositivo I2C detectado. Revisa VCC/GND/SDA/SCL y AD0.");
        } else {
            ESP_LOGI(TAG, "%d dispositivo(s) I2C detectado(s). 0x68/0x69 = MPU6050.", found);
        }

        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
