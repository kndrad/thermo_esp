#include "FreeRTOSConfig.h"  // IWYU pragma: keep
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "led.hpp"
#include "one_wire.hpp"

static const char* TAG = "MAIN";

const gpio_num_t led_esp_gpio  = GPIO_NUM_2;
const gpio_num_t led_ok_gpio   = GPIO_NUM_18;
const gpio_num_t led_err_gpio  = GPIO_NUM_19;
const gpio_num_t led_wire_gpio = GPIO_NUM_22;

const gpio_num_t wire_gpio = GPIO_NUM_23;

void setup_leds() {
    led::configure(led_esp_gpio);
    led::configure(led_ok_gpio);
    led::configure(led_err_gpio);
    led::configure(led_wire_gpio);
};

void test_one_wire(onewire::Master& master, onewire::DS18B20& ds18b20) {
    ESP_LOGI(TAG, "=== RUNNING TEMPERATURE TEST ===");
    gpio_set_level(led_err_gpio, 0);
    gpio_set_level(led_ok_gpio, 0);
    gpio_set_level(led_wire_gpio, 0);
    gpio_set_level(led_esp_gpio, 1);
    ets_delay_us(3000);

    bool detected = master.init();
    ESP_LOGI(TAG, "Master init result: %s", detected ? "SUCCESS" : "FAILED");

    if (detected) {
        ESP_LOGI(TAG, "DS18B20 DETECTED!");
        bool ext_power    = ds18b20.is_externally_powered();
        float temperature = ds18b20.read_temperature();
        ESP_LOGI(TAG, "External power: %s", ext_power ? "YES" : "NO");
        ESP_LOGI(TAG, "Temperature: %.3f°C", temperature);

        gpio_set_level(led_wire_gpio, 1);
        if (temperature > -999.0F) {
            ESP_LOGI(TAG, "SUCCESS");
            gpio_set_level(led_ok_gpio, 1);
            gpio_set_level(led_err_gpio, 0);
        }
        ets_delay_us(3000);
    } else {
        gpio_set_level(led_wire_gpio, 0);
        gpio_set_level(led_ok_gpio, 0);
        gpio_set_level(led_err_gpio, 1);
        ESP_LOGI(TAG, "DS18B20 NOT DETECTED.");
        ESP_LOGI(TAG, "ONE WIRE IO current level: %d", gpio_get_level(wire_gpio));
    }
    gpio_set_level(led_esp_gpio, 0);
    ets_delay_us(2000);
    ESP_LOGI(TAG, "============================");
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "STARTED");
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    vTaskDelay(pdMS_TO_TICKS(1000));

    setup_leds();
    onewire::Master master{wire_gpio};
    onewire::DS18B20 ds18b20{master};

    while (true) {
        test_one_wire(master, ds18b20);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
