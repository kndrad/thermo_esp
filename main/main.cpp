
#include <ctime>

#include <time.h>

#include "FreeRTOSConfig.h"  // IWYU pragma: keep
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "include/lcd_hd44780.hpp"
#include "include/one_wire.hpp"

#include <sys/time.h>

static const char* TAG = "MAIN";

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== LCD TEST STARTED ===");

    onewire::Master master{GPIO_NUM_4};
    onewire::DS18B20 sensor{master};
    lcd::HD44780 display{};

    while (true) {
        display.print_float(sensor.read_temperature());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
