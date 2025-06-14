#pragma once

#include <vector>

#include "driver/gpio.h"
#include "esp_log.h"  // IWYU pragma: keep
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "portmacro.h"     // IWYU pragma: keep
#include "soc/gpio_num.h"  // IWYU pragma: keep

namespace led {
static const char* TAG           = "LED";
static const uint16_t blink_rate = 40000 / 100;

void blink_n(const gpio_num_t pin, int n = 5) {
    int level = gpio_get_level(pin);

    for (int i = 0; i < n; i++) {
        gpio_set_level(pin, !level);
        vTaskDelay(pdMS_TO_TICKS(blink_rate));
        gpio_set_level(pin, level);
        vTaskDelay(pdMS_TO_TICKS(blink_rate));
    }
}

void blink_n(const std::vector<gpio_num_t>& pins, int n = 5) {
    for (const auto& pin : pins) {
        blink_n(pin, 5);
    }
}

void configure(const gpio_num_t pin) {
    gpio_config_t config{
        .pin_bit_mask = (1ULL << pin),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&config));
}

void configure(std::vector<gpio_num_t>& pins) {
    for (const auto& pin : pins) {
        configure(pin);
    }
}
};  // namespace led
