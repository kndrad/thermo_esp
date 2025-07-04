#include "include/one_wire.hpp"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "portmacro.h"
#include "rom/ets_sys.h"

namespace onewire {

Bus::Bus() : gpio_num(DEFAULT_GPIO_NUM) {
    ESP_LOGI(TAG, "Initializing GPIO %d", gpio_num);

    gpio_config_t config = {.pin_bit_mask = (1ULL << gpio_num),
                            .mode         = GPIO_MODE_INPUT_OUTPUT_OD,
                            .pull_up_en   = GPIO_PULLUP_ENABLE,
                            .pull_down_en = GPIO_PULLDOWN_DISABLE,
                            .intr_type    = GPIO_INTR_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&config));
    ESP_LOGI(TAG, "Initial GPIO level: %d", gpio_get_level(gpio_num));
}

Bus::Bus(gpio_num_t gpio_num) : gpio_num(gpio_num) {
    ESP_LOGI(TAG, "Initializing GPIO %d", gpio_num);

    gpio_config_t config = {.pin_bit_mask = (1ULL << gpio_num),
                            .mode         = GPIO_MODE_INPUT_OUTPUT_OD,
                            .pull_up_en   = GPIO_PULLUP_ENABLE,
                            .pull_down_en = GPIO_PULLDOWN_DISABLE,
                            .intr_type    = GPIO_INTR_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&config));
    ESP_LOGI(TAG, "Initial GPIO level: %d", gpio_get_level(gpio_num));
}

bool Bus::reset() {
    ESP_LOGD(TAG, "Starting reset sequence");

    int initial_level = gpio_get_level(gpio_num);
    ESP_LOGD(TAG, "Pre-reset level: %d", initial_level);

    // Reset pulse
    gpio_set_level(gpio_num, 0);
    ets_delay_us(480);

    gpio_set_level(gpio_num, 1);
    ets_delay_us(70);

    bool present = (gpio_get_level(gpio_num) == 0);
    ESP_LOGD(TAG, "Presence detected: %s", present ? "YES" : "NO");

    ets_delay_us(410);

    int final_level = gpio_get_level(gpio_num);
    ESP_LOGD(TAG, "Post-reset level: %d", final_level);

    return present;
}

bool Bus::read_bit() {
    gpio_set_level(gpio_num, 0);
    ets_delay_us(3);
    gpio_set_level(gpio_num, 1);
    ets_delay_us(10);

    bool bit = gpio_get_level(gpio_num);
    ets_delay_us(53);

    return bit;
}

void Bus::write_bit(bool bit) {
    if (bit) {
        // Write 1: short low pulse
        gpio_set_level(gpio_num, 0);
        ets_delay_us(6);
        gpio_set_level(gpio_num, 1);
        ets_delay_us(64);
    } else {
        // Write 0: long low pulse
        gpio_set_level(gpio_num, 0);
        ets_delay_us(60);
        gpio_set_level(gpio_num, 1);
        ets_delay_us(10);
    }
}

void Bus::write_byte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        write_bit(data & 0x01);
        data >>= 1;
    }
}

uint8_t Bus::read_byte() {
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        data >>= 1;
        if (read_bit()) {
            data |= 0x80;
        }
    }
    return data;
}

// Master implementation
Master::Master(gpio_num_t gpio_num) : bus(gpio_num) {}

bool Master::init() {
    return bus.reset();
}

void Master::write_byte(uint8_t data) {
    bus.write_byte(data);
}

uint8_t Master::read_byte() {
    return bus.read_byte();
}

// Slave implementation
Slave::Slave(Master& master) : master(master) {}

void Slave::skip_rom() {
    master.write_byte(0xCC);
}

void Slave::read_rom() {
    master.write_byte(0x33);
}

void Slave::match_rom(uint64_t rom_code) {
    master.write_byte(0x55);
    for (int i = 0; i < 8; i++) {
        master.write_byte(static_cast<uint8_t>(rom_code & 0xFF));
        rom_code >>= 8;
    }
}

void Slave::search_rom() {
    master.write_byte(0xF0);
}

void Slave::alarm_search() {
    master.write_byte(0xEC);
}

// DS18B20 implementation
DS18B20::DS18B20(Master& master) : Slave(master) {}

float DS18B20::read_temperature() {
    if (!master.init()) {
        return -999.0F;
    }

    skip_rom();
    convert_temp();
    vTaskDelay(pdMS_TO_TICKS(750));

    if (!master.init()) {
        return -999.0F;
    }

    skip_rom();
    read_scratchpad();

    uint8_t temp_lsb = master.read_byte();
    uint8_t temp_msb = master.read_byte();

    for (int i = 0; i < 7; i++) {
        master.read_byte();
    }

    int16_t raw = static_cast<int16_t>((temp_msb << 8) | temp_lsb);
    return static_cast<float>(raw) / 16.0F;
}

bool DS18B20::is_externally_powered() {
    if (!master.init()) {
        return false;
    }

    skip_rom();
    master.write_byte(0xB4);
    return master.read_byte() != 0;
}

void DS18B20::convert_temp() {
    master.write_byte(0x44);
}

void DS18B20::read_scratchpad() {
    master.write_byte(0xBE);
}

};  // namespace onewire
