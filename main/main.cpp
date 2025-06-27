#include "FreeRTOSConfig.h"  // IWYU pragma: keep
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "one_wire.hpp"

static const char* TAG = "MAIN";

// HD44780 Command Definitions
const uint8_t LCD_4BITMODE = 0x00;
const uint8_t LCD_2LINE    = 0x08;
const uint8_t LCD_5x8DOTS  = 0x00;

const uint8_t LCD_FUNCTIONSET = 0x20;

const uint8_t LCD_DISPLAYCONTROL = 0x08;
const uint8_t LCD_DISPLAYON      = 0x04;
const uint8_t LCD_CURSOROFF      = 0x00;
const uint8_t LCD_BLINKOFF       = 0x00;

const uint8_t LCD_ENTRYMODESET        = 0x04;
const uint8_t LCD_ENTRYLEFT           = 0x02;
const uint8_t LCD_ENTRYSHIFTDECREMENT = 0x00;

const uint8_t LCD_CLEARDISPLAY = 0x01;
const uint8_t LCD_RETURNHOME   = 0x02;
const uint8_t LCD_SETDDRAMADDR = 0x80;

const uint8_t function_set = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;

/*
4-bit parallel communication with the HD44780 LCD controller.
Send 8-bit commands/data as two 4-bit nibbles:
First nibble: Upper 4 bits (D7-D4),
Second nibble: Lower 4 bits (D3-D0)

LCD_EN must be pulsed high then low to latch data.
Register Select (RS) - Set 0 for READING INSTRUCTIONS (IR) or 1 for WRITING DATA (DR) to the LCD
controller
*/
const gpio_num_t LCD_EN = GPIO_NUM_19;  // LCD Enable (pulsed high and low)
const gpio_num_t LCD_RS = GPIO_NUM_18;  // LCD RS signal (0 for IR, 1 for DR)

const gpio_num_t LCD_DB7 = GPIO_NUM_14;  // LCD DATA BIT 7
const gpio_num_t LCD_DB6 = GPIO_NUM_27;  // LCD DATA BIT 6
const gpio_num_t LCD_DB5 = GPIO_NUM_26;  // LCD DATA BIT 5
const gpio_num_t LCD_DB4 = GPIO_NUM_25;  // LCD DATA BIT 4

const gpio_num_t LCD_ADC0_BTN = GPIO_NUM_36;  // LCD: ADC_CH0 for keypad buttons)

const gpio_num_t LCD_BACKLIGHT = GPIO_NUM_32;  // Shield D8 (Digital 10) - Optional

void setup_pins() {
    gpio_config_t en_out    = {.pin_bit_mask = (1ULL << LCD_EN),
                               .mode         = GPIO_MODE_OUTPUT,
                               .pull_up_en   = GPIO_PULLUP_DISABLE,
                               .pull_down_en = GPIO_PULLDOWN_DISABLE,
                               .intr_type    = GPIO_INTR_DISABLE};
    gpio_config_t rs_out    = {.pin_bit_mask = (1ULL << LCD_RS),
                               .mode         = GPIO_MODE_OUTPUT,
                               .pull_up_en   = GPIO_PULLUP_DISABLE,
                               .pull_down_en = GPIO_PULLDOWN_DISABLE,
                               .intr_type    = GPIO_INTR_DISABLE};
    gpio_config_t data_bits = {.pin_bit_mask = (1ULL << LCD_DB7) | (1ULL << LCD_DB6) |
                                               (1ULL << LCD_DB5) | (1ULL << LCD_DB4),
                               .mode         = GPIO_MODE_OUTPUT,
                               .pull_up_en   = GPIO_PULLUP_DISABLE,
                               .pull_down_en = GPIO_PULLDOWN_DISABLE,
                               .intr_type    = GPIO_INTR_DISABLE};
    gpio_config_t adc_pin   = {.pin_bit_mask = (1ULL << LCD_ADC0_BTN),
                               .mode         = GPIO_MODE_INPUT,
                               .pull_up_en   = GPIO_PULLUP_ENABLE,
                               .pull_down_en = GPIO_PULLDOWN_DISABLE,
                               .intr_type    = GPIO_INTR_DISABLE};
    gpio_config_t backlight = {.pin_bit_mask = (1ULL << LCD_BACKLIGHT),
                               .mode         = GPIO_MODE_INPUT,
                               .pull_up_en   = GPIO_PULLUP_DISABLE,
                               .pull_down_en = GPIO_PULLDOWN_DISABLE,
                               .intr_type    = GPIO_INTR_DISABLE};

    ESP_ERROR_CHECK(gpio_config(&rs_out));
    ESP_ERROR_CHECK(gpio_config(&en_out));
    ESP_ERROR_CHECK(gpio_config(&data_bits));
    ESP_ERROR_CHECK(gpio_config(&backlight));
    ESP_ERROR_CHECK(gpio_config(&adc_pin));

    gpio_set_level(LCD_RS, 0);
    gpio_set_level(LCD_EN, 0);
    gpio_set_level(LCD_DB4, 0);
    gpio_set_level(LCD_DB5, 0);
    gpio_set_level(LCD_DB6, 0);
    gpio_set_level(LCD_DB7, 0);
    gpio_set_level(LCD_BACKLIGHT, 1);

    ESP_LOGI(TAG, "GPIO pins configured successfully");
}

void pulse_enable() {
    gpio_set_level(LCD_EN, 1);
    esp_rom_delay_us(1);  // > 450ns
    gpio_set_level(LCD_EN, 0);
    esp_rom_delay_us(50);
}

void write4bits(uint8_t value) {
    gpio_set_level(LCD_DB7, (value >> 3) & 0x01);  // Extract bit 3
    gpio_set_level(LCD_DB6, (value >> 2) & 0x01);  // Extract bit 2
    gpio_set_level(LCD_DB5, (value >> 1) & 0x01);  // Extract bit 1
    gpio_set_level(LCD_DB4, (value >> 0) & 0x01);  // Extract bit 0

    pulse_enable();
}

void send_command(uint8_t value) {
    gpio_set_level(LCD_RS, 0);

    write4bits(value >> 4);    // upper nibble
    write4bits(value & 0x0F);  // lower nibble

    esp_rom_delay_us(50);
}

void write(uint8_t value) {
    gpio_set_level(LCD_RS, 1);  // Data mode

    write4bits(value >> 4);    // upper nibble
    write4bits(value & 0x0F);  // lower nibble

    esp_rom_delay_us(50);
}

void write_char(char c) {
    write((uint8_t)c);
}

void write_str(const char* str) {
    while (*str) {
        write_char(*str++);
    }
}

void set_cursor(uint8_t col, uint8_t row) {
    uint8_t address = 0;

    if (row == 0) {
        address = 0x00 + col;  // First line starts at 0x00
    } else if (row == 1) {
        address = 0x40 + col;  // Second line starts at 0x40
    }

    send_command(LCD_SETDDRAMADDR | address);
}

// HD44780 datasheet Figure 24 - 4-bit initialization sequence
void init() {
    ESP_LOGI(TAG, "Starting LCD initialization...");

    vTaskDelay(pdMS_TO_TICKS(50));  // Wait > 40ms

    gpio_set_level(LCD_RS, 0);  // Command mode

    // Initial 8-bit mode setup sequence
    write4bits(0x03);
    esp_rom_delay_us(4500);  // Wait > 4.1ms

    write4bits(0x03);
    esp_rom_delay_us(4500);  // Wait > 4.1ms

    write4bits(0x03);
    esp_rom_delay_us(150);  // Wait > 100μs

    write4bits(0x02);
    esp_rom_delay_us(150);  // Wait > 100μs

    // 4-bit mode
    send_command(LCD_FUNCTIONSET | function_set);  // 4-bit, 2-line, 5x8
    esp_rom_delay_us(50);

    send_command(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF);
    esp_rom_delay_us(50);

    send_command(LCD_CLEARDISPLAY);
    vTaskDelay(pdMS_TO_TICKS(2));  // takes 1.52ms

    send_command(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
    esp_rom_delay_us(50);

    ESP_LOGI(TAG, "LCD initialization completed");
}

void command_clear() {
    send_command(LCD_CLEARDISPLAY);
    vTaskDelay(pdMS_TO_TICKS(2));  // takes time
}

void command_home() {
    send_command(LCD_RETURNHOME);
    vTaskDelay(pdMS_TO_TICKS(2));  // takes time
}

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== LCD TEST STARTED ===");

    vTaskDelay(pdMS_TO_TICKS(500));

    setup_pins();
    vTaskDelay(pdMS_TO_TICKS(100));

    init();
    vTaskDelay(pdMS_TO_TICKS(500));

    onewire::Master master{onewire::DEFAULT_GPIO_NUM};
    onewire::DS18B20 sensor{master};
    vTaskDelay(pdMS_TO_TICKS(500));

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        float temp = sensor.read_temperature();
        ESP_LOGI(TAG, "Temperature: %.3f°C", temp);

        char str[] = "TEMP: %.3f C";
        char str2[100];
        sprintf(str2, str, temp);
        set_cursor(0, 0);

        write_str(str2);
    }
}
