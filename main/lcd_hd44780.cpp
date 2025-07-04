
#include "include/lcd_hd44780.hpp"

#include <ctime>
#include <vector>

#include <time.h>

#include "FreeRTOSConfig.h"  // IWYU pragma: keep
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "include/one_wire.hpp"

#include <sys/time.h>

static const char* TAG = "LCD_HD44780";

namespace lcd {

// Sets DB5=1 - which is a instruction identifier.
const uint8_t ENTER_FUNCSET = 0x20;  // 0010 0000

// DB4=0 is DL bit.
// DL=0 means "switch to 4bit interface mode"
// DL=1 means "use 8-bit interface"
const uint8_t FUNCSET_4BIT = 0x00;  // 0000 0000

// N=1 (bit 3 set): 2-line display mode
// N=0 (bit 3 clear): 1-line display mode
const uint8_t FUNCSET_2LINE = 0x08;  // 0000 1000

// F=0: 5×8 dot character font
// F=1: 5×10 dot character font
const uint8_t FUNCSET_5X8F = 0x00;  // 0000 0000

const uint8_t CLEAR_DISPLAY = 0x01;  // Clears display, sets ddram addr to 0 in addr counter
const uint8_t RETURN_HOME   = 0x02;

const uint8_t ENTER_ENTRYMODESET          = 0x04;
const uint8_t ENTRYMODESET_DDRAM_ADDR_INC = 0x02;
const uint8_t ENTRYMODESET_RMOVE          = ENTER_ENTRYMODESET | ENTRYMODESET_DDRAM_ADDR_INC;
const uint8_t ENTRYMODESET_SHIFT_DISPLAY  = 0x01;

const uint8_t CONTROL_DISPLAY    = 0x08;
const uint8_t DISPLAY_ON         = 0x04;
const uint8_t DISPLAY_CURSOR_OFF = 0x00;
const uint8_t DISPLAY_BLINK_OFF  = 0x00;

const uint8_t SET_DDRAM_ADDR = 0x80;

/*
4-bit parallel communication with the HD44780 LCD controller.
Send 8-bit commands/data as two 4-bit nibbles:
First nibble: Upper 4 bits (D7-D4),
Second nibble: Lower 4 bits (D3-D0)

LCD_EN must be pulsed high then low to latch data.
Register Select (RS) - Set 0 for READING INSTRUCTIONS (IR) or 1 for WRITING DATA (DR) to the LCD
controller
*/
const gpio_num_t DEFAULT_EN        = GPIO_NUM_19;  // LCD Enable (pulsed high and low)
const gpio_num_t DEFAULT_RS        = GPIO_NUM_18;  // LCD RS signal (0 for IR, 1 for DR)4
const gpio_num_t DEFAULT_DB7       = GPIO_NUM_21;  // DATA BIT 7
const gpio_num_t DEFAULT_DB6       = GPIO_NUM_25;  // DATA BIT 6
const gpio_num_t DEFAULT_DB5       = GPIO_NUM_26;  // DATA BIT 5
const gpio_num_t DEFAULT_DB4       = GPIO_NUM_27;  // DATA BIT 4
const gpio_num_t DEFAULT_DB3       = GPIO_NUM_NC;  // db3-db0 disabled when 4bit dl is used
const gpio_num_t DEFAULT_DB2       = GPIO_NUM_NC;
const gpio_num_t DEFAULT_DB1       = GPIO_NUM_NC;
const gpio_num_t DEFAULT_DB0       = GPIO_NUM_NC;
const gpio_num_t DEFAULT_A0        = GPIO_NUM_36;  // ADC_CH0 pin lcd buttons control
const gpio_num_t DEFAULT_BACKLIGHT = GPIO_NUM_32;  // Optional backlight control

HD44780::HD44780()
    : en(DEFAULT_EN),
      rs(DEFAULT_RS),
      db7(DEFAULT_DB7),
      db6(DEFAULT_DB6),
      db5(DEFAULT_DB5),
      db4(DEFAULT_DB4),
      db3(DEFAULT_DB3),
      db2(DEFAULT_DB2),
      db1(DEFAULT_DB1),
      db0(DEFAULT_DB0),
      a0(DEFAULT_A0),
      backlight(DEFAULT_BACKLIGHT) {
    std::vector<gpio_config_t> configs{
        {.pin_bit_mask = (1ULL << en),
         .mode         = GPIO_MODE_OUTPUT,
         .pull_up_en   = GPIO_PULLUP_DISABLE,
         .pull_down_en = GPIO_PULLDOWN_DISABLE,
         .intr_type    = GPIO_INTR_DISABLE},
        {.pin_bit_mask = (1ULL << rs),
         .mode         = GPIO_MODE_OUTPUT,
         .pull_up_en   = GPIO_PULLUP_DISABLE,
         .pull_down_en = GPIO_PULLDOWN_DISABLE,
         .intr_type    = GPIO_INTR_DISABLE},
        {.pin_bit_mask = (1ULL << db7) | (1ULL << db6) | (1ULL << db5) | (1ULL << db4),
         .mode         = GPIO_MODE_OUTPUT,
         .pull_up_en   = GPIO_PULLUP_DISABLE,
         .pull_down_en = GPIO_PULLDOWN_DISABLE,
         .intr_type    = GPIO_INTR_DISABLE},
        {.pin_bit_mask = (1ULL << a0),
         .mode         = GPIO_MODE_INPUT,
         .pull_up_en   = GPIO_PULLUP_DISABLE,
         .pull_down_en = GPIO_PULLDOWN_DISABLE,
         .intr_type    = GPIO_INTR_DISABLE},
        {.pin_bit_mask = (1ULL << backlight),
         .mode         = GPIO_MODE_INPUT,
         .pull_up_en   = GPIO_PULLUP_DISABLE,
         .pull_down_en = GPIO_PULLDOWN_DISABLE,
         .intr_type    = GPIO_INTR_DISABLE},
    };

    for (auto config : configs) {
        ESP_ERROR_CHECK(gpio_config(&config));
    }

    gpio_set_level(rs, 0);
    gpio_set_level(en, 0);
    gpio_set_level(db4, 0);
    gpio_set_level(db5, 0);
    gpio_set_level(db6, 0);
    gpio_set_level(db7, 0);
    gpio_set_level(backlight, 1);

    ESP_LOGI(TAG, "GPIO pins configured successfully");

    init();
}

void HD44780::send_bits(uint8_t val) const {
    gpio_set_level(db7, (val >> 3) & 0x01);  // bit 3
    gpio_set_level(db6, (val >> 2) & 0x01);  // bit 2
    gpio_set_level(db5, (val >> 1) & 0x01);  // bit 1
    gpio_set_level(db4, (val >> 0) & 0x01);  // bit 0

    // "pulse enable"
    gpio_set_level(en, 1);
    esp_rom_delay_us(1);  // > 450ns
    gpio_set_level(en, 0);
    esp_rom_delay_us(50);
}

void HD44780::ir(uint8_t instruction) const {
    gpio_set_level(rs, 0);  // sets to IR

    send_bits(instruction >> 4);
    send_bits(instruction & 0x0F);

    esp_rom_delay_us(50);
}

void HD44780::dr(uint8_t data) const {
    gpio_set_level(rs, 1);  // sets to DR

    send_bits(data >> 4);
    send_bits(data & 0x0F);

    esp_rom_delay_us(50);
}

// HD44780 4-bit initialization sequence.
void HD44780::init() const {
    ESP_LOGI(TAG, "initialization...");

    vTaskDelay(pdMS_TO_TICKS(50));  // > 40ms

    gpio_set_level(rs, 0);

    // Initial 8-bit mode setup sequence
    send_bits(0x03);         // DB7=0, DB6=0, DB5=1, DB4=1 (0011)
    esp_rom_delay_us(4500);  // > 4.1ms
    send_bits(0x03);
    esp_rom_delay_us(4500);  // > 4.1ms
    send_bits(0x03);
    esp_rom_delay_us(150);  // > 100μs

    // Since the LCD is still in 8-bit mode, it
    // sees: `0010 xxxx`:
    send_bits(0x02);        // DB7=0, DB6=0, DB5=1, DB4=0 (0010).
    esp_rom_delay_us(150);  // > 100μs

    // 0x20 | 0x00 | 0x08 | 0x00 = 0x28 = 0010 1000
    // Breakdown: 0010 1000
    // DL bit (DB4) data length: 0 = 4-bit interface
    // N bit (DB3) lines: 1 = 2 lines
    // F bit (DB2) font: 0 = 5×8 dots
    // Function Set command (bits 7-5 = 001)
    ir(ENTER_FUNCSET | FUNCSET_4BIT | FUNCSET_2LINE | FUNCSET_5X8F);  // 4-bit, 2-line, 5x8
    esp_rom_delay_us(50);

    ir(CONTROL_DISPLAY | DISPLAY_ON | DISPLAY_CURSOR_OFF | DISPLAY_BLINK_OFF);
    esp_rom_delay_us(50);

    ir(CLEAR_DISPLAY);
    vTaskDelay(pdMS_TO_TICKS(2));  //  1.52ms

    ir(ENTER_ENTRYMODESET | ENTRYMODESET_RMOVE | ENTRYMODESET_SHIFT_DISPLAY);
    esp_rom_delay_us(50);

    ESP_LOGI(TAG, "init done");
}

void HD44780::print_char(char value) const {
    uint8_t uint8_v = (uint8_t)value;
    dr(uint8_v);
}

void HD44780::print_string(const char* value) const {
    while (*value) {
        print_char(*value++);
    }
}

uint8_t CursorPosition::addr() const {
    if (row == 0) {
        return 0x00 + col;  // First line starts at 0x00
    }
    return 0x40 + col;  // Second line starts at 0x40
};

void HD44780::set_cursor(const CursorPosition& pos) const {
    uint8_t addr = pos.addr();
    ir(SET_DDRAM_ADDR | addr);
}

void HD44780::clear_display() {
    ir(CLEAR_DISPLAY);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void HD44780::return_home() {
    ir(RETURN_HOME);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void HD44780::print_float(float value) const {
    char stringf_buf[] = "%.2f";
    char line1[64];
    sprintf(line1, stringf_buf, value);

    setenv("TZ", "UTC+2", 1);
    tzset();
    time_t now;
    char line2[64];
    struct tm* timeinfo;

    time(&now);
    timeinfo = localtime(&now);
    localtime_r(&now, timeinfo);
    strftime(line2, sizeof(line2), "%H%M%S %d%m%Y", timeinfo);

    set_cursor({0, 0});  // line 1
    print_string(line1);
    dr(0xDF);
    print_char('C');

    set_cursor({0, 1});  // line 2
    print_string(line2);

    ESP_LOGI(TAG, "%s %s", line1, line2);
}
};  // namespace lcd
