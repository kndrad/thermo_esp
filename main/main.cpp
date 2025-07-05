#include <cstdio>
#include <ctime>
#include <string>
#include <vector>

#include "FreeRTOSConfig.h"  // IWYU pragma: keep
#include "esp_log.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "include/lcd_hd44780.hpp"
#include "include/one_wire.hpp"

#include <sys/time.h>

static const char* TAG = "MAIN";

namespace realtime {

// Each pin has 40k ohm pulldown resistor to GND.
const gpio_num_t SCLK_PIN   = GPIO_NUM_33;  // SERIAL CLOCK - syncs data movement
const gpio_num_t DATAIO_PIN = GPIO_NUM_23;  // I/O data line.
const gpio_num_t CE_PIN     = GPIO_NUM_22;  // CHIP ENABLE, RST in some data sheets.

enum Register : uint8_t {
    SECONDS_WRITE = 0x80,
    SECONDS_READ  = 0x81,
    MINUTES_WRITE = 0x82,
    MINUTES_READ  = 0x83,
    HOURS_WRITE   = 0x84,
    HOURS_READ    = 0x85,
    DATE_WRITE    = 0x86,
    DATE_READ     = 0x87,
    MONTH_WRITE   = 0x88,
    MONTH_READ    = 0x89,
    DAY_WRITE     = 0x8A,
    DAY_READ      = 0x8B,
    YEAR_WRITE    = 0x8C,
    YEAR_READ     = 0x8D,
    CONTROL_WRITE = 0x8E,
    CONTROL_READ  = 0x8F
};

class DS1302 {
  private:
    const gpio_num_t ce;
    const gpio_num_t dataio;
    const gpio_num_t sclk;

  public:
    DS1302() : ce(CE_PIN), dataio(DATAIO_PIN), sclk(SCLK_PIN) {
        std::vector<gpio_config_t> configs{
            {.pin_bit_mask = (1ULL << ce),
             .mode         = GPIO_MODE_OUTPUT,
             .pull_up_en   = GPIO_PULLUP_DISABLE,
             .pull_down_en = GPIO_PULLDOWN_DISABLE,
             .intr_type    = GPIO_INTR_DISABLE},
            {.pin_bit_mask = (1ULL << dataio),
             .mode         = GPIO_MODE_INPUT_OUTPUT,
             .pull_up_en   = GPIO_PULLUP_DISABLE,
             .pull_down_en = GPIO_PULLDOWN_DISABLE,
             .intr_type    = GPIO_INTR_DISABLE},
            {.pin_bit_mask = (1ULL << sclk),
             .mode         = GPIO_MODE_OUTPUT,
             .pull_up_en   = GPIO_PULLUP_DISABLE,
             .pull_down_en = GPIO_PULLDOWN_DISABLE,
             .intr_type    = GPIO_INTR_DISABLE},
        };

        for (auto config : configs) {
            ESP_ERROR_CHECK(gpio_config(&config));
        }
        gpio_set_level(ce, 0);
        esp_rom_delay_us(1);
        gpio_set_level(sclk, 0);

        write(CONTROL_WRITE, 0x00);
        ESP_LOGI(TAG, "DS1302 configured");
    }

    void write(const Register& reg, const uint8_t data) {
        gpio_set_level(ce, 1);  // ce high inits data transfers
        esp_rom_delay_us(1);

        send_byte(static_cast<uint8_t>(reg));
        send_byte(data);

        gpio_set_level(ce, 0);
        esp_rom_delay_us(1);
    }

    uint8_t read(const Register& reg) {
        gpio_set_level(ce, 1);  // ce high inits data transfers
        esp_rom_delay_us(1);

        send_byte(static_cast<uint8_t>(reg));
        uint8_t data = receive_byte();

        gpio_set_level(ce, 0);
        esp_rom_delay_us(1);
        return data;
    }

    void start() {
        uint8_t val = read(SECONDS_READ);
        write(SECONDS_WRITE, val & 0x7F);  // Clear CH bit to start clock
        ESP_LOGI("DS1302", "Clock STARTED");
    }

    void stop() {
        uint8_t val = read(SECONDS_READ);
        write(SECONDS_WRITE, val | 0x80);  // set clock HALT flag bit
        ESP_LOGI("DS1302", "Clock STOPPED");
    }

  private:
    void send_byte(uint8_t data) {
        for (uint8_t i = 0; i < 8; i++) {
            gpio_set_level(dataio, (data >> i) & 0x01);

            esp_rom_delay_us(1);  // clock cycle start
            gpio_set_level(sclk, 1);
            esp_rom_delay_us(1);
            gpio_set_level(sclk, 0);
            esp_rom_delay_us(1);  // clock cycle end
        }
    }

    uint8_t receive_byte() {
        uint8_t data = 0;

        gpio_set_direction(dataio, GPIO_MODE_INPUT);
        for (uint8_t i = 0; i < 8; i++) {
            esp_rom_delay_us(1);      // clock cycle start
            gpio_set_level(sclk, 1);  // rising edge ...
            esp_rom_delay_us(1);
            gpio_set_level(sclk, 0);
            // -- got output here --
            uint8_t bit  = gpio_get_level(dataio);  // ... falling edge
            data        |= (bit << i);
            // -- continue cycle --
            esp_rom_delay_us(1);  // clock cycle end
        }
        gpio_set_direction(dataio, GPIO_MODE_INPUT_OUTPUT);

        return data;
    }
};

static uint8_t uint8_bcd_conv(uint8_t decimal) {
    return ((decimal / 10) << 4) | (decimal % 10);
}

static uint8_t bcd_uint8_conv(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

static void timefmt(char* buf, size_t buf_size, uint8_t hours, uint8_t minutes, uint8_t seconds) {
    snprintf(buf,
             buf_size,
             "%02d:%02d:%02d",
             bcd_uint8_conv(hours),
             bcd_uint8_conv(minutes),
             bcd_uint8_conv(seconds & 0x7F));
}

static void datetimefmt(char* buf,
                        size_t buf_size,
                        uint8_t hours,
                        uint8_t minutes,
                        uint8_t seconds,
                        uint8_t date,
                        uint8_t month,
                        uint8_t year) {
    snprintf(buf,
             buf_size,
             "20%02d-%02d-%02d %02d:%02d:%02d",
             bcd_uint8_conv(year),
             bcd_uint8_conv(month),
             bcd_uint8_conv(date),
             bcd_uint8_conv(hours),
             bcd_uint8_conv(minutes),
             bcd_uint8_conv(seconds & 0x7F));
}
};  // namespace realtime

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== LCD TEST STARTED ===");

    onewire::Master master{GPIO_NUM_4};
    onewire::DS18B20 sensor{master};
    lcd::HD44780 display{};
    realtime::DS1302 rtclock{};

    // --- rtc sequence ----
    rtclock.stop();
    rtclock.write(realtime::Register::SECONDS_WRITE, realtime::uint8_bcd_conv(0) | 0x80);
    rtclock.write(realtime::Register::MINUTES_WRITE, realtime::uint8_bcd_conv(47));
    rtclock.write(realtime::Register::HOURS_WRITE, realtime::uint8_bcd_conv(20));
    rtclock.write(realtime::Register::DATE_WRITE, realtime::uint8_bcd_conv(5));
    rtclock.write(realtime::Register::MONTH_WRITE, realtime::uint8_bcd_conv(7));
    rtclock.write(realtime::Register::YEAR_WRITE, realtime::uint8_bcd_conv(25));
    rtclock.start();
    // -- end rtc seq --

    // Is running?
    uint8_t sec_check = rtclock.read(realtime::Register::SECONDS_READ);
    ESP_LOGI(TAG,
             "Clock started. Seconds register: 0x%02X (CH bit: %s)",
             sec_check,
             (sec_check & 0x80) ? "SET (stopped)" : "CLEAR (running)");

    ESP_LOGI(TAG, "Initial time set to 20:47:00");
    while (true) {
        uint8_t seconds = rtclock.read(realtime::Register::SECONDS_READ);
        uint8_t minutes = rtclock.read(realtime::Register::MINUTES_READ);
        uint8_t hours   = rtclock.read(realtime::Register::HOURS_READ);
        uint8_t date    = rtclock.read(realtime::Register::DATE_READ);
        uint8_t month   = rtclock.read(realtime::Register::MONTH_READ);
        uint8_t year    = rtclock.read(realtime::Register::YEAR_READ);

        char time_buf[32];
        realtime::timefmt(time_buf, sizeof(time_buf), hours, minutes, seconds);

        display.set_cursor({0, 0});
        display.print_string(time_buf);
        display.print_char(' ');
        float temp = sensor.read_temperature();
        display.print_temperature(temp);
        ESP_LOGI("LCD", "%s %.2f", time_buf, temp);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
