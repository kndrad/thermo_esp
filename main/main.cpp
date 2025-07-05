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
const gpio_num_t CLK_PIN  = GPIO_NUM_33;  // SERIAL CLOCK - syncs data movement
const gpio_num_t DATA_PIN = GPIO_NUM_23;  // I/O data line.
const gpio_num_t EN_PIN   = GPIO_NUM_22;  // CHIP ENABLE, RST in some data sheets.

uint8_t SECONDS_WRITE = 0x80, SECONDS_READ = 0x81, MINUTES_WRITE = 0x82, MINUTES_READ = 0x83,
        HOURS_WRITE = 0x84, HOURS_READ = 0x85, DATE_WRITE = 0x86, DAY_READ = 0x87,
        MONTH_WRITE = 0x88, MONTH_READ = 0x89, DAY_WRITE = 0x8A, DOW_READ = 0x8B, YEAR_WRITE = 0x8C,
        YEAR_READ = 0x8D, WRPROTECT = 0x8E, BURST = 0xBE;

typedef struct {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t dow;
} DateTime;

static uint8_t dec2bcd(uint8_t dec) {
    return ((dec / 10 * 16) + (dec % 10));
}

static uint8_t bcd2dec(uint8_t bcd) {
    return ((bcd / 16 * 10) + (bcd % 16));
}

class DS1302 {
  private:
    const gpio_num_t en_pin;
    const gpio_num_t data_pin;
    const gpio_num_t clk_pin;

  public:
    DS1302() : en_pin(EN_PIN), data_pin(DATA_PIN), clk_pin(CLK_PIN) {
        std::vector<gpio_config_t> configs{
            {.pin_bit_mask = (1ULL << en_pin),
             .mode         = GPIO_MODE_OUTPUT,
             .pull_up_en   = GPIO_PULLUP_DISABLE,
             .pull_down_en = GPIO_PULLDOWN_DISABLE,
             .intr_type    = GPIO_INTR_DISABLE},
            {.pin_bit_mask = (1ULL << data_pin),
             .mode         = GPIO_MODE_INPUT_OUTPUT,
             .pull_up_en   = GPIO_PULLUP_DISABLE,
             .pull_down_en = GPIO_PULLDOWN_DISABLE,
             .intr_type    = GPIO_INTR_DISABLE},
            {.pin_bit_mask = (1ULL << clk_pin),
             .mode         = GPIO_MODE_OUTPUT,
             .pull_up_en   = GPIO_PULLUP_DISABLE,
             .pull_down_en = GPIO_PULLDOWN_DISABLE,
             .intr_type    = GPIO_INTR_DISABLE},
        };

        for (auto config : configs) {
            ESP_ERROR_CHECK(gpio_config(&config));
        }

        gpio_set_direction(data_pin, GPIO_MODE_INPUT);
        gpio_set_level(en_pin, 0);
        gpio_set_level(clk_pin, 0);

        ESP_LOGI(TAG, "DS1302 configured");
    }

    void write_datetime(DateTime* dt) {
        prep_wr(WRPROTECT);
        write_byte(0x00);

        gpio_set_level(en_pin, 0);

        prep_wr(BURST);
        write_byte(dec2bcd(dt->second % 60));
        write_byte(dec2bcd(dt->minute % 60));
        write_byte(dec2bcd(dt->hour % 24));
        write_byte(dec2bcd(dt->day % 32));
        write_byte(dec2bcd(dt->month % 13));
        write_byte(dec2bcd(dt->dow % 8));
        write_byte(dec2bcd(dt->year % 100));
        write_byte(0x80);

        gpio_set_level(en_pin, 0);
    }

    void read_datetime(DateTime* dt) {
        prep_rd(BURST);

        dt->second = bcd2dec(read_byte() & SECONDS_READ);
        dt->minute = bcd2dec(read_byte() & MINUTES_READ);
        dt->hour   = bcd2dec(read_byte() & HOURS_READ);
        dt->day    = bcd2dec(read_byte() & DAY_READ);
        dt->month  = bcd2dec(read_byte() & MONTH_READ);
        dt->dow    = bcd2dec(read_byte() & DOW_READ);
        dt->year   = bcd2dec(read_byte() & YEAR_READ);

        gpio_set_level(en_pin, 0);
    }

    // true means stopped (halt flag set to 1)
    void halt(bool flag) {
        uint8_t regs[8];
        prep_rd(BURST);

        for (int b = 0; b < 8; b++) {
            regs[b] = read_byte();
        }
        gpio_set_level(en_pin, 0);

        if (flag) {
            regs[0] = 0x80;
        } else {
            regs[0] &= ~0x80;
        }
        regs[7] = 0x80;

        prep_wr(WRPROTECT);
        write_byte(0x00);
        gpio_set_level(en_pin, 0);

        prep_wr(BURST);
        for (int b = 0; b < 8; b++) {
            write_byte(regs[b]);
        }
        gpio_set_level(en_pin, 0);
    }

    void start() {
        halt(0);
        ESP_LOGI("DS1302", "Clock STARTED (halt 0)");
    }

    void stop() {
        halt(1);
        ESP_LOGI("DS1302", "Clock STOPPED (halt 1)");
    }

    void next_bit() {
        gpio_set_level(clk_pin, 1);
        esp_rom_delay_us(1);
        gpio_set_level(clk_pin, 0);
        esp_rom_delay_us(1);
    }

    void write_byte(uint8_t value) {
        for (uint8_t i = 0; i < 8; i++) {
            gpio_set_level(data_pin, (value & 0x01) ? 1 : 0);
            next_bit();
            value >>= 1;
        }
    }

    uint8_t read_byte() {
        uint8_t byte = 0;

        for (uint8_t b = 0; b < 8; b++) {
            if (gpio_get_level(data_pin) == 1) {
                byte |= 0x01 << b;
            }
            next_bit();
        }

        return byte;
    }

    void prep_wr(uint8_t addr) {
        gpio_set_direction(data_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(en_pin, 1);

        uint8_t cmd = 0x80 | addr;
        write_byte(cmd);
    }

    void prep_rd(uint8_t addr) {
        gpio_set_direction(data_pin, GPIO_MODE_OUTPUT);
        gpio_set_level(en_pin, 1);

        uint8_t cmd = 0x81 | addr;
        write_byte(cmd);

        gpio_set_direction(data_pin, GPIO_MODE_INPUT);
    }
};

static void ftime(char* buf, size_t buf_size, uint8_t hours, uint8_t minutes, uint8_t seconds) {
    snprintf(
        buf, buf_size, "%02d:%02d:%02d", bcd2dec(hours), bcd2dec(minutes), bcd2dec(seconds & 0x7F));
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
             bcd2dec(year),
             bcd2dec(month),
             bcd2dec(date),
             bcd2dec(hours),
             bcd2dec(minutes),
             bcd2dec(seconds & 0x7F));
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
    realtime::DateTime rtc_dt{
        .year = 25, .month = 7, .day = 6, .hour = 1, .minute = 22, .second = 30, .dow = 7};
    rtclock.write_datetime(&rtc_dt);
    // -- end rtc seq --

    while (true) {
        realtime::DateTime dt_now{};
        rtclock.read_datetime(&dt_now);

        display.set_cursor({0, 0});
        char buf[32];
        realtime::ftime(buf, sizeof(buf), dt_now.hour, dt_now.minute, dt_now.second);
        display.print_string(buf);
        display.print_char(' ');
        float temp = sensor.read_temperature();
        display.print_temperature(temp);
        ESP_LOGI("LCD", "LINE1: %s %.2f degree C", buf, temp);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
