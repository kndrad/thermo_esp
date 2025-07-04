#pragma once

#include <cstdint>

#include "driver/gpio.h"

namespace lcd {

extern const gpio_num_t DEFAULT_EN;
extern const gpio_num_t DEFAULT_RS;
extern const gpio_num_t DEFAULT_DB7;
extern const gpio_num_t DEFAULT_DB6;
extern const gpio_num_t DEFAULT_DB5;
extern const gpio_num_t DEFAULT_DB4;
extern const gpio_num_t DEFAULT_DB3;  // db3-db0 disabled when 4bit dl is used
extern const gpio_num_t DEFAULT_DB2;
extern const gpio_num_t DEFAULT_DB1;
extern const gpio_num_t DEFAULT_DB0;
extern const gpio_num_t DEFAULT_A0;
extern const gpio_num_t DEFAULT_BACKLIGHT;

extern const uint8_t ENTER_FUNCSET;
extern const uint8_t FUNCSET_4BIT;
extern const uint8_t FUNCSET_2LINE;
extern const uint8_t FUNCSET_5X8F;

extern const uint8_t CLEAR_DISPLAY;
extern const uint8_t RETURN_HOME;

extern const uint8_t ENTER_ENTRYMODESET;
extern const uint8_t ENTRYMODESET_DDRAM_ADDR_INC;
extern const uint8_t ENTRYMODESET_RMOVE;
extern const uint8_t ENTRYMODESET_SHIFT_DISPLAY;

extern const uint8_t CONTROL_DISPLAY;
extern const uint8_t DISPLAY_ON;
extern const uint8_t DISPLAY_CURSOR_OFF;
extern const uint8_t DISPLAY_BLINK_OFF;

extern const uint8_t SET_DDRAM_ADDR;

struct CursorPosition {
    uint8_t col;
    uint8_t row;
    uint8_t addr() const;
};

class HD44780 {
  public:
    const gpio_num_t en;
    const gpio_num_t rs;
    const gpio_num_t db7;
    const gpio_num_t db6;
    const gpio_num_t db5;
    const gpio_num_t db4;
    const gpio_num_t db3;  // db3-db0 disabled when 4bit dl is used
    const gpio_num_t db2;
    const gpio_num_t db1;
    const gpio_num_t db0;
    const gpio_num_t a0;
    const gpio_num_t backlight;

    HD44780();
    void print_char(char value) const;
    void print_string(const char* value) const;
    void set_cursor(const CursorPosition& pos) const;
    void clear_display();
    void return_home();
    void print_float(float value) const;

  private:
    void init() const;
    void send_bits(uint8_t value) const;
    void ir(uint8_t instruction) const;
    void dr(uint8_t data) const;
};
};  // namespace lcd
