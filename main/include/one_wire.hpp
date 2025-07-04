#pragma once

#include <cstdint>

#include "hal/gpio_types.h"
#include "soc/gpio_num.h"

namespace onewire {

static const char* TAG                   = "1WIRE";
static const gpio_num_t DEFAULT_GPIO_NUM = GPIO_NUM_4;

class Bus {
  public:
    Bus();
    Bus(gpio_num_t gpio_num);
    bool reset();
    bool read_bit();
    void write_bit(bool bit);
    void write_byte(uint8_t data);
    uint8_t read_byte();

  private:
    const gpio_num_t gpio_num{};
};

class Master {
  public:
    Master(gpio_num_t gpio_num);

    bool init();
    void write_byte(uint8_t data);
    uint8_t read_byte();

  private:
    Bus bus;
};

class Slave {
  public:
    Slave(Master& master);

  protected:
    Master& master;

    void skip_rom();
    void read_rom();
    void match_rom(uint64_t rom_code);
    void search_rom();
    void alarm_search();
};

class DS18B20 : public Slave {
  public:
    DS18B20(Master& master);

    float read_temperature();
    bool is_externally_powered();

  private:
    void convert_temp();
    void read_scratchpad();
};

};  // namespace onewire
