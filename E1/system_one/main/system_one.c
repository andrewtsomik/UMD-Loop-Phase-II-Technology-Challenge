#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_reg.h"
#include "esp_rom_sys.h"
#include "hw.h"

#define LED_R       4
#define LED_G      18
#define LED_B      13

#define TARGET      0xA5u
#define NEAR_BAND   0x10u

// bit 0 first — LSB is the leftmost switch on your breadboard
static const uint8_t DATA_PIN[8] = { 14, 16, 17, 19, 23, 27, 32, 33 };

static uint8_t read_switches(void)
{
    uint32_t lo = REG_READ(GPIO_IN_REG);    // pins 0–31
    uint32_t hi = REG_READ(GPIO_IN1_REG);   // pins 32–39
    uint8_t  v  = 0;

    for (int i = 0; i < 8; i++) {
        uint8_t  p   = DATA_PIN[i];
        uint32_t bit = (p < 32) ? (lo >> p) : (hi >> (p - 32));
        v |= (uint8_t)((bit & 1u) << i);
    }
    return v;
}

static void all_off(void)
{
    hw_clear(LED_R);
    hw_clear(LED_G);
    hw_clear(LED_B);
}

void app_main(void)
{
    hw_pad_out(LED_R);
    hw_pad_out(LED_G);
    hw_pad_out(LED_B);
    all_off();

    for (int i = 0; i < 8; i++)
        hw_pad_in(DATA_PIN[i], HW_PULL_DOWN);

    uint8_t last = (uint8_t)~read_switches();   // force a first update

    while (1) {
        uint8_t guess = read_switches();

        if (guess != last) {
            last = guess;
            uint32_t delta = (guess > TARGET) ? guess - TARGET
                                              : TARGET - guess;
            all_off();
            if (delta == 0)              hw_set(LED_G);
            else if (delta <= NEAR_BAND) hw_set(LED_B);
            else                         hw_set(LED_R);

            esp_rom_printf("0x%02X  delta 0x%02X\n",
                           (unsigned)guess, (unsigned)delta);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
