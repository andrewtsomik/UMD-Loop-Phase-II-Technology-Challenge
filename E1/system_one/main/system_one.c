#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/uart_reg.h"
#include "esp_rom_sys.h"
#include "hw.h"

#define LED_CLOSE   4     // yellow
#define LED_FAR    18     // red
#define LED_EXACT  13     // green

#define TARGET      0xA5u
#define NEAR_BAND   0x10u

// ---------- UART0 receive, register level ----------

static uint8_t uart_rx_byte(void)
{
    while (((REG_READ(UART_STATUS_REG(0)) >> UART_RXFIFO_CNT_S)
             & UART_RXFIFO_CNT_V) == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));       // yield so the WDT stays quiet
    }
    return REG_READ(UART_FIFO_AHB_REG(0)) & 0xFF;
}

static int hex_digit(uint8_t c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static uint32_t read_hex(void)
{
    uint32_t v = 0;
    int digits = 0;

    esp_rom_printf("hex> ");
    while (1) {
        uint8_t c = uart_rx_byte();

        if (c == '\r' || c == '\n') {
            if (digits) { esp_rom_printf("\n"); return v; }
            continue;
        }
        int d = hex_digit(c);
        if (d < 0) continue;                 // ignores "0x", spaces, junk

        v = (v << 4) | (uint32_t)d;
        digits++;
        esp_rom_printf("%c", c);             // monitor gives no local echo
    }
}

// ---------- main ----------

static void all_off(void)
{
    hw_clear(LED_CLOSE);
    hw_clear(LED_FAR);
    hw_clear(LED_EXACT);
}

void app_main(void)
{
    hw_pad_out(LED_CLOSE);
    hw_pad_out(LED_FAR);
    hw_pad_out(LED_EXACT);
    all_off();

    esp_rom_printf("\nSystem One | target 0x%02X | "
                   "green=exact yellow=close red=far\n\n", TARGET);

    while (1) {
        uint32_t guess = read_hex();
        uint32_t delta = (guess > TARGET) ? guess - TARGET : TARGET - guess;

        all_off();
        if (delta == 0)              hw_set(LED_EXACT);
        else if (delta <= NEAR_BAND) hw_set(LED_CLOSE);
        else                         hw_set(LED_FAR);

        esp_rom_printf("0x%02X  delta 0x%02X\n\n",
                       (unsigned)guess, (unsigned)delta);
    }
}
