#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/ledc_reg.h"
#include "soc/dport_reg.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/uart_reg.h"
#include "esp_rom_sys.h"
#include "hw.h"

#define SERVO_PIN      5               // H1 header, S pin

#define PERIOD_US      20000u          // 50 Hz
#define PULSE_MIN_US   1000u           // 0 degrees
#define PULSE_MAX_US   2000u           // 180 degrees
#define DUTY_RES_BITS  16u
#define DUTY_MAX       (1u << DUTY_RES_BITS)

#define US_TO_DUTY(us) ((uint32_t)((uint64_t)(us) * DUTY_MAX / PERIOD_US))

// ---------- LEDC setup ----------

static void ledc_timer_init(void)
{
    // LEDC, unlike GPIO, is clock-gated. Ungate and release reset first.
    DPORT_REG_SET_BIT(DPORT_PERIP_CLK_EN_REG, DPORT_LEDC_CLK_EN);
    DPORT_REG_CLR_BIT(DPORT_PERIP_RST_EN_REG, DPORT_LEDC_RST);

    // 80 MHz APB / (6250/256) / 2^16  =  50.000 Hz exactly
    REG_WRITE(LEDC_HSTIMER0_CONF_REG,
              (1u    << LEDC_TICK_SEL_HSTIMER0_S)  |   // 1 = APB_CLK
              (6250u << LEDC_DIV_NUM_HSTIMER0_S)   |   // 10.8 fixed point
              (DUTY_RES_BITS << LEDC_HSTIMER0_DUTY_RES_S));

    REG_SET_BIT(LEDC_HSTIMER0_CONF_REG, LEDC_HSTIMER0_RST);
    REG_CLR_BIT(LEDC_HSTIMER0_CONF_REG, LEDC_HSTIMER0_RST);
}

static void servo_attach(uint32_t pin)
{
    REG_WRITE(LEDC_HSCH0_HPOINT_REG, 0);
    REG_WRITE(LEDC_HSCH0_CONF0_REG,
              (0u << LEDC_TIMER_SEL_HSCH0_S) |         // follow HS timer 0
              LEDC_SIG_OUT_EN_HSCH0);

    // Same pad setup as System One, then repoint the matrix at LEDC
    // instead of GPIO_OUT. One register decides what drives the pin.
    hw_pad_out(pin);
    REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + 4*pin, LEDC_HS_SIG_OUT0_IDX);
}

static void servo_write_us(uint32_t pulse_us)
{
    REG_WRITE(LEDC_HSCH0_DUTY_REG, US_TO_DUTY(pulse_us) << 4);  // 4 frac bits
    REG_WRITE(LEDC_HSCH0_CONF1_REG, LEDC_DUTY_START_HSCH0);     // latch it
}

static uint32_t angle_to_us(uint32_t deg)
{
    if (deg > 180) deg = 180;
    return PULSE_MIN_US + (deg * (PULSE_MAX_US - PULSE_MIN_US)) / 180u;
}

// ---------- UART input, decimal ----------

static uint8_t uart_rx_byte(void)
{
    while (((REG_READ(UART_STATUS_REG(0)) >> UART_RXFIFO_CNT_S)
             & UART_RXFIFO_CNT_V) == 0) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return REG_READ(UART_FIFO_AHB_REG(0)) & 0xFF;
}

static uint32_t read_decimal(void)
{
    uint32_t v = 0;
    int digits = 0;

    esp_rom_printf("angle> ");
    while (1) {
        uint8_t c = uart_rx_byte();

        if (c == '\r' || c == '\n') {
            if (digits) { esp_rom_printf("\n"); return v; }
            continue;
        }
        if (c < '0' || c > '9') continue;

        v = v * 10u + (uint32_t)(c - '0');
        if (v > 9999u) v = 9999u;
        digits++;
        esp_rom_printf("%c", c);
    }
}

// ---------- main ----------

void app_main(void)
{
    ledc_timer_init();
    servo_attach(SERVO_PIN);
    servo_write_us(angle_to_us(90));          // centre on boot

    esp_rom_printf("\nSystem Two | SG90 on GPIO%d | enter 0-180\n\n", SERVO_PIN);

    while (1) {
        uint32_t deg = read_decimal();

        if (deg > 180u) {
            esp_rom_printf("out of range: %u (valid 0-180), ignored\n\n",
                           (unsigned)deg);
            continue;
        }
        uint32_t us = angle_to_us(deg);
        servo_write_us(us);

        esp_rom_printf("%u deg  ->  %u us  ->  duty %u\n\n",
                       (unsigned)deg, (unsigned)us,
                       (unsigned)US_TO_DUTY(us));
    }
}
