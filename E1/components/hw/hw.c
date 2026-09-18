#include "hw.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_periph.h"    // GPIO_PIN_MUX_REG[]
#include "soc/gpio_sig_map.h"   // SIG_GPIO_OUT_IDX

static void route_to_matrix(uint32_t pin, uint32_t extra)
{
    REG_WRITE(GPIO_PIN_MUX_REG[pin],
              (PIN_FUNC_GPIO << MCU_SEL_S) | (2u << FUN_DRV_S) | extra);
}

void hw_pad_out(uint32_t pin)
{
    route_to_matrix(pin, 0);
    REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + 4*pin, SIG_GPIO_OUT_IDX);
    if (pin < 32) REG_WRITE(GPIO_ENABLE_W1TS_REG,  1u << pin);
    else          REG_WRITE(GPIO_ENABLE1_W1TS_REG, 1u << (pin - 32));
}

void hw_pad_in(uint32_t pin, hw_pull_t pull)
{
    uint32_t extra = FUN_IE;
    if (pull == HW_PULL_UP)   extra |= FUN_PU;
    if (pull == HW_PULL_DOWN) extra |= FUN_PD;

    route_to_matrix(pin, extra);
    if (pin < 32) REG_WRITE(GPIO_ENABLE_W1TC_REG,  1u << pin);
    else          REG_WRITE(GPIO_ENABLE1_W1TC_REG, 1u << (pin - 32));
}

void hw_set(uint32_t pin)
{
    if (pin < 32) REG_WRITE(GPIO_OUT_W1TS_REG,  1u << pin);
    else          REG_WRITE(GPIO_OUT1_W1TS_REG, 1u << (pin - 32));
}

void hw_clear(uint32_t pin)
{
    if (pin < 32) REG_WRITE(GPIO_OUT_W1TC_REG,  1u << pin);
    else          REG_WRITE(GPIO_OUT1_W1TC_REG, 1u << (pin - 32));
}

void hw_write(uint32_t pin, bool level) { level ? hw_set(pin) : hw_clear(pin); }

bool hw_read(uint32_t pin)
{
    return (pin < 32) ? (REG_READ(GPIO_IN_REG)  >> pin)        & 1u
                      : (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1u;
}
