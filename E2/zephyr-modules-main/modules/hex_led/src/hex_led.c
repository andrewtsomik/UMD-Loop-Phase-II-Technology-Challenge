#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <hex_led.h>

#define DT_DRV_COMPAT mycompany_hex_led

struct hex_led_config {
    struct gpio_dt_spec close;
    struct gpio_dt_spec far;
    struct gpio_dt_spec exact;
};

static int hex_led_init(const struct device *dev)
{
    const struct hex_led_config *config = dev->config;
    int ret;

    ret = gpio_pin_configure_dt(&config->close,
                                GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        return ret;
    }

    ret = gpio_pin_configure_dt(&config->far,
                                GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        return ret;
    }

    ret = gpio_pin_configure_dt(&config->exact,
                                GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

static int hex_led_set_impl(const struct device *dev,
                            enum hex_led_state state)
{
    const struct hex_led_config *config = dev->config;
    int ret;

    ret = gpio_pin_set_dt(&config->close, 0);
    if (ret < 0) {
        return ret;
    }

    ret = gpio_pin_set_dt(&config->far, 0);
    if (ret < 0) {
        return ret;
    }

    ret = gpio_pin_set_dt(&config->exact, 0);
    if (ret < 0) {
        return ret;
    }

    switch (state) {
    case HEX_LED_OFF:
        break;

    case HEX_LED_CLOSE:
        ret = gpio_pin_set_dt(&config->close, 1);
        break;

    case HEX_LED_EXACT:
        ret = gpio_pin_set_dt(&config->exact, 1);
        break;

    case HEX_LED_FAR:
        ret = gpio_pin_set_dt(&config->far, 1);
        break;

    default:
        return -EINVAL;
    }

    return ret;
}

static const struct hex_led_driver_api hex_led_api = {
    .set = hex_led_set_impl,
};

#define HEX_LED_DEFINE(inst)                                      \
    static const struct hex_led_config hex_led_config_##inst = { \
        .close = GPIO_DT_SPEC_INST_GET(inst, close_gpios),       \
        .far = GPIO_DT_SPEC_INST_GET(inst, far_gpios),           \
        .exact = GPIO_DT_SPEC_INST_GET(inst, exact_gpios),       \
    };                                                           \
                                                                 \
    DEVICE_DT_INST_DEFINE(inst,                                  \
                          hex_led_init,                           \
                          NULL,                                  \
                          NULL,                                  \
                          &hex_led_config_##inst,                 \
                          POST_KERNEL,                            \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,   \
                          &hex_led_api);

DT_INST_FOREACH_STATUS_OKAY(HEX_LED_DEFINE)
