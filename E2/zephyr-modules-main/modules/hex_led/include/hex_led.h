
#ifndef HEX_LED_H
#define HEX_LED_H

#include <zephyr/device.h>

enum hex_led_state {
    HEX_LED_OFF,
    HEX_LED_CLOSE,
    HEX_LED_EXACT,
    HEX_LED_FAR,
};

struct hex_led_driver_api {
    int (*set)(const struct device *dev,
               enum hex_led_state state);
};

static inline int hex_led_set(const struct device *dev,
                              enum hex_led_state state)
{
    const struct hex_led_driver_api *api = dev->api;

    return api->set(dev, state);
}

#endif /* HEX_LED_H */

