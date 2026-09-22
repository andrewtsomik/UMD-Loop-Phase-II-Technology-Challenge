#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include <errno.h>
#include <stdint.h>

#include "receiver.h"

LOG_MODULE_REGISTER(receiver, LOG_LEVEL_INF);

/*

* Obtain the UART specified by the receiver node in devicetree.
  */
  #define RECEIVER_NODE DT_NODELABEL(receiver)

#if !DT_NODE_EXISTS(RECEIVER_NODE)
#error "No receiver node found in the device tree"
#endif

#if !DT_NODE_HAS_PROP(RECEIVER_NODE, uart)
#error "Receiver node must have a 'uart' property"
#endif

static const struct device *const receiver_uart =
DEVICE_DT_GET(DT_PHANDLE(RECEIVER_NODE, uart));

int receiver_init(void)
{
if (!device_is_ready(receiver_uart)) {
LOG_ERR("UART device is not ready");
return -ENODEV;
}

LOG_INF("Receiver initialized");

return 0;

}

int receiver_read(uint8_t *value)
{
unsigned char data;
int ret;

if (value == NULL) {
    return -EINVAL;
}

ret = uart_poll_in(receiver_uart, &data);

if (ret == 0) {
    *value = (uint8_t)data;

    LOG_DBG("Received value: %u", *value);

    return 0;
}

/*
 * uart_poll_in() returns -1 when there is currently
 * no data waiting in the UART receive buffer.
 */
if (ret == -1) {
    return -EAGAIN;
}

return ret;

}
