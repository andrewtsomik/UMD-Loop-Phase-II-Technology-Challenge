#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <errno.h>

#include "servo.h"

#define SERVO_NODE DT_NODELABEL(servo)

static const struct pwm_dt_spec servo_pwm =
    PWM_DT_SPEC_GET(SERVO_NODE);

static uint32_t angle_to_pulse_us(uint32_t angle)
{
    uint32_t min_pulse = DT_PROP(SERVO_NODE, min_pulse);
    uint32_t max_pulse = DT_PROP(SERVO_NODE, max_pulse);
    uint32_t min_angle = DT_PROP(SERVO_NODE, min_angle);
    uint32_t max_angle = DT_PROP(SERVO_NODE, max_angle);

    angle = CLAMP(angle, min_angle, max_angle);

    return min_pulse +
           ((angle - min_angle) * (max_pulse - min_pulse)) /
           (max_angle - min_angle);
}

int servo_init(void)
{
    if (!device_is_ready(servo_pwm.dev)) {
        return -ENODEV;
    }

    return servo_set_angle(90);
}

int servo_set_angle(uint32_t angle)
{
    uint32_t pulse_us = angle_to_pulse_us(angle);

    return pwm_set_dt(&servo_pwm,
                      PWM_USEC(20000),
                      PWM_USEC(pulse_us));
}
