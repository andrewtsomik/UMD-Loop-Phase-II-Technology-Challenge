#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include <tone.h>

#define TONE_NODE DT_NODELABEL(tone)

static const struct pwm_dt_spec tone_pwm =
    PWM_DT_SPEC_GET(TONE_NODE);

int tone_init(void)
{
    if (!pwm_is_ready_dt(&tone_pwm)) {
        return -1;
    }

    return pwm_set_pulse_dt(&tone_pwm, 0);
}

int tone_play(uint32_t duration_ms, uint32_t frequency_hz)
{
    if (frequency_hz == 0) {
        return -1;
    }

    uint32_t period_ns = 1000000000U / frequency_hz;
    uint32_t pulse_ns = period_ns / 2;

    int ret = pwm_set_dt(&tone_pwm, period_ns, pulse_ns);

    if (ret != 0) {
        return ret;
    }

    k_msleep(duration_ms);

    return pwm_set_pulse_dt(&tone_pwm, 0);
}
