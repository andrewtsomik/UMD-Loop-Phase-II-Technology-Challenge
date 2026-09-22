#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <servo.h>
#include <hex_led.h>
#include <tone.h>
#include <receiver.h>

#define TARGET      0x3FF44564u
#define NEAR_BAND   16u

#define STACK_SIZE  2048
#define PRIORITY    5
#define QUEUE_SIZE  8

/* Tone command limits */
#define TONE_MIN_HZ   20u
#define TONE_MAX_HZ   20000u
#define TONE_MIN_MS   1u
#define TONE_MAX_MS   10000u

K_MSGQ_DEFINE(hex_msgq, sizeof(uint32_t), QUEUE_SIZE, 4);
K_MSGQ_DEFINE(angle_msgq, sizeof(uint32_t), QUEUE_SIZE, 4);
K_MSGQ_DEFINE(tone_msgq, sizeof(uint32_t) * 2, QUEUE_SIZE, 4);
K_MSGQ_DEFINE(receiver_msgq, sizeof(uint8_t), QUEUE_SIZE, 4);

K_THREAD_STACK_DEFINE(led_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(servo_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(input_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(tone_stack, STACK_SIZE);
K_THREAD_STACK_DEFINE(receiver_stack, STACK_SIZE);

K_MUTEX_DEFINE(print_mutex);

/* All task output goes through this so lines from different threads
 * don't interleave. */
#define LOCKED_PRINTF(...)                      \
    do {                                        \
        k_mutex_lock(&print_mutex, K_FOREVER);  \
        printf(__VA_ARGS__);                    \
        k_mutex_unlock(&print_mutex);           \
    } while (0)

static struct k_thread receiver_thread_data;
static struct k_thread led_thread_data;
static struct k_thread servo_thread_data;
static struct k_thread input_thread_data;
static struct k_thread tone_thread_data;

static const struct device *leds =
    DEVICE_DT_GET(DT_NODELABEL(hex_led));

static const struct device *uart_dev =
    DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

struct tone_command {
    uint32_t duration_ms;
    uint32_t frequency_hz;
};

/* ---------- Input task ---------- */

static void input_task(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    char line[32];
    size_t pos = 0;

    LOCKED_PRINTF("\nCommands:\n"
                  "  hex <value>\n"
                  "  angle <1-180>\n"
                  "  tone <time(ms)> <frequency(Hz)>\n\n"
                  "> ");

    while (1) {
        unsigned char c;

        if (uart_poll_in(uart_dev, &c) != 0) {
            k_msleep(10);
            continue;
        }

        /* Ignore carriage return; process newline */
        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            line[pos] = '\0';

            if (pos > 0) {

                if (strncmp(line, "hex ", 4) == 0) {

                    uint32_t value =
                        (uint32_t)strtoul(&line[4], NULL, 16);

                    if (k_msgq_put(&hex_msgq,
                                   &value,
                                   K_NO_WAIT) != 0) {
                        LOCKED_PRINTF("\nHex queue full\n");
                    }
                }

                else if (strncmp(line, "angle ", 6) == 0) {

                    uint32_t angle =
                        (uint32_t)strtoul(&line[6], NULL, 10);

                    if (angle >= 1 && angle <= 180) {

                        if (k_msgq_put(&angle_msgq,
                                       &angle,
                                       K_NO_WAIT) != 0) {
                            LOCKED_PRINTF("\nAngle queue full\n");
                        }

                    } else {
                        LOCKED_PRINTF("\nAngle must be 1-180\n");
                    }
                }

                else if (strncmp(line, "tone ", 5) == 0) {

                    char *end;

                    uint32_t duration =
                        (uint32_t)strtoul(&line[5], &end, 10);

                    while (*end == ' ') {
                        end++;
                    }

                    uint32_t frequency =
                        (uint32_t)strtoul(end, NULL, 10);

                    struct tone_command command = {
                        .duration_ms = duration,
                        .frequency_hz = frequency
                    };

                    if (duration < TONE_MIN_MS || duration > TONE_MAX_MS ||
                        frequency < TONE_MIN_HZ || frequency > TONE_MAX_HZ) {
                        LOCKED_PRINTF("\nUsage: tone <%u-%u ms> <%u-%u Hz>\n",
                                      TONE_MIN_MS, TONE_MAX_MS,
                                      TONE_MIN_HZ, TONE_MAX_HZ);
                    } else if (k_msgq_put(&tone_msgq,
                                          &command,
                                          K_NO_WAIT) != 0) {
                        LOCKED_PRINTF("\nTone queue full\n");
                    }
                }

                else {
                    LOCKED_PRINTF("\nUnknown command: %s\n", line);
                }
            }

            pos = 0;
            LOCKED_PRINTF("> ");
        }

        else if (pos < sizeof(line) - 1) {
            line[pos++] = c;
	    putchar(c);
        }
    }
}

/* ---------- LED task ---------- */

static void led_task(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    uint32_t guess;

    hex_led_set(leds, HEX_LED_OFF);

    while (1) {
        k_msgq_get(&hex_msgq, &guess, K_FOREVER);

        uint32_t delta =
            (guess > TARGET) ?
            guess - TARGET :
            TARGET - guess;

        if (delta == 0) {
            hex_led_set(leds, HEX_LED_EXACT);
        }
        else if (delta <= NEAR_BAND) {
            hex_led_set(leds, HEX_LED_CLOSE);
        }
        else {
            hex_led_set(leds, HEX_LED_FAR);
        }

        LOCKED_PRINTF("\nHex: 0x%08X  delta: 0x%X\n",
                      (unsigned)guess,
                      (unsigned)delta);
    }
}

/* ---------- Servo task ---------- */

static void servo_task(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    uint32_t angle;

    servo_init();

    while (1) {
        k_msgq_get(&angle_msgq, &angle, K_FOREVER);

        servo_set_angle(angle);

        LOCKED_PRINTF("\nServo: %u degrees\n", (unsigned)angle);
    }
}

/* ---------- Tone task ---------- */

static void tone_task(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct tone_command command;

    int ret = tone_init();
    if (ret != 0) {
        LOCKED_PRINTF("Tone init failed: %d\n", ret);
        return;
    }

    while (1) {
        k_msgq_get(&tone_msgq, &command, K_FOREVER);

        ret = tone_play(command.duration_ms, command.frequency_hz);
        if (ret != 0) {
            LOCKED_PRINTF("\nTone play failed: %d\n", ret);
            continue;
        }

        LOCKED_PRINTF("\nTone: %u Hz for %u ms\n",
                      (unsigned)command.frequency_hz,
                      (unsigned)command.duration_ms);
    }
}

/* ---------- Receiver task ---------- */

static void receiver_task(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    uint8_t value;

    if (receiver_init() != 0) {
        LOCKED_PRINTF("Receiver initialization failed!\n");
        return;
    }

    LOCKED_PRINTF("\nReceiver ready. Waiting for 0-255 values...\n");

    while (1) {

        if (receiver_read(&value) == 0) {

            /* One UART byte represents one signal value (0-255). */
            LOCKED_PRINTF("\nReceived signal: %u (0x%02X)\n",
                          (unsigned)value,
                          (unsigned)value);

            /* Hand the value to whichever subsystem consumes it. */
            if (k_msgq_put(&receiver_msgq, &value, K_NO_WAIT) != 0) {
                LOCKED_PRINTF("Receiver queue full; value discarded\n");
            }
        }

        k_msleep(5000);
    }
}

/* ---------- Main ---------- */

int main(void)
{
    if (!device_is_ready(leds)) {
        printf("LED device not ready!\n");
        return 0;
    }

    if (!device_is_ready(uart_dev)) {
        printf("UART not ready!\n");
        return 0;
    }

    k_thread_create(&input_thread_data,
                    input_stack,
                    K_THREAD_STACK_SIZEOF(input_stack),
                    input_task,
                    NULL, NULL, NULL,
                    PRIORITY,
                    0,
                    K_NO_WAIT);

    k_thread_create(&led_thread_data,
                    led_stack,
                    K_THREAD_STACK_SIZEOF(led_stack),
                    led_task,
                    NULL, NULL, NULL,
                    PRIORITY,
                    0,
                    K_NO_WAIT);

    k_thread_create(&servo_thread_data,
                    servo_stack,
                    K_THREAD_STACK_SIZEOF(servo_stack),
                    servo_task,
                    NULL, NULL, NULL,
                    PRIORITY,
                    0,
                    K_NO_WAIT);

    k_thread_create(&tone_thread_data,
                    tone_stack,
                    K_THREAD_STACK_SIZEOF(tone_stack),
                    tone_task,
                    NULL, NULL, NULL,
                    PRIORITY,
                    0,
                    K_NO_WAIT);

    k_thread_create(&receiver_thread_data,
                    receiver_stack,
                    K_THREAD_STACK_SIZEOF(receiver_stack),
                    receiver_task,
                    NULL, NULL, NULL,
                    PRIORITY,
                    0,
                    K_NO_WAIT);

    return 0;
}
