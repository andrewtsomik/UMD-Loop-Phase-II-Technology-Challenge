#pragma once
#include <stdint.h>
#include <stdbool.h>

void hw_pad_out(uint32_t pin);              // configure pad as push-pull output
typedef enum { HW_PULL_NONE, HW_PULL_UP, HW_PULL_DOWN } hw_pull_t;
void hw_pad_in(uint32_t pin, hw_pull_t pull);
void hw_set(uint32_t pin);
void hw_clear(uint32_t pin);
void hw_write(uint32_t pin, bool level);
bool hw_read(uint32_t pin);
