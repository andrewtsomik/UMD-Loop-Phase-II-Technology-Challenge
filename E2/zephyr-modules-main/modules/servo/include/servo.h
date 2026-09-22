#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

int servo_init(void);

int servo_set_angle(uint32_t angle);

#endif /* SERVO_H */
