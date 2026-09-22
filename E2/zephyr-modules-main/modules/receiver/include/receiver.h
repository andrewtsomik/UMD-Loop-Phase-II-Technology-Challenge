#ifndef RECEIVER_H
#define RECEIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**

* @brief Initialize the receiver.
*
* @return 0 on success.
* @return Negative errno value on failure.
  */
  int receiver_init(void);

/**

* @brief Attempt to receive one signal value.
*
* This function is non-blocking.
*
* @param value Pointer where the received value will be stored.
*
* @return 0 if a value was received.
* @return -EAGAIN if no data is currently available.
* @return Negative errno value on failure.
  */
  int receiver_read(uint8_t *value);

#ifdef __cplusplus
}
#endif

#endif /* RECEIVER_H */
