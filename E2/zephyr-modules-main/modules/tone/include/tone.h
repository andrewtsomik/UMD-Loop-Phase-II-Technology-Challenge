#ifndef TONE_H
#define TONE_H

#include <stdint.h>

/**

* Initialize the tone generator.
*
* @return 0 on success, negative error code on failure.
  */
  int tone_init(void);

/**

* Play a tone.
*
* @param duration_ms Duration of the tone in milliseconds.
* @param level       Output level/duty cycle from 0-100.
*
* @return 0 on success, negative error code on failure.
  */
  int tone_play(uint32_t duration_ms, uint32_t level);

/**

* Stop the tone immediately.
  */
  void tone_stop(void);

#endif
