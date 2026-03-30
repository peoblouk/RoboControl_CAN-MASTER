#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stddef.h>
#include <stdint.h>

void status_led_start(const uint8_t *node_ids, size_t node_count);

#endif // STATUS_LED_H
