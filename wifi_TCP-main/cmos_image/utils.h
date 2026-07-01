#ifndef __UTILS_H__
#define __UTILS_H__

#include <stdint.h>

#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

void peek_buffer(uint8_t *buf, uint32_t size);

#endif
