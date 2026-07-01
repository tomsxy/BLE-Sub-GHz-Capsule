#include "utils.h"

#include <zephyr/kernel.h>

void peek_buffer(uint8_t *buf, uint32_t size) {
    uint32_t i = 0;

    for (; i < 10; i++)
    {
        printk("%02X ", buf[i]);
    }
    printk("\n");
    for (i = size - 21; i < size - 1; i++)
    {
        printk("%02X ", buf[i]);
    }
    printf("\n");
}
