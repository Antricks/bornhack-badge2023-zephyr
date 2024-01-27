#pragma once
#include <zephyr/kernel.h>

void hexdump(const uint8_t *buf, uint32_t len); 
void hexdump(const char *prefix, const uint8_t *buf, uint32_t len);
