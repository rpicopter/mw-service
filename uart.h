#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "msg.h"

int uart_init(const char *path);
void uart_close();

int uart_read(uint8_t *buf, int size);
int uart_write(uint8_t *buf, int count);

#endif

