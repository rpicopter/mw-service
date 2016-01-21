#ifndef DEBUG_H
#define DEBUG_H

#include <stdint.h>

#define DBG_MODULES  0b11111000  
#define DBG_MW       0b10000000
#define DBG_MSP      0b01000000
#define DBG_SHM      0b00100000
#define DBG_UART     0b00010000

#define DBG_LEVELS   0b00000111  
#define DBG_ERROR    0b00000001
#define DBG_WARNING  0b00000010
#define DBG_VERBOSE  0b00000100



void dbg_init(uint8_t caps);
void dbg(uint8_t mask, const char *format, ...);
char *dbg_printHEX(const uint8_t *s, int len);

#endif

