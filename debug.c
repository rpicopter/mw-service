#include "debug.h"
#include <stdio.h>
#include <stdarg.h>

static uint8_t dbg_mask;

static void dbg_print_prefix(uint8_t mask) {
	if (mask & DBG_ERROR) printf("(E) ");
	else if (mask & DBG_WARNING) printf("(W) ");
	else if (mask & DBG_VERBOSE) printf("(I) ");

	if (mask & DBG_MSP) printf("MSP: ");
	else if (mask & DBG_SHM) printf("SHM: ");
	else if (mask & DBG_MW) printf("MW: ");
	else if (mask & DBG_UART) printf("UART: ");
	else if (mask & DBG_MSG) printf("MSG: ");
	else printf(": ");
}

char *dbg_printHEX(const uint8_t *s, int len) {
	static char buf[0xFF];
        int i;

	if (len>(0xFF/3)) {  //3 bytes for every character in s
		sprintf(buf,"[STRING TOO LONG]\0");
		return buf;
	}

        for (i = 0; i < len; ++i) sprintf(buf+(3*i),"%02x ", s[i]);
	buf[3*i] = '\0';

	return buf;
}

void dbg_init(uint8_t caps) {
	dbg_mask = caps;
	if (dbg_mask & DBG_VERBOSE) dbg_mask |= DBG_WARNING;
	if (dbg_mask & DBG_WARNING) dbg_mask |= DBG_ERROR;
}

void dbg(uint8_t mask, const char *format, ...)
{
    va_list args;
    va_start(args, format);

    if((mask & dbg_mask & DBG_MODULES) && (mask & dbg_mask & DBG_LEVELS)) {
	dbg_print_prefix(mask);
	vprintf(format, args);
    }

    va_end(args);
}

