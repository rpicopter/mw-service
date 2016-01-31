//Based on: http://www.ibm.com/developerworks/aix/library/au-endianc/index.html?ca=drs-

#include "endian.h"

const int _i = 1;
#define is_bigendian() ( (*(char*)&_i) == 0 )

uint16_t *reverse16 (uint8_t *c) {
    static uint16_t s;
    uint8_t *p = (uint8_t *)&s;

    if (!is_bigendian()) {
        p[0] = c[0];
        p[1] = c[1];
    } else {
        p[0] = c[1];
        p[1] = c[0];
    }

    return &s;
}

uint32_t *reverse32 (uint8_t *c) {
    static uint32_t i;
    uint8_t *p = (uint8_t *)&i;

    if (!is_bigendian()) {
        p[0] = c[0];
        p[1] = c[1];
        p[2] = c[2];
        p[3] = c[3];
    } else {
        p[0] = c[3];
        p[1] = c[2];
        p[2] = c[1];
        p[3] = c[0];
    }

    return &i;
}

