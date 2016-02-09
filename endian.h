#ifndef ENDIAN_H
#define ENDIAN_H

#include <stdint.h>

uint16_t *fromBigEndian16 (uint8_t *c);

uint32_t *fromBigEndian32 (uint8_t *c);

uint16_t *fromLittleEndian16 (uint8_t *c);

uint32_t *fromLittleEndian32 (uint8_t *c);

uint16_t *reverse16 (uint8_t *c);

uint32_t *reverse32 (uint8_t *c);


#endif
