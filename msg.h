#ifndef MSG_H
#define MSG_H

#include <string.h>
#include <stdint.h>
#include "debug.h"


#define get_bit(n,k) (n & ( 1 << k )) >> k

#define MSG_MAX_DATA_LEN 32 //the maximum length of a data in a MSP protocol (padded)

struct S_MSG {
	uint8_t message_id;
	uint8_t size; 
	uint8_t data[MSG_MAX_DATA_LEN];
};

uint16_t msg_get_crc_error_count();

uint16_t msg_get_rx_count();

uint16_t msg_get_tx_count();

uint8_t msg_crc(const struct S_MSG *msg);

uint8_t msg_serialize(uint8_t *target, const struct S_MSG *msg);

uint8_t msg_parse(struct S_MSG *target, const uint8_t *buf, uint8_t buf_len);

#endif
