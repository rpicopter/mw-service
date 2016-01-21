#ifndef SHM_H
#define SHM_H

#include <stdint.h>
#include "msg.h"

#define SHM_SIZE 0xFF
#define SHM_MSG_SIZE 36 //should be the max size of a MW message (should be 34 but need to confirm if allignment is an issue)

typedef uint8_t SHM_MSG[SHM_MSG_SIZE];

struct S_SHM_BLOCK {
	SHM_MSG msg[SHM_SIZE];
	uint8_t size[SHM_SIZE];
	uint16_t counter[SHM_SIZE]; //gets incremented when there is put operation
};


void _shm_init(struct S_SHM_BLOCK *shm_block);

void _shm_get(uint8_t *target, struct S_SHM_BLOCK *shm_block, uint8_t id);

void _shm_put(struct S_SHM_BLOCK *shm_block, uint8_t id, uint8_t *msg, uint8_t msg_size);

uint8_t shm_server_init();

void shm_server_end();

uint8_t shm_client_init();

void shm_client_end();

void shm_put_incoming(struct S_MSG *msg);

void shm_put_outgoing(struct S_MSG *msg);

void shm_get_incoming(struct S_MSG *target, uint8_t id);

void shm_get_outgoing(struct S_MSG *target, uint8_t id);

uint8_t shm_scan_incoming_f(struct S_MSG *target, uint8_t *filter, uint8_t filter_len);

uint8_t shm_scan_incoming(struct S_MSG *target);

uint8_t shm_scan_outgoing_f(struct S_MSG *target, uint8_t *filter, uint8_t filter_len);

uint8_t shm_scan_outgoing(struct S_MSG *target);

#endif

