#include "shm.h"
#include "debug.h"
#include <string.h>

void shm_init(struct S_SHM_BLOCK *shm_block) {
	dbg(DBG_SHM|DBG_VERBOSE,"Initializing block: %p\n",shm_block);
}

void shm_get(uint8_t *target, struct S_SHM_BLOCK *shm_block, uint8_t id) {
	dbg(DBG_SHM|DBG_VERBOSE,"Getting block id: %i of block: %p, size: %u\n",id,shm_block,shm_block->size[id]);
	memcpy(target,shm_block->msg[id],shm_block->size[id]);
}

void shm_put(struct S_SHM_BLOCK *shm_block, uint8_t id, uint8_t *msg, uint8_t msg_size) {
	dbg(DBG_SHM|DBG_VERBOSE,"Setting %u bytes of block id: %i of block: %p\n",msg_size,id,shm_block);
	memcpy(shm_block->msg[id],msg,msg_size);
	shm_block->size[id] = msg_size;
	shm_block->counter[id]++;
	if (shm_block->counter[id] == UINT16_MAX) shm_block->counter[id] = 0;
}


