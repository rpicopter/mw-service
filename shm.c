#include "shm.h"
#include "debug.h"
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <semaphore.h>

static int fd_out,fd_in;
static struct S_SHM_BLOCK *block_out,*block_in;
static uint16_t compare_in[SHM_SIZE], compare_out[SHM_SIZE];
static sem_t *sem_out, *sem_in;


void _shm_init(struct S_SHM_BLOCK *shm_block) {
	dbg(DBG_SHM|DBG_VERBOSE,"Initializing block: %p\n",shm_block);
}

void _shm_get(uint8_t *target, struct S_SHM_BLOCK *shm_block, uint8_t id) {
	dbg(DBG_SHM|DBG_VERBOSE,"Getting block id: %i of block: %p, size: %u\n",id,shm_block,shm_block->size[id]);
	memcpy(target,shm_block->msg[id],shm_block->size[id]);
}

void _shm_put(struct S_SHM_BLOCK *shm_block, uint8_t id, uint8_t *msg, uint8_t msg_size) {
	dbg(DBG_SHM|DBG_VERBOSE,"Setting %u bytes of block id: %i of block: %p\n",msg_size,id,shm_block);
	memcpy(shm_block->msg[id],msg,msg_size);
	shm_block->size[id] = msg_size;
	shm_block->counter[id]++;
	if (shm_block->counter[id] == UINT16_MAX) shm_block->counter[id] = 0;
}

uint8_t shm_server_init() {
	int i;

	fd_out = shm_open("/mw-outgoing", O_CREAT | O_RDWR, 0666);
	if (fd_out == -1) {
		dbg(DBG_SHM|DBG_ERROR,"outgoing shm_open %s\n",strerror(errno));
		return 1;
	}

	if (ftruncate(fd_out, sizeof(struct S_SHM_BLOCK)) == -1) {
		dbg(DBG_SHM|DBG_ERROR,"outgoing ftruncate %s\n",strerror(errno));
		return 2;
	}	

	block_out = mmap(NULL, sizeof(struct S_SHM_BLOCK), PROT_READ|PROT_WRITE, MAP_SHARED,fd_out,0);
	if (block_out == MAP_FAILED) {
		dbg(DBG_SHM|DBG_ERROR,"outgoing mmap %s\n",strerror(errno));
		return 3;
	}


	fd_in = shm_open("/mw-incoming", O_CREAT | O_RDWR, 0666);
	if (fd_in == -1) {
		dbg(DBG_SHM|DBG_ERROR,"incoming shm_open %s\n",strerror(errno));
		return 4;
	}

	if (ftruncate(fd_in, sizeof(struct S_SHM_BLOCK)) == -1) {
		dbg(DBG_SHM|DBG_ERROR,"outgoing ftruncate %s\n",strerror(errno));
		return 5;
	}	

	block_in = mmap(NULL, sizeof(struct S_SHM_BLOCK), PROT_READ|PROT_WRITE, MAP_SHARED,fd_in,0);
	if (block_in == MAP_FAILED) {
		dbg(DBG_SHM|DBG_ERROR,"incoming mmap %s\n",strerror(errno));
		return 6;
	}

	_shm_init(block_out);
	_shm_init(block_in);

	for (i=0;i<SHM_SIZE;i++) {
		compare_in[i] = block_in->counter[i];
		compare_out[i] = block_out->counter[i];
	}

	sem_out=sem_open("/mw-outgoing-sem", O_CREAT, 0666, 0);
	if(sem_out == SEM_FAILED) {
		dbg(DBG_SHM|DBG_ERROR,"sem_out %s\n",strerror(errno));
		return 7;
	}

	sem_in=sem_open("/mw-incoming-sem", O_CREAT, 0666, 0);
	if(sem_in == SEM_FAILED) {
		dbg(DBG_SHM|DBG_ERROR,"sem_in %s\n",strerror(errno));
		return 8;
	}

	dbg(DBG_SHM|DBG_VERBOSE,"Server initialized.\n");

	return 0;
}

void shm_server_end() {
	munmap(block_in,sizeof(struct S_SHM_BLOCK));
	munmap(block_out,sizeof(struct S_SHM_BLOCK));
}

void shm_client_end() {
	munmap(block_in,sizeof(struct S_SHM_BLOCK));
	munmap(block_out,sizeof(struct S_SHM_BLOCK));
}

uint8_t shm_client_init() {
	int i;
	fd_out = shm_open("/mw-outgoing", O_RDWR, 0666);
	if (fd_out == -1) {
		dbg(DBG_SHM|DBG_ERROR,"outgoing shm_open %s\n",strerror(errno));
		return 1;
	}

	block_out = mmap(NULL, sizeof(struct S_SHM_BLOCK), PROT_READ|PROT_WRITE, MAP_SHARED,fd_out,0);
	if (block_out == MAP_FAILED) {
		dbg(DBG_SHM|DBG_ERROR,"outgoing mmap %s\n",strerror(errno));
		return 2;
	}


	fd_in = shm_open("/mw-incoming",O_RDWR, 0666);
	if (fd_in == -1) {
		dbg(DBG_SHM|DBG_ERROR,"incoming shm_open %s\n",strerror(errno));
		return 3;
	}

	block_in = mmap(NULL, sizeof(struct S_SHM_BLOCK), PROT_READ|PROT_WRITE, MAP_SHARED,fd_in,0);
	if (block_in == MAP_FAILED) {
		dbg(DBG_SHM|DBG_ERROR,"incoming mmap %s\n",strerror(errno));
		return 4;
	}

	for (i=0;i<SHM_SIZE;i++) {
		compare_in[i] = block_in->counter[i];
		compare_out[i] = block_out->counter[i];
	}

        sem_out=sem_open("/mw-outgoing-sem", 0);
        if(sem_out == SEM_FAILED) {
                dbg(DBG_SHM|DBG_ERROR,"sem_out %s\n",strerror(errno));
                return 7;
        }

        sem_in=sem_open("/mw-incoming-sem", 0);
        if(sem_in == SEM_FAILED) {
                dbg(DBG_SHM|DBG_ERROR,"sem_in %s\n",strerror(errno));
                return 8;
        }

	dbg(DBG_SHM|DBG_VERBOSE,"Client initialized.\n");

	return 0;
}

void shm_put_incoming(struct S_MSG *msg) {
	_shm_put(block_in, msg->message_id, (uint8_t*)msg, sizeof(struct S_MSG));
}

void shm_put_outgoing(struct S_MSG *msg) {
	_shm_put(block_out, msg->message_id, (uint8_t*)msg, sizeof(struct S_MSG));
}

void shm_get_incoming(struct S_MSG *target, uint8_t id) {
	_shm_get((uint8_t*)target,block_in,id);
}

void shm_get_outgoing(struct S_MSG *target, uint8_t id) {
	_shm_get((uint8_t*)target,block_out,id);
}

uint8_t shm_scan_incoming_f(struct S_MSG *target, uint8_t *filter, uint8_t filter_len) {
	static uint8_t i;
	static uint8_t _v;
	for (i=0;i<filter_len;i++) {
		_v = filter[i];
		if (block_in->counter[_v]!=compare_in[_v]) {
			dbg(DBG_SHM|DBG_VERBOSE,"Found incoming record: %u\n",_v);
			_shm_get((uint8_t*)target,block_in,_v);
			compare_in[_v] = block_in->counter[_v];
			return 1;
		}
	}
	return 0;
}

uint8_t shm_scan_incoming(struct S_MSG *target) {
	static uint8_t i;
	for (i=0;i<SHM_SIZE;i++)
		if (block_in->counter[i]!=compare_in[i]) {
			dbg(DBG_SHM|DBG_VERBOSE,"Found incoming record: %u\n",i);
			_shm_get((uint8_t*)target,block_in,i);
			compare_in[i] = block_in->counter[i];
			return 1;
		}
	return 0;
}

uint8_t shm_scan_outgoing_f(struct S_MSG *target, uint8_t *filter, uint8_t filter_len) {
	static uint8_t i;
	static uint8_t _v;
	for (i=0;i<filter_len;i++) {
		_v = filter[i];
		if (block_out->counter[_v]!=compare_out[_v]) {
			dbg(DBG_SHM|DBG_VERBOSE,"Found outgoing record: %u\n",_v);
			_shm_get((uint8_t*)target,block_out,_v);
			compare_out[_v] = block_out->counter[_v];
			return 1;
		}
	}
	return 0;
}

uint8_t shm_scan_outgoing(struct S_MSG *target) {
	static uint8_t i;
	for (i=0;i<SHM_SIZE;i++)
		if (block_out->counter[i]!=compare_out[i]) {
			dbg(DBG_SHM|DBG_VERBOSE,"Found outgoing record: %u\n",i);
			_shm_get((uint8_t*)target,block_out,i);
			compare_out[i] = block_out->counter[i];
			return 1;
		}
	return 0;
}

