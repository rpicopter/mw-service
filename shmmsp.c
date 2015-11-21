#include "shmmsp.h"
#include "debug.h"

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

uint8_t shmmsp_server_init() {
	int i;

	fd_out = shm_open("/mw-outgoing", O_CREAT | O_RDWR, 0666);
	if (fd_out == -1) {
		dbg(DBG_SHMMSP|DBG_ERROR,"outgoing shm_open %s\n",strerror(errno));
		return 1;
	}

	if (ftruncate(fd_out, sizeof(struct S_SHM_BLOCK)) == -1) {
		dbg(DBG_SHMMSP|DBG_ERROR,"outgoing ftruncate %s\n",strerror(errno));
		return 2;
	}	

	block_out = mmap(NULL, sizeof(struct S_SHM_BLOCK), PROT_READ|PROT_WRITE, MAP_SHARED,fd_out,0);
	if (block_out == MAP_FAILED) {
		dbg(DBG_SHMMSP|DBG_ERROR,"outgoing mmap %s\n",strerror(errno));
		return 3;
	}


	fd_in = shm_open("/mw-incoming", O_CREAT | O_RDWR, 0666);
	if (fd_in == -1) {
		dbg(DBG_SHMMSP|DBG_ERROR,"incoming shm_open %s\n",strerror(errno));
		return 4;
	}

	if (ftruncate(fd_in, sizeof(struct S_SHM_BLOCK)) == -1) {
		dbg(DBG_SHMMSP|DBG_ERROR,"outgoing ftruncate %s\n",strerror(errno));
		return 5;
	}	

	block_in = mmap(NULL, sizeof(struct S_SHM_BLOCK), PROT_READ|PROT_WRITE, MAP_SHARED,fd_in,0);
	if (block_in == MAP_FAILED) {
		dbg(DBG_SHMMSP|DBG_ERROR,"incoming mmap %s\n",strerror(errno));
		return 6;
	}

	shm_init(block_out);
	shm_init(block_in);

	for (i=0;i<SHM_SIZE;i++) {
		compare_in[i] = block_in->counter[i];
		compare_out[i] = block_out->counter[i];
	}

	sem_out=sem_open("/mw-outgoing-sem", O_CREAT, 0666, 0);
	if(sem_out == SEM_FAILED) {
		dbg(DBG_SHMMSP|DBG_ERROR,"sem_out %s\n",strerror(errno));
		return 7;
	}

	sem_in=sem_open("/mw-incoming-sem", O_CREAT, 0666, 0);
	if(sem_in == SEM_FAILED) {
		dbg(DBG_SHMMSP|DBG_ERROR,"sem_in %s\n",strerror(errno));
		return 8;
	}

	dbg(DBG_SHMMSP|DBG_VERBOSE,"Server initialized.\n");

	return 0;
}

void shmmsp_server_end() {
	munmap(block_in,sizeof(struct S_SHM_BLOCK));
	munmap(block_out,sizeof(struct S_SHM_BLOCK));
}

void shmmsp_client_end() {
	munmap(block_in,sizeof(struct S_SHM_BLOCK));
	munmap(block_out,sizeof(struct S_SHM_BLOCK));
}

uint8_t shmmsp_client_init() {
	int i;
	fd_out = shm_open("/mw-outgoing", O_RDWR, 0666);
	if (fd_out == -1) {
		dbg(DBG_SHMMSP|DBG_ERROR,"outgoing shm_open %s\n",strerror(errno));
		return 1;
	}

	block_out = mmap(NULL, sizeof(struct S_SHM_BLOCK), PROT_READ|PROT_WRITE, MAP_SHARED,fd_out,0);
	if (block_out == MAP_FAILED) {
		dbg(DBG_SHMMSP|DBG_ERROR,"outgoing mmap %s\n",strerror(errno));
		return 2;
	}


	fd_in = shm_open("/mw-incoming", O_RDONLY, 0666);
	if (fd_in == -1) {
		dbg(DBG_SHMMSP|DBG_ERROR,"incoming shm_open %s\n",strerror(errno));
		return 3;
	}

	block_in = mmap(NULL, sizeof(struct S_SHM_BLOCK), PROT_READ, MAP_SHARED,fd_in,0);
	if (block_in == MAP_FAILED) {
		dbg(DBG_SHMMSP|DBG_ERROR,"incoming mmap %s\n",strerror(errno));
		return 4;
	}

	for (i=0;i<SHM_SIZE;i++) {
		compare_in[i] = block_in->counter[i];
		compare_out[i] = block_out->counter[i];
	}

        sem_out=sem_open("/mw-outgoing-sem", 0);
        if(sem_out == SEM_FAILED) {
                dbg(DBG_SHMMSP|DBG_ERROR,"sem_out %s\n",strerror(errno));
                return 7;
        }

        sem_in=sem_open("/mw-incoming-sem", 0);
        if(sem_in == SEM_FAILED) {
                dbg(DBG_SHMMSP|DBG_ERROR,"sem_in %s\n",strerror(errno));
                return 8;
        }

	dbg(DBG_SHMMSP|DBG_VERBOSE,"Client initialized.\n");

	return 0;
}

void shmmsp_put_incoming(struct S_MSP_MSG *msg) {
	shm_put(block_in, msg->message_id, (uint8_t*)msg, sizeof(struct S_MSP_MSG));
}

void shmmsp_put_outgoing(struct S_MSP_MSG *msg) {
	shm_put(block_out, msg->message_id, (uint8_t*)msg, sizeof(struct S_MSP_MSG));
}

void shmmsp_get_incoming(struct S_MSP_MSG *target, uint8_t id) {
	shm_get((uint8_t*)target,block_in,id);
}

void shmmsp_get_outgoing(struct S_MSP_MSG *target, uint8_t id) {
	shm_get((uint8_t*)target,block_out,id);
}

uint8_t shmmsp_scan_incoming(struct S_MSP_MSG *target) {
	static uint8_t i;
	for (i=0;i<SHM_SIZE;i++)
		if (block_in->counter[i]!=compare_in[i]) {
			dbg(DBG_SHMMSP|DBG_VERBOSE,"Found incoming record id: %u\n",target->message_id);
			shm_get((uint8_t*)target,block_in,i);
			compare_in[i] = block_in->counter[i];
			return 1;
		}
	return 0;
}

uint8_t shmmsp_scan_outgoing(struct S_MSP_MSG *target) {
	static uint8_t i;
	for (i=0;i<SHM_SIZE;i++)
		if (block_out->counter[i]!=compare_out[i]) {
			dbg(DBG_SHMMSP|DBG_VERBOSE,"Found outgoing record id: %u\n",target->message_id);
			shm_get((uint8_t*)target,block_out,i);
			compare_out[i] = block_out->counter[i];
			return 1;
		}
	return 0;
}

