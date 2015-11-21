#ifndef SHMMSP_H
#define SHMMSP_H

#include "shm.h"
#include "msp.h"

uint8_t shmmsp_server_init();

void shmmsp_server_end();

uint8_t shmmsp_client_init();

void shmmsp_client_end();

void shmmsp_put_incoming(struct S_MSP_MSG *msg);

void shmmsp_put_outgoing(struct S_MSP_MSG *msg);

void shmmsp_get_incoming(struct S_MSP_MSG *target, uint8_t id);

void shmmsp_get_outgoing(struct S_MSP_MSG *target, uint8_t id);

uint8_t shmmsp_scan_incoming(struct S_MSP_MSG *target);

uint8_t shmmsp_scan_outgoing(struct S_MSP_MSG *target);

#endif

