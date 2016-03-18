/*
 * shm.h - SHM implementation
 * SHM is a globally defined memory block that is accessible by the server and the clients
 * The block contains all the possible MSP messages
 * When server runs it will read the outgoing block and forward the messages to MW
 * It will also listen for messages coming from MW board and write them back to the incoming block

 * Gregory Dymarek - March 2016
*/

#ifndef SHM_H
#define SHM_H

#include <stdint.h>
#include "msg.h"

#define SHM_SIZE 0xFF //number of possible MSP messages
#define SHM_MSG_SIZE 36 //should be the max size of a MW message (should be 34 but need to confirm if allignment is an issue)
//MSP_BOXNAMES is a special case as we gonna parse it immediately before storing so no need to store the entire message

typedef uint8_t SHM_MSG[SHM_MSG_SIZE];

struct S_SHM_BLOCK {
	SHM_MSG msg[SHM_SIZE];
	uint8_t size[SHM_SIZE];
	uint16_t counter[SHM_SIZE]; //gets incremented when there is put operation
};


/* Sets up SHM as server (creates is if needed)
 * Returns 0 on success
 */
uint8_t shm_server_init();

/* Closes the SHM server
 * It will not remove the block from the memory so clients can continue to access it
 */
void shm_server_end();

/* Sets up SHM as client
 * This requires the server to create the block in the first instance.
 * Returns 0 on success
 */
uint8_t shm_client_init();

/* Closes the SHM client
 */
void shm_client_end();

/* Puts a single message into the incoming block
 */
void shm_put_incoming(struct S_MSG *msg);

/* Puts a single message into the outgoing block
 */
void shm_put_outgoing(struct S_MSG *msg);

/* Gets a single message with given id from incoming block
 */
void shm_get_incoming(struct S_MSG *target, uint8_t id);

/* Gets a single message with given id from outgoing block
 */
void shm_get_outgoing(struct S_MSG *target, uint8_t id);

/* Looks for new message in the outgoing SHM using the provided filter 
 * It will only scan for messages that are in the filter
 * Returns:
 * 0 - no new message has been found
 * 1 - message found (target will be populated with the message details)
 */
uint8_t shm_scan_incoming_f(struct S_MSG *target, uint8_t *filter, uint8_t filter_len);

/* Looks for new message in the incoming SHM
 * Returns:
 * 0 - no new message has been found
 * 1 - message found (target will be populated with the message details)
 */
uint8_t shm_scan_incoming(struct S_MSG *target);

/* Looks for new message in the incoming SHM using the provided filter 
 * It will only scan for messages that are in the filter
 * Returns:
 * 0 - no new message has been found
 * 1 - message found (target will be populated with the message details)
 */
uint8_t shm_scan_outgoing_f(struct S_MSG *target, uint8_t *filter, uint8_t filter_len);

/* Looks for new message in the outgoing SHM
 * Returns:
 * 0 - no new message has been found
 * 1 - message found (target will be populated with the message details)
 */
uint8_t shm_scan_outgoing(struct S_MSG *target);

/* Helper function
 * Gets a single messages with given ID from the provided block
 */
void _shm_get(uint8_t *target, struct S_SHM_BLOCK *shm_block, uint8_t id);

/* Helper function
 * Sets a single messages with given ID into the provided block
 */
void _shm_put(struct S_SHM_BLOCK *shm_block, uint8_t id, uint8_t *msg, uint8_t msg_size);

/* Helper function
 * Initializes a SHM block
 */   
void _shm_init(struct S_SHM_BLOCK *shm_block);



#endif

