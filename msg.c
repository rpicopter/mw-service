#include "msg.h"

static uint16_t crc_error = 0;
static uint16_t msg_tx_count = 0;
static uint16_t msg_rx_count = 0;

uint16_t msg_get_crc_error_count() {
	return crc_error;
}

uint16_t msg_get_rx_count() {
	return msg_rx_count;
}

uint16_t msg_get_tx_count() {
	return msg_tx_count;
}

/*
        Helper function to calculate CRC as per MSP protocol
*/
uint8_t msg_crc(const struct S_MSG *msg) {
    uint8_t i;
    uint8_t crc = msg->size;
    crc^= msg->message_id;
    for (i=0;i<msg->size;i++) crc^= msg->data[i];
    return crc;
}

//returns number of bytes written into target
uint8_t msg_serialize(uint8_t *target, const struct S_MSG *msg) {
	msg_tx_count++;

	dbg(DBG_MSG|DBG_VERBOSE,"Serializing, id: %u, data size: %u\n",msg->message_id,msg->size);
	target[0] = '$';
	target[1] = 'M';
	target[2] = '<';
	target[3] = msg->size;
	target[4] = msg->message_id;
	memcpy(target+5,msg->data,msg->size);
	target[5+msg->size] = msg_crc(msg); 	

	return 6+msg->size;
}

//returns 0 if no message found, otherwise returns number of bytes consumed (in case of error returned message_id will be nulled)
uint8_t msg_parse(struct S_MSG *target, const uint8_t *buf, uint8_t buf_len) {
	dbg(DBG_MSG|DBG_VERBOSE,"Parsing, data size: %u\n",buf_len);	

	uint8_t crc;
	uint8_t state = 0;

	uint8_t i = 0;
	uint8_t j = 0;


	for (i=0;i<buf_len && state<=6;i++) {
		switch (state) {
			case 0: //preamble
				if (buf[i]=='$') state++;
				break;
			case 1: //preamble
				if (buf[i]=='M') state++;
				else state=0;
				break;
			case 2: //direction
				if (buf[i]=='>') state++;
				else state=0;
				break;
			case 3: //size
				target->size = buf[i];
				state++;
				break;
			case 4: //message id
				target->message_id = buf[i];
				state++;
				break;
			case 5: //data
				if (j<target->size) {
					target->data[j++] = buf[i];
				}
				if (j==target->size) state++;
				break;
			case 6: //crc 
				msg_rx_count++;
				crc = msg_crc(target);
				if (crc == buf[i]) state++;
				else state+=2;					
				break;
		} 
	}

	if (state<=6) { //end of buffer without message (i.e. message not complete)
		dbg(DBG_MSG|DBG_VERBOSE,"Message not found.\n");	
		target->message_id = 0;
		return 0;
	}

	if (state==7) {//message found		
		//target is set
		dbg(DBG_MSG|DBG_VERBOSE,"Found message id: %u found. Consumed bytes: %u\n",target->message_id,i);	
		return i;
	}

	if (state==8) { //something wrong
		dbg(DBG_MSG|DBG_WARNING,"CRC Error. Expected: %02x, calculated: %02x. Consumed bytes: %u\n",buf[i],crc,i);
		crc_error++;
		target->message_id = 0;
		return i;
	}

	return 0;
} 
