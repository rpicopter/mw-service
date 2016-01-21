#include "msg.h"

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
	dbg(DBG_MW|DBG_VERBOSE,"Serializing, id: %u, data size: %u\n",msg->message_id,msg->size);
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
	dbg(DBG_MW|DBG_VERBOSE,"Parsing, data size: %u\n",buf_len);	
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
				crc = msg_crc(target);
				if (crc == buf[i]) state++;
				else {
					dbg(DBG_MW|DBG_ERROR,"CRC Error. Expected: %02x, calculated: %02x\n",buf[i],crc);
					state+=2;	
				}
				break;
		} 
	}

	if (state<=6) { //end of buffer without message (i.e. message not complete)
		target->message_id = 0;
		return 0;
	}

	if (state==7) {//message found		
		//target is set
		return i;
	}

	if (state==8) { //something wrong
		target->message_id = 0;
		return i;
	}

	return 0;
} 
