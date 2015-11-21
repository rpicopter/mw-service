#include "debug.h"
#include "msp.h"
#include <string.h>
#include "endian.h"

#define MSP_MIN_MSG_LEN 5 //shortest message length, used to verify messages from UART 


uint8_t msp_crc(const struct S_MSP_MSG *msg) {
	uint8_t i;
	uint8_t crc = msg->size;
	crc^= msg->message_id;
	for (i=0;i<msg->size;i++) crc^= msg->data[i];
	return crc;
}

void msp_IDENT(struct S_MSP_MSG *target) {
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_IDENT\n");
	target->message_id = 100;
	target->size = 0;
}

void msp_parse_IDENT(struct S_MSP_IDENT *ident, struct S_MSP_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_IDENT\n");
	ident->version = msg->data[0];
	ident->multitype = msg->data[1];
	ident->msp_version = msg->data[2];
	ident->capability = reverse32(msg->data+3);

	dbg(DBG_MSP|DBG_VERBOSE,"version: %02x, multitype: %02x, msp_version: %02x, capability %08x\n",ident->version,ident->multitype,ident->msp_version,ident->capability);
}


void msp_STATUS(struct S_MSP_MSG *target) {
        dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_STATUS\n");
        target->message_id = 101;
        target->size = 0;
}

void msp_parse_STATUS(struct S_MSP_STATUS *status, struct S_MSP_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_STATUS\n");
	status->cycleTime = reverse16(msg->data); 
	status->i2c_errors_count = reverse16(msg->data+2); 
	status->sensor = reverse16(msg->data+4); 
	status->flag = reverse32(msg->data+6); 
	status->currentSet = msg->data[10];
	
	dbg(DBG_MSP|DBG_VERBOSE,"cycleTime: %04x, i2c_errors_count: %04x, sensor: %04x, flag: %08x, currentSet: %02x\n",status->cycleTime,status->i2c_errors_count,status->sensor,status->flag,status->currentSet);
}

void msp_RAW_IMU(struct S_MSP_MSG *target) {
        dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_RAW_IMU\n");
        target->message_id = 102;
        target->size = 0;
}

void msp_parse_RAW_IMU(struct S_MSP_RAW_IMU *imu, struct S_MSP_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing RAW_IMU\n");
	imu->accx = reverse16(msg->data);
	imu->accy = reverse16(msg->data+2);
	imu->accz = reverse16(msg->data+4);
	imu->gyrx = reverse16(msg->data+6);
	imu->gyry = reverse16(msg->data+8);
	imu->gyrz = reverse16(msg->data+10);
	imu->magx = reverse16(msg->data+12);
	imu->magy = reverse16(msg->data+14);
	imu->magz = reverse16(msg->data+16);

	dbg(DBG_MSP|DBG_VERBOSE,"AX: %04x, AY: %04x, AZ: %04x, GX: %04x, GY: %04x, GZ: %04x, MX: %04x, MY: %04x, MZ: %04x\n",imu->accx,imu->accy,imu->accz,imu->gyrx,imu->gyry,imu->gyrz,imu->magx,imu->magy,imu->magz);
}

void msp_SERVO(struct S_MSP_MSG *target) {
        dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_SERVO\n");
        target->message_id = 103;
        target->size = 0;
}

void msp_parse_SERVO(struct S_MSP_SERVO *servo, struct S_MSP_MSG *msg) {
	int i;
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_SERVO\n");
	for (i=0;i<8;i++)
		servo->servo[i] = reverse16(msg->data+(i*2));

	dbg(DBG_MSP|DBG_VERBOSE,"S0: %04x, S1: %04x, S2: %04x, S3: %04x, S3: %04x, S4: %04x, S5: %04x, S6: %04x, S7: %04x\n",servo->servo[0],servo->servo[1],servo->servo[2],servo->servo[3],servo->servo[4],servo->servo[5],servo->servo[6],servo->servo[7]);
}

void msp_RC(struct S_MSP_MSG *target) {
        dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_RC\n");
        target->message_id = 105;
        target->size = 0;
}

void msp_parse_RC(struct S_MSP_RC *rc, struct S_MSP_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_RC\n");
	rc->roll = reverse16(msg->data);
	rc->pitch = reverse16(msg->data+2);
	rc->yaw = reverse16(msg->data+4);
	rc->throttle = reverse16(msg->data+6);
	rc->aux1 = reverse16(msg->data+8);
	rc->aux2 = reverse16(msg->data+10);
	rc->aux3 = reverse16(msg->data+12);
	rc->aux4 = reverse16(msg->data+14);

	dbg(DBG_MSP|DBG_VERBOSE,"R: %04x, P: %04x, Y: %04x, T: %04x, A1: %04x, A2: %04x, A3: %04x, A4: %04x\n",rc->roll,rc->pitch,rc->yaw,rc->throttle, rc->aux1, rc->aux2, rc->aux3, rc->aux4);
}


uint8_t msp_toUART(uint8_t *target, uint8_t *target_len, const struct S_MSP_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"To UART message id: %u, data size: %u\n",msg->message_id,msg->size);
	target[0] = '$';
	target[1] = 'M';
	target[2] = '<';
	target[3] = msg->size;
	target[4] = msg->message_id;
	memcpy(target+5,msg->data,msg->size);
	target[5+msg->size] = msp_crc(msg); 
	(*target_len) = 6+msg->size;

	return *target_len;
}

#define UART_MSG_MAX 0xFF
static uint8_t buffer[UART_MSG_MAX];
static uint8_t w_ptr = 0, r_ptr = 0;

void msp_queueUART(const uint8_t *uart_msg, const int uart_msg_len) {
	dbg(DBG_MSP|DBG_VERBOSE,"Queuing UART data, len: %i\n",uart_msg_len);
	int i;
	for (i=0;i<uart_msg_len;i++) {
		buffer[w_ptr++] = uart_msg[i];
		if (w_ptr>=UART_MSG_MAX) w_ptr=0;
		if (w_ptr == r_ptr) { 
			dbg(DBG_MSP|DBG_ERROR,"buffer overflow!!\n");
			return;
		}
	}	
}

//returns 0 if no message found, -1 if error, 1 if ok 
int8_t msp_fromUART(struct S_MSP_MSG *target) {
	uint8_t crc;
	uint8_t state = 0;
	uint8_t ptr = r_ptr;

	uint8_t i = 0;
	
	while ((ptr!=w_ptr) && (state<=6)) {
		switch (state) {
			case 0: //preamble
				if (buffer[ptr]=='$') state++;
				break;
			case 1: //preamble
				if (buffer[ptr]=='M') state++;
				else state=0;
				break;
			case 2: //direction
				if (buffer[ptr]=='>') state++;
				else state=0;
				break;
			case 3: //size
				target->size = buffer[ptr];
				state++;
				break;
			case 4: //message id
				target->message_id = buffer[ptr];
				state++;
				break;
			case 5: //data
				if (i<target->size)
					target->data[i++] = buffer[ptr];
				if (i==target->size) state++;
				break;
			case 6: //crc 
				crc = msp_crc(target);
				if (crc == buffer[ptr]) state++;
				else {
					dbg(DBG_MSP|DBG_ERROR,"CRC Error. Expected: %02x, calculated: %02x\n",buffer[ptr],crc);
					state+=2;	
				}
				break;
		} 
		ptr++;
		if (ptr>UART_MSG_MAX) ptr=0;
	}

	if (state==7) { //message ok
		r_ptr = ptr;
		return 1;
	}

	if (state==8) { //something wrong
		r_ptr = ptr;
		return -1;
	}

	return 0; //we got only part message

} 

