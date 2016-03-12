#include "debug.h"
#include "msp.h"
#include <string.h>
#include "endian.h"



void msp_IDENT(struct S_MSG *target) {
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_IDENT\n");
	target->message_id = 100;
	target->size = 0;
}

void msp_parse_IDENT(struct S_MSP_IDENT *ident, struct S_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_IDENT\n");
	ident->version = msg->data[0];
	ident->multitype = msg->data[1];
	ident->msp_version = msg->data[2];
	ident->capability = *reverse32(msg->data+3);

	dbg(DBG_MSP|DBG_VERBOSE,"version: %02x, multitype: %02x, msp_version: %02x, capability %08x\n",ident->version,ident->multitype,ident->msp_version,ident->capability);
}


void msp_STATUS(struct S_MSG *target) {
        dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_STATUS\n");
        target->message_id = 101;
        target->size = 0;
}

void msp_parse_STATUS(struct S_MSP_STATUS *status, struct S_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_STATUS\n");
	status->cycleTime = *reverse16(msg->data); 
	status->i2c_errors_count = *reverse16(msg->data+2); 
	status->sensor = *reverse16(msg->data+4); 
	status->flag = *reverse32(msg->data+6); 
	status->currentSet = msg->data[10];
	
	dbg(DBG_MSP|DBG_VERBOSE,"cycleTime: %04x, i2c_errors_count: %04x, sensor: %04x, flag: %08x, currentSet: %02x\n",status->cycleTime,status->i2c_errors_count,status->sensor,status->flag,status->currentSet);
}

void msp_RAW_IMU(struct S_MSG *target) {
        dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_RAW_IMU\n");
        target->message_id = 102;
        target->size = 0;
}

void msp_parse_RAW_IMU(struct S_MSP_RAW_IMU *imu, struct S_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing RAW_IMU\n");
	imu->accx = *reverse16(msg->data);
	imu->accy = *reverse16(msg->data+2);
	imu->accz = *reverse16(msg->data+4);
	imu->gyrx = *reverse16(msg->data+6);
	imu->gyry = *reverse16(msg->data+8);
	imu->gyrz = *reverse16(msg->data+10);
	imu->magx = *reverse16(msg->data+12);
	imu->magy = *reverse16(msg->data+14);
	imu->magz = *reverse16(msg->data+16);

	dbg(DBG_MSP|DBG_VERBOSE,"AX: %04x, AY: %04x, AZ: %04x, GX: %04x, GY: %04x, GZ: %04x, MX: %04x, MY: %04x, MZ: %04x\n",imu->accx,imu->accy,imu->accz,imu->gyrx,imu->gyry,imu->gyrz,imu->magx,imu->magy,imu->magz);
}

void msp_SERVO(struct S_MSG *target) {
        dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_SERVO\n");
        target->message_id = 103;
        target->size = 0;
}

void msp_parse_SERVO(struct S_MSP_SERVO *servo, struct S_MSG *msg) {
	uint8_t i;
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_SERVO\n");
	for (i=0;i<8;i++)
		servo->servo[i] = *reverse16(msg->data+(i*2));

	dbg(DBG_MSP|DBG_VERBOSE,"S0: %04x, S1: %04x, S2: %04x, S3: %04x, S3: %04x, S4: %04x, S5: %04x, S6: %04x, S7: %04x\n",servo->servo[0],servo->servo[1],servo->servo[2],servo->servo[3],servo->servo[4],servo->servo[5],servo->servo[6],servo->servo[7]);
}

void msp_RC(struct S_MSG *target) {
        dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_RC\n");
        target->message_id = 105;
        target->size = 0;
}

void msp_SET_RAW_RC(struct S_MSG *target, struct S_MSP_RC *rc) {
	    target->message_id = 200;
        target->size = 16;
        memcpy(target->data,reverse16(&rc->roll),2);
        memcpy(target->data+2,reverse16(&rc->pitch),2);
        memcpy(target->data+4,reverse16(&rc->yaw),2);
        memcpy(target->data+6,reverse16(&rc->throttle),2);
        memcpy(target->data+8,reverse16(&rc->aux1),2);
        memcpy(target->data+10,reverse16(&rc->aux2),2);
        memcpy(target->data+12,reverse16(&rc->aux3),2);
        memcpy(target->data+14,reverse16(&rc->aux4),2);
}

void msp_parse_RC(struct S_MSP_RC *rc, struct S_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_RC\n");
	rc->roll = *reverse16(msg->data);
	rc->pitch = *reverse16(msg->data+2);
	rc->yaw = *reverse16(msg->data+4);
	rc->throttle = *reverse16(msg->data+6);
	rc->aux1 = *reverse16(msg->data+8);
	rc->aux2 = *reverse16(msg->data+10);
	rc->aux3 = *reverse16(msg->data+12);
	rc->aux4 = *reverse16(msg->data+14);

	dbg(DBG_MSP|DBG_VERBOSE,"R: %04x, P: %04x, Y: %04x, T: %04x, A1: %04x, A2: %04x, A3: %04x, A4: %04x\n",rc->roll,rc->pitch,rc->yaw,rc->throttle, rc->aux1, rc->aux2, rc->aux3, rc->aux4);
}

void msp_BOXIDS(struct S_MSG *target) {
    dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_BOXIDS\n");
    target->message_id = 119;
    target->size = 0;
}

void msp_parse_BOXIDS(struct S_MSP_BOXCONFIG *boxconf, struct S_MSG *msg) {
	uint8_t i,bid;
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_BOXIDS\n");
	memset(boxconf->supported,0,CHECKBOXITEMS);
	dbg(DBG_MSP|DBG_VERBOSE,"Supported boxes: ");
	for (i=0;i<msg->size;i++) {
		bid = *(msg->data+i); //boxid
		boxconf->supported[ bid ] = 1;
		dbg(DBG_MSP|DBG_VERBOSE,"%u ",bid);
	}
	dbg(DBG_MSP|DBG_VERBOSE,"\n");
}

void msp_BOX(struct S_MSG *target) {
    dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_BOX\n");
    target->message_id = 113;
    target->size = 0;
}

void msp_parse_BOX(struct S_MSP_BOXCONFIG *box, struct S_MSG *msg) {
	uint8_t i,j = 0;
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_BOX\n");
	memset(box->active,0,CHECKBOXITEMS);
	dbg(DBG_MSP|DBG_VERBOSE,"Box config: ");
	for (i=0;i<CHECKBOXITEMS;i++) {
		if (box->supported[i]) {
			box->active[i] = *reverse16(&msg->data[2*j]);
			dbg(DBG_MSP|DBG_VERBOSE,"%u=%u ",i,box->active[i]);
			j++;
		}
	}
	dbg(DBG_MSP|DBG_VERBOSE,"\n");
}

void msp_SET_BOX(struct S_MSG *target, struct S_MSP_BOXCONFIG *box) {
	uint8_t i,j = 0;
    dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_SET_BOX\n");
    target->message_id = 203;
    dbg(DBG_MSP|DBG_VERBOSE,"Setting box config: ");
	for (i=0;i<CHECKBOXITEMS;i++) {
		if (box->supported[i]) {
			memcpy(target->data+(j*2),reverse16(&box->active[i]),2);
			dbg(DBG_MSP|DBG_VERBOSE,"%u=%u ",i,box->active[i]);
			j++;
		}
	}
	target->size = 2*j;
}

/* USER DEFINED MESSAGES */
void msp_STICKCOMBO(struct S_MSG *target, struct S_MSP_STICKCOMBO *stickcombo) {
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_STICKCOMBO\n");
	target->message_id = 52;
	target->size = 1;
	target->data[0] = stickcombo->combo;
}

void msp_parse_STICKCOMBO(struct S_MSP_STICKCOMBO *stickcombo, struct S_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_STICKCOMBO\n");
	stickcombo->combo = msg->data[0];
}

void msp_custom(struct S_MSG *target, uint8_t id, uint8_t *data, uint8_t length) {
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing custom message: %u\n",id);
	target->message_id = id;
	target->size = length;
	memcpy(target->data,data,length);
}


