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
	ident->capability = reverse32(msg->data+3);

	dbg(DBG_MSP|DBG_VERBOSE,"version: %02x, multitype: %02x, msp_version: %02x, capability %08x\n",ident->version,ident->multitype,ident->msp_version,ident->capability);
}


void msp_STATUS(struct S_MSG *target) {
        dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_STATUS\n");
        target->message_id = 101;
        target->size = 0;
}

void msp_parse_STATUS(struct S_MSP_STATUS *status, struct S_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_STATUS\n");
	status->cycleTime = reverse16(msg->data); 
	status->i2c_errors_count = reverse16(msg->data+2); 
	status->sensor = reverse16(msg->data+4); 
	status->flag = reverse32(msg->data+6); 
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

void msp_SERVO(struct S_MSG *target) {
        dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_SERVO\n");
        target->message_id = 103;
        target->size = 0;
}

void msp_parse_SERVO(struct S_MSP_SERVO *servo, struct S_MSG *msg) {
	int i;
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_SERVO\n");
	for (i=0;i<8;i++)
		servo->servo[i] = reverse16(msg->data+(i*2));

	dbg(DBG_MSP|DBG_VERBOSE,"S0: %04x, S1: %04x, S2: %04x, S3: %04x, S3: %04x, S4: %04x, S5: %04x, S6: %04x, S7: %04x\n",servo->servo[0],servo->servo[1],servo->servo[2],servo->servo[3],servo->servo[4],servo->servo[5],servo->servo[6],servo->servo[7]);
}

void msp_RC(struct S_MSG *target) {
        dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_RC\n");
        target->message_id = 105;
        target->size = 0;
}

void msp_parse_RC(struct S_MSP_RC *rc, struct S_MSG *msg) {
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


