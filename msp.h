#ifndef MSP_H
#define MSP_H

#include <stdint.h>
#include "msg.h"


struct S_MSP_IDENT {
	uint8_t version;
	uint8_t multitype;
	uint8_t msp_version;
	uint32_t capability;	
}; 


struct S_MSP_STATUS {
	uint16_t cycleTime;
	uint16_t i2c_errors_count;
	uint16_t sensor;
	uint32_t flag;
	uint8_t currentSet;
};

struct S_MSP_RAW_IMU {
	int16_t accx;
	int16_t accy;
	int16_t accz;
	int16_t gyrx;
	int16_t gyry;
	int16_t gyrz;
	int16_t magx;
	int16_t magy;
	int16_t magz;
};

struct S_MSP_SERVO {
	uint16_t servo[8]; 
};

struct S_MSP_RC {
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
	uint16_t throttle;
	uint16_t aux1;
	uint16_t aux2;
	uint16_t aux3;
	uint16_t aux4;	
};


void msp_IDENT(struct S_MSG *target);
void msp_parse_IDENT(struct S_MSP_IDENT *target, struct S_MSG *msg);

void msp_STATUS(struct S_MSG *target);
void msp_parse_STATUS(struct S_MSP_STATUS *status, struct S_MSG *msg);

void msp_RAW_IMU(struct S_MSG *target);
void msp_parse_RAW_IMU(struct S_MSP_RAW_IMU *imu, struct S_MSG *msg);

void msp_SERVO(struct S_MSG *target);
void msp_parse_SERVO(struct S_MSP_SERVO *servo, struct S_MSG *msg);

void msp_RC(struct S_MSG *target);
void msp_SET_RAW_RC(struct S_MSG *target, struct S_MSP_RC *rc);
void msp_parse_RC(struct S_MSP_RC *status, struct S_MSG *msg);

#endif

