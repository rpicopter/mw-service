#ifndef MSP_H
#define MSP_H

#include <stdint.h>
#include "msg.h"


#define RC_LO 1100-1 //used for stick combos; based on MultiWii.h
#define RC_HI 1900+1
#define RC_CE 1500

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

enum box {
  BOXARM, //0
  BOXANGLE, //1
  BOXHORIZON, //2
  BOXBARO, //3
  BOXVARIO, //4
  BOXMAG, //5
  BOXHEADFREE, //6
  BOXHEADADJ, // 7 acquire heading for HEADFREE mode
  BOXCAMSTAB,// 8
  BOXCAMTRIG, //9
  BOXGPSHOME, //10
  BOXGPSHOLD, //11
  BOXPASSTHRU, //12
  BOXBEEPERON, //13
  BOXLEDMAX, //14 we want maximum illumination
  BOXLEDLOW, //15 low/no lights
  BOXLLIGHTS, //16 enable landing lights at any altitude
  BOXCALIB, //17
  BOXGOV, //18
  BOXOSD, //19
  BOXGPSNAV, //20
  BOXLAND, //21
  CHECKBOXITEMS //22
};

//EXTENDED_AUX_STATES not supported
struct S_MSP_BOXCONFIG {
	uint8_t supported[CHECKBOXITEMS];
	uint16_t active[CHECKBOXITEMS];
};

enum stick {
	STICKARM,
	STICKDISARM,
	STICKGYROCALIB,
	STICKACCCALIB,
	STICKMAGCALIB	
};

struct S_MSP_STICKCOMBO {
	uint8_t combo;
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

void msp_BOXIDS(struct S_MSG *target);
void msp_parse_BOXIDS(struct S_MSP_BOXCONFIG *boxconf, struct S_MSG *msg);

void msp_BOX(struct S_MSG *target);
void msp_parse_BOX(struct S_MSP_BOXCONFIG *box, struct S_MSG *msg);

void msp_SET_BOX(struct S_MSG *target, struct S_MSP_BOXCONFIG *box);

/* USER DEFINED MESSAGES */
void msp_STICKCOMBO(struct S_MSG *target, struct S_MSP_STICKCOMBO *stickcombo);
void msp_parse_STICKCOMBO(struct S_MSP_STICKCOMBO *stickcombo, struct S_MSG *msg);

void msp_custom(struct S_MSG *target, uint8_t id, uint8_t *data, uint8_t length);


#endif

