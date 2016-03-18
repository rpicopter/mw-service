#ifndef MSP_H
#define MSP_H

#include <stdint.h>
#include "msg.h"


#define RC_LO 1100-1 //used for stick combos; based on MultiWii.h
#define RC_HI 1900+1
#define RC_CE 1500

#define MSP_PRIVATE              1     //in+out message      to be used for a generic framework : MSP + function code (LIST/GET/SET) + data. no code yet

#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan and more
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed, ground course
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         altitude, variometer
#define MSP_ANALOG               110   //out message         vbat, powermetersum, rssi if available on RX
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         P I D coeff (9 are used currently)
#define MSP_BOX                  113   //out message         BOX setup (number is dependant of your setup)
#define MSP_MISC                 114   //out message         powermeter trig
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_BOXIDS               119   //out message         get the permanent IDs associated to BOXes
#define MSP_SERVO_CONF           120   //out message         Servo settings

#define MSP_NAV_STATUS           121   //out message         Returns navigation status
#define MSP_NAV_CONFIG           122   //out message         Returns navigation parameters

#define MSP_CELLS                130   //out message         FRSKY Battery Cell Voltages

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          P I D coeff (9 are used currently)
#define MSP_SET_BOX              203   //in message          BOX setup (number is dependant of your setup)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_SET_WP               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define MSP_SELECT_SETTING       210   //in message          Select Setting Number (0-2)
#define MSP_SET_HEAD             211   //in message          define a new heading hold direction
#define MSP_SET_SERVO_CONF       212   //in message          Servo settings
#define MSP_SET_MOTOR            214   //in message          PropBalance function
#define MSP_SET_NAV_CONFIG       215   //in message          Sets nav config parameters - write to the eeprom  

#define MSP_SET_ACC_TRIM         239   //in message          set acc angle trim values
#define MSP_ACC_TRIM             240   //out message         get acc angle trim values
#define MSP_BIND                 241   //in message          no param

#define MSP_EEPROM_WRITE         250   //in message          no param

#define MSP_DEBUGMSG             253   //out message         debug string buffer
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

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

enum multitype {
	MULTITYPENONE0, //0
	MULTITYPETRI, //1
	MULTITYPEQUADP, //2
	MULTITYPEQUADX, //3
	MULTITYPEBI, //4
	MULTITYPEGIMBAL, //5
	MULTITYPEY6, //6
	MULTITYPEHEX6, //7
	MULTITYPEFLYING_WING, //8
	MULTITYPEY4, //9
	MULTITYPEHEX6X, //10
	MULTITYPEOCTOX8, //11
	MULTITYPEOCTOFLATP, //12
	MULTITYPEOCTOFLATX, //13
	MULTITYPEAIRPLANE, //14
	MULTITYPEHELI_120_CCPM, //15
	MULTITYPEHELI_90_DEG, //16
	MULTITYPEVTAIL4, //17
	MULTITYPEHEX6H, //18
	MULTITYPENONE19,
	MULTITYPEDUALCOPTER, //20
	MULTITYPESINGLECOPTER //21
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
/* This message is used for all BOX related processing */
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

struct S_MSP_LOCALSTATUS {
	uint16_t crc_error_count; //on receiving end	
	uint16_t rx_count;
	uint16_t tx_count;
};

/* Gets arm status for the quadcopter based on flag in status message */
uint8_t msp_is_armed(struct S_MSP_STATUS *status);


/* Parsers and constructors 
 * By default each message has a create function and a parse function
 */
void mspmsg_IDENT_create(struct S_MSG *target); //creates a MSP_INDENT message
void mspmsg_IDENT_parse(struct S_MSP_IDENT *target, struct S_MSG *msg); //parses msg into MSP_INDENT structure

void mspmsg_STATUS_create(struct S_MSG *target); 
void mspmsg_STATUS_parse(struct S_MSP_STATUS *status, struct S_MSG *msg);

void mspmsg_RAW_IMU_create(struct S_MSG *target);
void mspmsg_RAW_IMU_parse(struct S_MSP_RAW_IMU *imu, struct S_MSG *msg);

void mspmsg_SERVO_create(struct S_MSG *target);
void mspmsg_SERVO_parse(struct S_MSP_SERVO *servo, struct S_MSG *msg);

void mspmsg_RC_create(struct S_MSG *target);
void mspmsg_SET_RAW_RC_create(struct S_MSG *target, struct S_MSP_RC *rc);
void mspmsg_RC_parse(struct S_MSP_RC *status, struct S_MSG *msg);

//BOXIDS = supported boxes
void mspmsg_BOXIDS_create(struct S_MSG *target);
void mspmsg_BOXIDS_parse(struct S_MSP_BOXCONFIG *boxconf, struct S_MSG *msg);

//BOX = value for all supported boxes
void mspmsg_BOX_create(struct S_MSG *target);
void mspmsg_BOX_parse(struct S_MSP_BOXCONFIG *box, struct S_MSG *msg);

void mspmsg_SET_BOX_create(struct S_MSG *target, struct S_MSP_BOXCONFIG *box); //create

/* USER DEFINED MESSAGES */
void mspmsg_STICKCOMBO_create(struct S_MSG *target, struct S_MSP_STICKCOMBO *stickcombo);
void mspmsg_STICKCOMBO_parse(struct S_MSP_STICKCOMBO *stickcombo, struct S_MSG *msg);

void mspmsg_LOCALSTATUS_create(struct S_MSG *target, struct S_MSP_LOCALSTATUS *status);
void mspmsg_LOCALSTATUS_parse(struct S_MSP_LOCALSTATUS *status, struct S_MSG *msg);

void mspmsg_custom_create(struct S_MSG *target, uint8_t id, uint8_t *data, uint8_t length);


#endif

