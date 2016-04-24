#include "debug.h"
#include "msp.h"
#include <string.h>
#include "endian.h"

const char pidnames[] =
  "ROLL;"
  "PITCH;"
  "YAW;"
  "ALT;"
  "Pos;"
  "PosR;"
  "NavR;"
  "LEVEL;"
  "MAG;"
  "VEL;"
;

const char boxnames[] = 
  "ARM;"
  "ANGLE;"
  "HORIZON;"
  "BARO;"
  "VARIO;"
  "MAG;"
  "HEADFREE;"
  "HEADADJ;"  
  "CAMSTAB;"
  "CAMTRIG;"
  "GPS HOME;"
  "GPS HOLD;"
  "PASSTHRU;"
  "BEEPER;"
  "LEDMAX;"
  "LEDLOW;"
  "LLIGHTS;"
  "CALIB;"
  "GOVERNOR;"
  "OSD SW;"
  "MISSION;"
  "LAND;"
 ;


uint8_t msp_get_pid_count() {
	return MAX_PID;
}

uint8_t msp_get_pidid(const char *name) {
	//tokenize the string of pid names
	char buf[256];
	strcpy(buf, pidnames);
	uint8_t i;
	char *ret;

	ret = strtok(buf,";");
	if (strcmp(name,ret)==0) return 0;

	for (i=1;i<MAX_PID;i++) {
		ret = strtok(NULL,";");
		if (strcmp(name,ret)==0) return i;
	}
	
	return UINT8_MAX;
}

const char* msp_get_pidname(uint8_t pid) {
	//tokenize the string of pid names
	static char buf[256];
	strcpy(buf, pidnames);
	uint8_t i;
	char *ret;
	ret = strtok(buf,";");
	for (i=1;i<=pid;i++)
		ret = strtok(NULL,";");
	return ret;
}

uint8_t msp_get_box_count() {
	return CHECKBOXITEMS;
}

const char* msp_get_boxname(uint8_t box) {
	//tokenize the string of pid names
	static char buf[256];
	strcpy(buf, boxnames);
	uint8_t i;
	char *ret;
	ret = strtok(buf,";");
	for (i=1;i<=box;i++)
		ret = strtok(NULL,";");
	return ret;
}

uint8_t msp_get_boxid(const char *name) {
	//tokenize the string of pid names
	char buf[256];
	strcpy(buf, boxnames);
	uint8_t i;
	char *ret;

	ret = strtok(buf,";");
	if (strcmp(name,ret)==0) return 0;

	for (i=1;i<CHECKBOXITEMS;i++) {
		ret = strtok(NULL,";");
		if (strcmp(name,ret)==0) return i;
	}
	
	return UINT8_MAX;	
}

uint8_t msp_is_armed(struct S_MSP_STATUS *status) {
	//the arm flag is the very last bit
	return get_bit(status->flag,0); 

}

/* constructors and parsers for all messages */
void mspmsg_IDENT_serialize(struct S_MSG *target, struct S_MSP_IDENT *src) {
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_IDENT\n");
	target->message_id = MSP_IDENT;
	target->size = 0;
	//TODO: data
}

void mspmsg_IDENT_parse(struct S_MSP_IDENT *ident, struct S_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_IDENT\n");
	ident->version = msg->data[0];
	ident->multitype = msg->data[1];
	ident->msp_version = msg->data[2];
	ident->capability = *(msg->data+3);

	dbg(DBG_MSP|DBG_VERBOSE,"version: %02x, multitype: %02x, msp_version: %02x, capability %08x\n",ident->version,ident->multitype,ident->msp_version,ident->capability);
}


void mspmsg_STATUS_serialize(struct S_MSG *target, struct S_MSP_STATUS *src) {
        dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_STATUS\n");
        target->message_id = MSP_STATUS;
        target->size = 0;
        //TODO: data
}

void mspmsg_STATUS_parse(struct S_MSP_STATUS *status, struct S_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_STATUS\n");
	status->cycleTime = *reverse16(msg->data); 
	status->i2c_errors_count = *reverse16(msg->data+2); 
	status->sensor = *(msg->data+4);
	status->flag = *(msg->data+6);  //we do not want to reverse flag as this is not a value
	status->currentSet = msg->data[10];
	
	dbg(DBG_MSP|DBG_VERBOSE,"cycleTime: %04x, i2c_errors_count: %04x, sensor: %04x, flag: %08x, currentSet: %02x\n",status->cycleTime,status->i2c_errors_count,status->sensor,status->flag,status->currentSet);
}

void mspmsg_RAW_IMU_serialize(struct S_MSG *target, struct S_MSP_RAW_IMU *src) {
        dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_RAW_IMU\n");
        target->message_id = MSP_RAW_IMU;
        target->size = 0;
        //TODO: data
}

void mspmsg_RAW_IMU_parse(struct S_MSP_RAW_IMU *imu, struct S_MSG *msg) {
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

void mspmsg_ATTITUDE_serialize(struct S_MSG *target, struct S_MSP_ATTITUDE *src) {
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_ATTITUDE\n");
    target->message_id = MSP_ATTITUDE;
    target->size = 0;
    //TODO: data	
}

void mspmsg_ATTITUDE_parse(struct S_MSP_ATTITUDE *target, struct S_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_ATTITUDE\n");
	target->angx = *reverse16(msg->data);
	target->angy = *reverse16(msg->data+2);
	target->heading = *reverse16(msg->data+4);
}

void mspmsg_ALTITUDE_serialize(struct S_MSG *target, struct S_MSP_ALTITUDE *src) {
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_ALTITUDE\n");
    target->message_id = MSP_ALTITUDE;
    target->size = 0;
    //TODO: data	
}

void mspmsg_ALTITUDE_parse(struct S_MSP_ALTITUDE *target, struct S_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_ALTITUDE\n");
	target->EstAlt = *reverse32(msg->data);
	target->vario = *reverse16(msg->data+4);
}

void mspmsg_RAW_GPS_serialize(struct S_MSG *target, struct S_MSP_RAW_GPS *src) {
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_ATTITUDE\n");
    target->message_id = MSP_RAW_GPS;
    target->size = 0;
    //TODO: data	
}

void mspmsg_RAW_GPS_parse(struct S_MSP_RAW_GPS *target, struct S_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_RAW_GPS\n");
	target->fix = *msg->data;
	target->num_sat = *(msg->data+1);
	target->lat = *reverse32(msg->data+2);
	target->lon = *reverse32(msg->data+6);
	target->alt = *reverse16(msg->data+10);
	target->speed = *reverse16(msg->data+12);
	target->ground_course = *reverse16(msg->data+14);
}

void mspmsg_SERVO_serialize(struct S_MSG *target, struct S_MSP_SERVO *src) {
        dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_SERVO\n");
        target->message_id = MSP_SERVO;
        target->size = 0;
        //TODO: data
}

void mspmsg_SERVO_parse(struct S_MSP_SERVO *servo, struct S_MSG *msg) {
	uint8_t i;
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_SERVO\n");
	for (i=0;i<8;i++)
		servo->servo[i] = *reverse16(msg->data+(i*2));

	dbg(DBG_MSP|DBG_VERBOSE,"S0: %04x, S1: %04x, S2: %04x, S3: %04x, S3: %04x, S4: %04x, S5: %04x, S6: %04x, S7: %04x\n",servo->servo[0],servo->servo[1],servo->servo[2],servo->servo[3],servo->servo[4],servo->servo[5],servo->servo[6],servo->servo[7]);
}

void mspmsg_RC_serialize(struct S_MSG *target, struct S_MSP_RC *src) {
        dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_RC\n");
        target->message_id = MSP_RC;
        target->size = 0;
        //TODO: data
}

void mspmsg_SET_RAW_RC_serialize(struct S_MSG *target, struct S_MSP_RC *src) {
	    target->message_id = MSP_SET_RAW_RC;
		if (src==NULL) {
			target->size = 0;
			return;
		}	    
        target->size = 16;
        memcpy(target->data,reverse16(&src->roll),2);
        memcpy(target->data+2,reverse16(&src->pitch),2);
        memcpy(target->data+4,reverse16(&src->yaw),2);
        memcpy(target->data+6,reverse16(&src->throttle),2);
        memcpy(target->data+8,reverse16(&src->aux1),2);
        memcpy(target->data+10,reverse16(&src->aux2),2);
        memcpy(target->data+12,reverse16(&src->aux3),2);
        memcpy(target->data+14,reverse16(&src->aux4),2);
}

void mspmsg_RC_parse(struct S_MSP_RC *rc, struct S_MSG *msg) {
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

void mspmsg_BOXIDS_serialize(struct S_MSG *target, struct S_MSP_BOXCONFIG *src) {
    dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_BOXIDS\n");
    target->message_id = MSP_BOXIDS;
    target->size = 0;
    //TODO: data
}

void mspmsg_BOXIDS_parse(struct S_MSP_BOXCONFIG *boxconf, struct S_MSG *msg) {
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

void mspmsg_BOX_serialize(struct S_MSG *target, struct S_MSP_BOXCONFIG *msg) {
    dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_BOX\n");
    target->message_id = MSP_BOX;
    target->size = 0;
    //TODO: data
}

void mspmsg_BOX_parse(struct S_MSP_BOXCONFIG *box, struct S_MSG *msg) {
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

void mspmsg_SET_BOX_serialize(struct S_MSG *target, struct S_MSP_BOXCONFIG *src) {
	uint8_t i,j = 0;
    dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_SET_BOX\n");
    target->message_id = MSP_SET_BOX;
	if (src==NULL) {
		target->size = 0;
		return;
	}

    dbg(DBG_MSP|DBG_VERBOSE,"Setting box config: ");
	for (i=0;i<CHECKBOXITEMS;i++) {
		if (src->supported[i]) {
			memcpy(target->data+(j*2),reverse16(&src->active[i]),2);
			dbg(DBG_MSP|DBG_VERBOSE,"%u=%u ",i,src->active[i]);
			j++;
		}
	}
	target->size = 2*j;
}

void mspmsg_PID_serialize(struct S_MSG *target, struct S_MSP_PIDITEMS *src) {
	uint8_t i;
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_PID\n");
	target->message_id = MSP_PID;
	
	if (src==NULL) {
		target->size = 0;
		return;
	}

	for (i=0;i<MAX_PID;i++)	{
		target->data[i*3] = src->pid[i].P8;
		target->data[i*3+1] = src->pid[i].I8;
		target->data[i*3+2] = src->pid[i].D8;
	}

	target->size = MAX_PID*3;
}


void mspmsg_PID_parse(struct S_MSP_PIDITEMS *piditems, struct S_MSG *msg) {
	uint8_t i;
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_PID\n");
	for (i=0;i<MAX_PID;i++)	{
		piditems->pid[i].P8 = msg->data[i*3];
		piditems->pid[i].I8 = msg->data[i*3+1];
		piditems->pid[i].D8 = msg->data[i*3+2];
	}
}

void mspmsg_SET_PID_serialize(struct S_MSG *target, struct S_MSP_PIDITEMS *src) {
	uint8_t i;
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_SET_PID\n");
	target->message_id = MSP_SET_PID;
	
	if (src==NULL) {
		target->size = 0;
		return;
	}

	for (i=0;i<MAX_PID;i++)	{
		target->data[i*3] = src->pid[i].P8;
		target->data[i*3+1] = src->pid[i].I8;
		target->data[i*3+2] = src->pid[i].D8;
	}

	target->size = MAX_PID*3;
}

/* USER DEFINED MESSAGES */
void mspmsg_STICKCOMBO_serialize(struct S_MSG *target, struct S_MSP_STICKCOMBO *src) {
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_STICKCOMBO\n");
	target->message_id = MSP_STICKCOMBO;
	if (src==NULL) {
		target->size = 0;
		return;
	}

	target->data[0] = src->combo;
	target->size = 1;
}

void mspmsg_STICKCOMBO_parse(struct S_MSP_STICKCOMBO *stickcombo, struct S_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_STICKCOMBO\n");
	stickcombo->combo = msg->data[0];
}

void mspmsg_LOCALSTATUS_serialize(struct S_MSG *target, struct S_MSP_LOCALSTATUS *src) {
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_LOCALSTATUS\n");
	target->message_id = MSP_LOCALSTATUS;
	if (src==NULL) {
		target->size = 0;
		return;
	}

	//we are communicating with the mw-service hence no need to reverse byte order
	memcpy(target->data,&src->crc_error_count,2);
	memcpy(target->data+2,&src->rx_count,2);
	memcpy(target->data+4,&src->tx_count,2);
	memcpy(target->data+6,&src->rssi,1);
	memcpy(target->data+7,&src->noise,1);

	target->size = 8;
}

void mspmsg_LOCALSTATUS_parse(struct S_MSP_LOCALSTATUS *status, struct S_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_LOCALSTATUS\n");
	memcpy(&status->crc_error_count,msg->data,2);
	memcpy(&status->rx_count,msg->data+2,2);
	memcpy(&status->tx_count,msg->data+4,2);
	memcpy(&status->rssi,msg->data+6,1);
	memcpy(&status->noise,msg->data+7,1);
}

void mspmsg_HOST_WIFI_parse(struct S_MSP_HOST_WIFI *status, struct S_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_HOST_WIFI\n");
	status->rssi = msg->data[0];
	status->noise = msg->data[1];

}

void mspmsg_HOST_WIFI_serialize(struct S_MSG *target, struct S_MSP_HOST_WIFI *src) {
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_HOST_WIFI\n");
	target->message_id = MSP_HOST_WIFI;
	if (src==NULL) {
		target->size = 0;
		return;
	}

	memcpy(target->data,&src->rssi,1);
	memcpy(target->data+1,&src->noise,1);

	target->size = 2;
}

void mspmsg_custom_serialize(struct S_MSG *target, uint8_t id, uint8_t *data, uint8_t length) {
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing custom message: %u\n",id);
	target->message_id = id;
	target->size = length;
	memcpy(target->data,data,length);
}


