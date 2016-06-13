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

uint8_t msp_is_boxactive(struct S_MSP_STATUS *status, struct S_MSP_BOXCONFIG *boxconf, uint8_t box) {
	uint8_t c = 0;
	uint8_t i;

	for (i=0;i<box;i++)
		if (boxconf->supported[i]) c++;

	if (status->flag & (1<<c)) return 1;

	return 0;
}

uint8_t msp_is_armed(struct S_MSP_STATUS *status) {
	//the arm flag is always the very last bit
	return get_bit(status->flag,0); 

}

/* constructors and parsers for all messages */
void mspmsg_IDENT_serialize(struct S_MSG *target) {
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_IDENT\n");
	target->message_id = MSP_IDENT;
	target->size = 0;
}

void mspmsg_IDENT_parse(struct S_MSP_IDENT *ident, struct S_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_IDENT\n");
	ident->version = msg->data[0];
	ident->multitype = msg->data[1];
	ident->msp_version = msg->data[2];
	ident->capability = *(msg->data+3);

	dbg(DBG_MSP|DBG_VERBOSE,"version: %02x, multitype: %02x, msp_version: %02x, capability %08x\n",ident->version,ident->multitype,ident->msp_version,ident->capability);
}


void mspmsg_STATUS_serialize(struct S_MSG *target) {
    dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_STATUS\n");
    target->message_id = MSP_STATUS;
    target->size = 0;
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

void mspmsg_MISC_serialize(struct S_MSG *target) {
    dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_MISC\n");
    target->message_id = MSP_MISC;
    target->size = 0;	
}

void mspmsg_MISC_parse(struct S_MSP_MISC *misc, struct S_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_MISC\n");
	misc->intPowerTrigger1 = *reverse16(msg->data);
	misc->minthrottle = *reverse16(msg->data+2);
	misc->maxthrottle = *reverse16(msg->data+4);
	misc->mincommand = *reverse16(msg->data+6);
	misc->failsafe_throttle = *reverse16(msg->data+8);
	misc->arm = *reverse16(msg->data+10);
	misc->lifetime = *reverse32(msg->data+12);
	misc->mag_declination = *reverse16(msg->data+16);
	misc->vbatscale = *(msg->data+18);
	misc->vbatlevel_warn1 = *(msg->data+19);
	misc->vbatlevel_warn2 = *(msg->data+20);
	misc->vbatlevel_crit = *(msg->data+21);
}

void mspmsg_SET_MISC_serialize(struct S_MSG *target, struct S_MSP_MISC *src) {
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing MSP_SET_MISC\n");
	target->message_id = MSP_SET_MISC;
	if (src==NULL) {
		target->size = 0;
		return;
	}	    

    memcpy(target->data,reverse16(&src->intPowerTrigger1),2);
    memcpy(target->data+2,reverse16(&src->minthrottle),2);
    memcpy(target->data+4,reverse16(&src->maxthrottle),2);
    memcpy(target->data+6,reverse16(&src->mincommand),2);
    memcpy(target->data+8,reverse16(&src->failsafe_throttle),2);
    memcpy(target->data+10,reverse16(&src->arm),2);
    memcpy(target->data+12,reverse32(&src->lifetime),4);
    memcpy(target->data+16,reverse16(&src->mag_declination),2);
    target->data[18] = src->vbatscale;
    target->data[19] = src->vbatlevel_warn1;	
    target->data[20] = src->vbatlevel_warn2;	
    target->data[21] = src->vbatlevel_crit;	

    target->size = 22;
}

void mspmsg_RAW_IMU_serialize(struct S_MSG *target) {
    dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_RAW_IMU\n");
    target->message_id = MSP_RAW_IMU;
    target->size = 0;
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

void mspmsg_ATTITUDE_serialize(struct S_MSG *target) {
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_ATTITUDE\n");
    target->message_id = MSP_ATTITUDE;
    target->size = 0;
}

void mspmsg_ATTITUDE_parse(struct S_MSP_ATTITUDE *target, struct S_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_ATTITUDE\n");
	target->angx = *reverse16(msg->data);
	target->angy = *reverse16(msg->data+2);
	target->heading = *reverse16(msg->data+4);
}

void mspmsg_ALTITUDE_serialize(struct S_MSG *target) {
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_ALTITUDE\n");
    target->message_id = MSP_ALTITUDE;
    target->size = 0;	
}

void mspmsg_ALTITUDE_parse(struct S_MSP_ALTITUDE *target, struct S_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_ALTITUDE\n");
	target->EstAlt = *reverse32(msg->data);
	target->vario = *reverse16(msg->data+4);
}

void mspmsg_RAW_GPS_serialize(struct S_MSG *target) {
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_ATTITUDE\n");
    target->message_id = MSP_RAW_GPS;
    target->size = 0;
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

void mspmsg_WP_serialize(struct S_MSG *target, uint8_t wp_no) {
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_WP\n");
    target->message_id = MSP_WP;
    target->data[0] = wp_no;
    target->size = 1;
    //TODO: data		
}

void mspmsg_WP_parse(struct S_MSP_WP *target, struct S_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_WP\n");
	target->wp_no = *msg->data;
	target->action = *(msg->data+1);
	target->lat = *reverse32(msg->data+2);
	target->lon = *reverse32(msg->data+6);
	target->alt_hold = *reverse32(msg->data+10);
	target->param1 = *reverse16(msg->data+14);
	target->param2 = *reverse16(msg->data+16);
	target->param3 = *reverse16(msg->data+18);
	target->flag = *reverse16(msg->data+20);
}
void mspmsg_NAV_CONFIG_serialize(struct S_MSG *target) {
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_NAV_CONFIG\n");
    target->message_id = MSP_NAV_CONFIG;
    target->size = 0;	
}

void mspmsg_NAV_CONFIG_parse(struct S_MSP_NAV_CONFIG *target, struct S_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_NAV_CONFIG\n");

	target->filtering = (msg->data[0] & 0b00000001)?1:0;
	target->lead_filter = (msg->data[0] & 0b00000010)?1:0;
	target->dont_reset_home_at_arm = (msg->data[0] & 0b00000100)?1:0;
	target->nav_controls_heading = (msg->data[0] & 0b00001000)?1:0;
	target->nav_tail_first = (msg->data[0] & 0b00010000)?1:0;
	target->nav_rth_takeoff_heading = (msg->data[0] & 0b00100000)?1:0;
	target->slow_nav = (msg->data[0] & 0b01000000)?1:0;
	target->wait_for_rth_alt = (msg->data[0] & 0b10000000)?1:0;

	target->ignore_throttle = (msg->data[1] & 0b00000001)?1:0;
	target->takeover_baro = (msg->data[1] & 0b00000010)?1:0;

	target->wp_radius = *reverse16(msg->data+2);        // in cm
	target->safe_wp_distance = *reverse16(msg->data+4);    // in meter
	target->nav_max_altitude = *reverse16(msg->data+6);  // in meter
	target->nav_speed_max = *reverse16(msg->data+8);       // in cm/s
	target->nav_speed_min = *reverse16(msg->data+10);       // in cm/s
	target->crosstrack_gain = *(msg->data+12);     // * 100 (0-2.56)
	target->nav_bank_max = *reverse16(msg->data+13);       // degree * 100; (3000 default)
	target->rth_altitude = *reverse16(msg->data+15);        // in meter
	target->land_speed  = *(msg->data+17);        // between 50 and 255 (100 approx = 50cm/sec)
	target->fence = *reverse16(msg->data+18);              // fence control in meters

	target->max_wp_number = *(msg->data+20);

	target->checksum = *(msg->data+21);
};

void mspmsg_NAV_CONFIG_SET_serialize(struct S_MSG *target, struct S_MSP_NAV_CONFIG *src) {
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_SET_NAV_CONFIG\n");
    target->message_id = MSP_SET_NAV_CONFIG;

    uint8_t b;

    b = src->filtering?1:0; b<<=1;
    b = src->lead_filter?1:0; b<<=1;
    b = src->dont_reset_home_at_arm?1:0; b<<=1;
    b = src->nav_controls_heading?1:0; b<<=1;
    b = src->nav_tail_first?1:0; b<<=1;
    b = src->nav_rth_takeoff_heading?1:0; b<<=1;
    b = src->slow_nav?1:0; b<<=1;
    b = src->wait_for_rth_alt?1:0;
    target->data[0] = b;

    b = src->ignore_throttle?1:0; b<<=1;
    b = src->takeover_baro?1:0;
    target->data[1] = b;

	memcpy(target->data+2,reverse16(&src->wp_radius),2);
	memcpy(target->data+4,reverse16(&src->safe_wp_distance),2);
	memcpy(target->data+6,reverse16(&src->nav_max_altitude),2);
	memcpy(target->data+8,reverse16(&src->nav_speed_max),2);
	memcpy(target->data+10,reverse16(&src->nav_speed_min),2);
	memcpy(target->data+12,&src->crosstrack_gain,1);
	memcpy(target->data+13,reverse16(&src->nav_bank_max),2);
	memcpy(target->data+15,reverse16(&src->rth_altitude),2);
	memcpy(target->data+17,&src->land_speed,1);
	memcpy(target->data+18,reverse16(&src->fence),2);

	memcpy(target->data+20,&src->max_wp_number,1);

	memcpy(target->data+21,&src->checksum,1); 

    target->size = 22;	

}

void mspmsg_SERVO_serialize(struct S_MSG *target) {
    dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_SERVO\n");
    target->message_id = MSP_SERVO;
    target->size = 0;
}

void mspmsg_SERVO_parse(struct S_MSP_SERVO *servo, struct S_MSG *msg) {
	uint8_t i;
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_SERVO\n");
	for (i=0;i<8;i++)
		servo->servo[i] = *reverse16(msg->data+(i*2));

	dbg(DBG_MSP|DBG_VERBOSE,"S0: %04x, S1: %04x, S2: %04x, S3: %04x, S3: %04x, S4: %04x, S5: %04x, S6: %04x, S7: %04x\n",servo->servo[0],servo->servo[1],servo->servo[2],servo->servo[3],servo->servo[4],servo->servo[5],servo->servo[6],servo->servo[7]);
}

void mspmsg_RC_serialize(struct S_MSG *target) {
        dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_RC\n");
        target->message_id = MSP_RC;
        target->size = 0;
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

void mspmsg_RC_TUNING_serialize(struct S_MSG *target) {
    dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_RC_TUNING\n");
    target->message_id = MSP_RC_TUNING;
    target->size = 0;
}

void mspmsg_RC_TUNING_parse(struct S_MSP_RC_TUNING *rct, struct S_MSG *msg) {
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_RC_TUNING\n");
	rct->rcRate = *(msg->data);
	rct->rcExpo = *(msg->data+1);
	rct->rollPitchRate = *(msg->data+2);
	rct->yawRate = *(msg->data+3);
	rct->dynThrPID = *(msg->data+4);
	rct->thrMid = *(msg->data+5);
	rct->thrExpo = *(msg->data+6);
}

void mspmsg_SET_RC_TUNING_serialize(struct S_MSG *target, struct S_MSP_RC_TUNING *src) {
    dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_SET_RC_TUNING\n");
    target->message_id = MSP_SET_RC_TUNING;

	*(target->data) = src->rcRate;
	*(target->data+1) = src->rcExpo;
	*(target->data+2) = src->rollPitchRate;
	*(target->data+3) = src->yawRate;
	*(target->data+4) = src->dynThrPID;
	*(target->data+5) = src->thrMid;
	*(target->data+6) = src->thrExpo;

    target->size = 7;
}

void mspmsg_BOXIDS_serialize(struct S_MSG *target) {
    dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_BOXIDS\n");
    target->message_id = MSP_BOXIDS;
    target->size = 0;
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

void mspmsg_BOX_serialize(struct S_MSG *target) {
    dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_BOX\n");
    target->message_id = MSP_BOX;
    target->size = 0;
}

void mspmsg_BOX_parse(struct S_MSP_BOXCONFIG *box, struct S_MSG *msg) {
	uint8_t i,j = 0;
	dbg(DBG_MSP|DBG_VERBOSE,"Parsing MSP_BOX\n");
	memset(box->value,0,CHECKBOXITEMS);
	dbg(DBG_MSP|DBG_VERBOSE,"Box config: ");
	for (i=0;i<CHECKBOXITEMS;i++) {
		if (box->supported[i]) {
			box->value[i] = *reverse16(&msg->data[2*j]);
			dbg(DBG_MSP|DBG_VERBOSE,"%u=%u ",i,box->value[i]);
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
			memcpy(target->data+(j*2),reverse16(&src->value[i]),2);
			dbg(DBG_MSP|DBG_VERBOSE,"%u=%u ",i,src->value[i]);
			j++;
		}
	}
	target->size = 2*j;
}

void mspmsg_PID_serialize(struct S_MSG *target) {
	dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_PID\n");
	target->message_id = MSP_PID;
	target->size = 0;
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

void mspmsg_SELECT_SETTING_serialize(struct S_MSG *target, uint8_t set) {
    dbg(DBG_MSP|DBG_VERBOSE,"Preparing message SELECT_SETTING\n");
    target->message_id = MSP_SELECT_SETTING;
    if (set>3) set=3;
    target->data[0] = set;
    target->size = 1;
}

void mspmsg_SET_HEAD_serialize(struct S_MSG *target, int16_t target_heading) {
    dbg(DBG_MSP|DBG_VERBOSE,"Preparing message SET_HEAD\n");
    target->message_id = MSP_SET_HEAD;
    memcpy(target->data,reverse16(&target_heading),2);
    target->size = 2;
}

void mspmsg_EEPROM_WRITE_serialize(struct S_MSG *target) {
    dbg(DBG_MSP|DBG_VERBOSE,"Preparing message MSP_EEPROM_WRITE\n");
    target->message_id = MSP_EEPROM_WRITE;
    target->size = 0;
    //TODO: data	
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


