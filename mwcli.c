#include "shm.h"
#include "msp.h"

#include <stdio.h>
#include <string.h>
#include <getopt.h>
#include <signal.h>
#include "debug.h"
#include "routines.h"

uint8_t verbosity = 0b11111111;
uint8_t stop = 0;

struct S_MSG msg;
uint8_t request_message;
int mode = -1;


void catch_signal(int sig)
{
        stop = 1;
}

void print_usage() {
        printf("Usage:\n");
	printf("-h\thelp\n");
        printf("-l\tlisten (sniffing) mode\n");
	printf("-p N\tsend message with id N\n"); 
}

int set_defaults(int c, char **a) {
        int option;
        while ((option = getopt(c, a,"hlp:")) != -1) {
                switch (option)  {
                        case 'l': mode = 1;  break;
			case 'p': 
				mode = 0;
				request_message = atoi(optarg);
				break;
                        default:
                                print_usage();
                                return -1;
                                break;
                }
        }

	if (mode==-1) {
		print_usage();
		return -1;
	}

        return 0;
}

void listen() {
	struct S_MSP_IDENT ident;
	struct S_MSP_STATUS status;
	struct S_MSP_RAW_IMU imu;
	struct S_MSP_SERVO servo;
	struct S_MSP_RC rc;

	while (!stop) {
		mssleep(100);
		if (shm_scan_incoming(&msg)) {
			switch (msg.message_id) {
				case 100: msp_parse_IDENT(&ident,&msg); break;
				case 101: msp_parse_STATUS(&status,&msg); break;
				case 102: msp_parse_RAW_IMU(&imu,&msg); break;
				case 103: msp_parse_SERVO(&servo,&msg); break;
				case 105: msp_parse_RC(&rc,&msg); break;
				default: printf("Msg ID: %u (NOT IMPLEMENTED)\n",msg.message_id);
			}
		}

	}
}

int main (int argc, char **argv)
{
	signal(SIGTERM, catch_signal);
        signal(SIGINT, catch_signal);

	if (set_defaults(argc,argv)) return -1;

	dbg_init(verbosity);	

	if (shm_client_init()) return -1;

	if (mode==1) { //-l was specified
		//request IDENT and STATUS for demo
		msp_IDENT(&msg); //prepare message
		shm_put_outgoing(&msg);  //send it to the service
		msp_STATUS(&msg);
		shm_put_outgoing(&msg);

		//run loop
		listen();
	} 
	else if (mode==0) { //-p was specified
		msg.message_id = request_message;
		msg.size = 0;
		shm_put_outgoing(&msg);
	}

	
	shm_client_end();

	return 0;
}

