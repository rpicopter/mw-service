/*
 * mwtest.c - a utility to test MultiWii connection using mw-service
 * It issues MSP_STATUS and MSP_IDENT request and waits for a response.

 * Gregory Dymarek - March 2016
*/


#include "shm.h" //defines routines to send and receive MW messages
#include "msp.h" //defines the structure for MW messages

#include <stdio.h>
#include <string.h>
#include <getopt.h>
#include <signal.h>
#include "debug.h"
#include "routines.h"

uint8_t verbosity = 0b11111111; //always run in full verbocity, equivalent to DBG_MODULES|DBG_LEVELS

uint8_t stop = 0;
uint8_t wait_s = 3;

struct S_MSG msg;
uint8_t request_message;



void catch_signal(int sig)
{
        stop = 1;
}

void print_usage() {
    printf("Usage:\n");
	printf("-h\thelp\n");
    printf("-w N\twait for response N sec (0 - forever)\n");
}

int set_defaults(int c, char **a) {
    int option;
    while ((option = getopt(c, a,"hw:")) != -1) {
        switch (option)  {
            case 'w': wait_s = atoi(optarg); break;
            default: print_usage(); return -1;
        }
    }
   	
   	return 0;
}

void listen() {
	struct S_MSP_IDENT ident;
	struct S_MSP_STATUS status;
	uint8_t counter = wait_s;
	uint16_t msg_counter = 0;

	while (!stop) {
		mssleep(1000);
		if (shm_scan_incoming(&msg)) { //check if there are any messages for us
			printf("Msg ID: %u\n",msg.message_id);
			msg_counter++;
			switch (msg.message_id) {
				case 100: 
					mspmsg_IDENT_parse(&ident,&msg);  
					//ident now contains the retrieved MSP_IDENT message
				break;
				case 101: 
					mspmsg_STATUS_parse(&status,&msg); 
					//status now contains the retrieved MSP_STATUS message
				break;
			}
		}
		counter--;
		if (counter==0) stop=1;
	}

	if (msg_counter) {
		printf("\nReceived %u messages.\n",msg_counter);
		printf("MultiWii connection seems to be working fine!\n");
	} else {
		printf("\nError! No messages received.\n");
		printf("Ensure your wiring is correct and that you have activated MSP in your MultiWii config\n");
	}
}

int main (int argc, char **argv)
{
	signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);

	if (set_defaults(argc,argv)) return -1;

	dbg_init(verbosity);

	if (shm_client_init()) {
		perror("Error initializing");
		return -1; //initiate channel to mw-service
	}

	mspmsg_IDENT_serialize(&msg); //prepare MSP_IDENT message
	shm_put_outgoing(&msg); //send it to the service

	mspmsg_STATUS_serialize(&msg); //preparing MSP_STATUS message
	shm_put_outgoing(&msg); //send it to the service

	//run loop
	listen();
	
	shm_client_end(); //close channel to mw-service

	return 0;
}

