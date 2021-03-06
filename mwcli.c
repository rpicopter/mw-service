/*
 * mwcli.c - a utility to send MW from command line
 * Gregory Dymarek - April 2016
*/


#include "shm.h" //defines routines to send and receive MW messages
#include "msp.h" //defines the structure for MW messages

#include <stdio.h>
#include <string.h>
#include <getopt.h>
#include <signal.h>
#include "debug.h"
#include "routines.h"

uint8_t block;
uint8_t msg_id;
uint8_t data[MSP_MAX_SIZE];
uint8_t data_len;

uint8_t verbosity;
int8_t mode = -1;

struct S_MSG msg;

void print_usage() {
    printf("Usage: mwcli -b [block] -i [ID] -d [data] -r\n");
	printf("-h\thelp\n");
    printf("use -d for writing data; -r for reading\n");
    printf("[block] 0 for outgoing, 1 for incoming\n");
    printf("[ID] message ID\n");
    printf("[data] space separated decimals (uint8_t), i.e. 182 24 53 3\n");
}

int set_defaults(int c, char **a) {
    int option,i;
    block = 0;
    msg_id = 0;
    data_len = 0;
    verbosity = 0b00000000;
    while ((option = getopt(c, a,"hvb:i:d:r")) != -1) {
        switch (option)  {
            case 'b': block = atoi(optarg); break;
            case 'v': verbosity = 0xFF; break;
            case 'i': msg_id = atoi(optarg); break;
            case 'd':
            	for (i=optind-1;i<c-1;i++) {
            		data[data_len++] = atoi(a[i]); 
            	}
                mode = 0;
            break;
            case 'r': 
                mode = 1;
            break;
            default: 
            	print_usage(); 
            	return -1;
        }
    }

	if ((mode==0) && data_len && msg_id) return 0;
    if ((mode==1) && msg_id) return 0;
	print_usage(); 
	return -1;
}

int main (int argc, char **argv)
{
    uint8_t i;
	if (set_defaults(argc,argv)) return -1;

	dbg_init(verbosity);

	if (shm_client_init()) {
		perror("Error initializing");
		return -1; //initiate channel to mw-service
	}

    if (mode==0) {  //write 
	   mspmsg_custom_serialize(&msg,msg_id,data,data_len);

	   if (verbosity) printf("Writing MsgID: %u, data length: %u\n",msg_id,data_len);
	   if (block==0) shm_put_outgoing(&msg); //send it to the service
	   else shm_put_incoming(&msg); //send it to the service
    } else if (mode==1) {
        if (verbosity) printf("Reading MsgID: %u\n",msg_id);
        if (block==0) shm_get_outgoing(&msg,msg_id); //send it to the service
        else shm_get_incoming(&msg,msg_id); //send it to the service
        if (verbosity) printf("Length: %u Data: %s\n",msg.size,dbg_printHEX(msg.data,msg.size));
        else {
            for (i=0;i<msg.size;i++)
                printf("%c",msg.data[i]);
            printf("\n");
        }
    }

	shm_client_end(); //close channel to mw-service

	return 0;
}

