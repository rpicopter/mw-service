#include "debug.h"
#include "msg.h"
#include "shm.h"
#include "uart.h"
#include "routines.h"

#include <stdio.h>
#include <getopt.h>
#include <string.h>
#include <signal.h>

#define LOOP_TIME 20

uint8_t stop = 0;

char uart_path[255] = "/dev/ttyUSB0";
uint8_t background = 0;
uint8_t verbosity = 0b11111111;

struct timespec current_time;

void catch_signal(int sig)
{
	dbg(DBG_WARNING|DBG_MW,"Received signal: %i\n",sig);
	stop = 1;
}

void print_usage() {
	printf("Usage:\n");
	printf("-h\thelp\n");
	printf("-b\trun in background [defaults: %u]\n",background);
	printf("-v\tverbosity flag [defaults: %u]\n",verbosity);
	printf("-u\tuart device path [defaults: %s]\n",uart_path);
}

int set_defaults(int c, char **a) {
	int option;
	while ((option = getopt(c, a,"hbv:u:")) != -1) {
		switch (option)  {
			case 'b': background = 1;  break;
			case 'v': verbosity = atoi(optarg); break;
			case 'u': strcpy(uart_path,optarg); break;
			default:
				print_usage();
				return -1;
				break;
		}
	}
	return 0;
} 


int main (int argc, char **argv)
{

	struct S_MSG msg;
	uint8_t buf[255];
	uint8_t len;

	struct timespec time_prev, time_cur;
	long dt_ms;

	int i,ret,buf_ptr = 0;

	//evaluate options
	if (set_defaults(argc,argv)) return -1;

	//initialize logger
	dbg_init(verbosity);

	//install handlers
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);

	clock_gettime(CLOCK_REALTIME, &time_cur);
	time_prev = time_cur;


	if (shm_server_init()) return -1;

  	if (uart_init(uart_path)) return -1;

	dbg(DBG_MW|DBG_VERBOSE,"Starting loop...\n");

	while (!stop) {
		//ensure loop works 
		clock_gettime(CLOCK_REALTIME, &time_cur);
		dt_ms = TimeSpecMS(TimeSpecDiff(&time_cur,&time_prev));
		if (dt_ms<LOOP_TIME) mssleep(LOOP_TIME-dt_ms);
		clock_gettime(CLOCK_REALTIME, &time_prev);


		if (buf_ptr>=255) {
			dbg(DBG_MW|DBG_ERROR,"Reading buffer overflow! Resetting.\n");
			buf_ptr = 0;
		}
		ret = uart_read(buf+buf_ptr,255-buf_ptr);
		if (ret>0) { //received some data
			buf_ptr+=ret;
			i = 0;
			while ((ret=msg_parse(&msg,buf+i,buf_ptr-i))) { //retrieve msg from buffer 			
				 i+=ret;
				 if (msg.message_id)
					shm_put_incoming(&msg);
			}
			if (i) { //rewind buffer by number of process bytes			
				memmove(buf,buf+i,buf_ptr-i);
				buf_ptr = buf_ptr - i;
			}
		}

		
		while ((ret=shm_scan_outgoing(&msg))) {
			if (ret==1) {
				ret = msg_serialize(buf,&msg);
				uart_write(buf,ret);
			}
		}
	}

	uart_close();

	shm_server_end();

	return 0;
}

