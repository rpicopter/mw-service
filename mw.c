#include "debug.h"
#include "msg.h"
#include "shm.h"
#include "msp.h"
#include "uart.h"
#include "gpio.h"
#include "routines.h"

#include <stdio.h>
#include <getopt.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <sys/select.h>

#define BUFFER_SIZE 255

uint8_t stop = 0;

char uart_path[255] = "/dev/ttyUSB0";
uint8_t background = 0;
uint8_t verbosity = 0b11111111;
uint8_t reset_pin = 25;

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
	printf("-p\tgpio to use for resetting mw [defaults: %u]\n",reset_pin);
}

int set_defaults(int c, char **a) {
	int option;
	while ((option = getopt(c, a,"hbv:u:p:")) != -1) {
		switch (option)  {
			case 'b': background = 1;  break;
			case 'v': verbosity = atoi(optarg); break;
			case 'u': strcpy(uart_path,optarg); break;
			case 'p': reset_pin = atoi(optarg); break;
			default:
				print_usage();
				return -1;
				break;
		}
	}
	return 0;
} 

void get_local_status(struct S_MSG *msg) {
	dbg(DBG_VERBOSE|DBG_MW,"Getting local status\n");
	uint16_t x;
	uint8_t buf[16];

	x = msg_get_crc_error_count();
	memcpy(buf,&x,2);
	x = msg_get_rx_count();
	memcpy(buf+2,&x,2);
	x = msg_get_tx_count();
	memcpy(buf+4,&x,2);

	msp_custom(msg,50,buf,6);
}

void reset_mw() {
	dbg(DBG_VERBOSE|DBG_MW,"Resetting mw on GPIO %u\n",reset_pin);
	gpio_export(reset_pin);
	gpio_set_value(reset_pin,GPIO_HIGH);
	mssleep(1000);
	gpio_set_value(reset_pin,GPIO_LOW);
	gpio_unexport(reset_pin);
}

int main (int argc, char **argv)
{

	struct S_MSG msg;
	uint8_t bufin[BUFFER_SIZE];
	uint8_t bufin_start=0,bufin_end=0;

	uint8_t bufout[BUFFER_SIZE];
	uint8_t bufout_start=0,bufout_end=0;


	int i,ret;

	//,ret,buf_ptr = 0;
	int fd = 0;
	fd_set rlist, wlist, elist;
	struct timeval tv;

	//evaluate options
	if (set_defaults(argc,argv)) return -1;

	//initialize logger
	dbg_init(verbosity);

	//install handlers
	signal(SIGTERM, catch_signal);
	signal(SIGINT, catch_signal);


	if (shm_server_init()) return -1;

	fd = uart_init(uart_path);

  	if (fd<0) return -1;

	dbg(DBG_MW|DBG_LEVELS,"Started\n");

	while (!stop) {
        tv.tv_sec = 0;
        tv.tv_usec = 10000; //every 10ms

        FD_ZERO(&rlist);
        FD_ZERO(&wlist);
        FD_ZERO(&elist);

        FD_SET(fd, &elist);
        FD_SET(fd, &rlist);
        if (bufout_end!=bufout_start)
        	FD_SET(fd, &wlist);


        ret = select(fd+1, &rlist, &wlist, &elist, &tv);

        if (FD_ISSET(fd, &elist)) {
            dbg(DBG_MW|DBG_ERROR,"Exception\n");
            break;
        }

        if (ret == -1) {
            dbg(DBG_MW|DBG_ERROR,"Select exception: %s\n",strerror(errno));
            stop = 1;
            break;
        }  

        if (FD_ISSET(fd, &wlist)) {
			i=uart_write(bufout+bufout_start,bufout_end-bufout_start);
			if (i<0) { 
				dbg(DBG_MW|DBG_ERROR,"Writing to device error (%i %i): %s\n",bufout_start,bufout_end,strerror(errno));
				stop = 1;
				break;
			}
			bufout_start+=i;
			
			if (i==(bufout_end-bufout_start)) {//we wrote everything
				bufout_end=bufout_start=0;
			} else { //partial write, rewind
				memmove(bufout,bufout+bufout_start,bufout_end-bufout_start);
				bufout_end -= bufout_start;
				bufout_start = 0;
			}        	
        }


        ret = 1; 
        while (ret && (BUFFER_SIZE-bufout_end>MSG_MAX_DATA_LEN)) { //if we have space in buffer
			ret = shm_scan_outgoing(&msg); 
			if (ret==1) {
				switch (msg.message_id)	{
					case 50: //this is our custom status message
						get_local_status(&msg);
						shm_put_incoming(&msg);
						break;
					case 51: 
						reset_mw();
						break;
					default: //add message to buffer for sending later
						bufout_end += msg_serialize(bufout+bufout_end,&msg);
				}
			}	
        }


        if (FD_ISSET(fd, &rlist)) {
        	if (BUFFER_SIZE-bufin_end<MSG_MAX_DATA_LEN) {
				dbg(DBG_MW|DBG_ERROR,"Reading buffer overflow! Resetting.\n");
				bufin_start=bufin_end = 0;
			}
        	ret = uart_read(bufin+bufin_end,BUFFER_SIZE-bufin_end);
        	if (ret>0) { //received some data
				bufin_end+=ret;
				while (ret=msg_parse(&msg,bufin+bufin_start,bufin_end-bufin_start)) { //retrieve msg from buffer 			
					bufin_start+=ret;
				 	if (msg.message_id)
						shm_put_incoming(&msg);
				}
				if (bufin_start) { //rewind buffer by number of process bytes			
					memmove(bufin,bufin+bufin_start,bufin_end-bufin_start);
					bufin_end-=bufin_start;
					bufin_start = 0;
				}
			}
       	}
	}

	uart_close();

	shm_server_end();

	dbg(DBG_MW|DBG_LEVELS,"End\n");

	return 0;
}

