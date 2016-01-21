#include "uart.h"
#include "debug.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <errno.h>

#define UART_MSG_MAX 0xFF

static int uart_fd = -1;

/* we use buffers to reduce number of system calls 

static uint8_t buffer_in[UART_MSG_MAX];
static uint8_t in_w_ptr = 0, in_r_ptr = 0;

static uint8_t buffer_out[UART_MSG_MAX];
static uint8_t out_ptr = 0;
*/

/* to be moved somewhere else
void _uart_recv_stash(const uint8_t *uart_msg, const int uart_msg_len) {
        dbg(DBG_UART|DBG_VERBOSE,"Queuing incoming UART data, len: %i\n",uart_msg_len);
        int i;
        for (i=0;i<uart_msg_len;i++) {
                buffer_in[in_w_ptr++] = uart_msg[i];
                if (in_w_ptr>=UART_MSG_MAX) in_w_ptr=0;
                if (in_w_ptr == in_r_ptr) { 
                        dbg(DBG_UART|DBG_ERROR,"buffer_in overflow!!\n");
                        return;
                }
        }       
}
*/


int uart_init(const char *path) {
        dbg(DBG_UART|DBG_VERBOSE,"Openining %s\n",path);
        uart_fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (uart_fd == -1) {
                dbg(DBG_UART|DBG_ERROR,"Failed to open UART device!\n");
                return -1;
        }

        struct termios options;
        tcgetattr(uart_fd, &options);
        options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
        options.c_iflag = IGNPAR;
        options.c_oflag = 0;
        options.c_lflag = 0;
        tcflush(uart_fd, TCIFLUSH);
        tcsetattr(uart_fd, TCSANOW, &options);
        return 0;
}

void uart_close() {
        dbg(DBG_UART|DBG_VERBOSE,"Closing.\n");
        close (uart_fd);
        uart_fd = -1;
}

int uart_read(uint8_t *buf, int size) {
        int ret;
        ret = read(uart_fd, (void*)buf, size);
        if (ret<0) {
		if (errno==EAGAIN) return 0;
                perror("UART: Error reading");
        }
        else if (ret>0) {
		dbg(DBG_UART|DBG_VERBOSE,"rx: \"%s\", len: %i\n",dbg_printHEX(buf,ret),ret);
	}

        return ret;
}

int uart_write(uint8_t *buf, int count) {
        int ret;
        ret=write(uart_fd, buf, count);
        if (ret<0) {
                perror("UART: Error writing");
        }
        else {
		dbg(DBG_UART|DBG_VERBOSE,"tx: \"%s\", len: %i\n",dbg_printHEX(buf,count),count);
	}
        return ret;
}

