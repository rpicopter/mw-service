#include "uart.h"
#include "debug.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <errno.h>


static int uart_fd = -1;

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

