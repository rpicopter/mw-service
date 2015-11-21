bin_PROGRAMS = mw mwcli
mw_SOURCES = mw.c debug.c msp.c shm.c shmmsp.c uart.c endian.c routines.c
mw_CFLAGS = -Wall
mw_LDFLAGS = 
mw_LDADD = -lrt -lpthread

mwcli_SOURCES = mwcli.c shm.c msp.c shmmsp.c endian.c debug.c routines.c 
mwcli_CFLAGS = -Wall
mwcli_LDFLAGS = 
mwcli_LDADD = -lrt -lpthread
