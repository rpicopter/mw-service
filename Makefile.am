lib_LTLIBRARIES=libmw_core.la
libmw_core_la_SOURCES=debug.c shm.c msg.c
libmw_core_la_CFLAGS=-fPIC
libmw_core_la_LDFLAGS=-fPIC
libmw_core_includedir = $(includedir)/mw
libmw_core_include_HEADERS = debug.h shm.h msg.h

bin_PROGRAMS = mw mwcli
mw_SOURCES = mw.c routines.c uart.c
mw_CFLAGS = -Wall
mw_LDFLAGS =  
mw_LDADD = -lmw_core -lrt -lpthread

mwcli_SOURCES = mwcli.c msp.c endian.c routines.c 
mwcli_CFLAGS = -Wall
mwcli_LDFLAGS = 
mwcli_LDADD = -lmw_core -lrt -lpthread
