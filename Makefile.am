lib_LTLIBRARIES=libmw_core.la
libmw_core_la_SOURCES=debug.c shm.c msg.c msp.c endian.c
libmw_core_la_CFLAGS=-fPIC
libmw_core_la_LDFLAGS=-fPIC
libmw_core_includedir = $(includedir)/mw
libmw_core_include_HEADERS = debug.h shm.h msg.h msp.h endian.h

bin_PROGRAMS = mw mwcli
mw_SOURCES = mw.c routines.c uart.c
mw_CFLAGS = -Wall
mw_LDFLAGS =  
mw_LDADD = -lmw_core -lrt -lpthread

mwtest_SOURCES = mwtest.c msp.c endian.c routines.c 
mwtest_CFLAGS = -Wall
mwtest_LDFLAGS = 
mwtest_LDADD = -lmw_core -lrt -lpthread
