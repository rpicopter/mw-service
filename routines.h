#ifndef ROUTINES_H
#define ROUTINES_H

#include <time.h>

void mssleep(unsigned int ms);

struct timespec *TimeSpecDiff(struct timespec *ts1, struct timespec *ts2);

long TimeSpecMS(struct timespec *dt);

#endif
