#include "routines.h"
#include <stdio.h>

void mssleep(unsigned int ms) {
  struct timespec tim;
   tim.tv_sec = ms/1000;
   tim.tv_nsec = 1000000L * (ms % 1000);
   if(nanosleep(&tim , &tim) < 0 )
   {
      printf("Nano sleep system call failed \n");
   }
}


struct timespec *TimeSpecDiff(struct timespec *ts1, struct timespec *ts2) //difference between ts1 and ts2
{
        static struct timespec ts;
        ts.tv_sec = ts1->tv_sec - ts2->tv_sec;
        ts.tv_nsec = ts1->tv_nsec - ts2->tv_nsec;
        if (ts.tv_nsec < 0) { //wrap around
                ts.tv_sec--;
                ts.tv_nsec += 1000000000;
        }
        return &ts;
}

long TimeSpecMS(struct timespec *dt) {
        return dt->tv_sec*1000 + dt->tv_nsec/1000000;
}

