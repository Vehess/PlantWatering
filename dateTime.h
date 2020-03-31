

#ifndef DATETIME_H
#define	DATETIME_H

#include <stdint.h>
#include <stdbool.h>


void dateTime_init();
void dateTime_engine();

typedef struct {    
    uint8_t year; // nb years from 2000 (ex : 11 for 2011)
    uint8_t month; // 1..12 , 12 == december
    uint8_t day;   // 1..28/29/30/31
    uint8_t hour;  // 0..23
    uint8_t min;   // 0..59
    uint8_t sec;   // 0..59
}date_time_t;

#define DATE_TIME_T_INIT {0,1,1,0,0,0}
extern const  date_time_t date_time_t_init;



#endif	/* DATETIME_H */

