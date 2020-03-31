/*
 * File:   GSM.h
 * Author: vso
 *
 * Created on 12 octobre 2016, 15:04
 */

#ifndef GSM_H
#define	GSM_H

#include <stdint.h>
#include <stdbool.h>

#include "dateTime.h"
#include "uart.h"

typedef enum {
    TILT            = 0,
    SECURITY        = 1,
    END             = 2,
    EMMERGENCY_STOP = 3,
}alert_type_t;

typedef struct {
    char year[3];
    char month[3];
    char day[3];
    char hour[3];
    char minute[3];
    char second[3];
}dateTime_ASCII_t;

#define DATE_TIME_ASCII_T_INIT {"YY","MM","DD","hh","mm","ss"}
extern const dateTime_ASCII_t dateTime_ASCII_t_init;

void GSM_init(void);
void GSM_engine(void);

uint32_t GSM_init_occured();
uint32_t GSM_init_error();

#define GSM_NB_CHAR_IMEI 15
const char * GSM_get_IMEI();
const date_time_t* GSM_getDateTime();
const dateTime_ASCII_t* GSM_getDateTime_ASCII();
bool GSM_dateTime_isValid();

typedef enum {
    SMS_TEST               = 0,
    MODE_TEST              = 1,
    SMS_PTI_START          = 2,
    MODE_GARAGE            = 3,
    EN_DANGER              = 4,
    INCLINAISON            = 5,
    OPERATIONNELLE         = 6,
    OPERATIONNELLE_GPS_HS  = 7,
    OPERATIONNELLE_GSM_HS  = 8,
    DISFONCTIONNEMENT      = 9,
}sms_type_t;

bool GSM_SMS_send( sms_type_t sms_type , uint8_t index_phoneNumber );
bool GSM_SMS_sender_busy();
bool GSM_SMS_sender_error();
void GSM_prepareMessage( uart_frame_t * p_frame, sms_type_t sms_type /*, uint8_t index_phone_number*/ );

#endif	/* GSM_H */







