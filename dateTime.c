
#include "dateTime.h"

#include "param.h"
#include "GSM.h"
#include "time.h"
#include "GPS.h"
#include "SMS.h"
#include "log.h"

const  date_time_t date_time_t_init = DATE_TIME_T_INIT;


/******************************************************************************/
void dateTime_init(){

}


/******************************************************************************/
void dateTime_engine(){

    if(  GSM_dateTime_isValid() == TRUE ) {

        // Send once a day a SMS
        // for that , save in EEPROM the last time that the SMS of day has been sent
        if( (param_dateTimeSaved_get()->year != GSM_getDateTime()->year) ||
            (param_dateTimeSaved_get()->month != GSM_getDateTime()->month) ||
            (param_dateTimeSaved_get()->day != GSM_getDateTime()->day) )
        {
            // current date is differente that date of last SMS of day send

            // > So send a new SMS of day
            if( GPS_what_is_GPS_fix_state() == GPS_HARDWARE_ERROR )
            {
                SMS_send( OPERATIONNELLE_GPS_HS );
            }
            else
            {
                SMS_send( OPERATIONNELLE );
            }

            log_add( LOG_DAILY );

            // > Save current time for next day
            param_dateTimeSaved_write( GSM_getDateTime() );
        }


        // Radio validation : play sentence once a day on radio
        if( param_get()->radioValidTime.enable )
        {
            static enum {
                STATE_CHECK_TIME          = 0,
                STATE_PLAY_FIRST_TIME     = 1,
                STATE_WAIT_BEFORE_REPEAT  = 2,
                STATE_PLAY_SECOND_TIME    = 3,
                STATE_END_IDLE            = 4
            } state = STATE_CHECK_TIME ;

            static timer_t timer_repeat = TIMER_T_NULL;

            switch( state ) {
                ////////////////////////////////////////////////////////////////////
                case STATE_CHECK_TIME :
                    if( (GSM_getDateTime()->hour == param_get()->radioValidTime.hour) &&
                        (GSM_getDateTime()->min  == param_get()->radioValidTime.min) )
                    {
                        state = STATE_PLAY_FIRST_TIME;
                    }
                    break;
                ////////////////////////////////////////////////////////////////////
                case STATE_PLAY_FIRST_TIME :
                    { //play sound -> "Machine" "En fonctionnement" on radio
                        const sound_t sound_list[] = { SOUND_STD_Machine , SOUND_STD_Operationnel };
                        sound_playList( true , sound_list , sizeof(sound_list) );
                        timer_repeat = time_timerStart_sec( 5 ); // Wait 5 secondes before repeate
                        state = STATE_WAIT_BEFORE_REPEAT;
                    }
                    break;
                ////////////////////////////////////////////////////////////////////
                case STATE_WAIT_BEFORE_REPEAT :
                    if( time_timerCheck( &timer_repeat) == true )
                    {
                        state = STATE_PLAY_SECOND_TIME;
                    }
                    break;
                ////////////////////////////////////////////////////////////////////
                case STATE_PLAY_SECOND_TIME :
                    { //play sound -> "Machine" "En fonctionnement" on radio
                        const sound_t sound_list[] = { SOUND_STD_Machine , SOUND_STD_Operationnel };
                        sound_playList( true , sound_list , sizeof(sound_list) );
                        state = STATE_END_IDLE;
                    }
                    break;
                ////////////////////////////////////////////////////////////////////
                case STATE_END_IDLE :

                    break;
            }
        }

    }

}

