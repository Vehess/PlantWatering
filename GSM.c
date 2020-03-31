/*
 * GSM.c
 *
 *  Created on: 19 mars 2020
 *      Author: Lenovo
 */

#include <xc.h>

#include "GSM.h"
#include "time.h"

#include "dateTime.h"
#include "GPS.h"
#include "uart.h"


#include "tool.h"
#include "knownPosition.h"
#include "param.h"
#include "monitorInclino.h"

#include "modeTest.h"

const dateTime_ASCII_t dateTime_ASCII_t_init = DATE_TIME_ASCII_T_INIT;


#define UART2_BAUDRATE_GSM 19200
#define UART2_MAX_TX_FRAME_SIZE (uint16_t)(176ul) //>170
#define UART2_MAX_RX_FRAME_SIZE (uint16_t)(64ul)  //>50

uint8_t uart2_TX_buffer[UART2_MAX_TX_FRAME_SIZE];
uint8_t uart2_RX_buffer[UART2_MAX_RX_FRAME_SIZE];

#define GSM_UART UART2
#define GSM_TIMEOUT_MS 500ul
#define GSM_TIMEOUT_SMS_TEXT_MS 25000ul

#define GSM_POK_enable()        LATFbits.LATF1 = 0 // inverted
#define GSM_POK_disable()       LATFbits.LATF1 = 1 // inverted

#define GSM_Sleep()             LATBbits.LATB4 = 1
#define GSM_Wake()              LATBbits.LATB4 = 0

#define GSM_POWER_enable()      LATGbits.LATG1 = 1
#define GSM_POWER_disable()     LATGbits.LATG1 = 0


/*******************************************************************************
 *
 * ======================= INIT GSM (PIN) ==================================
 *
 *        |     |<      INIT       >|<   OP   >
 *        |     |                   |
 *  ms    | 500 | 100 | 3000 | 1000 |
 *        |     |     |      |      |
 * POWER  |_____|-----|------|------|----------
 *        |     |     |      |      |
 * POK    |_____|_____|------|______|__________
 *        |     |     |      |      |
 * WAKE   |_____|_____|______|------|----------
 *        |     |     |      |      |
 *
 *
 ******************************************************************************/


static const char * AT_CMD_GET_DATE_TIME = "AT+CCLK?\r";
static const char * AT_CMD_GET_IMEI = "AT+CGSN\r";


static char save_IMEI[GSM_NB_CHAR_IMEI+1] = "";
static date_time_t GSM_dateTime = {0,0,0,0,0,0};
static bool GSM_dateTime_valid = false;


const char machine[] = "Machine ";
const char inclinaison[] = "inclinaison";
const char garage[] = "mode garage\n";
const char test_mode[] = "Mode Test\n";
const char testing[] = "en test ";
const char danger[] = "en danger";
const char unknown_position[] = "position inconnue";
const char anchor[] = "point d'ancrage ";


const char operationnelle[] = "operationnelle";
const char malfunction[] = "disfonctionnement ";
const char GPS[] = "GPS";
const char GSM[] = "GSM";

const char text_separator[] = "\n";

dateTime_ASCII_t dateTime_ASCII;
//pente to do


/******************************************************************************/
const char* GSM_get_IMEI() {
    return save_IMEI;
}

/******************************************************************************/
const date_time_t* GSM_getDateTime() {
    return &GSM_dateTime;
}

/******************************************************************************/
const dateTime_ASCII_t* GSM_getDateTime_ASCII() {
    return &dateTime_ASCII;
}

/******************************************************************************/
bool GSM_dateTime_isValid() {
    return GSM_dateTime_valid;
}

/******************************************************************************/
static bool extract_IMEI( const uart_frame_t * p_frame ) {
    // Expected response : "\r\n355278058005359\r\n0\r"
    const uint16_t expected_len = 2 + 15 + 2 + 1 + 1;
    if( p_frame->len != expected_len )
        return false;

    if( uart_frame_startWith(p_frame , "\r\n") == false )
        return false;

    if( uart_frame_endWith(p_frame , "\r\n0\r") == false )
        return false;

    memcpy( save_IMEI , &p_frame->p_data[2] , 15 );
    save_IMEI[15] = '\0';

    NOP();
    return true;
}



/******************************************************************************/
static bool extract_dateTime( const uart_frame_t * p_frame ) {

    GSM_dateTime_valid = false; // Will be set to true if parse is OK

    // Expected response : "+CCLK: \"16/10/07,08:04:08+08\"\r\n0\r"
    const uint16_t expected_len = 7 + 1 + 8 + 1 + 8 + 1 + 7;
    if( p_frame->len != expected_len )
        return false;

    if( uart_frame_startWith(p_frame , "+CCLK: \"") == false )
        return false;

    if( uart_frame_endWith(p_frame , "\"\r\n0\r") == false )
        return false;

    date_time_t date_time;

    // extract date
    bool error = false;
    date_time.year =  tool_parseToUint32( &p_frame->p_data[8] , 2 , false , &error );
    if( error ) return false;
    date_time.month = tool_parseToUint32( &p_frame->p_data[8+3] , 2 , false , &error );
    if( error ) return false;
    date_time.day =   tool_parseToUint32( &p_frame->p_data[8+3+3] , 2 , false , &error );
    if( error ) return false;
    date_time.hour =  tool_parseToUint32( &p_frame->p_data[8+3+3+3] , 2 , false , &error );
    if( error ) return false;
    date_time.min =   tool_parseToUint32( &p_frame->p_data[8+3+3+3+3] , 2 , false , &error );
    if( error ) return false;
    date_time.sec =   tool_parseToUint32( &p_frame->p_data[8+3+3+3+3+3] , 2 , false , &error );
    if( error ) return false;

    // Check GSM is sync with BTS ( if not, it start in year 2000 : 00/01/01 ...)
    // So we can check that year is at least 2016 to valid the date
    if( date_time.year < 16 ) {
        return false;
    }

    // Expected response : "+CCLK: \"16/10/07,08:04:08+08\"\r\n0\r"
    dateTime_ASCII.year  [0]  = p_frame->p_data[ 8];
    dateTime_ASCII.year  [1]  = p_frame->p_data[ 9];
    dateTime_ASCII.year  [2]  = 0x00;
    dateTime_ASCII.month [0]  = p_frame->p_data[11];
    dateTime_ASCII.month [1]  = p_frame->p_data[12];
    dateTime_ASCII.month [2]  = 0x00;
    dateTime_ASCII.day   [0]  = p_frame->p_data[14];
    dateTime_ASCII.day   [1]  = p_frame->p_data[15];
    dateTime_ASCII.day   [2]  = 0x00;
    dateTime_ASCII.hour  [0]  = p_frame->p_data[17];
    dateTime_ASCII.hour  [1]  = p_frame->p_data[18];
    dateTime_ASCII.hour  [2]  = 0x00;
    dateTime_ASCII.minute[0]  = p_frame->p_data[20];
    dateTime_ASCII.minute[1]  = p_frame->p_data[21];
    dateTime_ASCII.minute[2]  = 0x00;
    dateTime_ASCII.second[0]  = p_frame->p_data[23];
    dateTime_ASCII.second[1]  = p_frame->p_data[24];
    dateTime_ASCII.second[2]  = 0x00;

    GSM_dateTime = date_time;
    GSM_dateTime_valid = true; // parse OK

    return true;
}


/******************************************************************************/
void GSM_init()
{
    // Cmde Alim GSM  // RG1
    // DTR_GSM        // RB4
    // Cmde POK       // RF1

    GSM_POWER_disable();
    GSM_Sleep();
    GSM_POK_disable();

    // Setpin as output
    TRISGbits.TRISG1 = 0; //GSM Power driver pin, is an output
    TRISFbits.TRISF1 = 0; //Set POK out
    TRISBbits.TRISB4 = 0; //Set DTR_GSM as output

    uart_init( GSM_UART , UART2_BAUDRATE_GSM ,
        uart2_TX_buffer , sizeof(uart2_TX_buffer) ,
        uart2_RX_buffer , sizeof(uart2_RX_buffer) );

}

typedef enum{

    STATE_INIT_START,                   // Statr a new INIT
    STATE_INIT_POWER_OFF,               // Let power down for some time
    STATE_INIT_POWER_ON,                // Power ON the GSM module
    STATE_INIT_POK_ON,                  // Enable POK
    STATE_INIT_POK_OFF_WAKE,            // Disable POK / WAKE GSM (DTR))
    STATE_INIT_SEND_IMEI,               // Send request to get IMEI
    STATE_INIT_RECV_IMEI,               // Received IMEI
    STATE_INIT_SEND_SET_MODE_TEXT,      // Send request to enable mode test (SMS)
    STATE_INIT_RECV_SET_MODE_TEXT,      // Received response to enable mode test (SMS)
    STATE_INIT_WAIT_END,                // Wait some time
    STATE_INIT_ERROR,                   // Manage error on init

    STATE_LOOP_OK,
    STATE_LOOP_ERROR,
    STATE_LOOP_WAIT,

    STATE_LOOP_SEND_DATE_TIME,
    STATE_LOOP_RECV_DATE_TIME,
    STATE_LOOP_CHECK_REQUEST,

    STATE_LOOP_SMS_SEND_NUMBER,
    STATE_LOOP_SMS_ACK_NUMBER,
    STATE_LOOP_SMS_SEND_MESSAGE,
    STATE_LOOP_SMS_ACK_MESSAGE,
    STATE_LOOP_SMS_OK,
    STATE_LOOP_SMS_ERROR,

}gsm_state_t;
static gsm_state_t state = STATE_INIT_START;


/******************************************************************************/
static const char * get_phoneNumber( uint8_t index_phoneNumber )
{
    return param_get()->phone_numbers[index_phoneNumber].number;
}

/******************************************************************************/

/**
 * \brief This function is the spine of the SMS that will be sent. It changes
 * the content of the message depending on the sms_type and the available data
 * comming from the GPS Trimble module.
 * @param p_frame
 * @param sms_type
 */
void GSM_prepareMessage( uart_frame_t * p_frame, sms_type_t sms_type /*, uint8_t index_phone_number*/ ) {
    #warning "SMS a verifier..."
//    uart_frame_add_string( p_frame , GSM_getDateTime_ASCII()->year );
//    uart_frame_add_string( p_frame , "/" );
//    uart_frame_add_string( p_frame , GSM_getDateTime_ASCII()->month );
//    uart_frame_add_string( p_frame , "/" );
//    uart_frame_add_string( p_frame , GSM_getDateTime_ASCII()->day );
//    uart_frame_add_string( p_frame , "," );
//    uart_frame_add_string( p_frame , GSM_getDateTime_ASCII()->hour );
//    uart_frame_add_string( p_frame , ":" );
//    uart_frame_add_string( p_frame , GSM_getDateTime_ASCII()->minute );
//    uart_frame_add_string( p_frame , text_separator );
    uart_frame_add_string( p_frame , machine );
    uart_frame_add_string_limitedNbrOfCharacter( p_frame , param_get()->machineName , sizeof(param_get()->machineName) );
    uart_frame_add_string( p_frame , text_separator );

    switch (sms_type)
    {
        case SMS_TEST                   : uart_frame_add_string( p_frame , testing );break;
        case MODE_TEST                  :
        {
            uart_frame_add_string( p_frame , test_mode );
            if( knownPosition_getCurrent() != NULL )
            {
                uart_frame_add_string( p_frame , anchor );
                uart_frame_add_string( p_frame , text_separator );
                uart_frame_add_string_limitedNbrOfCharacter( p_frame , knownPosition_getCurrent()->name, sizeof(knownPosition_getCurrent()->name));
            }
            else
            {
                uart_frame_add_string( p_frame , unknown_position );
                uart_frame_add_string( p_frame , text_separator );
                if( GPS_what_is_the_position_ASCII()->latitude_ASCII[0] != ' ' )
                {
                    uart_frame_add_string( p_frame , GPS_what_is_the_position_ASCII()->latitude_ASCII );
                    uart_frame_add_string( p_frame , GPS_what_is_the_position_ASCII()->longitude_ASCII );
                }
                uart_frame_add_string( p_frame , text_separator );
            }
        }break;
        case MODE_GARAGE                :
        {
            if( modeTest_isEnable() == true )/////////////////////////////////////
            {
                uart_frame_add_string( p_frame , testing );
            }
            uart_frame_add_string( p_frame , garage );
            uart_frame_add_string( p_frame , text_separator );

            if( knownPosition_getCurrent() != NULL )
            {
                uart_frame_add_string( p_frame , anchor );
                uart_frame_add_string( p_frame , text_separator );
                uart_frame_add_string_limitedNbrOfCharacter( p_frame , knownPosition_getCurrent()->name, sizeof(knownPosition_getCurrent()->name));
            }
            else
            {
                uart_frame_add_string( p_frame , unknown_position );
                uart_frame_add_string( p_frame , text_separator );
                if( GPS_what_is_the_position_ASCII()->latitude_ASCII[0] != ' ' )
                {
                    uart_frame_add_string( p_frame , GPS_what_is_the_position_ASCII()->latitude_ASCII );
                    uart_frame_add_string( p_frame , GPS_what_is_the_position_ASCII()->longitude_ASCII );
                }
                uart_frame_add_string( p_frame , text_separator );
            }



        }break;

        case EN_DANGER :
        {
            uart_frame_add_string( p_frame , danger );
            uart_frame_add_string( p_frame , text_separator );
            if( monitorInclino_isAlerte() == true )
            {
                uart_frame_add_string( p_frame , inclinaison );
                uart_frame_add_string( p_frame , text_separator );
            }
            else
            {
                uart_frame_add_string( p_frame , text_separator );
            }
            if( knownPosition_getCurrent() != NULL )
            {
                uart_frame_add_string( p_frame , anchor );
                uart_frame_add_string( p_frame , text_separator );
                uart_frame_add_string_limitedNbrOfCharacter( p_frame , knownPosition_getCurrent()->name, sizeof(knownPosition_getCurrent()->name));
            }
            else
            {
                uart_frame_add_string( p_frame , unknown_position );
                uart_frame_add_string( p_frame , text_separator );
                if( GPS_what_is_the_position_ASCII()->latitude_ASCII[0] != ' ' )
                {
                    uart_frame_add_string( p_frame , GPS_what_is_the_position_ASCII()->latitude_ASCII );
                    uart_frame_add_string( p_frame , GPS_what_is_the_position_ASCII()->longitude_ASCII );
                }
            }
        }break;

        case OPERATIONNELLE             : uart_frame_add_string( p_frame , operationnelle );break;

        case OPERATIONNELLE_GPS_HS      :
        {
            uart_frame_add_string( p_frame , operationnelle );
            uart_frame_add_string( p_frame , text_separator );
            uart_frame_add_string( p_frame , malfunction );
            uart_frame_add_string( p_frame , GPS );
        }break;

        case OPERATIONNELLE_GSM_HS      :
        {
            uart_frame_add_string( p_frame , operationnelle );
            uart_frame_add_string( p_frame , text_separator );
            uart_frame_add_string( p_frame , malfunction );
            uart_frame_add_string( p_frame , GSM );
        }break;
    }
}

typedef struct {
    bool send_sms;
    uint8_t sms_retry;
    uint8_t sms_error; // 0 ok else error
    uint8_t index_phoneNumber;
    sms_type_t sms_type;
}request_sms_t;

#define REQUEST_SMS_T_INIT {false,0,0}
static request_sms_t request_sms = REQUEST_SMS_T_INIT;

static uint32_t gsm_init_occured = 0;
/******************************************************************************/
uint32_t GSM_init_occured() {
    return gsm_init_occured;
}

static uint8_t gsm_init_error = 0;
/******************************************************************************/
uint32_t GSM_init_error() {
    return gsm_init_error;
}



static uint8_t gsm_loop_error_cpt = 0;

/******************************************************************************/
bool GSM_SMS_send( sms_type_t sms_type , uint8_t index_phoneNumber ) {
    if( request_sms.send_sms )
        return false;
    request_sms.send_sms = true;
    request_sms.sms_error = false;
    request_sms.index_phoneNumber = index_phoneNumber;
    request_sms.sms_type = sms_type;
    return true;
}

/******************************************************************************/
bool GSM_SMS_sender_busy() {
    return request_sms.send_sms;
}

/******************************************************************************/
bool GSM_SMS_sender_error() {
    return request_sms.sms_error;
}

/******************************************************************************/
void GSM_engine(void) {

    static timer_t timer = TIMER_T_NULL;

    switch( state ) {

        //-----------------------------------------------------------------------------
        //-------------------------- INIT ---------------------------------------------
        //-----------------------------------------------------------------------------

        ////////////////////////////////////////////////////////////////////////
        case STATE_INIT_START:{
            gsm_init_occured++;
            GSM_POWER_disable();
            GSM_POK_disable();
            GSM_Sleep();
            timer = time_timerStart_ms( 200ul );
            state = STATE_INIT_POWER_OFF; // ---------------------------------->
        }break;

        ////////////////////////////////////////////////////////////////////////
        case STATE_INIT_POWER_OFF:{
            if( time_timerCheck( &timer ) == true ) {
                GSM_POWER_enable();
                //GSM_POK_disable();
                //GSM_Sleep();
                timer = time_timerStart_ms( 1000ul );
                state = STATE_INIT_POWER_ON; // ------------------------------->
            }
        }break;

        ////////////////////////////////////////////////////////////////////////
        case STATE_INIT_POWER_ON:{
            if( time_timerCheck( &timer ) == true ) {
                //GSM_POWER_enable();
                GSM_POK_enable();
                //GSM_Sleep();
                timer = time_timerStart_ms( 3000ul );
                state = STATE_INIT_POK_ON; // --------------------------------->
            }
        }break;

        ////////////////////////////////////////////////////////////////////////
        case STATE_INIT_POK_ON:{
            if( time_timerCheck( &timer ) == true ) {
                //GSM_POWER_enable();
                GSM_POK_disable();
                GSM_Wake();
                timer = time_timerStart_ms( 1000ul );
                state = STATE_INIT_POK_OFF_WAKE; // --------------------------->
            }
        }break;

        ////////////////////////////////////////////////////////////////////////
        case STATE_INIT_POK_OFF_WAKE:{
            if( time_timerCheck( &timer ) == true ) {
                state = STATE_INIT_SEND_IMEI; // ------------------------------>
            }
        }break;

        ////////////////////////////////////////////////////////////////////////
        case STATE_INIT_SEND_IMEI:{
            uart_frame_t * p_tx_frame = uart_prepare_tx_frame( GSM_UART );
            if( p_tx_frame != NULL ) {
                uart_frame_add_string( p_tx_frame, AT_CMD_GET_IMEI );
                uart_start_tx_frame( GSM_UART );
                uart_start_rx_frame( GSM_UART );
                timer = time_timerStart_ms( GSM_TIMEOUT_MS );
                state = STATE_INIT_RECV_IMEI; // ------------------------------>
            }
        }break;

        ////////////////////////////////////////////////////////////////////////
        case STATE_INIT_RECV_IMEI:{
            uart_frame_t * p_rx_frame = uart_get_rx_frame( GSM_UART );
            if( p_rx_frame != NULL ) {
                // We receive a frame
                bool ret = extract_IMEI( p_rx_frame );
                if( ret == false ) {
                    // parse error
                    gsm_init_error = 1;
                    state = STATE_INIT_ERROR; // ------------------------------>
                }
                else {
                    state = STATE_INIT_SEND_SET_MODE_TEXT; // ----------------->
                }
            }
            else if( time_timerCheck( &timer ) == true ) {
                // timeout error
                gsm_init_error = 2;
                state = STATE_INIT_ERROR; // ---------------------------------->
            }
        }break;

        ////////////////////////////////////////////////////////////////////////
        case STATE_INIT_SEND_SET_MODE_TEXT:{
            uart_frame_t * p_tx_frame = uart_prepare_tx_frame( GSM_UART );
            if( p_tx_frame != NULL ) {
                uart_frame_add_string( p_tx_frame, "AT+CMGF=1\r" );
                uart_start_tx_frame( GSM_UART );
                uart_start_rx_frame( GSM_UART );
                timer = time_timerStart_ms( GSM_TIMEOUT_MS );
                state = STATE_INIT_RECV_SET_MODE_TEXT; // --------------------->
            }
        }break;

        ////////////////////////////////////////////////////////////////////////
        case STATE_INIT_RECV_SET_MODE_TEXT:{
            uart_frame_t * p_rx_frame = uart_get_rx_frame( GSM_UART );
            if( p_rx_frame != NULL ) {
                // We receive a frame
                bool ret = uart_frame_startWith( p_rx_frame , "0\r" );
                if( ret == false ) {
                    // parse error
                    gsm_init_error = 3;
                    state = STATE_INIT_ERROR; // ------------------------------>
                }
                else {
                    timer = time_timerStart_ms( 1000 );
                    state = STATE_INIT_WAIT_END; // --------------------------->
                }
            }
            else if( time_timerCheck( &timer ) == true ) {
                // timeout error
                gsm_init_error = 4;
                state = STATE_INIT_ERROR; // ---------------------------------->
            }
        }break;

        ////////////////////////////////////////////////////////////////////////
        case STATE_INIT_WAIT_END:{
            gsm_init_error = 0;
            if( time_timerCheck( &timer ) == true ) {
                timer = time_timerStart_ms( 30000ul ); // Wait 30s before to be able to send a first SMS
                state = STATE_LOOP_WAIT; // ----------------------------------->
            }
        }break;

        ////////////////////////////////////////////////////////////////////////
        case STATE_INIT_ERROR:{
            if( gsm_init_error == 0)
                gsm_init_error = 0xFF;
            state = STATE_INIT_START; // -------------------------------------->
        }break;

        //-----------------------------------------------------------------------------
        //-------------------------- LOOP ---------------------------------------------
        //-----------------------------------------------------------------------------

        ////////////////////////////////////////////////////////////////////////
        case STATE_LOOP_WAIT:{
            if( time_timerCheck( &timer ) == true ) {
                state = STATE_LOOP_SEND_DATE_TIME; // ------------------------->
            }
        }break;

        ////////////////////////////////////////////////////////////////////////
        case STATE_LOOP_SEND_DATE_TIME:{
            uart_frame_t * p_tx_frame = uart_prepare_tx_frame( GSM_UART );
            if( p_tx_frame != NULL ) {
                uart_frame_add_string( p_tx_frame, AT_CMD_GET_DATE_TIME );
                uart_start_tx_frame( GSM_UART );
                uart_start_rx_frame( GSM_UART );
                timer = time_timerStart_ms( GSM_TIMEOUT_MS );
                state = STATE_LOOP_RECV_DATE_TIME; // ------------------------->
            }
        }break;

        ////////////////////////////////////////////////////////////////////////
        case STATE_LOOP_RECV_DATE_TIME:{
            uart_frame_t * p_rx_frame = uart_get_rx_frame( GSM_UART );
            if( p_rx_frame != NULL ) {
                // We receive a frame
                bool ret = extract_dateTime( p_rx_frame );
                if( ret == false ) {
                    state = STATE_LOOP_ERROR; // ------------------------------>
                }
                else{
                    state = STATE_LOOP_CHECK_REQUEST; // ---------------------->
                }
            }
            else if( time_timerCheck( &timer ) == true ) {
                // end of timeout
                state = STATE_LOOP_ERROR; // ---------------------------------->
            }
        }break;

        ////////////////////////////////////////////////////////////////////////
        case STATE_LOOP_CHECK_REQUEST:{
            if( request_sms.send_sms ) {
                state = STATE_LOOP_SMS_SEND_NUMBER; // ------------------------>
            }
            else {
                state = STATE_LOOP_OK; // ------------------------------------->
            }
        }break;

        ////////////////////////////////////////////////////////////////////////
        case STATE_LOOP_SMS_SEND_NUMBER:{
            uart_frame_t * p_tx_frame = uart_prepare_tx_frame( GSM_UART );
            if( p_tx_frame != NULL ) {
                uart_frame_add_string( p_tx_frame , "AT+CMGS=\"" );
                uart_frame_add_string( p_tx_frame ,  get_phoneNumber( request_sms.index_phoneNumber ) );
                uart_frame_add_string( p_tx_frame , "\"\r" );

                uart_start_tx_frame( GSM_UART );
                uart_start_rx_frame( GSM_UART );
                timer = time_timerStart_ms( GSM_TIMEOUT_MS );
                state = STATE_LOOP_SMS_ACK_NUMBER; // ------------------------->
            }
        }break;

        ////////////////////////////////////////////////////////////////////////
        case STATE_LOOP_SMS_ACK_NUMBER:{
            uart_frame_t * p_rx_frame = uart_get_rx_frame( GSM_UART );
            if( p_rx_frame != NULL ) {
                // We receive a frame
                bool ret = uart_frame_startWith( p_rx_frame , "\r\n>" );
                if( ret == false ) {
                    request_sms.sms_error = 1;
                    state = STATE_LOOP_SMS_ERROR; // -------------------------->
                }
                else
                    state = STATE_LOOP_SMS_SEND_MESSAGE; // ------------------->
            }
            else if( time_timerCheck( &timer ) == true ) {
                // end of timeout
                request_sms.sms_error = 2;
                state = STATE_LOOP_SMS_ERROR; // ------------------------------>
            }
        }break;

        ////////////////////////////////////////////////////////////////////////
        case STATE_LOOP_SMS_SEND_MESSAGE:{
            uart_frame_t * p_tx_frame = uart_prepare_tx_frame( GSM_UART );
            if( p_tx_frame != NULL ) {
                GSM_prepareMessage( p_tx_frame , request_sms.sms_type ); // add message ///////////////////////////////////////////////////////////
                uart_frame_add_byte(  p_tx_frame , 0x1A ); // add end of message : <SUB>

                uart_start_tx_frame( GSM_UART );
                uart_start_rx_frame( GSM_UART );
                timer = time_timerStart_ms( GSM_TIMEOUT_SMS_TEXT_MS );
                state = STATE_LOOP_SMS_ACK_MESSAGE; // ------------------------>
            }
        }break;

        ////////////////////////////////////////////////////////////////////////
        case STATE_LOOP_SMS_ACK_MESSAGE:{
            uart_frame_t * p_rx_frame = uart_get_rx_frame( GSM_UART );
            if( p_rx_frame != NULL ) {
                // We receive a frame
                bool ret_check_beg = uart_frame_startWith( p_rx_frame , "+CMGS:" );
                bool ret_check_end = uart_frame_endWith( p_rx_frame , "\r\n0\r" );
                if( (ret_check_beg == false) || (ret_check_end == false) ) {
                    request_sms.sms_error = 3;
                    state = STATE_LOOP_SMS_ERROR; // -------------------------->
                }
                else
                    state = STATE_LOOP_SMS_OK; // ----------------------------->
            }
            else if( time_timerCheck( &timer ) == true ) {
                // end of timeout
                request_sms.sms_error = 4;
                state = STATE_LOOP_SMS_ERROR; // ------------------------------>
            }
        }break;

        ////////////////////////////////////////////////////////////////////////
        case STATE_LOOP_SMS_OK:{
            request_sms.sms_retry = 0;
            request_sms.send_sms = false;
            request_sms.sms_error = 0;
            state = STATE_LOOP_OK; // ----------------------------------------->
        }break;

        ////////////////////////////////////////////////////////////////////////
        case STATE_LOOP_SMS_ERROR:{
            request_sms.sms_retry ++;
            if( request_sms.sms_error==0 )
                request_sms.sms_error = 0xFF;
            if( request_sms.sms_retry < 2 ) {
                state = STATE_LOOP_SMS_SEND_NUMBER; // ----------------------->
            }
            else{
                request_sms.sms_retry = 0;
                request_sms.send_sms = false;
                state = STATE_LOOP_ERROR; // ---------------------------------->
            }
        }break;


        ////////////////////////////////////////////////////////////////////////
        case STATE_LOOP_OK:{
            gsm_loop_error_cpt = 0; // Clear error counter
            timer = time_timerStart_ms( 500ul );
            state = STATE_LOOP_WAIT; // --------------------------------------->
        }break;

        ////////////////////////////////////////////////////////////////////////
        case STATE_LOOP_ERROR:{
            gsm_loop_error_cpt++;
            if( gsm_loop_error_cpt > 16)
                state = STATE_INIT_START; // ---------------------------------->
            else {
                timer = time_timerStart_ms( 500ul );
                state = STATE_LOOP_WAIT; // ----------------------------------->
            }
        }break;
    }

}








