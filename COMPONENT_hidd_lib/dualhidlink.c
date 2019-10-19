/*
 * Copyright 2019, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/********************************************************************************
*
* File Name: dualhidlink.c
*
* Abstract: This file implements HID application transport that supports both the Bluetooth(BT) Classic
*               and LE
* Functions:
*
*******************************************************************************/
#ifdef DUAL_MODE_HIDD

#include "dualhidlink.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_batmon.h"

wiced_bt_transport_t active_transport=0;

uint32_t dualhidlink_sleep_handler(wiced_sleep_poll_type_t type );
void blehidlink_init(void);
void bthidlink_init(void);
void bthidlink_determineNextState(void);
void blehidlink_determineNextState(void);


wiced_sleep_config_t    dualhidlink_sleep_config = {
    WICED_SLEEP_MODE_NO_TRANSPORT,  //sleep_mode
    0,                              //host_wake_mode
    0,                              //device_wake_mode
    WICED_SLEEP_WAKE_SOURCE_GPIO | WICED_SLEEP_WAKE_SOURCE_KEYSCAN | WICED_SLEEP_WAKE_SOURCE_QUAD,  //device_wake_source
    255,                            //must set device_wake_gpio_num to 255 for WICED_SLEEP_MODE_NO_TRANSPORT
    dualhidlink_sleep_handler        //sleep_permit_handler
};
wiced_sleep_allow_check_callback dualhidlink_registered_app_sleep_handler = NULL;


/////////////////////////////////////////////////////////////////////////////////////////////
/// Abstract link layer initialize
/////////////////////////////////////////////////////////////////////////////////////////////
void wiced_dual_mode_hidd_link_init()
{
    uint8_t *host_address;

    //Setup Battery Service
    wiced_hal_batmon_init();

    //hid link init
    blehidlink_init();
    bthidlink_init();

    //configure sleep
    wiced_sleep_configure( &dualhidlink_sleep_config );

    switch (active_transport)
    {
        case BT_TRANSPORT_BR_EDR:
            ble_hidd_link.subState = BLEHIDLINK_DISCONNECTED;
            bthidlink_determineNextState();
            break;
        case BT_TRANSPORT_LE:
            bt_hidd_link.subState = BTHIDLINK_DISCONNECTED;
            blehidlink_determineNextState();
            break;

        default:
            blehidlink_determineNextState();
            bthidlink_determineNextState();
            break;
    }

}

/////////////////////////////////////////////////////////////////////////////////
/// register application sleep permit handler
///
/// \param cb - pointer to application callback function
/////////////////////////////////////////////////////////////////////////////////
void wiced_dual_mode_hidd_link_register_sleep_permit_handler(wiced_sleep_allow_check_callback sleep_handler)
{
    dualhidlink_registered_app_sleep_handler = sleep_handler;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Sleep permit query to check if sleep (normal or SDS) is allowed and sleep time
///
/// \param type - sleep poll type
///
/// \return   sleep permission or sleep time, depending on input param
////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t dualhidlink_sleep_handler(wiced_sleep_poll_type_t type )
{
    uint32_t ret = WICED_SLEEP_NOT_ALLOWED;

    switch(type)
    {
        case WICED_SLEEP_POLL_TIME_TO_SLEEP:
            ret = WICED_SLEEP_MAX_TIME_TO_SLEEP;

            //query application for sleep time
            if (dualhidlink_registered_app_sleep_handler)
                ret = dualhidlink_registered_app_sleep_handler(type);

            break;

        case WICED_SLEEP_POLL_SLEEP_PERMISSION:
            ret = WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;

            //query application for sleep permit first
            if (dualhidlink_registered_app_sleep_handler)
                ret = dualhidlink_registered_app_sleep_handler(type);

            if (!ble_hidd_link.allowSDS)
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;

            if (wiced_hidd_is_transport_detection_polling_on())
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;

            //due to sniff+SDS is not supported in core FW, at this time, only allow SDS when disconnected
            if (bt_hidd_link.subState != BTHIDLINK_DISCONNECTED)
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;

            //save to AON before entering SDS
            if (ret == WICED_SLEEP_ALLOWED_WITH_SHUTDOWN)
            {
                wiced_ble_hidd_link_aon_action_handler(BLEHIDLINK_SAVE_TO_AON);
                wiced_bt_hidd_link_aon_action_handler(BTHIDLINK_SAVE_TO_AON);
            }
            break;

    }

    return ret;
}

#endif //#ifdef DUAL_MODE_HIDD
