/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
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
* File Name: hidd_lib.c
*
* Abstract: This file implements HID application transport that supports both the Bluetooth(BT) Classic
*               and LE
* Functions:
*
*******************************************************************************/
#include "wiced.h"
#include "wiced_sleep.h"
#include "wiced_timer.h"
#include "hidd_lib.h"
#include "wiced_bt_ota_firmware_upgrade.h"

#ifdef SUPPORT_EPDS
#define PMU_CONFIG_FLAGS_ENABLE_SDS 0x00002000
extern uint32_t g_foundation_config_PMUflags;
#endif

typedef struct
{
    // is shutdown sleep (SDS) allowed?
    uint8_t allowDeepSleep:1;
    uint8_t allowHIDOFF:1;

    /// allow SDS timer
    wiced_timer_t allowDeepSleepTimer;
    wiced_sleep_allow_check_callback  p_app_sleep_handler;

} hidd_sleep_t;

static hidd_sleep_t hidd_sleep;

////////////////////////////////////////////////////////////////////////////////
/// This function is the timeout handler for allowsleep_timer
////////////////////////////////////////////////////////////////////////////////
static void allow_sleep_timer_cb( uint32_t arg )
{
    hidd_deep_sleep_not_allowed(0); // sleep is now allowed
}

////////////////////////////////////////////////////////////////////////////////
/// This function returns if sleep is allowed
////////////////////////////////////////////////////////////////////////////////
uint8_t hidd_is_deep_sleep_allowed()
{
    return hidd_sleep.allowDeepSleep;
}

////////////////////////////////////////////////////////////////////////////////
/// This function returns if allow to sleep timer is running
////////////////////////////////////////////////////////////////////////////////
uint8_t hidd_is_deep_sleep_timer_running()
{
    return wiced_is_timer_in_use(&hidd_sleep.allowDeepSleepTimer);
}

////////////////////////////////////////////////////////////////////////////////
/// This function allows device to sleep
////////////////////////////////////////////////////////////////////////////////
void hidd_set_deep_sleep_allowed( uint8_t allowed )
{
    hidd_sleep.allowDeepSleep = allowed;
    if (hidd_is_deep_sleep_timer_running())
    {
//        WICED_BT_TRACE("\ndeepSleep timer stopped");
        wiced_stop_timer(&hidd_sleep.allowDeepSleepTimer);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// This function not allowing the device to sleep for period of time
////////////////////////////////////////////////////////////////////////////////
void hidd_deep_sleep_not_allowed( uint32_t milliseconds )
{
    hidd_set_deep_sleep_allowed(milliseconds ? WICED_FALSE : WICED_TRUE);
    if (!hidd_is_deep_sleep_allowed())
    {
        wiced_start_timer(&hidd_sleep.allowDeepSleepTimer, milliseconds);
//        WICED_BT_TRACE("\ndeepSleep not allowed for %d milliseconds", milliseconds);
    }
    else
    {
//        WICED_BT_TRACE("\ndeepSleep allowed");
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Sleep permit query to check if sleep (normal or SDS) is allowed and sleep time
///
/// \param type - sleep poll type
///
/// \return   sleep permission or sleep time, depending on input param
////////////////////////////////////////////////////////////////////////////////////////////////////////////
static uint32_t HIDD_sleep_handler( wiced_sleep_poll_type_t type )
{
    uint32_t ret = WICED_SLEEP_NOT_ALLOWED;

#if SLEEP_ALLOWED
    switch(type)
    {
    case WICED_SLEEP_POLL_TIME_TO_SLEEP:
        //query application for sleep time
        ret = hidd_sleep.p_app_sleep_handler ? hidd_sleep.p_app_sleep_handler(type) : WICED_SLEEP_MAX_TIME_TO_SLEEP;

 #if SFI_DEEP_SLEEP
        // In 20835, sfi CS may contains glitch that wakes up Flash result in high current. Apply workaround to put sfi into powerdown
        if ((ret == WICED_SLEEP_MAX_TIME_TO_SLEEP) && ( pmu_attemptSleepState == 5 ))
        {
            sfi_exit_deep_power_down(FALSE);
            sfi_enter_deep_power_down();
        }
 #endif
        break;

    case WICED_SLEEP_POLL_SLEEP_PERMISSION:
 #if SLEEP_ALLOWED > 1
        //query application for sleep permit
        ret = (hidd_sleep.p_app_sleep_handler) ? hidd_sleep.p_app_sleep_handler(type) : WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;

        // if we are allowed to shutdown, check further if we really should shutdown
        if ( (ret == WICED_SLEEP_ALLOWED_WITH_SHUTDOWN)
  #ifdef FATORY_TEST_SUPPORT
              && !force_sleep_in_HID_mode
  #endif
           )
        {
            // any of the following is true, we don't allow shutdown
            if ( !hidd_is_deep_sleep_allowed()
                 || wiced_hidd_is_transport_detection_polling_on()
  #ifdef BLE_SUPPORT
                 || (blelink.second_conn_state == BLEHIDLINK_2ND_CONNECTION_PENDING)
  #endif
  #if defined(BR_EDR_SUPPORT) && is_20835Family
                 //due to sniff+SDS is not supported in core FW, at this time, only allow SDS when disconnected
                 || (bt_hidd_link.subState != HIDLINK_DISCONNECTED)
  #endif
  #ifdef OTA_FIRMWARE_UPGRADE
                 || wiced_ota_fw_upgrade_is_active()
  #endif
              )
           {
               // change to no shutdown
               ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
           }
       }

       // if we are shutting down, prepare for shutdown
       if (ret == WICED_SLEEP_ALLOWED_WITH_SHUTDOWN)
       {
// if support epds, we need to determine if it should enter epds or hidoff
  #ifdef SUPPORT_EPDS
           if (!wiced_hidd_link_is_disconnected() || !hidd_sleep.allowHIDOFF)
           {
               // enter ePDS
               g_foundation_config_PMUflags &= ~PMU_CONFIG_FLAGS_ENABLE_SDS;
           }
           else
           {
               static uint8_t showHIDOFF = 1;
               if (showHIDOFF)
               {
                   showHIDOFF = 0;
                   WICED_BT_TRACE("\nHIDOFF");
               }
               /* allow ePDS */
               g_foundation_config_PMUflags |= PMU_CONFIG_FLAGS_ENABLE_SDS;
           }
  #endif
  #if is_SDS_capable
            hidd_link_aon_action_handler(HIDD_LINK_SAVE_TO_AON);
  #endif
        }
 #else // SLEEP_ALLOWED == 1
        ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
 #endif
        break;
    }
#endif // SLEEP_ALLOWED

    if(ret == WICED_SLEEP_ALLOWED_WITH_SHUTDOWN)
    {
        WICED_BT_TRACE("\nSDS");
    }

    return ret;
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// Abstract link layer initialize
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_sleep_configure( wiced_sleep_config_t * hidd_link_sleep_config )
{
    // Take over sleep handler
    hidd_sleep.p_app_sleep_handler = hidd_link_sleep_config->sleep_permit_handler;
    hidd_link_sleep_config->sleep_permit_handler = HIDD_sleep_handler;

    //configure sleep
#if !defined(CYW55572A1) // 55572A1 wiced_sleep_configure() is not ready
    wiced_sleep_configure( hidd_link_sleep_config );
#endif
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void hidd_allowed_hidoff( wiced_bool_t en )
{
    hidd_sleep.allowHIDOFF = en ? 1 : 0;
}

////////////////////////////////////////////////////////////////////////////////
/// hidd_sleep_init
////////////////////////////////////////////////////////////////////////////////
void hidd_sleep_init()
{
    //timer to allow shut down sleep (SDS)
    wiced_init_timer( &hidd_sleep.allowDeepSleepTimer, allow_sleep_timer_cb, 0, WICED_MILLI_SECONDS_TIMER );
}
