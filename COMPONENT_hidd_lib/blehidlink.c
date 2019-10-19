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
* File Name: blehidlink.c
*
* Abstract: This file implements the BLE HID application transport
*
* Functions:
*
*******************************************************************************/
#ifndef BT_HIDD_ONLY
#include "spar_utils.h"
#include "blehidlink.h"
#include "blehostlist.h"
#include "wiced_hal_mia.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_event.h"
#include "wiced_bt_l2c.h"
#include "wiced_hal_batmon.h"
#include "wiced_hal_nvram.h"
#include "wiced_memory.h"
#include "blehidhci.h"

tBleHidLink ble_hidd_link = {{15, 15, 20, 300}, 1, 0, };

uint16_t blehostlist_flags = 0;
wiced_bt_device_link_keys_t  blehostlist_link_keys = {0, };
wiced_bool_t blehidlink_connection_param_updated = WICED_FALSE;
uint8_t wake_from_SDS_timer_timeout = 0;
wiced_bt_ble_advert_mode_t app_adv_mode = BTM_BLE_ADVERT_OFF;

#ifdef FATORY_TEST_SUPPORT
uint8_t factory_mode = 0;
extern uint8_t force_sleep_in_HID_mode;
#endif

const wiced_bt_cfg_settings_t * wiced_bt_hid_cfg_settings_ptr = NULL;

void blehidlink_init();
void blehidlink_determineNextState(void);
void blehidlink_connParamUpdate_timerCb( uint32_t arg);
void blehidlink_reconnect_timerCb( uint32_t arg);
void blehidlink_allowsleeptimerCb( uint32_t arg);
void blehidlink_connectionIdle_timerCb(INT32 args, UINT32 overTimeInUs);
#ifdef ALLOW_SDS_IN_DISCOVERABLE
void blehidlink_stateswitchtimerCb( uint32_t arg);
void blehidlink_discoverabletimerCb(INT32 args, UINT32 overTimeInUs);
#endif
void blehidlink_enterDiscoverable(uint32_t SDS_allow);
void blehidlink_enterConnected(void);
void blehidlink_enterDisconnected(void);
void blehidlink_enterReconnecting(void);
void blehidlink_setState(uint8_t newState);
uint32_t blehidlink_sleep_handler(wiced_sleep_poll_type_t type );

#ifdef EASY_PAIR
void blehidlink_easyPair(uint32_t arg);
void blehidlink_easyPair_timerCb( uint32_t arg);
#endif

#ifdef DUAL_MODE_HIDD
extern wiced_bt_transport_t active_transport;
#endif

//Local identity key ID
#define  VS_LOCAL_IDENTITY_ID WICED_NVRAM_VSID_START

wiced_sleep_config_t    blehidlink_sleep_config = {
    WICED_SLEEP_MODE_NO_TRANSPORT,  //sleep_mode
    0,                              //host_wake_mode
    0,                              //device_wake_mode
    WICED_SLEEP_WAKE_SOURCE_GPIO | WICED_SLEEP_WAKE_SOURCE_KEYSCAN | WICED_SLEEP_WAKE_SOURCE_QUAD,  //device_wake_source
    255,                            //must set device_wake_gpio_num to 255 for WICED_SLEEP_MODE_NO_TRANSPORT
    blehidlink_sleep_handler        //sleep_permit_handler
};
wiced_sleep_allow_check_callback blehidlink_registered_app_sleep_handler = NULL;

PLACE_DATA_IN_RETENTION_RAM blehid_aon_save_content_t   blehid_aon_data;

/////////////////////////////////////////////////////////////////////////////////////////////
/// Abstract link layer initialize
/////////////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_init()
{
    //Setup Battery Service
    wiced_hal_batmon_init();

    blehidlink_init();

    //configure sleep
    wiced_sleep_configure( &blehidlink_sleep_config );

    blehidlink_determineNextState();

}

/////////////////////////////////////////////////////////////////////////////////////////////
/// ble hid link init
/////////////////////////////////////////////////////////////////////////////////////////////
void blehidlink_init()
{
    //read HID host from NVRAM
    wiced_ble_hidd_host_info_init();

#ifdef DUAL_MODE_HIDD
    if(wiced_ble_hidd_host_info_is_bonded())
    {
        WICED_BT_TRACE("BT_TRANSPORT_LE!\n");
        active_transport = BT_TRANSPORT_LE;
    }
#endif

    ///connection idle timer that can be supported in uBCS mode
    osapi_createTimer(&ble_hidd_link.conn_idle_timer, blehidlink_connectionIdle_timerCb, 0);

    //timer to auto reconnect when disconnected
    wiced_init_timer( &ble_hidd_link.reconnect_timer, blehidlink_reconnect_timerCb, 0, WICED_MILLI_SECONDS_TIMER );

#ifdef EASY_PAIR
    //timer for easy pair
    wiced_init_timer( &ble_hidd_link.easyPair_timer, blehidlink_easyPair_timerCb, 0, WICED_MILLI_SECONDS_TIMER );
#endif

    //timer to allow shut down sleep (SDS)
    wiced_init_timer( &ble_hidd_link.allowSDS_timer, blehidlink_allowsleeptimerCb, 0, WICED_MILLI_SECONDS_TIMER );

#ifdef ALLOW_SDS_IN_DISCOVERABLE
    ble_hidd_link.state_switch_timeout_in_ms = 1000; // 1 seconds

    /// timer to switch from DISCOVERABLE to BLEHIDLINK_ADVERTISING_IN_uBCS_UNDIRECTED
    wiced_init_timer( &ble_hidd_link.state_switch_timer, blehidlink_stateswitchtimerCb, 0, WICED_MILLI_SECONDS_TIMER );

    ///discoverable timer that can be supported in uBCS mode
    osapi_createTimer(&ble_hidd_link.discoverable_timer, blehidlink_discoverabletimerCb, 0);
#endif

#ifdef AUTO_RECONNECT
    ble_hidd_link.auto_reconnect = WICED_TRUE;
#endif

}

/////////////////////////////////////////////////////////////////////////////////////////////
/// determine next action on cold boot
/////////////////////////////////////////////////////////////////////////////////////////////
void blehidlink_determineNextState_on_cold_boot(void)
{
    //if low battery shutdown, enter disconnected
    if (wiced_hal_batmon_is_low_battery_shutdown())
    {
        blehidlink_enterDisconnected();
        return;
    }

    if(wiced_ble_hidd_host_info_is_bonded())
    {
        WICED_BT_TRACE("bonded info in NVRAM\n");

#ifdef AUTO_RECONNECT
        if (ble_hidd_link.auto_reconnect && !wiced_hal_batmon_is_low_battery_shutdown())
        {
            wiced_start_timer(&ble_hidd_link.reconnect_timer,10000); //auto reconnect after 10 seconds.
        }
#endif
        blehidlink_enterDisconnected();

#ifdef START_ADV_WHEN_POWERUP_NO_CONNECTED
        wiced_ble_hidd_host_info_add_to_resolving_list();
        blehidlink_enterReconnecting();
#endif
    }
    else
    {
        blehidlink_enterDisconnected();

#ifdef START_ADV_WHEN_POWERUP_NO_CONNECTED
 #ifdef ALLOW_SDS_IN_DISCOVERABLE
        //For cold boot, give it more time before allowing SDS; otherwise, it might reset.
        ble_hidd_link.state_switch_timeout_in_ms = 5000; // 5 seconds. For cold boot, give it more time before allowing SDS; otherwise, it might reset.
 #endif
        blehidlink_enterDiscoverable(WICED_TRUE);
#endif
    }
}

extern BOOL8 btsnd_hcic_ble_set_adv_enable (UINT8 adv_enable);

/////////////////////////////////////////////////////////////////////////////////////////////
/// determine next action when wake from SDS
/////////////////////////////////////////////////////////////////////////////////////////////
void blehidlink_determineNextState_on_wake_from_SDS(void)
{
    //restore embeded controller info for the LE link (peer device info, bonded, encrypted, connection parameters etc.)
    memcpy(&emConInfo_devInfo, &ble_hidd_link.resume_emconinfo, sizeof(EMCONINFO_DEVINFO));

    //check if osapi app timer timeout
    if (ble_hidd_link.osapi_app_timer_running)
    {
        uint64_t time_passed_in_ms = (clock_SystemTimeMicroseconds64() - ble_hidd_link.osapi_app_timer_start_instant)/1000;
        //is it advertising timer?
        if (ble_hidd_link.osapi_app_timer_running & BLEHIDLINK_ADV_CONNECTABLE_UNDIRECTED_TIMER)
        {
            // if time passed more than 60 seconds (adv timer timeout value)
            if (time_passed_in_ms >= 60000)
            {
                WICED_BT_TRACE("discoverable timer timeout!!\n");
                wake_from_SDS_timer_timeout = BLEHIDLINK_ADV_CONNECTABLE_UNDIRECTED_TIMER | 1;
            }
        }
        //is it connection idle timer
        else if (ble_hidd_link.osapi_app_timer_running & BLEHIDLINK_CONNECTION_IDLE_TIMER)
        {
            WICED_BT_TRACE("ble_hidd_link.conn_idle_timeout=%d, time_passed_in_ms=%d\n", ble_hidd_link.conn_idle_timeout, (uint32_t)time_passed_in_ms);
            // if time passed more than connection idle timeout value
            if ((time_passed_in_ms >= ble_hidd_link.conn_idle_timeout*1000) || ((ble_hidd_link.conn_idle_timeout - time_passed_in_ms/1000) <= 1))
            {
                WICED_BT_TRACE("connection idle timer timeout!!\n");
                wake_from_SDS_timer_timeout = BLEHIDLINK_CONNECTION_IDLE_TIMER | 1;
            }
            else
            {
                uint64_t remaining_time_in_ms = ble_hidd_link.conn_idle_timeout*1000 - time_passed_in_ms;
                //WICED_BT_TRACE("remaining_time_in_ms = %d\n", (uint32_t)remaining_time_in_ms);
                //restart connection idle timer w/remaining time
                osapi_activateTimer( &ble_hidd_link.conn_idle_timer, remaining_time_in_ms*1000); //timout in micro seconds.
            }
        }

        ble_hidd_link.osapi_app_timer_running = 0;
    }

#if defined(ENDLESS_LE_ADVERTISING_WHILE_DISCONNECTED) || defined(ALLOW_SDS_IN_DISCOVERABLE)
    if ((BLEHIDLINK_ADVERTISING_IN_uBCS_DIRECTED == ble_hidd_link.resumeState) ||
        (BLEHIDLINK_ADVERTISING_IN_uBCS_UNDIRECTED == ble_hidd_link.resumeState))
    {
        //stop advertising.
        //NOTE: wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF) can't be used to stop advertising here. Due to wiced stack didn't save adv status before/exit SDS.
        btsnd_hcic_ble_set_adv_enable (BTM_BLE_ADVERT_OFF);

        //check if wake up due to receiving LE connect request
        if (wiced_blehidd_is_wakeup_from_conn_req())
        {
            WICED_BT_TRACE("wake from CONNECT req\n");
            if (!wiced_hal_batmon_is_low_battery_shutdown())
            {
                if (wiced_ble_hidd_host_info_is_bonded() && (BLEHIDLINK_ADVERTISING_IN_uBCS_DIRECTED == ble_hidd_link.resumeState))
                {
                    wiced_ble_hidd_host_info_add_to_resolving_list();
                    blehidlink_enterReconnecting();
                }
                else
                {
 #ifdef WHITE_LIST_FOR_ADVERTISING
                    //if advertising white list is enabled before enter SDS
                    if (wiced_ble_hidd_host_info_is_bonded() && ble_hidd_link.adv_white_list_enabled)
                    {
                        uint8_t *bonded_bdadr = (uint8_t *)wiced_ble_hidd_host_info_get_bdaddr();

                        //add to white list
                        wiced_bt_ble_update_advertising_white_list(WICED_TRUE, bonded_bdadr);

                        //update advertising filer policy to use white list to filter scan and connect request
                        wiced_btm_ble_update_advertisement_filter_policy(ble_hidd_link.adv_white_list_enabled);
                    }
 #endif
                    blehidlink_enterDiscoverable(WICED_FALSE);
                }
            }
        }
        else
        {
            WICED_BT_TRACE("set Disconnected state\n");
            blehidlink_setState(BLEHIDLINK_DISCONNECTED);
        }
    }
    else
#endif
    {
        //set subState to resumeState
        blehidlink_setState(ble_hidd_link.resumeState);
    }

    if ((BLEHIDLINK_DISCONNECTED == ble_hidd_link.subState) && !wiced_hal_batmon_is_low_battery_shutdown())
    {
        //poll user activity and action accordingly
        if(ble_hidd_link.pollReportUserActivityCallback)
        {
            ble_hidd_link.pollReportUserActivityCallback();
        }

#if defined(ENDLESS_LE_ADVERTISING_WHILE_DISCONNECTED) || defined(ALLOW_SDS_IN_DISCOVERABLE)
        //if no user activity and not wake up due to application timer timeout. restart adv again
        if ((BLEHIDLINK_DISCONNECTED == ble_hidd_link.subState) && !wake_from_SDS_timer_timeout)
        {
 #ifdef ENDLESS_LE_ADVERTISING_WHILE_DISCONNECTED
            //if  it is bonded, start low duty cycle directed advertising again.
            if (wiced_ble_hidd_host_info_is_bonded() && (BLEHIDLINK_ADVERTISING_IN_uBCS_DIRECTED == ble_hidd_link.resumeState))
            {
                uint8_t *bdAddr;
                uint8_t bdAddrType;


                if (wiced_ble_hidd_host_info_get_first_host(&bdAddr,&bdAddrType))
                {
                    //NOTE!!! wiced_bt_start_advertisement could modify the value of bdAddr, so MUST use a copy.
                    uint8_t tmp_bdAddr[BD_ADDR_LEN];
                    memcpy(tmp_bdAddr, bdAddr, BD_ADDR_LEN);

                    // start high duty cycle directed advertising.
                    if (wiced_bt_start_advertisements(BTM_BLE_ADVERT_DIRECTED_LOW, bdAddrType, tmp_bdAddr))
                    {
                        WICED_BT_TRACE("Failed to start low duty cycle directed advertising!!!\n");
                    }

                    blehidlink_setState(BLEHIDLINK_ADVERTISING_IN_uBCS_DIRECTED);
                }
                else
                {
                    WICED_BT_TRACE("Fatal!!! we shouldn't get here!!\n");
                }
            }
            else
 #endif
            {
 #ifdef ALLOW_SDS_IN_DISCOVERABLE
            blehidlink_enterDiscoverable(WICED_TRUE);
 #endif
            }
        }
#endif
    }
    else if ((BLEHIDLINK_CONNECTED == ble_hidd_link.subState) && !wiced_hal_batmon_is_low_battery_shutdown())
    {
        if (wake_from_SDS_timer_timeout & BLEHIDLINK_CONNECTION_IDLE_TIMER)
        {
            //disconnect the link
            wiced_ble_hidd_link_disconnect();
        }
    }

}

/////////////////////////////////////////////////////////////////////////////////////////////
/// determine next action on cold boot or wake from SDS
/////////////////////////////////////////////////////////////////////////////////////////////
void blehidlink_determineNextState(void)
{
    if(!wiced_hal_mia_is_reset_reason_por())
    {
        WICED_BT_TRACE("wake from SDS\n");
        blehidlink_determineNextState_on_wake_from_SDS();
    }
    else
    {
        WICED_BT_TRACE("cold boot\n");
        blehidlink_determineNextState_on_cold_boot();
    }

    //always reset to 0
    wake_from_SDS_timer_timeout = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// LE connection up indication
/////////////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_connected(void)
{
    if (ble_hidd_link.conn_idle_timeout)
    {
    // start the connection idle timer
    osapi_activateTimer( &ble_hidd_link.conn_idle_timer, ble_hidd_link.conn_idle_timeout * 1000000UL); //timout in micro seconds.
    ble_hidd_link.osapi_app_timer_start_instant = clock_SystemTimeMicroseconds64();
    ble_hidd_link.osapi_app_timer_running |= BLEHIDLINK_CONNECTION_IDLE_TIMER;
    ble_hidd_link.osapi_app_timer_running |= 1;
    }

    blehidlink_enterConnected();
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// LE connection down indication
/////////////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_disconnected(void)
{
    //stop the connection idle timer
    osapi_deactivateTimer(&ble_hidd_link.conn_idle_timer);

    ble_hidd_link.osapi_app_timer_running &= ~BLEHIDLINK_CONNECTION_IDLE_TIMER;
    if ((ble_hidd_link.osapi_app_timer_running >> 1) == 0)
    {
        ble_hidd_link.osapi_app_timer_running = 0; // no more application osapi timer is running
    }

#ifdef AUTO_RECONNECT
    //reconnect back if bonded with host and not in the process of lowbattery shut down
    //and disconnect is not due to virtual cable unplug
    if(ble_hidd_link.auto_reconnect && wiced_ble_hidd_host_info_is_bonded() && !wiced_hal_batmon_is_low_battery_shutdown() && !ble_hidd_link.pendingStateTransiting)
        wiced_start_timer(&ble_hidd_link.reconnect_timer,500); //auto reconnect in 500 ms
#endif
    //clear link encrypted flag when disconnected
    wiced_blehidd_set_link_encrypted_flag(WICED_FALSE);

    blehidlink_enterDisconnected();
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// LE advertising (except high duty cycle directed adv) stop indication
/////////////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_adv_stop(void)
{
    WICED_BT_TRACE("wiced_ble_hidd_link_adv_stop\n");

#ifdef ALLOW_SDS_IN_DISCOVERABLE
    //stop discoverable timer
    osapi_deactivateTimer(&ble_hidd_link.discoverable_timer);

    ble_hidd_link.osapi_app_timer_running &= ~BLEHIDLINK_ADV_CONNECTABLE_UNDIRECTED_TIMER;
    if ((ble_hidd_link.osapi_app_timer_running >> 1) == 0)
    {
        ble_hidd_link.osapi_app_timer_running = 0; // no more application osapi timer is running
    }
#endif

    //if not connected, must be adv timeout
    if (!wiced_ble_hidd_link_is_connected())
    {
        blehidlink_setState(BLEHIDLINK_DISCONNECTED);
    }

    if (ble_hidd_link.second_conn_state == BLEHIDLINK_2ND_CONNECTION_ALLOWED)
    {
        //set 2nd connection state to default
        ble_hidd_link.second_conn_state = BLEHIDLINK_2ND_CONNECTION_NOT_ALLOWED;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// High duty cycle directed advertising stop indication
/////////////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_directed_adv_stop(void)
{
    WICED_BT_TRACE("blehidlink_DirectedAdvStop\n");

#if defined(ENDLESS_LE_ADVERTISING_WHILE_DISCONNECTED) && !defined(SUPPORT_EPDS)
    blehidlink_setState(BLEHIDLINK_ADVERTISING_IN_uBCS_DIRECTED);
#else
 #ifdef WHITE_LIST_FOR_ADVERTISING
    //update advertising filer policy to use white list to filter scan and connect request
    wiced_btm_ble_update_advertisement_filter_policy(0x03);

    ble_hidd_link.adv_white_list_enabled = 0x03;
 #endif

    // start undirected connectable advertising.
    if (wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL))
        WICED_BT_TRACE("Failed to undirected connectable advertising!!!\n");

 #ifdef ALLOW_SDS_IN_DISCOVERABLE
    osapi_activateTimer( &ble_hidd_link.discoverable_timer, 60000000UL); //60 seconds. timout in micro seconds.
    ble_hidd_link.osapi_app_timer_start_instant = clock_SystemTimeMicroseconds64();
    ble_hidd_link.osapi_app_timer_running |= BLEHIDLINK_ADV_CONNECTABLE_UNDIRECTED_TIMER;
    ble_hidd_link.osapi_app_timer_running |= 1;

    blehidlink_setState(BLEHIDLINK_ADVERTISING_IN_uBCS_UNDIRECTED);
 #endif
#endif
}

/////////////////////////////////////////////////////////////////////////////////
/// register application sleep permit handler
///
/// \param cb - pointer to application callback function
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_register_sleep_permit_handler(wiced_sleep_allow_check_callback sleep_handler)
{
    blehidlink_registered_app_sleep_handler = sleep_handler;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Sleep permit query to check if sleep (normal or SDS) is allowed and sleep time
///
/// \param type - sleep poll type
///
/// \return   sleep permission or sleep time, depending on input param
////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t blehidlink_sleep_handler(wiced_sleep_poll_type_t type )
{
    uint32_t ret = WICED_SLEEP_NOT_ALLOWED;

    switch(type)
    {
        case WICED_SLEEP_POLL_TIME_TO_SLEEP:
            ret = WICED_SLEEP_MAX_TIME_TO_SLEEP;

            //query application for sleep time
            if (blehidlink_registered_app_sleep_handler)
                ret = blehidlink_registered_app_sleep_handler(type);

            break;

        case WICED_SLEEP_POLL_SLEEP_PERMISSION:
            ret = WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;

            //query application for sleep permit first
            if (blehidlink_registered_app_sleep_handler)
                ret = blehidlink_registered_app_sleep_handler(type);

            if (!ble_hidd_link.allowSDS)
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;

            if (ble_hidd_link.second_conn_state == BLEHIDLINK_2ND_CONNECTION_PENDING)
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;

            if (wiced_hidd_is_transport_detection_polling_on()
#ifdef FATORY_TEST_SUPPORT
                && !force_sleep_in_HID_mode
#endif
                )
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;

            //save to AON before entering SDS
            if (ret == WICED_SLEEP_ALLOWED_WITH_SHUTDOWN)
            {
                wiced_ble_hidd_link_aon_action_handler(BLEHIDLINK_SAVE_TO_AON);
            }

            break;

    }

    return ret;
}

/////////////////////////////////////////////////////////////////////////////////
/// check if it is currently connected
///
/// \return TRUE/FALSE
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t  wiced_ble_hidd_link_is_connected(void)
{
    return (ble_hidd_link.subState == BLEHIDLINK_CONNECTED);
}

/////////////////////////////////////////////////////////////////////////////////
/// Check if it is discoverable (i.e. connetable undirected advertising)
///
/// \return TRUE/FALSE
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t  wiced_ble_hidd_link_is_discoverable(void)
{
    return (ble_hidd_link.subState == BLEHIDLINK_DISCOVERABLE);
}

/////////////////////////////////////////////////////////////////////////////////
/// Connect
/// As LE slave, it means start LE advertising
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_connect(void)
{
    //add host to Resolving List after key press interrupt is handled to avoid delay
    wiced_ble_hidd_host_info_add_to_resolving_list();

    if(wiced_ble_hidd_host_info_is_bonded())
    {
        WICED_BT_TRACE("Bonded: SubState:%d\n",ble_hidd_link.subState);
        switch(ble_hidd_link.subState)
        {
            case BLEHIDLINK_DISCONNECTED:
                blehidlink_enterReconnecting();
                break;
            default:
                 //WICED_BT_TRACE("wiced_ble_hidd_link_connect(bonded):%d\n",ble_hidd_link.subState);
                break;
        }
    }
    else
    {
        switch(ble_hidd_link.subState)
        {
            case BLEHIDLINK_DISCONNECTED:
                blehidlink_enterDiscoverable(WICED_TRUE);
                break;
            default:
                //WICED_BT_TRACE("wiced_ble_hidd_link_connect(not bonded):%d\n",ble_hidd_link.subState);
                break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// Disconnect
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_disconnect(void)
{
    wiced_bt_gatt_disconnect(ble_hidd_link.gatts_conn_id);
}

/////////////////////////////////////////////////////////////////////////////////
/// Add new observer for link state changed.
/// Whenever link state changed, the observer will be notified.
///
/// \param observer - pointer to the callback function
///
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_add_state_observer(wiced_ble_hidd_state_change_callback_t* observer)
{
    LinkStateObserver* ob = (LinkStateObserver*)wiced_memory_permanent_allocate(sizeof(LinkStateObserver));

    // If allocation was OK, put this registration in the SL
    if(ob)
    {
        ob->callback = observer;
        ob->next = ble_hidd_link.firstStateObserver;
        ble_hidd_link.firstStateObserver = ob;
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// become disconnected
/////////////////////////////////////////////////////////////////////////////////
void blehidlink_enterDisconnected(void)
{
    WICED_BT_TRACE("enterDisconnected\n");

#ifdef WHITE_LIST_FOR_ADVERTISING
    //clear white list
    wiced_bt_ble_clear_white_list();
#endif

#ifdef ENDLESS_LE_ADVERTISING_WHILE_DISCONNECTED
    blehidlink_enterReconnecting();
#else
    blehidlink_setState(BLEHIDLINK_DISCONNECTED);
#endif

    if (ble_hidd_link.pendingStateTransiting)
    {
        ble_hidd_link.pendingStateTransiting = 0;
        if (ble_hidd_link.stateTransitingFunc)
        {
            ble_hidd_link.stateTransitingFunc(1);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// become discoverable. i.e. start connectable undirected advertising
/////////////////////////////////////////////////////////////////////////////////
void blehidlink_enterDiscoverable(uint32_t SDS_allow)
{
#ifdef FATORY_TEST_SUPPORT
    //do not start LE advertising when factory_mode == 1
    if (factory_mode)
        return;
#endif

    //if low battery shutdown, do nothing
    if (wiced_hal_batmon_is_low_battery_shutdown())
    {
        return;
    }

    WICED_BT_TRACE("enterDiscoverable\n");

    // start undirected connectable advertising.
    if (wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL))
        WICED_BT_TRACE("Failed to start undirected connectable advertising!!!\n");

    blehidlink_setState(BLEHIDLINK_DISCOVERABLE);

#ifdef ALLOW_SDS_IN_DISCOVERABLE
    osapi_activateTimer( &ble_hidd_link.discoverable_timer, 60000000UL); //60 seconds. timout in micro seconds.
    ble_hidd_link.osapi_app_timer_start_instant = clock_SystemTimeMicroseconds64();
    ble_hidd_link.osapi_app_timer_running |= BLEHIDLINK_ADV_CONNECTABLE_UNDIRECTED_TIMER;
    ble_hidd_link.osapi_app_timer_running |= 1;

    if (SDS_allow)
    {
        //switch to advertising w/ SDS after 1 second
        if (wiced_is_timer_in_use(&ble_hidd_link.state_switch_timer))
        {
            wiced_stop_timer(&ble_hidd_link.state_switch_timer);
        }
        wiced_start_timer(&ble_hidd_link.state_switch_timer,ble_hidd_link.state_switch_timeout_in_ms);
    }

    //set it back to 1 second
    ble_hidd_link.state_switch_timeout_in_ms = 1000; // 1 second
#endif

}

/////////////////////////////////////////////////////////////////////////////////
///allow discoverable while in connection, i.e. start connectable undirected advertisingnected
/////////////////////////////////////////////////////////////////////////////////
void blehidlink_allowDiscoverable(void)
{
    WICED_BT_TRACE("allowDiscoverable\n");

    //do nothing if we are in DISCOVERABLE state alreay
    if (ble_hidd_link.second_conn_state || (ble_hidd_link.subState == BLEHIDLINK_DISCOVERABLE))
        return;

    //stop advertising anyway
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);

#ifdef WHITE_LIST_FOR_ADVERTISING
    //set advertising filer policy to default (white list is not used)
    wiced_btm_ble_update_advertisement_filter_policy(0);

    ble_hidd_link.adv_white_list_enabled = 0;
#endif

    if (ble_hidd_link.subState != BLEHIDLINK_CONNECTED)
    {
        blehidlink_enterDiscoverable(WICED_TRUE);
    }
    else
    {
        //save the existing connection's info
        memcpy(&ble_hidd_link.existing_emconinfo, &emConInfo_devInfo, sizeof(EMCONINFO_DEVINFO));

        // start undirected connectable advertising.
        if (wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL))
            WICED_BT_TRACE("Failed to start undirected connectable advertising!!!\n");

#ifdef ALLOW_SDS_IN_DISCOVERABLE
        //start discoverable timer in normal mode
        osapi_activateTimer( &ble_hidd_link.discoverable_timer, 60000000UL); //60 seconds. timout in micro seconds.
        ble_hidd_link.osapi_app_timer_start_instant = clock_SystemTimeMicroseconds64();
        ble_hidd_link.osapi_app_timer_running |= BLEHIDLINK_ADV_CONNECTABLE_UNDIRECTED_TIMER;
        ble_hidd_link.osapi_app_timer_running = 1;
#endif
        ble_hidd_link.second_conn_state = BLEHIDLINK_2ND_CONNECTION_ALLOWED;
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// set new link state and notify observers.
///
/// \param newState - the new link state
/////////////////////////////////////////////////////////////////////////////////
void blehidlink_setState(uint8_t newState)
{
    LinkStateObserver* tmpObs = ble_hidd_link.firstStateObserver;

    if(newState != ble_hidd_link.subState)
    {
        ble_hidd_link.subState = newState;

#ifdef DUAL_MODE_HIDD
        //if current active transport is BR/EDR, do not notify observer
        if (active_transport == BT_TRANSPORT_BR_EDR)
            return;
#endif

        while(tmpObs)
        {
            if(tmpObs->callback)
            {
                tmpObs->callback(newState);
            }

            tmpObs = tmpObs->next;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// Become connected.
/// Start SMP bonding/pairing process if not bonded but encryption is required
/////////////////////////////////////////////////////////////////////////////////
void blehidlink_enterConnected(void)
{
    uint8_t *bdAddr;
    uint8_t bdAddrType;


    WICED_BT_TRACE("blehidlink_enterConnected\n");
#ifdef ALLOW_SDS_IN_DISCOVERABLE
    //stop discoverable timer
    osapi_deactivateTimer(&ble_hidd_link.discoverable_timer);

    ble_hidd_link.osapi_app_timer_running &= ~BLEHIDLINK_ADV_CONNECTABLE_UNDIRECTED_TIMER;
    if ((ble_hidd_link.osapi_app_timer_running >> 1) == 0)
    {
        ble_hidd_link.osapi_app_timer_running = 0; // no more application osapi timer is running
    }
#endif
    //check if match with bonded device bd_addr
    if (wiced_ble_hidd_host_info_get_number() && wiced_ble_hidd_host_info_get_first_host(&bdAddr,&bdAddrType))
    {
        if ((ble_hidd_link.gatts_peer_addr_type == bdAddrType) && (memcmp(bdAddr, ble_hidd_link.gatts_peer_addr, BD_ADDR_LEN)== 0)
            && wiced_ble_hidd_host_info_is_bonded())
        {
            WICED_BT_TRACE("set device bonded flag\n");
            // this is bonded device, we have the bonding info..
            wiced_blehidd_set_device_bonded_flag(WICED_TRUE);
        }
        else
        {
            WICED_BT_TRACE("clear device bonded flag\n");
            // not a bonded device.
            wiced_blehidd_set_device_bonded_flag(WICED_FALSE);
        }
    }

    blehidlink_setState(BLEHIDLINK_CONNECTED);
    // In any case, stop advertising
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);

    if(wiced_bt_hid_cfg_settings_ptr->security_requirement_mask)
    {
        if (!wiced_blehidd_is_device_bonded())
        {
                WICED_BT_TRACE("send security req: \n");
                wiced_bt_dev_sec_bond(ble_hidd_link.gatts_peer_addr, ble_hidd_link.gatts_peer_addr_type, BT_TRANSPORT_LE, 0, NULL);
        }
    }

}

/////////////////////////////////////////////////////////////////////////////////
/// Reconnecting to previous HID host.
/// i.e. start high duty cycle directed advertising
/////////////////////////////////////////////////////////////////////////////////
void blehidlink_enterReconnecting(void)
{
    uint8_t *bdAddr;
    uint8_t bdAddrType;
    uint8_t i;

    //if low battery shutdown, do nothing
    if (wiced_hal_batmon_is_low_battery_shutdown())
    {
        return;
    }

#ifdef FATORY_TEST_SUPPORT
    //do not start LE advertising when factory_mode == 1
    if (factory_mode)
        return;
#endif

    WICED_BT_TRACE("enterReconnecting\n");
    //if reconnect timer is running, stop it
    if (wiced_is_timer_in_use(&ble_hidd_link.reconnect_timer))
    {
        wiced_stop_timer(&ble_hidd_link.reconnect_timer);
    }

    if (wiced_ble_hidd_host_info_get_first_host(&bdAddr,&bdAddrType))
    {
        //NOTE!!! wiced_bt_start_advertisement could modify the value of bdAddr, so MUST use a copy.
        uint8_t tmp_bdAddr[BD_ADDR_LEN];
        memcpy(tmp_bdAddr, bdAddr, BD_ADDR_LEN);

#ifdef WHITE_LIST_FOR_ADVERTISING
        //add to white list
        wiced_bt_ble_update_advertising_white_list(WICED_TRUE, tmp_bdAddr);
#endif
        // start high duty cycle directed advertising.
        if (wiced_bt_start_advertisements(BTM_BLE_ADVERT_DIRECTED_HIGH, bdAddrType, tmp_bdAddr))
            WICED_BT_TRACE("Failed to start high duty cycle directed advertising!!!\n");

        blehidlink_setState(BLEHIDLINK_RECONNECTING);
    }
    else
    {
        blehidlink_enterDiscoverable(WICED_TRUE);
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// handler in MPAF(i.e. application) thread when application is polled
/////////////////////////////////////////////////////////////////////////////////
int blehidlink_pollTimerExpiredAction(void* unused)
{
    if(ble_hidd_link.pollReportUserActivityCallback && ble_hidd_link.appPoll_enabled)
    {
        ble_hidd_link.pollReportUserActivityCallback();
    }

    return 0;
}

/////////////////////////////////////////////////////////////////////////////////
/// handler in BCS thread when application is polled
/// It will invoke/serialize the handler in MPAF thread
///
/// \param task -don't care
/// \param context - don't care
/////////////////////////////////////////////////////////////////////////////////
void blehidlink_pollTimerExpiredNotice(void* task, uint32_t context)
{
    wiced_app_event_serialize(blehidlink_pollTimerExpiredAction, NULL);
}

/////////////////////////////////////////////////////////////////////////////////
/// Enable application poll
///
/// \param enable - WICED_TRUE/WICED_FALSE
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_enable_poll_callback(wiced_bool_t enable)
{
    if (ble_hidd_link.appPoll_enabled == enable)
        return;

    ble_hidd_link.appPoll_enabled = enable;
    wiced_hidd_register_callback_for_poll_event(BT_TRANSPORT_LE, wiced_blehidd_get_peer_addr(), enable, blehidlink_pollTimerExpiredNotice);
}

/////////////////////////////////////////////////////////////////////////////////
/// register application callback function when application is polled
///
/// \param cb - pointer to application callback function
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_register_poll_callback(blehidlink_no_param_fp *cb)
{
    ble_hidd_link.pollReportUserActivityCallback = cb;
}

/////////////////////////////////////////////////////////////////////////////////
/// send HID report as GATT notification
///
/// \param reportID - report ID
/// \param reportType - report type.
/// \param data - pointer to report data
/// \param length - length of the report data
///
/// \return 0 - successful
///         others - failed
/////////////////////////////////////////////////////////////////////////////////
uint8_t wiced_ble_hidd_link_send_report(uint8_t reportID, wiced_hidd_report_type_t reportType, uint8_t *data, uint8_t length)
{
    wiced_bt_gatt_status_t rptSentStatus = wiced_blehidd_send_report(ble_hidd_link.gatts_conn_id, reportID, reportType, data, length);

    if(rptSentStatus)
    {
        if (rptSentStatus == WICED_BT_GATT_CCC_CFG_ERR)
        {
            WICED_BT_TRACE("reportID:%d notification FALSE\n",reportID);
        }
        else
        {
            // Something did not match
            WICED_BT_TRACE("SendRpt failed, %d, %d, %d, 0x%x\n", reportID, length, reportType, rptSentStatus);
        }
    }

    //Start a timer to make sure the packet is sent over the air before going to HID Off
    //This is a work around while we determine where we should prevent HID Off
    if (wiced_is_timer_in_use(&ble_hidd_link.allowSDS_timer))
    {
        wiced_stop_timer(&ble_hidd_link.allowSDS_timer);
    }
    wiced_start_timer(&ble_hidd_link.allowSDS_timer,1000);// 1 second. timeout in ms
    ble_hidd_link.allowSDS = 0;


    // Whenever there is an activity, restart the connection idle timer
    if (ble_hidd_link.conn_idle_timeout)
    {
        osapi_activateTimer( &ble_hidd_link.conn_idle_timer, ble_hidd_link.conn_idle_timeout * 1000000UL); //timout in micro seconds.
        ble_hidd_link.osapi_app_timer_start_instant = clock_SystemTimeMicroseconds64();
        ble_hidd_link.osapi_app_timer_running |= BLEHIDLINK_CONNECTION_IDLE_TIMER;
        ble_hidd_link.osapi_app_timer_running |= 1;
    }

    return rptSentStatus;
}

/////////////////////////////////////////////////////////////////////////////////
/// virtual cable unplug.
/// This function will remove all HID host information and start connectable undirected advertising
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_virtual_cable_unplug(void)
{
    if (wiced_ble_hidd_host_info_is_bonded())
    {
        uint8_t *bonded_bdadr = wiced_ble_hidd_host_info_get_bdaddr ();
#ifdef WHITE_LIST_FOR_ADVERTISING
        //stop advertising anyway before white list operation
        wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);

        //remove from white list
        WICED_BT_TRACE("remove from white list : %B\n", bonded_bdadr);
        wiced_bt_ble_update_advertising_white_list(WICED_FALSE, bonded_bdadr);

        //clear whitlist
        wiced_bt_ble_clear_white_list();
#endif

        WICED_BT_TRACE("remove bonded device : %B\n", bonded_bdadr);
        wiced_bt_dev_delete_bonded_device(bonded_bdadr);

#ifdef DUAL_MODE_HIDD
        active_transport = 0;
#endif
    }

    WICED_BT_TRACE("Removing all bonded info\n");
    wiced_ble_hidd_host_info_delete_all();

    WICED_BT_TRACE("clear device bonded flag\n");
    wiced_blehidd_set_device_bonded_flag(WICED_FALSE);

#ifdef WHITE_LIST_FOR_ADVERTISING
    //set advertising filer policy to default (white list not used)
    wiced_btm_ble_update_advertisement_filter_policy(0);

    ble_hidd_link.adv_white_list_enabled = 0;
#endif

    if (ble_hidd_link.subState == BLEHIDLINK_CONNECTED)
    {

#ifdef EASY_PAIR
        ble_hidd_link.pendingStateTransiting = 1;
        ble_hidd_link.stateTransitingFunc = blehidlink_easyPair;
#else
        ble_hidd_link.pendingStateTransiting = 0;
        ble_hidd_link.stateTransitingFunc = NULL;
 #ifdef ALLOW_SDS_IN_DISCOVERABLE
        //For virtual cable unplug, give it more time before allowing SDS; otherwise, it might reset.
        ble_hidd_link.state_switch_timeout_in_ms = 5000; // 5 seconds.  give it more time before allowing SDS; otherwise, it might reset.
 #endif
#endif
        //disconnect the link
        wiced_ble_hidd_link_disconnect();
    }
    else if (ble_hidd_link.subState == BLEHIDLINK_RECONNECTING)
    {
        wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);

#ifdef EASY_PAIR
        blehidlink_easyPair(0);
#else
        blehidlink_enterDiscoverable(WICED_TRUE);
#endif

    }
    else if (ble_hidd_link.subState != BLEHIDLINK_DISCOVERABLE)
    {
#ifdef EASY_PAIR
        blehidlink_easyPair(0);
#else
        blehidlink_enterDiscoverable(WICED_TRUE);
#endif
    }

    //if reconnect timer is running, stop it
    if (wiced_is_timer_in_use(&ble_hidd_link.reconnect_timer))
    {
        wiced_stop_timer(&ble_hidd_link.reconnect_timer);
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// set ble HID link prefered conneciton parameters
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_set_preferred_conn_params(uint16_t conn_min_interval, uint16_t conn_max_interval, uint16_t slavelatency, uint16_t timeout)
{
    ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_INTERVAL_MIN]  = conn_min_interval;
    ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_INTERVAL_MAX]  = conn_max_interval;
    ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_SLAVE_LATENCY] = slavelatency;
    ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_TIMEOUT]       = timeout;
}

/////////////////////////////////////////////////////////////////////////////////
/// set ble HID link connection Idle timer timeout value in seconds (default is 0, i.e. no timeout)
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_set_connection_Idle_timeout_value(uint16_t value)
{
    ble_hidd_link.conn_idle_timeout  = value;
}

/////////////////////////////////////////////////////////////////////////////////
/// request asymmetric slave latency.
/// this is useful when master doesn't accept the connection parameter update req
/// slave can enable asymmetric slave latency to lower power consumption
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_set_slave_latency(uint16_t slaveLatencyinmS)
{
    UINT16 latency_plus_one = slaveLatencyinmS/wiced_blehidd_get_connection_interval() * 4/5;

    ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_SLAVE_LATENCY] = latency_plus_one - 1;
    ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_INTERVAL_MIN] =
    ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_INTERVAL_MAX] = wiced_blehidd_get_connection_interval();

    WICED_BT_TRACE("wiced_ble_hidd_link_set_slave_latency: interval=%d, slavelatency=%d\n",
               ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_INTERVAL_MIN],
               ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_SLAVE_LATENCY]);
    wiced_blehidd_set_asym_slave_latency(wiced_blehidd_get_connection_handle(), ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_SLAVE_LATENCY]);
}

/////////////////////////////////////////////////////////////////////////////////
/// handler when received LE Connection Update Complete event
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_conn_update_complete(void)
{
    WICED_BT_TRACE("ConnParamUpdate:Interval:%d, Latency:%d, Supervision TO:%d\n",
                 wiced_blehidd_get_connection_interval(),wiced_blehidd_get_slave_latency(),wiced_blehidd_get_supervision_timeout());

#ifndef  SKIP_CONNECT_PARAM_UPDATE_EVEN_IF_NO_PREFERED
    if ((wiced_blehidd_get_connection_interval() < ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_INTERVAL_MIN]) ||
        (wiced_blehidd_get_connection_interval() > ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_INTERVAL_MAX]) ||
        (wiced_blehidd_get_slave_latency() != ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_SLAVE_LATENCY]))
    {
#ifdef ASSYM_SLAVE_LATENCY
        //if actual slavelatency is smaller than desired slave latency, set asymmetric slave latency in the slave side
        if (wiced_blehidd_get_connection_interval()*(wiced_blehidd_get_slave_latency() + 1) <
                ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_INTERVAL_MIN] * (ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_SLAVE_LATENCY] + 1))
            wiced_ble_hidd_link_set_slave_latency(ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_INTERVAL_MIN]*(ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_SLAVE_LATENCY]+1)*5/4);
#else
        wiced_ble_hidd_link_conn_param_update();
#endif
        blehidlink_connection_param_updated = WICED_TRUE;
    }
#endif
}

/////////////////////////////////////////////////////////////////////////////////
/// request Connection Parameter Update
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_conn_param_update(void)
{
    if ((wiced_blehidd_get_connection_interval() < ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_INTERVAL_MIN]) ||
            (wiced_blehidd_get_connection_interval() > ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_INTERVAL_MAX]) ||
            (wiced_blehidd_get_slave_latency() != ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_SLAVE_LATENCY]))
    {
        WICED_BT_TRACE("send conn param request\n");
        wiced_bt_l2cap_update_ble_conn_params(ble_hidd_link.gatts_peer_addr,
                          ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_INTERVAL_MIN],
                          ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_INTERVAL_MAX],
                          ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_SLAVE_LATENCY],
                          ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_TIMEOUT]);
    }
    blehidlink_connection_param_updated = WICED_TRUE;
}

/////////////////////////////////////////////////////////////////////////////////
/// timeout handler
/// \param arg - don't care
/////////////////////////////////////////////////////////////////////////////////
void blehidlink_reconnect_timerCb( uint32_t arg)
{
    WICED_BT_TRACE("Reconnect\n");

    wiced_ble_hidd_link_connect();
}

/////////////////////////////////////////////////////////////////////////////////
/// timeout handler
/// \param arg - don't care
/// \param overTimeInUs - don't care
/////////////////////////////////////////////////////////////////////////////////
void blehidlink_connectionIdle_timerCb(INT32 args, UINT32 overTimeInUs)
{
    WICED_BT_TRACE("connection Idle timeout\n");

    //disconnect the link
    wiced_ble_hidd_link_disconnect();
}

////////////////////////////////////////////////////////////////////////////////////
/// save/restore contents to/from Always On Memory when entering/exiting SDS
///
/// \param type - BLEHIDLINK_SAVE_TO_AON or BLEHIDLINK_RESTORE_FROM_AON
////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_aon_action_handler(uint8_t  type)
{
    if (type == BLEHIDLINK_RESTORE_FROM_AON)
    {
        WICED_BT_TRACE("WICED_BT_AON_DRIVER_RESTORE\n");

        ble_hidd_link.resumeState = blehid_aon_data.blehidlink_state;
        ble_hidd_link.gatts_conn_id = blehid_aon_data.gatts_conn_id;
        ble_hidd_link.gatts_peer_addr_type = blehid_aon_data.gatts_peer_addr_type;
        memcpy(ble_hidd_link.gatts_peer_addr, blehid_aon_data.gatts_peer_addr, BD_ADDR_LEN);
        blehostlist_flags = blehid_aon_data.blehostlist_flags;

        //restore emconinfo
        ble_hidd_link.resume_emconinfo.connHandle = blehid_aon_data.emconinfo.connHandle;
        ble_hidd_link.resume_emconinfo.flag       = blehid_aon_data.emconinfo.flag;
        ble_hidd_link.resume_emconinfo.peerAddressType = blehid_aon_data.emconinfo.peerAddressType;
        memcpy(ble_hidd_link.resume_emconinfo.peerAddress, blehid_aon_data.emconinfo.peerAddress, BD_ADDR_LEN);
        ble_hidd_link.resume_emconinfo.connInterval = blehid_aon_data.emconinfo.connInterval;
        ble_hidd_link.resume_emconinfo.connLatency  = blehid_aon_data.emconinfo.connLatency;
        ble_hidd_link.resume_emconinfo.supervisionTimeout = blehid_aon_data.emconinfo.supervisionTimeout;

        //restore application timer info
        ble_hidd_link.osapi_app_timer_start_instant = blehid_aon_data.osapi_app_timer_start_instant;
        ble_hidd_link.osapi_app_timer_running = blehid_aon_data.osapi_app_timer_running;

#ifdef WHITE_LIST_FOR_ADVERTISING
        ble_hidd_link.adv_white_list_enabled = blehid_aon_data.adv_white_list_enabled;
#endif
    }
    else
    {
#if !defined(CYW20819A1) /* Slimboot is not supported in 20819A1 */
        // save all output GPIO values in the saved cfgs before entering uBCS mode
        wiced_hal_gpio_slimboot_reenforce_outputpin_value();

        blehid_aon_data.blehidlink_state = ble_hidd_link.subState;
        blehid_aon_data.gatts_conn_id = ble_hidd_link.gatts_conn_id;
        blehid_aon_data.gatts_peer_addr_type = ble_hidd_link.gatts_peer_addr_type;
        memcpy(blehid_aon_data.gatts_peer_addr, ble_hidd_link.gatts_peer_addr, BD_ADDR_LEN);
        blehid_aon_data.blehostlist_flags = blehostlist_flags;

        //save emconinfo
        blehid_aon_data.emconinfo.connHandle = emConInfo_devInfo.connHandle;
        blehid_aon_data.emconinfo.flag       = emConInfo_devInfo.flag;
        blehid_aon_data.emconinfo.peerAddressType = emConInfo_devInfo.peerAddressType;
        memcpy(blehid_aon_data.emconinfo.peerAddress, emConInfo_devInfo.peerAddress, BD_ADDR_LEN);
        blehid_aon_data.emconinfo.connInterval = emConInfo_devInfo.connInterval;
        blehid_aon_data.emconinfo.connLatency  = emConInfo_devInfo.connLatency;
        blehid_aon_data.emconinfo.supervisionTimeout = emConInfo_devInfo.supervisionTimeout;

        //save application timer info
        blehid_aon_data.osapi_app_timer_start_instant = ble_hidd_link.osapi_app_timer_start_instant;
        blehid_aon_data.osapi_app_timer_running = ble_hidd_link.osapi_app_timer_running;

#ifdef WHITE_LIST_FOR_ADVERTISING
        blehid_aon_data.adv_white_list_enabled = ble_hidd_link.adv_white_list_enabled;
#endif
#endif
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// timeout handler
///
/// \param arg - don't care
/////////////////////////////////////////////////////////////////////////////////
void blehidlink_allowsleeptimerCb( uint32_t arg)
{
    ble_hidd_link.allowSDS = 1;
}

#ifdef ALLOW_SDS_IN_DISCOVERABLE
/////////////////////////////////////////////////////////////////////////////////
/// timeout handler
///
/// \param arg - don't care
/////////////////////////////////////////////////////////////////////////////////
void blehidlink_stateswitchtimerCb( uint32_t arg)
{
    if (BLEHIDLINK_DISCOVERABLE == ble_hidd_link.subState)
    {
        blehidlink_setState(BLEHIDLINK_ADVERTISING_IN_uBCS_UNDIRECTED);
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// timeout handler
///
/// \param args - don't care
/// \param overTimeInUs - don't care
/////////////////////////////////////////////////////////////////////////////////
void blehidlink_discoverabletimerCb(INT32 args, UINT32 overTimeInUs)
{
    WICED_BT_TRACE("blehidlink_discoverabletimerCb!!!\n");
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
}
#endif

#ifdef EASY_PAIR
/////////////////////////////////////////////////////////////////////////////////
/// initialize easy pair data
/////////////////////////////////////////////////////////////////////////////////
void easyPairInit()
{
    ble_hidd_link.easyPair_deviceIndex = INVALID_INDEX;
    ble_hidd_link.easyPair.availableSlots = MAX_DEVICES;
    memset((void*)&ble_hidd_link.easyPair.device[0], 0, sizeof(EASY_PAIR_CANDIDATE)*MAX_DEVICES);
}

/////////////////////////////////////////////////////////////////////////////////
/// check if device existed or not
///
/// \param  p_scan_result - pointer to scan result data
///
/// \return 0 - not existed
///         1 - existed
/////////////////////////////////////////////////////////////////////////////////
uint8_t easyPairCheckExistDevices(wiced_bt_ble_scan_results_t *p_scan_result)
{
    uint8_t j, ret = 0;

    for(j = 0; j < MAX_DEVICES; j++)
    {
        //check to see if device i is valid
        if(ble_hidd_link.easyPair.device[j].valid == VALID_ID)
        {
            //if so, check to see if bd addr matches
            if(!memcmp( p_scan_result->remote_bd_addr , ble_hidd_link.easyPair.device[j].wd_addr , BD_ADDR_LEN))
            {
                //sum the rssi
                ble_hidd_link.easyPair.device[j].rssi_total += p_scan_result->rssi;
                ble_hidd_link.easyPair.device[j].rssi_count++;
                return 1;
            }
        }
    }

    return ret;
}

/////////////////////////////////////////////////////////////////////////////////
/// handles the scan results
///
/// \param  p_scan_result - pointer to scan result data
/// \param  p_adv_data - pointer to advertising data
///
/// \return 0 - not existed
///         1 - existed
/////////////////////////////////////////////////////////////////////////////////
void blehidlink_easyPair_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
{
    uint8_t *p_data;
    uint8_t length;
    uint8_t temp;
    uint8_t manu = 0;
    uint8_t tx = 0;
    uint8_t manufacture_id[3] = {0x00, 0x0F, 0x01};

    if ( p_scan_result )
    {
        // Advertisement data should have Advertisement type BTM_BLE_ADVERT_TYPE_MANUFACTURER
        p_data = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_MANUFACTURER, &length );

        //easy pair requires manufacturer id to be 0x00 0x0F 0x01
        if ( ( p_data == NULL ) || ( length != 3 ) || ( memcmp( p_data, manufacture_id, 3 ) != 0 ) )
        {
            // wrong device
            return;
        }
        else
        {
            manu = 1;
        }

        // Advertisement data should have Advertisement type BTM_BLE_ADVERT_TYPE_TX_POWER
        p_data = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_TX_POWER, &length );

        //easy pair tx power must be 0x00
        if ( ( p_data == NULL ) || ( length != 1 ) || (p_data[0] != 0 ) )
        {
            // wrong device
            return;
        }
        else
        {
            tx = 1;
        }


        //if both match proceed check to see if device exists.  if not, add it.
        if(manu & tx)
        {
            //this is a valid candidate
            //check to see if it is already in the list
            if(easyPairCheckExistDevices(p_scan_result))
            {
                return;
            }

            //if there is a room, add the device
            if(ble_hidd_link.easyPair.availableSlots != 0)
            {
                temp = MAX_DEVICES - ble_hidd_link.easyPair.availableSlots;
                ble_hidd_link.easyPair.device[temp].addressType = p_scan_result->ble_addr_type;
                memcpy(ble_hidd_link.easyPair.device[temp].wd_addr, p_scan_result->remote_bd_addr, BD_ADDR_LEN);
                ble_hidd_link.easyPair.device[temp].rssi_total =  p_scan_result->rssi;
                ble_hidd_link.easyPair.device[temp].rssi_count++;
                ble_hidd_link.easyPair.device[temp].valid = VALID_ID;
                ble_hidd_link.easyPair.availableSlots--;
            }
        }

    }
    else
    {
        WICED_BT_TRACE( "Scan completed\n" );
        if (ble_hidd_link.easyPair_deviceIndex != INVALID_INDEX)
        {
            // start high duty cycle directed advertising.
            wiced_bt_start_advertisements(BTM_BLE_ADVERT_DIRECTED_HIGH,
                                       ble_hidd_link.easyPair.device[ble_hidd_link.easyPair_deviceIndex].addressType,
                                       ble_hidd_link.easyPair.device[ble_hidd_link.easyPair_deviceIndex].wd_addr);

            blehidlink_setState(BLEHIDLINK_RECONNECTING);

        }
        else
        {
            blehidlink_setState(BLEHIDLINK_DISCONNECTED);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// timeout hander
/// this function will determine if there is a valid easy pair
/// candidate to connect to
///
/// \param arg - don't care
/////////////////////////////////////////////////////////////////////////////////
void blehidlink_easyPair_timerCb(uint32_t arg)
{
    uint8_t i;
    uint8_t highest_rssi_index = INVALID_INDEX;
    int32_t highest_rssi=INVALID_RSSI;
    int32_t rssi_avg;

    WICED_BT_TRACE("blehidlink_easyPair_timerCb\n");

    //stop timer
    if (wiced_is_timer_in_use(&ble_hidd_link.easyPair_timer))
    {
        wiced_stop_timer(&ble_hidd_link.easyPair_timer);
    }

    // find the average rssi of each device and
    // also find which one has the highest rssi to connect to
    for(i = 0; i < MAX_DEVICES; i++)
    {
        if(ble_hidd_link.easyPair.device[i].valid == VALID_ID)
        {
                rssi_avg = ble_hidd_link.easyPair.device[i].rssi_total / ble_hidd_link.easyPair.device[i].rssi_count;

                if( rssi_avg > highest_rssi)
                {
                    highest_rssi = rssi_avg;
                    highest_rssi_index = i;
                }
        }
    }

    //if a valid candid was found, make a connection to valid
    // device with highest rssi
    if(highest_rssi_index != INVALID_INDEX)
    {
        ble_hidd_link.easyPair_deviceIndex = highest_rssi_index;
    }

    //turn off scans
    wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_NONE, 0, blehidlink_easyPair_scan_result_cback );
}

/////////////////////////////////////////////////////////////////////////////////
/// register for notification of LE Advertising Report Event and
/// enable scan and start scan timer
/////////////////////////////////////////////////////////////////////////////////
void blehidlink_easyPair_scan(void)
{
    wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_HIGH_DUTY, 0, blehidlink_easyPair_scan_result_cback);

    //start easy pair scan timer
    wiced_start_timer(&ble_hidd_link.easyPair_timer, EASY_PAIR_SCAN_TIMEOUT *1000); // start 5 seconds timer. timeout in ms
}

/////////////////////////////////////////////////////////////////////////////////
/// start easy pair
/////////////////////////////////////////////////////////////////////////////////
void blehidlink_easyPair(uint32_t arg)
{
    WICED_BT_TRACE("blehidlink_easyPair\n");

    easyPairInit();
    blehidlink_easyPair_scan();
}
#endif //#ifdef EASY_PAIR

/*
 * hidd lib link default management callback
 */
wiced_bt_management_cback_t *app_management_cback_ptr = NULL;
wiced_result_t (*app_init_ptr)(void) = NULL;
wiced_result_t ble_hidd_lib_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_device_link_keys_t *pLinkKeys;
    wiced_bt_ble_advert_mode_t curr_adv_mode;
    uint8_t *p_keys;
    wiced_bt_device_address_t         bda = { 0 };

    // library should not hard coded to reference application data structure. The application should initialize usage by passing the pointer.
    // For backward compatiblity, the legacy code may not initialize the pointer, in that case, we assign the pointer to the hard coded data.
    // This code should be removed once all application is changed to call wiced_ble_hidd_start to initialize wiced_bt_hid_cfg_settings_ptr.
    if (!wiced_bt_hid_cfg_settings_ptr)
    {
        WICED_BT_TRACE("HID configuration is not initialized by wiced_ble_hidd_start()\n");
        extern const wiced_bt_cfg_settings_t wiced_bt_hid_cfg_settings;
        wiced_bt_hid_cfg_settings_ptr = &wiced_bt_hid_cfg_settings;
    }

    WICED_BT_TRACE("=== BT stack cback event %d\n", event);

    if (app_management_cback_ptr)
    {
        result = app_management_cback_ptr(event, p_event_data);
        if (result != WICED_NOT_FOUND) // if application has handled the event
        {
            return result;
        }
        // not handled by app, default result back to success and continue with default handler
        result = WICED_BT_SUCCESS;
    }

    // pass event to HCI control host if it is enabled
    hci_control_le_handle_event(event, p_event_data);

    // hidd_lib default handler
    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            hci_control_le_enable_trace();
            wiced_bt_dev_read_local_addr(bda);
            WICED_BT_TRACE("Address: [ %B]\n", bda);
            if (app_init_ptr)
            {
                result = app_init_ptr();
            }
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT\n");
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_SC_ONLY|BTM_LE_AUTH_REQ_BOND;              /* LE sec bonding */
            p_event_data->pairing_io_capabilities_ble_request.max_key_size = 16;
            p_event_data->pairing_io_capabilities_ble_request.init_keys = 0x0F; //(BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_PLK);
            p_event_data->pairing_io_capabilities_ble_request.resp_keys = 0x0F; //(BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_PLK);
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            if(p_event_data->pairing_complete.transport == BT_TRANSPORT_LE)
            {
                wiced_bt_dev_ble_pairing_info_t * p_info;
                p_info =  &p_event_data->pairing_complete.pairing_complete_info.ble;
                WICED_BT_TRACE( "LE Pairing Complete: %x\n",p_info->reason);

                //bonding successful
                if (!p_info->reason )
                {
                    WICED_BT_TRACE( "BONDED successful\n");
                    if (!wiced_blehidd_is_device_bonded())
                    {
                        WICED_BT_TRACE( "set device bonded flag\n");
                        wiced_blehidd_set_device_bonded_flag(WICED_TRUE);
                    }

#ifdef CONNECTED_ADVERTISING_SUPPORTED
                    //If there is a connection existing, delete pairing information and disconnect existing connection
                    if (ble_hidd_link.second_conn_state == BLEHIDLINK_2ND_CONNECTION_PENDING)
                    {
                        ble_hidd_link.second_conn_state = BLEHIDLINK_2ND_CONNECTION_NOT_ALLOWED;

                        if (wiced_ble_hidd_host_info_is_bonded())
                        {
                            uint8_t *bonded_bdadr = (uint8_t *)wiced_ble_hidd_host_info_get_bdaddr();

                            WICED_BT_TRACE("remove bonded device : %B\n", bonded_bdadr);
                            wiced_bt_dev_delete_bonded_device(bonded_bdadr);
                        }

                        WICED_BT_TRACE("Removing all bonded info\n");
                        wiced_ble_hidd_host_info_delete_all();

                        //disconnect existing connection
                        wiced_bt_gatt_disconnect(ble_hidd_link.existing_connection_gatts_conn_id);
                    }
#endif
                    //SMP result callback: successful
                    wiced_ble_hidd_host_info_add_first(ble_hidd_link.gatts_peer_addr, ble_hidd_link.gatts_peer_addr_type, &blehostlist_link_keys, blehostlist_flags);
                }
                else
                {
                    //SMP result callback: failed
                    WICED_BT_TRACE( " BONDED failed\n");
#ifdef CONNECTED_ADVERTISING_SUPPORTED
                    //If this is from the new connection
                    if (ble_hidd_link.second_conn_state == BLEHIDLINK_2ND_CONNECTION_PENDING)
                    {
                        uint16_t temp_gatts_conn_id = ble_hidd_link.gatts_conn_id;

                        WICED_BT_TRACE("delete the new connection: %d\n", temp_gatts_conn_id);

                        ble_hidd_link.second_conn_state = BLEHIDLINK_2ND_CONNECTION_NOT_ALLOWED;

                        //recover current connection gatt connection id
                        ble_hidd_link.gatts_conn_id = ble_hidd_link.existing_connection_gatts_conn_id;

                        //disconnect new connection
                        wiced_bt_gatt_disconnect(temp_gatts_conn_id);

                        //restore embeded controller info for the LE link (peer device info, bonded, encrypted, connection parameters etc.)
                        memcpy(&emConInfo_devInfo, &ble_hidd_link.existing_emconinfo, sizeof(EMCONINFO_DEVINFO));
                    }
                    else
#endif
                    wiced_ble_hidd_host_info_delete(ble_hidd_link.gatts_peer_addr, ble_hidd_link.gatts_peer_addr_type);
                }
            }
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            WICED_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT %B\n", (uint8_t *)&p_event_data->paired_device_link_keys_update);
            memcpy(&blehostlist_link_keys, &p_event_data->paired_device_link_keys_update, sizeof(wiced_bt_device_link_keys_t));
            wiced_trace_array((uint8_t *)(&(blehostlist_link_keys.key_data)), BTM_SECURITY_KEY_DATA_LEN);
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            WICED_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT\n");
            pLinkKeys = (wiced_bt_device_link_keys_t *)wiced_ble_hidd_host_info_get_link_keys();
            WICED_BT_TRACE("link_key:%s [%B] [%B]\n", pLinkKeys ? "Yes" : "No", pLinkKeys->bd_addr, p_event_data->paired_device_link_keys_request.bd_addr);
            if (pLinkKeys && !memcmp(pLinkKeys->bd_addr, p_event_data->paired_device_link_keys_request.bd_addr, BD_ADDR_LEN))
            {
                memcpy(&p_event_data->paired_device_link_keys_request, pLinkKeys, sizeof(wiced_bt_device_link_keys_t));
            }
            else
            {
                WICED_BT_TRACE("link_key not available\n");
                result = WICED_BT_ERROR;
            }
            break;

        case BTM_SECURITY_REQUEST_EVT:
            WICED_BT_TRACE("BTM_SECURITY_REQUEST_EVT\n");
             /* Use the default security */
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr,  WICED_BT_SUCCESS);
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            WICED_BT_TRACE("BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT\n");
            /* save keys to NVRAM */
            p_keys = (uint8_t *)&p_event_data->local_identity_keys_update;
            wiced_hal_write_nvram ( VS_LOCAL_IDENTITY_ID, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
            WICED_BT_TRACE("local keys save to NVRAM result: %d \n", result);
            wiced_trace_array(p_event_data->local_identity_keys_update.local_key_data, BTM_SECURITY_LOCAL_KEY_DATA_LEN);
            break;

        case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            WICED_BT_TRACE("BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT\n");
            /* read keys from NVRAM */
            p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
            wiced_hal_read_nvram( VS_LOCAL_IDENTITY_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
            WICED_BT_TRACE("local keys read from NVRAM result: %d\n",  result);
            if (!result)
                wiced_trace_array(p_keys, BTM_SECURITY_LOCAL_KEY_DATA_LEN);
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            curr_adv_mode = app_adv_mode;
            app_adv_mode = p_event_data->ble_advert_state_changed;

            WICED_BT_TRACE( "Advertisement State Change: %d -> %d\n", curr_adv_mode, app_adv_mode);
            //if high duty cycle directed advertising stops
            if ( (curr_adv_mode == BTM_BLE_ADVERT_DIRECTED_HIGH) &&
                     (app_adv_mode == BTM_BLE_ADVERT_DIRECTED_LOW))
            {
                wiced_ble_hidd_link_directed_adv_stop();
            }
#if !defined(ENDLESS_LE_ADVERTISING_WHILE_DISCONNECTED) || !defined(SUPPORT_EPDS)
            // btstack will switch to low adv mode automatically when high adv mode timeout,
            // for HIDD, we want to stop adv instead
            else if ((curr_adv_mode == BTM_BLE_ADVERT_UNDIRECTED_HIGH) &&
                         (app_adv_mode == BTM_BLE_ADVERT_UNDIRECTED_LOW))
            {
                wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
            }
#endif
            // if we are reconnecting and adv stops, we enter disconnected state
            if (ble_hidd_link.subState == BLEHIDLINK_RECONNECTING && !app_adv_mode)
            {
                blehidlink_setState(BLEHIDLINK_DISCONNECTED);
#ifdef AUTO_RECONNECT
                if(ble_hidd_link.auto_reconnect && wiced_ble_hidd_host_info_is_bonded() && !wiced_hal_batmon_is_low_battery_shutdown())
                    wiced_ble_hidd_link_connect();
#endif
            }
            break;

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
                WICED_BT_TRACE( "Scan State Change: %d\n", p_event_data->ble_scan_state_changed );
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            WICED_BT_TRACE("BTM_ENCRYPTION_STATUS_EVT");
            if (p_event_data->encryption_status.result == WICED_SUCCESS)
            {
                WICED_BT_TRACE(" link encrypted\n");
                wiced_blehidd_set_link_encrypted_flag(WICED_TRUE);
            }
            else
            {
                WICED_BT_TRACE(" Encryption failed:%d\n", p_event_data->encryption_status.result);
            }
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            WICED_BT_TRACE(" BTM_BLE_CONNECTION_PARAM_UPDATE status:%d\n", p_event_data->ble_connection_param_update.status);
            if (!p_event_data->ble_connection_param_update.status)
            {
                wiced_ble_hidd_link_conn_update_complete();
            }
            break;

        default:
            WICED_BT_TRACE("unhandled management_cback event: %d!!!\n", event );
            break;
    }
    return result;
}

/////////////////////////////////////////////////////////////////////////////////
/// wiced_ble_hidd_start
///
/// \param p_bt_management_cback  - application bt_management callback function
///        p_bt_cfg_settings      - bt configuration setting
///        wiced_bt_cfg_buf_pools - buffer pool configuration
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_start(const char * appName, wiced_result_t (*p_bt_app_init)(),
                          wiced_bt_management_cback_t   * p_bt_management_cback,
                          const wiced_bt_cfg_settings_t * p_bt_cfg_settings,
                          const wiced_bt_cfg_buf_pool_t * p_bt_cfg_buf_pools)
{
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
    WICED_BT_TRACE("%s start\n",appName);

    app_management_cback_ptr = p_bt_management_cback;
    app_init_ptr = p_bt_app_init;
    if (!p_bt_cfg_settings || !p_bt_cfg_buf_pools)
    {
        WICED_BT_TRACE("bt or buff_pool configration is undefined\n");
    }
    else
    {
        wiced_bt_hid_cfg_settings_ptr = p_bt_cfg_settings;
        wiced_bt_stack_init (ble_hidd_lib_management_cback, p_bt_cfg_settings, p_bt_cfg_buf_pools);
    }
}

#endif //#ifndef BT_HIDD_ONLY
