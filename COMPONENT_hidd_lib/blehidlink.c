/*
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
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
#ifdef BLE_SUPPORT
#include "spar_utils.h"
#include "blehidlink.h"
#include "wiced_hal_mia.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_event.h"
#include "wiced_bt_l2c.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_ota_firmware_upgrade.h"
#include "wiced_hal_batmon.h"
#include "wiced_hal_nvram.h"
#include "wiced_memory.h"
#include "hidd_lib.h"

tBleHidLink ble_hidd_link = {{15, 15, 20, 300}, 0, 0, };

uint16_t blehostlist_flags = 0;
wiced_bool_t blehidlink_connection_param_updated = WICED_FALSE;
uint8_t wake_from_SDS_timer_timeout = 0;
wiced_bt_ble_advert_mode_t app_adv_mode = BTM_BLE_ADVERT_OFF;

#ifdef FATORY_TEST_SUPPORT
uint8_t factory_mode = 0;
extern uint8_t force_sleep_in_HID_mode;
#endif

void blehidlink_reconnect_timerCb( uint32_t arg);
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

#ifdef EASY_PAIR
void blehidlink_easyPair(uint32_t arg);
void blehidlink_easyPair_timerCb( uint32_t arg);
#endif

PLACE_DATA_IN_RETENTION_RAM blehid_aon_save_content_t   blehid_aon_data;

/////////////////////////////////////////////////////////////////////////////////////////////
/// ble hid link init
/////////////////////////////////////////////////////////////////////////////////////////////
void blehidlink_init()
{
    ///connection idle timer that can be supported in uBCS mode
    osapi_createTimer(&ble_hidd_link.conn_idle_timer, blehidlink_connectionIdle_timerCb, 0);

    //timer to auto reconnect when disconnected
    wiced_init_timer( &ble_hidd_link.reconnect_timer, blehidlink_reconnect_timerCb, 0, WICED_MILLI_SECONDS_TIMER );

#ifdef EASY_PAIR
    //timer for easy pair
    wiced_init_timer( &ble_hidd_link.easyPair_timer, blehidlink_easyPair_timerCb, 0, WICED_MILLI_SECONDS_TIMER );
#endif

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
void blehidlink_determineNextState_on_boot(void)
{
    //if low battery shutdown, enter disconnected
    if (wiced_hal_batmon_is_low_battery_shutdown())
    {
        blehidlink_enterDisconnected();
        return;
    }

    if(wiced_hidd_is_paired())
    {
        WICED_BT_TRACE("\nbonded info in NVRAM");

#ifdef AUTO_RECONNECT
        if (ble_hidd_link.auto_reconnect && !wiced_hal_batmon_is_low_battery_shutdown())
        {
            wiced_start_timer(&ble_hidd_link.reconnect_timer,10000); //auto reconnect after 10 seconds.
        }
#endif
        blehidlink_enterDisconnected();

#ifdef START_ADV_WHEN_POWERUP_NO_CONNECTED
        blehidlink_enterReconnecting();
#endif
    }
    else
    {
        WICED_BT_TRACE("\nno bonding");
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
                WICED_BT_TRACE("\ndiscoverable timer timeout!!");
                wake_from_SDS_timer_timeout = BLEHIDLINK_ADV_CONNECTABLE_UNDIRECTED_TIMER | 1;
            }
        }
        //is it connection idle timer
        else if (ble_hidd_link.osapi_app_timer_running & BLEHIDLINK_CONNECTION_IDLE_TIMER)
        {
            WICED_BT_TRACE("\nble_hidd_link.conn_idle_timeout=%d, time_passed_in_ms=%d", ble_hidd_link.conn_idle_timeout, (uint32_t)time_passed_in_ms);
            // if time passed more than connection idle timeout value
            if ((time_passed_in_ms >= ble_hidd_link.conn_idle_timeout*1000) || ((ble_hidd_link.conn_idle_timeout - time_passed_in_ms/1000) <= 1))
            {
                WICED_BT_TRACE("\nconnection idle timer timeout!!");
                wake_from_SDS_timer_timeout = BLEHIDLINK_CONNECTION_IDLE_TIMER | 1;
            }
            else
            {
                uint64_t remaining_time_in_ms = ble_hidd_link.conn_idle_timeout*1000 - time_passed_in_ms;
                //WICED_BT_TRACE("\nremaining_time_in_ms = %d", (uint32_t)remaining_time_in_ms);
                //restart connection idle timer w/remaining time
                osapi_activateTimer( &ble_hidd_link.conn_idle_timer, remaining_time_in_ms*1000); //timout in micro seconds.
            }
        }

        ble_hidd_link.osapi_app_timer_running = 0;
    }

#if is_SDS_capable && (defined(ENDLESS_LE_ADVERTISING_WHILE_DISCONNECTED) || defined(ALLOW_SDS_IN_DISCOVERABLE))
    if ((BLEHIDLINK_ADVERTISING_IN_uBCS_DIRECTED == ble_hidd_link.resumeState) ||
        (BLEHIDLINK_ADVERTISING_IN_uBCS_UNDIRECTED == ble_hidd_link.resumeState))
    {
        //stop advertising.
        //NOTE: wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF) can't be used to stop advertising here.
        // Due to wiced stack didn't save adv status before/exit SDS.
        //btsnd_hcic_ble_set_adv_enable (BTM_BLE_ADVERT_OFF);

        //check if wake up due to receiving LE connect request
        if (wiced_blehidd_is_wakeup_from_conn_req())
        {
            WICED_BT_TRACE("\nwake from CONNECT req");
            if (!wiced_hal_batmon_is_low_battery_shutdown())
            {
                if (hidd_host_isBonded() && (BLEHIDLINK_ADVERTISING_IN_uBCS_DIRECTED == ble_hidd_link.resumeState))
                {
                    blehidlink_enterReconnecting();
                }
                else
                {
 #ifdef WHITE_LIST_FOR_ADVERTISING
                    //if advertising white list is enabled before enter SDS
                    if (hidd_host_isBonded() && ble_hidd_link.adv_white_list_enabled)
                    {
                        //add to white list
                        wiced_bt_ble_update_advertising_white_list(WICED_TRUE, wiced_hidd_host_addr());

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
            WICED_BT_TRACE("\nset Disconnected state");
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

#if is_SDS_capable && (defined(ENDLESS_LE_ADVERTISING_WHILE_DISCONNECTED) || defined(ALLOW_SDS_IN_DISCOVERABLE))
        //if no user activity and not wake up due to application timer timeout. restart adv again
        if ((BLEHIDLINK_DISCONNECTED == ble_hidd_link.subState) && !wake_from_SDS_timer_timeout)
        {
 #ifdef ENDLESS_LE_ADVERTISING_WHILE_DISCONNECTED
            //if  it is bonded, start low duty cycle directed advertising again.
            if (hidd_host_isBonded() && (BLEHIDLINK_ADVERTISING_IN_uBCS_DIRECTED == ble_hidd_link.resumeState))
            {
                //NOTE!!! wiced_bt_start_advertisement could modify the value of bdAddr, so MUST use a copy.
                uint8_t tmp_bdAddr[BD_ADDR_LEN];
                memcpy(tmp_bdAddr, wiced_hidd_host_addr(), BD_ADDR_LEN);

                // start high duty cycle directed advertising.
                if (wiced_bt_start_advertisements(BTM_BLE_ADVERT_DIRECTED_LOW, wiced_hidd_host_addr_type(), tmp_bdAddr))
                {
                    WICED_BT_TRACE("\nFailed to start low duty cycle directed advertising!!!");
                }

                blehidlink_setState(BLEHIDLINK_ADVERTISING_IN_uBCS_DIRECTED);
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
/// ble hidd link get current advertizing mode
/////////////////////////////////////////////////////////////////////////////////////////////
wiced_bt_ble_advert_mode_t wiced_ble_hidd_link_get_adv_mode(void)
{
    return app_adv_mode;
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// determine next action on cold boot or wake from SDS
/////////////////////////////////////////////////////////////////////////////////////////////
void blehidlink_determineNextState(void)
{
    if(!wiced_hal_mia_is_reset_reason_por())
    {
#if is_208xxFamily
        // in 20819, this is HIDOFF wake up
        WICED_BT_TRACE("\nwake from HIDOFF");
        blehidlink_determineNextState_on_boot();
#else
        WICED_BT_TRACE("\nwake from SDS");
        blehidlink_determineNextState_on_wake_from_SDS();
#endif
    }
    else
    {
        WICED_BT_TRACE("\ncold boot");
        blehidlink_determineNextState_on_boot();
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
    if(ble_hidd_link.auto_reconnect && (wiced_hidd_host_transport() == BT_TRANSPORT_LE) && !wiced_hal_batmon_is_low_battery_shutdown() && !ble_hidd_link.pendingStateTransiting)
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
    WICED_BT_TRACE("\nwiced_ble_hidd_link_adv_stop");

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
    WICED_BT_TRACE("\nblehidlink_DirectedAdvStop");

//#if defined(ENDLESS_LE_ADVERTISING_WHILE_DISCONNECTED) && is_SDS_capable)
#if 0
    blehidlink_setState(BLEHIDLINK_ADVERTISING_IN_uBCS_DIRECTED);
#else
 #ifdef WHITE_LIST_FOR_ADVERTISING
    //update advertising filer policy to use white list to filter scan and connect request
    wiced_btm_ble_update_advertisement_filter_policy(0x03);

    ble_hidd_link.adv_white_list_enabled = 0x03;
 #endif

    // start undirected connectable advertising.
    if (wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL))
        WICED_BT_TRACE("\nFailed to undirected connectable advertising!!!");

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
/// check if the current ble link state is in the requested state
///
/// \return TRUE/FALSE
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t  wiced_ble_hidd_link_state_is(uint8_t state)
{
    return (ble_hidd_link.subState == state);
}

/////////////////////////////////////////////////////////////////////////////////
/// Connect
/// As LE slave, it means start LE advertising
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_connect(void)
{
    if(hidd_host_isBonded())
    {
        switch(ble_hidd_link.subState)
        {
            case BLEHIDLINK_DISCONNECTED:
                blehidlink_enterReconnecting();
                break;
            default:
                 //WICED_BT_TRACE("\nwiced_ble_hidd_link_connect(bonded):%d",ble_hidd_link.subState);
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
                //WICED_BT_TRACE("\nwiced_ble_hidd_link_connect(not bonded):%d",ble_hidd_link.subState);
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
//    WICED_BT_TRACE("\nLE enterDisconnected");

#ifdef WHITE_LIST_FOR_ADVERTISING
    //clear white list
    wiced_bt_ble_clear_white_list();
#endif

#ifdef AUTO_RECONNECT
    if ((wiced_hidd_host_transport() == BT_TRANSPORT_LE))
    {
        blehidlink_enterReconnecting();
    }
    else
#endif
    blehidlink_setState(BLEHIDLINK_DISCONNECTED);

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

    WICED_BT_TRACE("\nenterDiscoverable");

    // start undirected connectable advertising.
    if (wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL))
        WICED_BT_TRACE("\nFailed to start undirected connectable advertising!!!");

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
    WICED_BT_TRACE("\nallowDiscoverable");

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
            WICED_BT_TRACE("\nFailed to start undirected connectable advertising!!!");

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
        WICED_BT_TRACE("\nLE state changed from %d to %d", ble_hidd_link.subState, newState);
        ble_hidd_link.subState = newState;

        hci_control_send_state_change(BT_TRANSPORT_LE, newState);

        //if current active transport is BR/EDR, do not notify observer
        if (wiced_hidd_host_transport() == BT_TRANSPORT_BR_EDR)
            return;

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
    WICED_BT_TRACE("\nblehidlink_enterConnected");

#ifdef ALLOW_SDS_IN_DISCOVERABLE
    //stop discoverable timer
    osapi_deactivateTimer(&ble_hidd_link.discoverable_timer);

    ble_hidd_link.osapi_app_timer_running &= ~BLEHIDLINK_ADV_CONNECTABLE_UNDIRECTED_TIMER;
    if ((ble_hidd_link.osapi_app_timer_running >> 1) == 0)
    {
        ble_hidd_link.osapi_app_timer_running = 0; // no more application osapi timer is running
    }
#endif

    hidd_host_setAddrType(ble_hidd_link.gatts_peer_addr, ble_hidd_link.gatts_peer_addr_type);

    // if dev does not agree with our setting, we change both to bonded
    if (wiced_blehidd_is_device_bonded() ^ hidd_host_isBonded())
    {
        if (hidd_host_isBonded())
        {
            // this is bonded device, we have the bonding info..
            WICED_BT_TRACE("\nset device bonded flag");
            wiced_blehidd_set_device_bonded_flag(WICED_TRUE);
        }
        else
        {
            // already bonded in dev, we update our record
            WICED_BT_TRACE("\nupdate bonded flag");
            hidd_host_setBonded(WICED_TRUE);
        }
    }

    blehidlink_setState(BLEHIDLINK_CONNECTED);
    // In any case, stop advertising
//    wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);

    if(wiced_hidd_cfg()->security_requirement_mask)
    {
        if (!wiced_blehidd_is_device_bonded())
        {
            WICED_BT_TRACE("\nsend security req:");
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
    WICED_BT_TRACE("\nblehidlink_enterReconnecting");
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

    WICED_BT_TRACE("\nenterReconnecting");
    //if reconnect timer is running, stop it
    if (wiced_is_timer_in_use(&ble_hidd_link.reconnect_timer))
    {
        wiced_stop_timer(&ble_hidd_link.reconnect_timer);
    }

    if (wiced_hidd_is_paired())
    {
        //NOTE!!! wiced_bt_start_advertisement could modify the value of bdAddr, so MUST use a copy.
        uint8_t tmp_bdAddr[BD_ADDR_LEN];
        memcpy(tmp_bdAddr, wiced_hidd_host_addr(), BD_ADDR_LEN);

#ifdef WHITE_LIST_FOR_ADVERTISING
        //add to white list
        wiced_bt_ble_update_advertising_white_list(WICED_TRUE, tmp_bdAddr);
#endif
        // start high duty cycle directed advertising.
        if (wiced_bt_start_advertisements(BTM_BLE_ADVERT_DIRECTED_HIGH, wiced_hidd_host_addr_type(), tmp_bdAddr))
            WICED_BT_TRACE("\nFailed to start high duty cycle directed advertising!!!");

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
            WICED_BT_TRACE("\nreportID:%d notification FALSE",reportID);
        }
        else
        {
            // Something did not match
            WICED_BT_TRACE("\nSendRpt failed, %d, %d, %d, 0x%x", reportID, length, reportType, rptSentStatus);
        }
    }

    //Start a timer to make sure the packet is sent over the air before going to HID Off
    //This is a work around while we determine where we should prevent HID Off
    wiced_hidd_deep_sleep_not_allowed(1000);// No deep sleep for 1 second.

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
    if (hidd_host_isBonded())
    {
        uint8_t *bonded_bdadr = wiced_hidd_host_addr();
#ifdef WHITE_LIST_FOR_ADVERTISING
        //stop advertising anyway before white list operation
        wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);

        //remove from white list
        WICED_BT_TRACE("\nremove from white list : %B", bonded_bdadr);
        wiced_bt_ble_update_advertising_white_list(WICED_FALSE, bonded_bdadr);

        //clear whitlist
        wiced_bt_ble_clear_white_list();
#endif

        WICED_BT_TRACE("\nremove bonded device : %B", bonded_bdadr);
        wiced_bt_dev_delete_bonded_device(bonded_bdadr);
    }

    WICED_BT_TRACE("\nRemoving all bonded info");
    wiced_hidd_host_remove_all();

    WICED_BT_TRACE("\nclear device bonded flag");
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
//        blehidlink_enterDiscoverable(WICED_TRUE);
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

    WICED_BT_TRACE("\nwiced_ble_hidd_link_set_slave_latency: interval=%d, slavelatency=%d",
               ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_INTERVAL_MIN],
               ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_SLAVE_LATENCY]);
    wiced_blehidd_set_asym_slave_latency(wiced_blehidd_get_connection_handle(), ble_hidd_link.prefered_conn_params[BLEHIDLINK_CONN_SLAVE_LATENCY]);
}

/////////////////////////////////////////////////////////////////////////////////
/// handler when received LE Connection Update Complete event
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_ble_hidd_link_conn_param_updated()
{
    return blehidlink_connection_param_updated;
}

/////////////////////////////////////////////////////////////////////////////////
/// handler when received LE Connection Update Complete event
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_conn_update_complete(void)
{
    WICED_BT_TRACE("\nConnParamUpdate:Interval:%d, Latency:%d, Supervision TO:%d",
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
        WICED_BT_TRACE("\nsend conn param request");
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
    WICED_BT_TRACE("\nReconnect");

    wiced_ble_hidd_link_connect();
}

/////////////////////////////////////////////////////////////////////////////////
/// timeout handler
/// \param arg - don't care
/// \param overTimeInUs - don't care
/////////////////////////////////////////////////////////////////////////////////
void blehidlink_connectionIdle_timerCb(INT32 args, UINT32 overTimeInUs)
{
    WICED_BT_TRACE("\nconnection Idle timeout");

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
        WICED_BT_TRACE("\nWICED_BT_AON_DRIVER_RESTORE");

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
#if !is_208xxFamily /* Slimboot is not supported in 20819A1 */
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
    WICED_BT_TRACE("\nblehidlink_discoverabletimerCb!!!");
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
        WICED_BT_TRACE("\nScan completed" );
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

    WICED_BT_TRACE("\nblehidlink_easyPair_timerCb");

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
    WICED_BT_TRACE("\nblehidlink_easyPair");

    easyPairInit();
    blehidlink_easyPair_scan();
}
#endif //#ifdef EASY_PAIR


#endif //#ifdef BLE_SUPPORT
