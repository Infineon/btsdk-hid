/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
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
* File Name: hidd_blelink.c
*
* Abstract: This file implements the BLE HID application transport
*
* Functions:
*
*******************************************************************************/
#ifdef BLE_SUPPORT
#include "spar_utils.h"
#include "hidd_lib.h"
#include "wiced_hal_mia.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_event.h"
#include "wiced_bt_l2c.h"
#include "wiced_hal_batmon.h"
#include "wiced_memory.h"

tBleHidLink blelink = {};

#ifdef FATORY_TEST_SUPPORT
uint8_t factory_mode = 0;
extern uint8_t force_sleep_in_HID_mode;
#endif

#ifdef EASY_PAIR
#endif

PLACE_DATA_IN_RETENTION_RAM blehid_aon_save_content_t   ble_aon_data;

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_set_adv_mode(wiced_bt_ble_advert_mode_t newMode)
{
    blelink.adv_mode = newMode;
}

/////////////////////////////////////////////////////////////////////////////////
/// timeout handler
/// \param arg - don't care
/// \param overTimeInUs - don't care
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_connectionIdle_timerCb(INT32 args, UINT32 overTimeInUs)
{
    WICED_BT_TRACE("\nconnection Idle timeout");

    //disconnect the link
    hidd_blelink_disconnect();
}

#ifdef EASY_PAIR
/////////////////////////////////////////////////////////////////////////////////
/// initialize easy pair data
/////////////////////////////////////////////////////////////////////////////////
void easyPairInit()
{
    blelink.easyPair_deviceIndex = INVALID_INDEX;
    blelink.easyPair.availableSlots = MAX_DEVICES;
    memset((void*)&blelink.easyPair.device[0], 0, sizeof(EASY_PAIR_CANDIDATE)*MAX_DEVICES);
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
        if(blelink.easyPair.device[j].valid == VALID_ID)
        {
            //if so, check to see if bd addr matches
            if(!memcmp( p_scan_result->remote_bd_addr , blelink.easyPair.device[j].wd_addr , BD_ADDR_LEN))
            {
                //sum the rssi
                blelink.easyPair.device[j].rssi_total += p_scan_result->rssi;
                blelink.easyPair.device[j].rssi_count++;
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
void hidd_blelink_easyPair_scan_result_cback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
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
            if(blelink.easyPair.availableSlots != 0)
            {
                temp = MAX_DEVICES - blelink.easyPair.availableSlots;
                blelink.easyPair.device[temp].addressType = p_scan_result->ble_addr_type;
                memcpy(blelink.easyPair.device[temp].wd_addr, p_scan_result->remote_bd_addr, BD_ADDR_LEN);
                blelink.easyPair.device[temp].rssi_total =  p_scan_result->rssi;
                blelink.easyPair.device[temp].rssi_count++;
                blelink.easyPair.device[temp].valid = VALID_ID;
                blelink.easyPair.availableSlots--;
            }
        }

    }
    else
    {
        WICED_BT_TRACE("\nScan completed" );
        if (blelink.easyPair_deviceIndex != INVALID_INDEX)
        {
            // start high duty cycle directed advertising.
            wiced_bt_start_advertisements(BTM_BLE_ADVERT_DIRECTED_HIGH,
                                       blelink.easyPair.device[blelink.easyPair_deviceIndex].addressType,
                                       blelink.easyPair.device[blelink.easyPair_deviceIndex].wd_addr);

            hidd_blelink_set_state(HIDLINK_LE_RECONNECTING);

        }
        else
        {
            hidd_blelink_set_state(HIDLINK_LE_DISCONNECTED);
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
void hidd_blelink_easyPair_timerCb(uint32_t arg)
{
    uint8_t i;
    uint8_t highest_rssi_index = INVALID_INDEX;
    int32_t highest_rssi=INVALID_RSSI;
    int32_t rssi_avg;

    WICED_BT_TRACE("\nhidd_blelink_easyPair_timerCb");

    //stop timer
    if (wiced_is_timer_in_use(&blelink.easyPair_timer))
    {
        wiced_stop_timer(&blelink.easyPair_timer);
    }

    // find the average rssi of each device and
    // also find which one has the highest rssi to connect to
    for(i = 0; i < MAX_DEVICES; i++)
    {
        if(blelink.easyPair.device[i].valid == VALID_ID)
        {
                rssi_avg = blelink.easyPair.device[i].rssi_total / blelink.easyPair.device[i].rssi_count;

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
        blelink.easyPair_deviceIndex = highest_rssi_index;
    }

    //turn off scans
    wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_NONE, 0, hidd_blelink_easyPair_scan_result_cback );
}

/////////////////////////////////////////////////////////////////////////////////
/// register for notification of LE Advertising Report Event and
/// enable scan and start scan timer
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_easyPair_scan(void)
{
    wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_HIGH_DUTY, 0, hidd_blelink_easyPair_scan_result_cback);

    //start easy pair scan timer
    wiced_start_timer(&blelink.easyPair_timer, EASY_PAIR_SCAN_TIMEOUT *1000); // start 5 seconds timer. timeout in ms
}

/////////////////////////////////////////////////////////////////////////////////
/// start easy pair
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_easyPair(uint32_t arg)
{
    WICED_BT_TRACE("\nhidd_blelink_easyPair");

    easyPairInit();
    hidd_blelink_easyPair_scan();
}
#endif //#ifdef EASY_PAIR

/////////////////////////////////////////////////////////////////////////////////////////////
/// ble hid link init
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_init()
{
    ///connection idle timer that can be supported in uBCS mode
    osapi_createTimer(&blelink.conn_idle_timer, hidd_blelink_connectionIdle_timerCb, 0);

#ifdef EASY_PAIR
    //timer for easy pair
    wiced_init_timer( &blelink.easyPair_timer, hidd_blelink_easyPair_timerCb, 0, WICED_MILLI_SECONDS_TIMER );
#endif

#ifdef ALLOW_SDS_IN_DISCOVERABLE
    blelink.state_switch_timeout_in_ms = 1000; // 1 seconds

    /// timer to switch from DISCOVERABLE to HIDLINK_LE_ADVERTISING_IN_uBCS_UNDIRECTED
    wiced_init_timer( &blelink.state_switch_timer, hidd_blelink_stateswitchtimerCb, 0, WICED_MILLI_SECONDS_TIMER );

    ///discoverable timer that can be supported in uBCS mode
    osapi_createTimer(&blelink.discoverable_timer, hidd_blelink_discoverabletimerCb, 0);
#endif

#ifdef AUTO_RECONNECT
    blelink.auto_reconnect = WICED_TRUE;
#endif

}

/////////////////////////////////////////////////////////////////////////////////
/// Reconnecting to previous HID host.
/// i.e. start high duty cycle directed advertising
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_enterReconnecting(void)
{
    WICED_BT_TRACE("\nhidd_blelink_enterReconnecting");
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
    hidd_link_stop_reconnect_timer();

    if (hidd_is_paired())
    {
        //NOTE!!! wiced_bt_start_advertisement could modify the value of bdAddr, so MUST use a copy.
        uint8_t tmp_bdAddr[BD_ADDR_LEN];
        memcpy(tmp_bdAddr, hidd_host_addr(), BD_ADDR_LEN);

#ifdef FILTER_ACCEPT_LIST_FOR_ADVERTISING
        //add to Filter Accept List
        wiced_bt_ble_update_advertising_filter_accept_list(WICED_TRUE, tmp_bdAddr);
#endif
        // start high duty cycle directed advertising.
        if (wiced_bt_start_advertisements(BTM_BLE_ADVERT_DIRECTED_HIGH, hidd_host_addr_type(), tmp_bdAddr))
            WICED_BT_TRACE("\nFailed to start high duty cycle directed advertising!!!");

        hidd_blelink_set_state(HIDLINK_LE_RECONNECTING);
    }
    else
    {
        hidd_blelink_enterDiscoverable(WICED_TRUE);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// determine next action on cold boot
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_determine_next_state_on_boot(void)
{
    //if low battery shutdown, enter disconnected
    if (wiced_hal_batmon_is_low_battery_shutdown())
    {
        hidd_blelink_enterDisconnected();
        return;
    }

    if(hidd_host_isBonded())
    {
        WICED_BT_TRACE("\nbonded info in NVRAM");

#ifdef AUTO_RECONNECT
        hidd_link_delayed_reconnect(10000); //auto reconnect after 10 seconds.
#endif
        hidd_blelink_enterDisconnected();

#ifdef START_ADV_WHEN_POWERUP_NO_CONNECTED
        hidd_blelink_enterReconnecting();
#endif
    }
    else
    {
        hidd_blelink_enterDisconnected();

#ifdef START_ADV_WHEN_POWERUP_NO_CONNECTED
 #ifdef ALLOW_SDS_IN_DISCOVERABLE
        //For cold boot, give it more time before allowing SDS; otherwise, it might reset.
        blelink.state_switch_timeout_in_ms = 5000; // 5 seconds. For cold boot, give it more time before allowing SDS; otherwise, it might reset.
 #endif
        hidd_blelink_enterDiscoverable(WICED_TRUE);
#endif
    }
}

#if !is_208xxFamily
/////////////////////////////////////////////////////////////////////////////////////////////
/// determine next action when wake from SDS
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_determine_next_state_on_wake_from_SDS(void)
{
    //restore embeded controller info for the LE link (peer device info, bonded, encrypted, connection parameters etc.)
    memcpy(&emConInfo_devInfo, &blelink.resume_emconinfo, sizeof(EMCONINFO_DEVINFO));

    //check if osapi app timer timeout
    if (blelink.osapi_app_timer_running)
    {
        uint64_t time_passed_in_ms = (clock_SystemTimeMicroseconds64() - blelink.osapi_app_timer_start_instant)/1000;
        //is it advertising timer?
        if (blelink.osapi_app_timer_running & BLEHIDLINK_ADV_CONNECTABLE_UNDIRECTED_TIMER)
        {
            // if time passed more than 60 seconds (adv timer timeout value)
            if (time_passed_in_ms >= 60000)
            {
                WICED_BT_TRACE("\ndiscoverable timer timeout!!");
                blelink.wake_from_SDS_timer_timeout_flag = BLEHIDLINK_ADV_CONNECTABLE_UNDIRECTED_TIMER | 1;
            }
        }
        //is it connection idle timer
        else if (blelink.osapi_app_timer_running & BLEHIDLINK_CONNECTION_IDLE_TIMER)
        {
            WICED_BT_TRACE("\nblelink.conn_idle_timeout=%d, time_passed_in_ms=%d", blelink.conn_idle_timeout, (uint32_t)time_passed_in_ms);
            // if time passed more than connection idle timeout value
            if ((time_passed_in_ms >= blelink.conn_idle_timeout*1000) || ((blelink.conn_idle_timeout - time_passed_in_ms/1000) <= 1))
            {
                WICED_BT_TRACE("\nconnection idle timer timeout!!");
                blelink.wake_from_SDS_timer_timeout_flag = BLEHIDLINK_CONNECTION_IDLE_TIMER | 1;
            }
            else
            {
                uint64_t remaining_time_in_ms = blelink.conn_idle_timeout*1000 - time_passed_in_ms;
                //WICED_BT_TRACE("\ = %d", (uint32_t)remaining_time_in_ms);
                //restart connection idle timer w/remaining time
                osapi_activateTimer( &blelink.conn_idle_timer, remaining_time_in_ms*1000); //timout in micro seconds.
            }
        }

        blelink.osapi_app_timer_running = 0;
    }

#if is_SDS_capable && (defined(ENDLESS_LE_ADVERTISING_WHILE_DISCONNECTED) || defined(ALLOW_SDS_IN_DISCOVERABLE))
    if ((HIDLINK_LE_ADVERTISING_IN_uBCS_DIRECTED == blelink.resumeState) ||
        (HIDLINK_LE_ADVERTISING_IN_uBCS_UNDIRECTED == blelink.resumeState))
    {
        extern BOOLEAN btsnd_hcic_ble_set_adv_enable (UINT8 adv_enable);
        //stop advertising.
        //NOTE: wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF) can't be used to stop advertising here. Due to wiced stack didn't save adv status before/exit SDS.
        btsnd_hcic_ble_set_adv_enable (BTM_BLE_ADVERT_OFF);

        //check if wake up due to receiving LE connect request
        if (wiced_blehidd_is_wakeup_from_conn_req())
        {
            WICED_BT_TRACE("\nwake from CONNECT req");
            if (!wiced_hal_batmon_is_low_battery_shutdown())
            {
                if (hidd_host_isBonded() && (HIDLINK_LE_ADVERTISING_IN_uBCS_DIRECTED == blelink.resumeState))
                {
                    hidd_blelink_enterReconnecting();
                }
                else
                {
 #ifdef FILTER_ACCEPT_LIST_FOR_ADVERTISING
                    //if advertising Filter Accept List is enabled before enter SDS
                    if (hidd_host_isBonded() && blelink.adv_filter_accept_list_enabled)
                    {
                        //add to Filter Accept List
                        wiced_bt_ble_update_advertising_filter_accept_list(WICED_TRUE, hidd_host_addr());

                        //update advertising filer policy to use Filter Accept List to filter scan and connect request
                        wiced_btm_ble_update_advertisement_filter_policy(blelink.adv_filter_accept_list_enabled);
                    }
 #endif
                    hidd_blelink_enterDiscoverable(WICED_FALSE);
                }
            }
        }
        else
        {
            WICED_BT_TRACE("\nset Disconnected state");
            hidd_blelink_set_state(HIDLINK_LE_DISCONNECTED);
        }
    }
    else
#endif
    {
        //set subState to resumeState
        hidd_blelink_set_state(blelink.resumeState);
    }

    if ((HIDLINK_LE_DISCONNECTED == blelink.subState) && !wiced_hal_batmon_is_low_battery_shutdown())
    {
        //poll user activity and action accordingly
        if(link.callbacks->p_app_poll_user_activities)
        {
            link.callbacks->p_app_poll_user_activities();
        }

#if is_SDS_capable && (defined(ENDLESS_LE_ADVERTISING_WHILE_DISCONNECTED) || defined(ALLOW_SDS_IN_DISCOVERABLE))
        //if no user activity and not wake up due to application timer timeout. restart adv again
        if ((HIDLINK_LE_DISCONNECTED == blelink.subState) && !blelink.wake_from_SDS_timer_timeout_flag)
        {
 #ifdef ENDLESS_LE_ADVERTISING_WHILE_DISCONNECTED
            //if  it is bonded, start low duty cycle directed advertising again.
            if (hidd_host_isBonded() && (HIDLINK_LE_ADVERTISING_IN_uBCS_DIRECTED == blelink.resumeState))
            {
                //NOTE!!! wiced_bt_start_advertisement could modify the value of bdAddr, so MUST use a copy.
                uint8_t tmp_bdAddr[BD_ADDR_LEN];
                memcpy(tmp_bdAddr, hidd_host_addr(), BD_ADDR_LEN);

                // start high duty cycle directed advertising.
                if (wiced_bt_start_advertisements(BTM_BLE_ADVERT_DIRECTED_LOW, hidd_host_addr_type(), tmp_bdAddr))
                {
                    WICED_BT_TRACE("\nFailed to start low duty cycle directed advertising!!!");
                }

                hidd_blelink_set_state(HIDLINK_LE_ADVERTISING_IN_uBCS_DIRECTED);
            }
            else
 #endif
            {
 #ifdef ALLOW_SDS_IN_DISCOVERABLE
                hidd_blelink_enterDiscoverable(WICED_TRUE);
 #endif
            }
        }
#endif
    }
    else if ((HIDLINK_LE_CONNECTED == blelink.subState) && !wiced_hal_batmon_is_low_battery_shutdown())
    {
        if (blelink.wake_from_SDS_timer_timeout_flag & BLEHIDLINK_CONNECTION_IDLE_TIMER)
        {
            //disconnect the link
            hidd_blelink_disconnect();
        }
    }

}
#endif

/////////////////////////////////////////////////////////////////////////////////////////////
/// ble hidd link get current advertizing mode
/////////////////////////////////////////////////////////////////////////////////////////////
wiced_bt_ble_advert_mode_t hidd_blelink_get_adv_mode(void)
{
    return blelink.adv_mode;
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// determine next action on cold boot or wake from SDS
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_determine_next_state(void)
{
    if(!wiced_hal_mia_is_reset_reason_por())
    {
        WICED_BT_TRACE("\nwake from shutdown");
#if !is_208xxFamily
        hidd_blelink_determine_next_state_on_wake_from_SDS();
        return;
#endif
    }
    else
    {
        WICED_BT_TRACE("\ncold boot");
    }
    hidd_blelink_determine_next_state_on_boot();

    //always reset to 0
    blelink.wake_from_SDS_timer_timeout_flag = 0;
}

/////////////////////////////////////////////////////////////////////////////////
/// Become connected.
/// Start SMP bonding/pairing process if not bonded but encryption is required
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_enterConnected(void)
{
    uint8_t *bdAddr;
    uint8_t bdAddrType;


    WICED_BT_TRACE("\nhidd_blelink_enterConnected");

#ifdef ALLOW_SDS_IN_DISCOVERABLE
    //stop discoverable timer
    osapi_deactivateTimer(&blelink.discoverable_timer);

    blelink.osapi_app_timer_running &= ~BLEHIDLINK_ADV_CONNECTABLE_UNDIRECTED_TIMER;
    if ((blelink.osapi_app_timer_running >> 1) == 0)
    {
        blelink.osapi_app_timer_running = 0; // no more application osapi timer is running
    }
#endif

    hidd_host_setAddrType(blelink.gatts_peer_addr, blelink.gatts_peer_addr_type);

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

    hidd_blelink_set_state(HIDLINK_LE_CONNECTED);
    // In any case, stop advertising
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);

    if(hidd_cfg()->security_requirement_mask)
    {
        if (!wiced_blehidd_is_device_bonded())
        {
            WICED_BT_TRACE("\nsend security req:");
            wiced_bt_dev_sec_bond(blelink.gatts_peer_addr, blelink.gatts_peer_addr_type, BT_TRANSPORT_LE, 0, NULL);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// LE connection up indication
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_connected(void)
{
    if (blelink.conn_idle_timeout)
    {
        // start the connection idle timer
        osapi_activateTimer( &blelink.conn_idle_timer, blelink.conn_idle_timeout * 1000000UL); //timout in micro seconds.
        blelink.osapi_app_timer_start_instant = clock_SystemTimeMicroseconds64();
        blelink.osapi_app_timer_running |= BLEHIDLINK_CONNECTION_IDLE_TIMER;
        blelink.osapi_app_timer_running |= 1;
    }

    hidd_blelink_enterConnected();
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// LE connection down indication
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_disconnected(void)
{
    //stop the connection idle timer
    osapi_deactivateTimer(&blelink.conn_idle_timer);

    blelink.osapi_app_timer_running &= ~BLEHIDLINK_CONNECTION_IDLE_TIMER;
    if ((blelink.osapi_app_timer_running >> 1) == 0)
    {
        blelink.osapi_app_timer_running = 0; // no more application osapi timer is running
    }

#ifdef AUTO_RECONNECT
    //reconnect back if bonded with host and not in the process of lowbattery shut down
    //and disconnect is not due to virtual cable unplug
    if(blelink.auto_reconnect && !blelink.pendingStateTransiting)
        hidd_link_delayed_reconnect(500); //auto reconnect in 500 ms
#endif
    //clear link encrypted flag when disconnected
    wiced_blehidd_set_link_encrypted_flag(WICED_FALSE);

    hidd_blelink_enterDisconnected();
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// LE advertising (except high duty cycle directed adv) stop indication
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_adv_stop(void)
{
    WICED_BT_TRACE("\nhidd_blelink_adv_stop");

#ifdef ALLOW_SDS_IN_DISCOVERABLE
    //stop discoverable timer
    osapi_deactivateTimer(&blelink.discoverable_timer);

    blelink.osapi_app_timer_running &= ~BLEHIDLINK_ADV_CONNECTABLE_UNDIRECTED_TIMER;
    if ((blelink.osapi_app_timer_running >> 1) == 0)
    {
        blelink.osapi_app_timer_running = 0; // no more application osapi timer is running
    }
#endif

    //if not connected, must be adv timeout
    if (!hidd_blelink_is_connected())
    {
        hidd_blelink_set_state(HIDLINK_LE_DISCONNECTED);
    }

    if (blelink.second_conn_state == BLEHIDLINK_2ND_CONNECTION_ALLOWED)
    {
        //set 2nd connection state to default
        blelink.second_conn_state = BLEHIDLINK_2ND_CONNECTION_NOT_ALLOWED;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// High duty cycle directed advertising stop indication
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_directed_adv_stop(void)
{
    WICED_BT_TRACE("\nhidd_blelink_DirectedAdvStop");

//#if defined(ENDLESS_LE_ADVERTISING_WHILE_DISCONNECTED) && is_SDS_capable
#if 0
    hidd_blelink_set_state(HIDLINK_LE_ADVERTISING_IN_uBCS_DIRECTED);
#else
 #ifdef FILTER_ACCEPT_LIST_FOR_ADVERTISING
    //update advertising filer policy to use Filter Accept List to filter scan and connect request
    wiced_btm_ble_update_advertisement_filter_policy(0x03);

    blelink.adv_filter_accept_list_enabled = 0x03;
 #endif

    // start undirected connectable advertising.
    if (wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL))
        WICED_BT_TRACE("\nFailed to undirected connectable advertising!!!");

 #ifdef ALLOW_SDS_IN_DISCOVERABLE
    osapi_activateTimer( &blelink.discoverable_timer, 60000000UL); //60 seconds. timout in micro seconds.
    blelink.osapi_app_timer_start_instant = clock_SystemTimeMicroseconds64();
    blelink.osapi_app_timer_running |= BLEHIDLINK_ADV_CONNECTABLE_UNDIRECTED_TIMER;
    blelink.osapi_app_timer_running |= 1;

    hidd_blelink_set_state(HIDLINK_LE_ADVERTISING_IN_uBCS_UNDIRECTED);
 #endif
#endif
}

/////////////////////////////////////////////////////////////////////////////////
/// check if the link state is in requested state
///
/// \return TRUE/FALSE
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t  hidd_blelink_state_is(uint8_t state)
{
    return (blelink.subState == state);
}

/////////////////////////////////////////////////////////////////////////////////
/// Connect
/// As LE peripheral, it means start LE advertising
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_connect(void)
{
    if(hidd_host_isBonded())
    {
        switch(blelink.subState)
        {
            case HIDLINK_LE_DISCONNECTED:
                hidd_blelink_enterReconnecting();
                break;
            default:
                 //WICED_BT_TRACE("\nhidd_blelink_connect(bonded):%d",blelink.subState);
                break;
        }
    }
    else
    {
        switch(blelink.subState)
        {
            case HIDLINK_LE_DISCONNECTED:
                hidd_blelink_enterDiscoverable(WICED_TRUE);
                break;
            default:
                //WICED_BT_TRACE("\nhidd_blelink_connect(not bonded):%d",blelink.subState);
                break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// Disconnect
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_disconnect(void)
{
    wiced_bt_gatt_disconnect(blelink.gatts_conn_id);
}

/////////////////////////////////////////////////////////////////////////////////
/// Add new observer for link state changed.
/// Whenever link state changed, the observer will be notified.
///
/// \param observer - pointer to the callback function
///
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_add_state_observer(wiced_ble_hidd_state_change_callback_t* observer)
{
    LinkStateObserver* ob = (LinkStateObserver*)wiced_memory_permanent_allocate(sizeof(LinkStateObserver));

    // If allocation was OK, put this registration in the SL
    if(ob)
    {
        ob->callback = observer;
        ob->next = blelink.firstStateObserver;
        blelink.firstStateObserver = ob;
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// become disconnected
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_enterDisconnected(void)
{
//    WICED_BT_TRACE("\nLE enterDisconnected");

#ifdef FILTER_ACCEPT_LIST_FOR_ADVERTISING
    //clear Filter Accept List
    wiced_bt_ble_clear_filter_accept_list();
#endif

    hidd_blelink_set_state(HIDLINK_LE_DISCONNECTED);

#ifdef AUTO_RECONNECT
    hidd_link_delayed_reconnect(AUTO_RECONNECT_DELAY);
#endif

    if (blelink.pendingStateTransiting)
    {
        blelink.pendingStateTransiting = 0;
        if (blelink.stateTransitingFunc)
        {
            blelink.stateTransitingFunc(1);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// become discoverable. i.e. start connectable undirected advertising
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_enterDiscoverable(uint32_t SDS_allow)
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

    hidd_blelink_set_state(HIDLINK_LE_DISCOVERABLE);

#ifdef ALLOW_SDS_IN_DISCOVERABLE
    osapi_activateTimer( &blelink.discoverable_timer, 60000000UL); //60 seconds. timout in micro seconds.
    blelink.osapi_app_timer_start_instant = clock_SystemTimeMicroseconds64();
    blelink.osapi_app_timer_running |= BLEHIDLINK_ADV_CONNECTABLE_UNDIRECTED_TIMER;
    blelink.osapi_app_timer_running |= 1;

    if (SDS_allow)
    {
        //switch to advertising w/ SDS after 1 second
        if (wiced_is_timer_in_use(&blelink.state_switch_timer))
        {
            wiced_stop_timer(&blelink.state_switch_timer);
        }
        wiced_start_timer(&blelink.state_switch_timer,blelink.state_switch_timeout_in_ms);
    }

    //set it back to 1 second
    blelink.state_switch_timeout_in_ms = 1000; // 1 second
#endif

}

/////////////////////////////////////////////////////////////////////////////////
///allow discoverable while in connection, i.e. start connectable undirected advertisingnected
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_allowDiscoverable(void)
{
    WICED_BT_TRACE("\nallowDiscoverable");

    //do nothing if we are in DISCOVERABLE state alreay
    if (blelink.second_conn_state || (blelink.subState == HIDLINK_LE_DISCOVERABLE))
        return;

    //stop advertising anyway
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);

#ifdef FILTER_ACCEPT_LIST_FOR_ADVERTISING
    //set advertising filer policy to default (Filter Accept List is not used)
    wiced_btm_ble_update_advertisement_filter_policy(0);

    blelink.adv_filter_accept_list_enabled = 0;
#endif

    if (blelink.subState != HIDLINK_LE_CONNECTED)
    {
        hidd_blelink_enterDiscoverable(WICED_TRUE);
    }
    else
    {
        //save the existing connection's info
        memcpy(&blelink.existing_emconinfo, &emConInfo_devInfo, sizeof(EMCONINFO_DEVINFO));

        // start undirected connectable advertising.
        if (wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL))
            WICED_BT_TRACE("\nFailed to start undirected connectable advertising!!!");

#ifdef ALLOW_SDS_IN_DISCOVERABLE
        //start discoverable timer in normal mode
        osapi_activateTimer( &blelink.discoverable_timer, 60000000UL); //60 seconds. timout in micro seconds.
        blelink.osapi_app_timer_start_instant = clock_SystemTimeMicroseconds64();
        blelink.osapi_app_timer_running |= BLEHIDLINK_ADV_CONNECTABLE_UNDIRECTED_TIMER;
        blelink.osapi_app_timer_running = 1;
#endif
        blelink.second_conn_state = BLEHIDLINK_2ND_CONNECTION_ALLOWED;
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// set new link state and notify observers.
///
/// \param newState - the new link state
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_set_state(uint8_t newState)
{
    LinkStateObserver* tmpObs = blelink.firstStateObserver;

    if(newState != blelink.subState)
    {
        WICED_BT_TRACE("\nLE state changed from %s to %s ", hidd_link_state_str(blelink.subState), hidd_link_state_str(newState));
        blelink.subState = newState;

        hci_control_send_state_change(BT_TRANSPORT_LE, newState);

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
/// handler in MPAF(i.e. application) thread when application is polled
/////////////////////////////////////////////////////////////////////////////////
int hidd_blelink_poll_timer_expired_action(void* unused)
{
    if(link.callbacks && link.callbacks->p_app_poll_user_activities && link.ble_pollEnabled)
    {
        link.callbacks->p_app_poll_user_activities();
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
void hidd_blelink_poll_timer_expired_notice(void* task, uint32_t context)
{
    wiced_app_event_serialize(hidd_blelink_poll_timer_expired_action, NULL);
}

/////////////////////////////////////////////////////////////////////////////////
/// Enable application poll
///
/// \param enable - WICED_TRUE/WICED_FALSE
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_enable_poll_callback(wiced_bool_t enable)
{
    wiced_hidd_register_callback_for_poll_event(BT_TRANSPORT_LE, wiced_blehidd_get_peer_addr(), enable, hidd_blelink_poll_timer_expired_notice);
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
uint8_t hidd_blelink_send_report(uint8_t reportID, wiced_hidd_report_type_t reportType, uint8_t *data, uint8_t length)
{
    wiced_bt_gatt_status_t rptSentStatus = wiced_blehidd_send_report(blelink.gatts_conn_id, reportID, reportType, data, length);

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
    hidd_deep_sleep_not_allowed(1000);// No deep sleep for 1 second.

    // Whenever there is an activity, restart the connection idle timer
    if (blelink.conn_idle_timeout)
    {
        osapi_activateTimer( &blelink.conn_idle_timer, blelink.conn_idle_timeout * 1000000UL); //timout in micro seconds.
        blelink.osapi_app_timer_start_instant = clock_SystemTimeMicroseconds64();
        blelink.osapi_app_timer_running |= BLEHIDLINK_CONNECTION_IDLE_TIMER;
        blelink.osapi_app_timer_running |= 1;
    }

    return rptSentStatus;
}

/////////////////////////////////////////////////////////////////////////////////
/// virtual cable unplug.
/// This function will remove all HID host information and start connectable undirected advertising
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_virtual_cable_unplug(void)
{
    if (hidd_host_isBonded())
    {
        uint8_t *bonded_bdadr = hidd_host_addr();
#ifdef FILTER_ACCEPT_LIST_FOR_ADVERTISING
        //stop advertising anyway before Filter Accept List operation
        wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);

        //remove from Filter Accept List
        WICED_BT_TRACE("\nremove from Filter Accept List : %B", bonded_bdadr);
        wiced_bt_ble_update_advertising_filter_accept_list(WICED_FALSE, bonded_bdadr);

        //clear whitlist
        wiced_bt_ble_clear_filter_accept_list();
#endif

        WICED_BT_TRACE("\nremove bonded device : %B", bonded_bdadr);
        wiced_bt_dev_delete_bonded_device(bonded_bdadr);
    }

    WICED_BT_TRACE("\nRemoving all bonded info");
    hidd_host_remove_all();

    WICED_BT_TRACE("\nclear device bonded flag");
    wiced_blehidd_set_device_bonded_flag(WICED_FALSE);

#ifdef FILTER_ACCEPT_LIST_FOR_ADVERTISING
    //set advertising filer policy to default (Filter Accept List not used)
    wiced_btm_ble_update_advertisement_filter_policy(0);

    blelink.adv_filter_accept_list_enabled = 0;
#endif

    if (blelink.subState == HIDLINK_LE_CONNECTED)
    {

#ifdef EASY_PAIR
        blelink.pendingStateTransiting = 1;
        blelink.stateTransitingFunc = hidd_blelink_easyPair;
#else
        blelink.pendingStateTransiting = 0;
        blelink.stateTransitingFunc = NULL;
 #ifdef ALLOW_SDS_IN_DISCOVERABLE
        //For virtual cable unplug, give it more time before allowing SDS; otherwise, it might reset.
        blelink.state_switch_timeout_in_ms = 5000; // 5 seconds.  give it more time before allowing SDS; otherwise, it might reset.
 #endif
#endif
        //disconnect the link
        hidd_blelink_disconnect();
    }
    else if (blelink.subState == HIDLINK_LE_RECONNECTING)
    {
        wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);

#ifdef EASY_PAIR
        hidd_blelink_easyPair(0);
#else
        hidd_blelink_enterDiscoverable(WICED_TRUE);
#endif

    }
    else if (blelink.subState != HIDLINK_LE_DISCOVERABLE)
    {
#ifdef EASY_PAIR
        hidd_blelink_easyPair(0);
#else
//        hidd_blelink_enterDiscoverable(WICED_TRUE);
#endif
    }

    //if reconnect timer is running, stop it
    hidd_link_stop_reconnect_timer();
}

/////////////////////////////////////////////////////////////////////////////////
/// set ble HID link connection Idle timer timeout value in seconds (default is 0, i.e. no timeout)
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_set_connection_Idle_timeout_value(uint16_t value)
{
    blelink.conn_idle_timeout  = value;
}

/////////////////////////////////////////////////////////////////////////////////
/// request asymmetric peripheral latency.
/// this is useful when central doesn't accept the connection parameter update req
/// peripheral can enable asymmetric peripheral latency to lower power consumption
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_set_peripheral_latency(uint16_t peripheralLatencyinmS)
{
    UINT16 latency_plus_one = peripheralLatencyinmS/wiced_blehidd_get_connection_interval() * 4/5;

    hidd_cfg()->ble_scan_cfg.conn_latency = latency_plus_one - 1;
    hidd_cfg()->ble_scan_cfg.conn_min_interval =
    hidd_cfg()->ble_scan_cfg.conn_max_interval = wiced_blehidd_get_connection_interval();

    WICED_BT_TRACE("\nhidd_blelink_set_peripheral_latency: interval=%d, peripherallatency=%d",
               hidd_cfg()->ble_scan_cfg.conn_min_interval,
               hidd_cfg()->ble_scan_cfg.conn_latency);
    wiced_blehidd_set_asym_peripheral_latency(wiced_blehidd_get_connection_handle(), hidd_cfg()->ble_scan_cfg.conn_latency);
}

/////////////////////////////////////////////////////////////////////////////////
/// handler when received LE Connection Update Complete event
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_blelink_conn_param_updated()
{
    return blelink.connection_param_updated;
}

/////////////////////////////////////////////////////////////////////////////////
/// handler when received LE Connection Update Complete event
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_conn_update_complete(void)
{
    WICED_BT_TRACE("\nConnParamUpdate:Interval:%d, Latency:%d, Supervision TO:%d",
                 wiced_blehidd_get_connection_interval(),wiced_blehidd_get_peripheral_latency(),wiced_blehidd_get_supervision_timeout());

#ifndef  SKIP_CONNECT_PARAM_UPDATE_EVEN_IF_NO_PREFERED
    if ((wiced_blehidd_get_connection_interval() < hidd_cfg()->ble_scan_cfg.conn_min_interval) ||
        (wiced_blehidd_get_connection_interval() > hidd_cfg()->ble_scan_cfg.conn_max_interval) ||
        (wiced_blehidd_get_peripheral_latency() != hidd_cfg()->ble_scan_cfg.conn_latency))
    {
#ifdef ASSYM_PERIPHERAL_LATENCY
        //if actual peripheral latency is smaller than desired peripheral latency, set asymmetric peripheral latency in the peripheral side
        if (wiced_blehidd_get_connection_interval()*(wiced_blehidd_get_peripheral_latency() + 1) <
                hidd_cfg()->ble_scan_cfg.conn_min_interval * (hidd_cfg()->ble_scan_cfg.conn_latency + 1))
            hidd_blelink_set_peripheral_latency(hidd_cfg()->ble_scan_cfg.conn_min_interval*(hidd_cfg()->ble_scan_cfg.conn_latency+1)*5/4);
#else
        hidd_blelink_conn_param_update();
#endif
        blelink.connection_param_updated = WICED_TRUE;
    }
#endif
}

/////////////////////////////////////////////////////////////////////////////////
/// request Connection Parameter Update
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_conn_param_update(void)
{
    if ((wiced_blehidd_get_connection_interval() < hidd_cfg()->ble_scan_cfg.conn_min_interval) ||
            (wiced_blehidd_get_connection_interval() > hidd_cfg()->ble_scan_cfg.conn_max_interval) ||
            (wiced_blehidd_get_peripheral_latency() != hidd_cfg()->ble_scan_cfg.conn_latency))
    {
        WICED_BT_TRACE("\nsend conn param request");
        wiced_bt_l2cap_update_ble_conn_params(blelink.gatts_peer_addr,
                          hidd_cfg()->ble_scan_cfg.conn_min_interval,
                          hidd_cfg()->ble_scan_cfg.conn_max_interval,
                          hidd_cfg()->ble_scan_cfg.conn_latency,
                          hidd_cfg()->ble_scan_cfg.conn_supervision_timeout);
    }
    blelink.connection_param_updated = WICED_TRUE;
}

////////////////////////////////////////////////////////////////////////////////////
/// save/restore contents to/from Always On Memory when entering/exiting SDS
///
/// \param type - HIDD_LINK_SAVE_TO_AON or HIDD_LINK_RESTORE_FROM_AON
////////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_aon_action_handler(uint8_t  type)
{
    if (type == HIDD_LINK_RESTORE_FROM_AON)
    {
        WICED_BT_TRACE("\nWICED_BT_AON_DRIVER_RESTORE");

        blelink.resumeState = ble_aon_data.hidd_blelink_state;
        blelink.gatts_conn_id = ble_aon_data.gatts_conn_id;
        blelink.gatts_peer_addr_type = ble_aon_data.gatts_peer_addr_type;
        memcpy(blelink.gatts_peer_addr, ble_aon_data.gatts_peer_addr, BD_ADDR_LEN);

        //restore emconinfo
        blelink.resume_emconinfo.connHandle = ble_aon_data.emconinfo.connHandle;
        blelink.resume_emconinfo.flag       = ble_aon_data.emconinfo.flag;
        blelink.resume_emconinfo.peerAddressType = ble_aon_data.emconinfo.peerAddressType;
        memcpy(blelink.resume_emconinfo.peerAddress, ble_aon_data.emconinfo.peerAddress, BD_ADDR_LEN);
        blelink.resume_emconinfo.connInterval = ble_aon_data.emconinfo.connInterval;
        blelink.resume_emconinfo.connLatency  = ble_aon_data.emconinfo.connLatency;
        blelink.resume_emconinfo.supervisionTimeout = ble_aon_data.emconinfo.supervisionTimeout;

        //restore application timer info
        blelink.osapi_app_timer_start_instant = ble_aon_data.osapi_app_timer_start_instant;
        blelink.osapi_app_timer_running = ble_aon_data.osapi_app_timer_running;

#ifdef FILTER_ACCEPT_LIST_FOR_ADVERTISING
        blelink.adv_filter_accept_list_enabled = ble_aon_data.adv_filter_accept_list_enabled;
#endif
    }
    else
    {
#if !defined(CYW20819A1) && !defined(CYW20820A1) /* Slimboot is not supported in 20819A1 */
        // save all output GPIO values in the saved cfgs before entering uBCS mode
        wiced_hal_gpio_slimboot_reenforce_outputpin_value();

        ble_aon_data.hidd_blelink_state = blelink.subState;
        ble_aon_data.gatts_conn_id = blelink.gatts_conn_id;
        ble_aon_data.gatts_peer_addr_type = blelink.gatts_peer_addr_type;
        memcpy(ble_aon_data.gatts_peer_addr, blelink.gatts_peer_addr, BD_ADDR_LEN);

        //save emconinfo
        ble_aon_data.emconinfo.connHandle = emConInfo_devInfo.connHandle;
        ble_aon_data.emconinfo.flag       = emConInfo_devInfo.flag;
        ble_aon_data.emconinfo.peerAddressType = emConInfo_devInfo.peerAddressType;
        memcpy(ble_aon_data.emconinfo.peerAddress, emConInfo_devInfo.peerAddress, BD_ADDR_LEN);
        ble_aon_data.emconinfo.connInterval = emConInfo_devInfo.connInterval;
        ble_aon_data.emconinfo.connLatency  = emConInfo_devInfo.connLatency;
        ble_aon_data.emconinfo.supervisionTimeout = emConInfo_devInfo.supervisionTimeout;

        //save application timer info
        ble_aon_data.osapi_app_timer_start_instant = blelink.osapi_app_timer_start_instant;
        ble_aon_data.osapi_app_timer_running = blelink.osapi_app_timer_running;

#ifdef FILTER_ACCEPT_LIST_FOR_ADVERTISING
        ble_aon_data.adv_filter_accept_list_enabled = blelink.adv_filter_accept_list_enabled;
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
void hidd_blelink_stateswitchtimerCb( uint32_t arg)
{
    if (HIDLINK_LE_DISCOVERABLE == blelink.subState)
    {
        hidd_blelink_set_state(HIDLINK_LE_ADVERTISING_IN_uBCS_UNDIRECTED);
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// timeout handler
///
/// \param args - don't care
/// \param overTimeInUs - don't care
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_discoverabletimerCb(INT32 args, UINT32 overTimeInUs)
{
    WICED_BT_TRACE("\nhidd_blelink_discoverabletimerCb!!!");
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
}
#endif


#endif //#ifdef BLE_SUPPORT
