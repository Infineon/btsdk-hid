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
* File Name: hidd_blelink.c
*
* Abstract: This file implements the LE HID application transport
*
* Functions:
*
*******************************************************************************/
#if BTSTACK_VER < 0x03000001
#ifdef BLE_SUPPORT

#include "wiced_bt_trace.h"
#include "wiced_hal_batmon.h"
#include "hidd_lib.h"

PLACE_DATA_IN_RETENTION_RAM blehid_aon_save_content_t   ble_aon_data;

void hidd_blelink_pr_link_key(wiced_bt_device_link_keys_t * p_link_keys)
{
    WICED_BT_TRACE("\nmask           :  x%02X", p_link_keys->key_data.le_keys_available_mask);
    WICED_BT_TRACE("\nLE AddrType   :  %d", p_link_keys->key_data.ble_addr_type);
    WICED_BT_TRACE("\nStatic AddrType:  %d", p_link_keys->key_data.static_addr_type);
    WICED_BT_TRACE("\nStatic Addr    :  %B", p_link_keys->key_data.static_addr);
    STRACE_ARRAY  ("\n  irk: ", &(p_link_keys->key_data.le_keys.irk), LINK_KEY_LEN);
#if SMP_INCLUDED == TRUE && SMP_LE_SC_INCLUDED == TRUE
    STRACE_ARRAY  ("\n pltk: ", &(p_link_keys->key_data.le_keys.pltk), LINK_KEY_LEN);
    STRACE_ARRAY  ("\npcsrk: ", &(p_link_keys->key_data.le_keys.pcsrk), LINK_KEY_LEN);
    STRACE_ARRAY  ("\n lltk: ", &(p_link_keys->key_data.le_keys.lltk), LINK_KEY_LEN);
    STRACE_ARRAY  ("\nlcsrk: ", &(p_link_keys->key_data.le_keys.lcsrk), LINK_KEY_LEN);
#else
    STRACE_ARRAY  ("\n ltk: ", &(p_link_keys->key_data.le_keys.ltk), LINK_KEY_LEN);
    STRACE_ARRAY  ("\ncsrk: ", &(p_link_keys->key_data.le_keys.csrk), LINK_KEY_LEN);
#endif
}

#if !is_20819Family
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
                hidd_blelink_start_timer( &blelink.conn_idle_timer, remaining_time_in_ms); //timout in milliseconds.
            }
        }

        blelink.osapi_app_timer_running = 0;
    }

 #if is_SDS_capable && (defined(ENDLESS_LE_ADVERTISING) || defined(ALLOW_SDS_IN_DISCOVERABLE))
    if ((HIDLINK_LE_ADVERTISING_IN_uBCS_DIRECTED == blelink.resumeState) ||
        (HIDLINK_LE_ADVERTISING_IN_uBCS_UNDIRECTED == blelink.resumeState))
    {
        extern wiced_bool_t btsnd_hcic_ble_set_adv_enable (UINT8 adv_enable);
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

 #if is_SDS_capable && (defined(ENDLESS_LE_ADVERTISING) || defined(ALLOW_SDS_IN_DISCOVERABLE))
        //if no user activity and not wake up due to application timer timeout. restart adv again
        if ((HIDLINK_LE_DISCONNECTED == blelink.subState) && !blelink.wake_from_SDS_timer_timeout_flag)
        {
  #ifdef ENDLESS_LE_ADVERTISING
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

#endif // BLE_SUPPORT

#endif // #if BTSTACK_VER < 0x03000001
