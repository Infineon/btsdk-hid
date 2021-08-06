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
* File Name: hidd_link.c
*
* Functions:
*
*******************************************************************************/
#include "wiced_bt_trace.h"
#include "wiced_hal_batmon.h"
#include "wiced_hal_mia.h"
#include "hidd_lib.h"

hidd_link_t link;

#if is_SDS_capable
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
void hidd_link_aon_action_handler(uint8_t  type)
{
 #ifdef BLE_SUPPORT
    hidd_blelink_aon_action_handler(type);
 #endif
 #ifdef BR_EDR_SUPPORT
    hidd_btlink_aon_action_handler(type);
 #endif
}
#endif

#ifdef SUPPORT_CODE_ENTRY
////////////////////////////////////////////////////////////////////////////////
/// Provide pin code to the BT transport. This should only be done in
/// response to a pin code request from the transport. This method
/// unconditionally flags that we are not requesting a pin code from the
/// application. Further action depends on the state and is as follows:
/// - DISCOVERABLE: if we have a (partial) connection, we assume that this
///      message is in response to a pin code request from us and pass
///      this response to the BT stack. Otherwise it is discarded.
/// - CONNECTED: Same as DISCOVERABLE
/// - All other state: Ignored.
////////////////////////////////////////////////////////////////////////////////
void hidd_link_pinCode(uint8_t pinCodeSize, uint8_t *pinCodeBuffer)
{
    // Flag that we are no longer requesting pin code from the application
    //hidd_link_pinCodeRequested = WICED_FALSE;

    // We are only interested in DISCOVERABLE and CONNECTED states
#if 0
    if ((subState == DISCOVERABLE || subState == CONNECTED) ||
        (hostList->isWaitForSdpEnabled(reconnectHostIndex) &&
         BT_MEMCMP(tmpAddr, &(*hostList)[reconnectHostIndex], sizeof(BD_ADDR)) == 0))
#else
    if ((bt_hidd_link.subState == HIDLINK_DISCOVERABLE || bt_hidd_link.subState == HIDLINK_CONNECTED))
#endif
    {
        wiced_bt_dev_pin_code_reply(link.hidd_link_passkeyreq_bdaddr, WICED_BT_SUCCESS, pinCodeSize, pinCodeBuffer);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// This method handles a pin code request from the BT core.
///  - INITIALIZED: Impossible. Ignore.
///  - DISCONNECTED: Impossible. Ignore.
///  - DISCOVERABLE: Acceptable to get a pin code request here.
///           If we are auto-pairing (enabled via auto-pairing HID report)
///           then handle the request locally using the configured code. Otherwise
///           request application to enter pin code entry state.
///           Note that auto-pairing can happen in DISCOVERABLE state as the
///           auto-pairing request comes over the control channel, which
///           may be open without the interrupt channel being open.
///  - CONNECTABLE: Implies other side lost the link key or the other side
///           is acting under false pretences.
///           Tell BT conn to disconnect but stay in this state.
///  - CONNECTED: We can get a pin code request in CONNECTED state as some
///           stacks open the interrupt channel before pairing.
///           If we are auto-pairing (enabled via auto-pairing HID report)
///           then handle the request locally using the configured code. Otherwise
///           flag that we have requested a pin code from the application and
///           request application to enter pin code entry state
///  - DISCONNECTING: We are already disconnecting. Ignore.
///  - RECONNECTING: Implies other side lost the link key or the other side
///           is acting under false pretences.
///           Tell BT conn to disconnect but stay in this state.
/// \param p_event_data remote device information.
////////////////////////////////////////////////////////////////////////////////
void hidd_link_pinCodeRequest(wiced_bt_dev_name_and_class_t *p_event_data)
{
    switch(bt_hidd_link.subState)
    {
        case HIDLINK_INITIALIZED:
        case HIDLINK_DISCONNECTED:
        case HIDLINK_DISCONNECTING:
            break;
        case HIDLINK_RECONNECTING:
        case HIDLINK_CONNECTABLE:
#if 0
            if(!(hostList->isWaitForSdpEnabled(reconnectHostIndex) &&
             BT_MEMCMP(bdAddr, &(*hostList)[reconnectHostIndex], sizeof(BD_ADDR)) == 0))
            {
                // If we were not waiting for the host to complete the connection, end it
                btHidConn->disconnect();
                break;
            }
#endif
        case HIDLINK_DISCOVERABLE:
        case HIDLINK_CONNECTED:
            //btLpm->pauseLpm();
#if 0
            // If we are in auto-pairing mode, we handle this locally
            if (autoPairingRequested)
            {
                // We are. Handle it locally.
                pinCode(autoPairingPinSize, autoPairingPin);

                // Auto-pairing is a one shot deatl
                autoPairingRequested = FALSE;
            }
            else
#endif
            {
                // Flag that we have requested pin code from the application
                //pinCodeRequested = TRUE;

                // Tell app to provide us pin code.
                if (link.callbacks && link.callbacks->p_app_enter_pincode_entry_mode)
                {
                    link.callbacks->p_app_enter_pincode_entry_mode();
                }
            }
            break;
    default:
        break;
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Provide a pass code to the transport. This should be done in response to a
/// pass code request from the transport. This method
/// unconditionally flags that we are not requesting a pin code from the
/// application. Further action depends on the state and is as follows:
///  - DISCOVERABLE: if we have a (partial) connection, we assume that this
///      message is in response to a pass code request from us and pass
///      this response to the BT stack. Otherwise it is discarded.
/// - All other state: Ignored.
/// NOTE: pinCodeBuffer is expected to be a null terminated string representation
///       of an unsigned interger between 0 and 999999, both inclusive.
////////////////////////////////////////////////////////////////////////////////
void hidd_link_passCode(uint8_t pinCodeSize, uint8_t *pinCodeBuffer)
{
    uint32_t passkey = 0;

    // Flag that we are no longer requesting pin code from the application
    //hidd_link_pinCodeRequested = WICED_FALSE;

    if(pinCodeSize && pinCodeBuffer)
    {
        if(bt_hidd_link.subState == HIDLINK_DISCOVERABLE)
        {
            uint8_t i;
            // Convert the char string to an unsigned int, base10
            //passkey = (UINT32) strtoul((const char*)pinCodeBuffer, NULL, 10);
            for (i=0; i<pinCodeSize; i++)
            {
              passkey = passkey * 10 + pinCodeBuffer[i] - '0';
            }
            WICED_BT_TRACE("\npasskey: %d",passkey);

            // Now pass the pass key to BTM.
            wiced_bt_dev_pass_key_req_reply(WICED_BT_SUCCESS, link.hidd_link_passkeyreq_bdaddr, passkey);
        }
    }
    else
    {
        // If app is not capable of responding to pass key, reject it
        wiced_bt_dev_pass_key_req_reply(WICED_BT_UNSUPPORTED, link.hidd_link_passkeyreq_bdaddr, 0);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// A pass key needs to be input by the user. This will be handled similar to pincode.
/// The app may enter a key right away or may defer it to a later point in time.
///  - INITIALIZED: Impossible. Ignore
///  - DISCONNECTED: Impossible. Ignore.
///  - DISCONNECTING: Possible race. Ignore because we are disconnecting any way.
///  - CONNECTED: Impossible - cannot do SSP when channels are already up. The other
///               may be incorrect. reject.
///  - DISCOVERABLE: Pairing in progress,
///
///  - CONNECTABLE: The peer may have lost the link key or acting under false pretences.
///                 Disconnect but stay in same state.
///  - RECONNECTING: THe peer may have lost the link key or acting under false pretences.
///                 Disconnect but stay in same state.
/// \param passKeyReq Metadata for the pass key request
////////////////////////////////////////////////////////////////////////////////
void hidd_link_passKeyRequest(wiced_bt_dev_user_key_req_t* passKeyReq)
{
    memcpy(link.hidd_link_passkeyreq_bdaddr, passKeyReq->bd_addr, BD_ADDR_LEN);

    //call back to application to enter Pass Key
    if (link.callbacks && link.callbacks->p_app_enter_passcode_entry_mode)
    {
        link.callbacks->p_app_enter_passcode_entry_mode();
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Provide a key press indication to the peer. If the subState is DISCOVERABLE,
/// uses BTM to send out a notification to the peer. Else ignored.
////////////////////////////////////////////////////////////////////////////////
void hidd_link_passCodeKeyPressReport(uint8_t key)
{
    switch(bt_hidd_link.subState)
    {
    case HIDLINK_CONNECTABLE:
    case HIDLINK_RECONNECTING:
#if 0
        if(!(hostList->isWaitForSdpEnabled(reconnectHostIndex) &&
             BT_MEMCMP(btHidConn->getBdAddr(), &(*hostList)[reconnectHostIndex], sizeof(BD_ADDR)) == 0))
        {
            break;
        }
#endif
        // else deliberate fallthrugh
    case HIDLINK_DISCOVERABLE:
        // Send the noti out
        wiced_bt_dev_send_key_press_notif(link.hidd_link_passkeyreq_bdaddr, (UINT8)key);
        break;

    default:
        break;
    }
}
#endif

////////////////////////////////////////////////////////////////////////////////
/// hidd_link_state_str -- this function returns link state string
///
/// \param state - link state
///
/// \return -- link state string
////////////////////////////////////////////////////////////////////////////////
const char * hidd_link_state_str(uint8_t state)
{
    const char * stateStr[] = {"initialized",
                               "disconnected",
                               "discoverable",
                               "connectable",
                               "connected",
                               "disconnecting",
                               "reconnecting",
                               "uBCS_direct_adv",
                               "uBCS_undirect_adv",
                               "invalid"};
    state &= HIDLINK_MASK;
    if (state >= HIDLINK_INVLIAD)
        state = HIDLINK_INVLIAD;
    return stateStr[state];
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// hidd_link_enable_poll_callback
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_link_enable_poll_callback(uint8_t transport, wiced_bool_t enable)
{
#ifdef BLE_SUPPORT
    if (transport == BT_TRANSPORT_LE && link.ble_pollEnabled != enable)
    {
        link.ble_pollEnabled = enable;
        WICED_BT_TRACE("\nBLE enableAppPoll:%d", enable);
        hidd_blelink_enable_poll_callback(enable);
    }
#endif
#ifdef BR_EDR_SUPPORT
    if (transport == BT_TRANSPORT_BR_EDR && link.bredr_pollEnabled != enable)
    {
        link.bredr_pollEnabled = enable;
        WICED_BT_TRACE("\nBT enableAppPoll:%d", enable);
        hidd_btlink_enable_poll_callback(enable);
    }
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// hidd_link_connect
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_link_connect()
{
#ifdef BLE_SUPPORT
    if (hidd_host_transport()==BT_TRANSPORT_LE)
    {
        // for BLE there is no channel, skip p_data[0]
        hidd_blelink_connect();
    }
#endif
#ifdef BR_EDR_SUPPORT
    if (hidd_host_transport() == BT_TRANSPORT_BR_EDR)
    {
        hidd_btink_connect();
    }
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// hidd_link_disconnect
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_link_disconnect()
{
#ifdef BLE_SUPPORT
    if (hidd_host_transport()==BT_TRANSPORT_LE)
    {
        // for BLE there is no channel, skip p_data[0]
        hidd_blelink_disconnect();
    }
#endif
#ifdef BR_EDR_SUPPORT
    if (hidd_host_transport() == BT_TRANSPORT_BR_EDR)
    {
        hidd_btlink_disconnect();
    }
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_link_is_connected()
{
#ifdef BLE_SUPPORT
    if (hidd_host_transport() == BT_TRANSPORT_LE)
    {
        return hidd_blelink_is_connected();
    }
#endif
#ifdef BR_EDR_SUPPORT
    if (hidd_host_transport() == BT_TRANSPORT_BR_EDR)
    {
        return hidd_btlink_is_connected();
    }
#endif
    return FALSE;
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_link_is_disconnected()
{
    wiced_bool_t state =
#ifdef BLE_SUPPORT
        hidd_blelink_is_disconnected()
 #if BR_EDR_SUPPORT
        &&
 #endif
#endif
#ifdef BR_EDR_SUPPORT
        hidd_btlink_is_disconnected()
#endif
        ;
    return state;
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_link_is_encrypted()
{
#ifdef BLE_SUPPORT
    if (hidd_host_transport() == BT_TRANSPORT_LE)
    {
        return wiced_blehidd_is_link_encrypted();
    } else
#endif
#ifdef BR_EDR_SUPPORT
    if (hidd_host_transport() == BT_TRANSPORT_BR_EDR)
    {
        return bt_hidd_link.encrypt_status.encrypted;
    } else
#endif
    return FALSE;
}

/**
 * Writes the data to NVRAM,
 * Application can write up to 255 bytes in one VS  id section
 *
 * @param[in] vs_id        : Volatile Section Identifier. Application can use
 *                           the VS ids from WICED_NVRAM_VSID_START to
 *                           WICED_NVRAM_VSID_END
 *
 * @param[in] data_length  : Length of the data to be written to the NVRAM,
 *
 * @param[in] p_data       : Pointer to the data to be written to the NVRAM
 *
 * @param[out] p_status    : Pointer to location where status of the call
 *                           is returned
 *
 *
 * @return  number of bytes written, 0 on error
 */
uint16_t hidd_write_nvram( uint16_t vs_id, uint16_t data_length, uint8_t * p_data, wiced_result_t * p_status)
{
    uint16_t rtv = wiced_hal_write_nvram(vs_id, data_length, p_data, p_status);

    hidd_nvram_deep_sleep();
    return rtv;
}

/** Reads the data from NVRAM
 *
 * @param[in]  vs_id       : Volatile Section Identifier. Application can use
 *                           the VS ids from WICED_NVRAM_VSID_START to
 *                           WICED_NVRAM_VSID_END
 *
 * @param[in]  data_length : Length of the data to be read from NVRAM
 *
 * @param[out] p_data      : Pointer to the buffer to which data will be copied
 *
 * @param[out] p_status    : Pointer to location where status of the call
 *                           is returned
 *
 * @return  the number of bytes read, 0 on failure
 */
uint16_t hidd_read_nvram( uint16_t vs_id, uint16_t data_length, uint8_t * p_data, wiced_result_t * p_status)
{
    uint16_t rtv = wiced_hal_read_nvram(vs_id, data_length, p_data, p_status);

    hidd_nvram_deep_sleep();
    return rtv;
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_link_virtual_cable_unplug(void)
{
    wiced_bt_transport_t transport = hidd_host_transport();
#ifdef BLE_SUPPORT
    if (transport==BT_TRANSPORT_LE)
    {
        hidd_blelink_virtual_cable_unplug();
        return;
    }
#endif
#ifdef BR_EDR_SUPPORT
    if (transport==BT_TRANSPORT_BR_EDR)
    {
        hidd_btlink_virtual_cable_unplug();
        return;
    }
#endif
}

/******************************************************************************************/
/******************************************************************************************/
/**
 * Function         hidd_link_send_data
 *
 *                  Sends input reports to host.
 *
 *  @param[in]      ch          : interupt or control channel (for BR/EDR only)
 *  @param[in]      type        : Report type
 *  @param[in]      ptr         : Report pointer
 *  @param[in]      len         : Report length
 *
 *  @return         for BR/EDR status code (see #wiced_bt_hidd_status_e)
 *                  for BLE sttatus code (see #wiced_bt_gatt_status_e)
 */
wiced_bt_hidd_status_t hidd_link_send_data(uint8_t ch, uint8_t type, uint8_t *ptr, uint16_t len)
{
    uint8_t id = *ptr;

    switch (hidd_host_transport()) {
#ifdef BLE_SUPPORT
    case BT_TRANSPORT_LE:
        if (hidd_gatts_set_data(ptr, len))
        {
            // Send the report
            return hidd_blelink_send_report(id, type, ++ptr, --len); // pointer and length don't include report ID for BLE
        }
        break;
#endif
#ifdef BR_EDR_SUPPORT
    case BT_TRANSPORT_BR_EDR:
        return wiced_bt_hidd_send_data(ch == HCI_CONTROL_HID_REPORT_CHANNEL_CONTROL, type, ptr, len);
        break;
#endif
    }
    return WICED_BT_HIDD_ERR_UNSUPPORTED;
}

/******************************************************************************************/
/******************************************************************************************/
wiced_bt_hidd_status_t hidd_link_send_report(void *ptr, uint16_t len)
{
    return hidd_link_send_data(HCI_CONTROL_HID_REPORT_CHANNEL_INTERRUPT, HCI_CONTROL_HID_REPORT_TYPE_INPUT, ptr, len);
}

#ifdef AUTO_RECONNECT
///////////////////////////////////////////////////////////////////////////////
/// hidd_link_delayed_reconnect(uint32_t delay)
///////////////////////////////////////////////////////////////////////////////
void hidd_link_delayed_reconnect(uint32_t delay)
{
    if (hidd_is_paired() && !wiced_hal_batmon_is_low_battery_shutdown())
    {
        wiced_start_timer(&link.reconnect_timer, delay); //auto reconnect after 10 seconds.
    }
}
#endif

/////////////////////////////////////////////////////////////////////////////////
/// timeout handler
/// \param arg - don't care
/////////////////////////////////////////////////////////////////////////////////
void hidd_link_reconnect_timerCb( uint32_t arg)
{
    WICED_BT_TRACE("\nReconnect");
    hidd_link_connect();
}

///////////////////////////////////////////////////////////////////////////////
/// hidd_link_stop_reconnect_timer()
///////////////////////////////////////////////////////////////////////////////
BOOL8 hidd_link_is_reconnect_timer_running()
{
    return wiced_is_timer_in_use(&link.reconnect_timer);
}

///////////////////////////////////////////////////////////////////////////////
/// hidd_link_stop_reconnect_timer()
///////////////////////////////////////////////////////////////////////////////
void hidd_link_stop_reconnect_timer()
{
    if (hidd_link_is_reconnect_timer_running())
    {
        wiced_stop_timer(&link.reconnect_timer);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// Abstract link layer initialize
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_link_init()
{
    // initialize host list management
    hidd_host_init();

    //timer to auto reconnect when disconnected
    wiced_init_timer( &link.reconnect_timer, hidd_link_reconnect_timerCb, 0, WICED_MILLI_SECONDS_TIMER );

    //hid link init
#ifdef BLE_SUPPORT
    hidd_blelink_init();
    hidd_blelink_determine_next_state();
#endif
#ifdef BR_EDR_SUPPORT
    hidd_btlink_init();
    hidd_btlink_determine_next_state();
#endif

    wiced_hal_mia_enable_mia_interrupt(TRUE);
    wiced_hal_mia_enable_lhl_interrupt(TRUE);//GPIO interrupt
}
