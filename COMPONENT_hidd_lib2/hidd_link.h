/*
 * Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
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

/*******************************************************************************
*
* File Name: dualhidlink.h
*
* Abstract: DUAL (BT/BLE) Host List definitions and API functions
*
* Functions:
*
*******************************************************************************/
#ifndef __HIDD_LINK_H__
#define __HIDD_LINK_H__

#include "hidd_blelink.h"
#include "hidd_btlink.h"
#include "hci_control_api.h"

#define BT_TRANSPORT_NONE 0

enum
{
    /// No activity
    HIDLINK_ACTIVITY_NONE                   = 0x00,

    /// Reportable activity, e.g. motion, scroll, key or button press, etc.
    HIDLINK_ACTIVITY_REPORTABLE             = 0x01,

    /// Non-reportable activity, e.g. key or button held down
    HIDLINK_ACTIVITY_NON_REPORTABLE         = 0x02,

    /// Activities that require a transport to be connected, reportable
    HIDLINK_ACTIVITY_CONNECTABLE            = HIDLINK_ACTIVITY_REPORTABLE,

    /// Connect button pressed
    HIDLINK_ACTIVITY_CONNECT_BUTTON_DOWN    = 0x80
};

enum hidd_link_state_e
{
        /// The ble_link is initialized but inactive. This is the initial state.
        HIDLINK_INITIALIZED,

        /// Initialized, but idle (disconnected, not discoverable, not connectable)
        HIDLINK_DISCONNECTED,

        /// Advertising with a discoverable or connectable advertising event
        ///   with a discoverable flag set in LE.
        HIDLINK_DISCOVERABLE,

        /// Connectable with host(s) we have been cabled with. We do page scans
        /// in this state and only accept connections from hosts we know
        HIDLINK_CONNECTABLE,

        /// We have a connection with a host over which we can pass reports. In this state
        ///   both interrupt and control channels are open
        HIDLINK_CONNECTED,

        /// We are in the process of disconnecting
        HIDLINK_DISCONNECTING,

        /// Reconnecting to previously bonded host(s)
        HIDLINK_RECONNECTING,

        /// directed Advertising in uBCS 8mode
        HIDLINK_ADVERTISING_IN_uBCS_DIRECTED,

        /// undirected Advertising in uBCS mode
        HIDLINK_ADVERTISING_IN_uBCS_UNDIRECTED,

        HIDLINK_INVLIAD,

        HIDLINK_LE = 0x80,

        /// Initialized, but idle (disconnected, not discoverable, not connectable)
        HIDLINK_LE_DISCONNECTED = HIDLINK_LE | HIDLINK_DISCONNECTED,

        /// Advertising with a discoverable or connectable advertising event
        ///   with a discoverable flag set in LE.
        HIDLINK_LE_DISCOVERABLE = HIDLINK_LE | HIDLINK_DISCOVERABLE,

        /// We have a connection with a host over which we can pass reports. In this state
        ///   both interrupt and control channels are open
        HIDLINK_LE_CONNECTED = HIDLINK_LE | HIDLINK_CONNECTED,

        /// Reconnecting to previously bonded host(s)
        HIDLINK_LE_RECONNECTING = HIDLINK_LE | HIDLINK_RECONNECTING,

        /// directed Advertising in uBCS 8mode
        HIDLINK_LE_ADVERTISING_IN_uBCS_DIRECTED = HIDLINK_LE | HIDLINK_ADVERTISING_IN_uBCS_DIRECTED,

        /// undirected Advertising in uBCS mode
        HIDLINK_LE_ADVERTISING_IN_uBCS_UNDIRECTED = HIDLINK_LE | HIDLINK_ADVERTISING_IN_uBCS_UNDIRECTED,

        HIDLINK_MASK = 0x7f,

};

enum
{
    HIDD_LINK_SAVE_TO_AON = 0,     /* Save context from SRAM to AON */
    HIDD_LINK_RESTORE_FROM_AON,    /* Restore context from AON to SRAM */
};

typedef void hidd_link_app_poll_callback_t(void);
typedef void hidd_link_app_connection_failed_callback_t(void);
typedef void hidd_link_app_enter_pincode_entry_mode_callback_t(void);
typedef void hidd_link_app_enter_passcode_entry_mode_callback_t(void);
typedef void hidd_link_app_exit_pin_and_passcode_entry_mode_callback_t(void);
typedef uint8_t hidd_link_app_get_idle_callback_t(void);
typedef uint8_t hidd_link_app_set_idle_callback_t(uint8_t idleRateIn4msUnits);
typedef uint8_t hidd_link_app_get_protocol_callback_t(void);
typedef uint8_t hidd_link_app_set_protocol_callback_t(uint8_t);
typedef uint8_t hidd_link_app_get_report_callback_t( uint8_t reportType, uint8_t reportId);
typedef uint8_t hidd_link_app_set_report_callback_t(uint8_t reportType, uint8_t *payload, uint16_t payloadSize);
typedef void hidd_link_app_rx_data_callback_t(uint8_t reportType, uint8_t *payload, uint16_t payloadSize);

typedef struct
{
    hidd_link_app_poll_callback_t                                *p_app_poll_user_activities;
    hidd_link_app_connection_failed_callback_t                   *p_app_connection_failed_notification;

    hidd_link_app_enter_pincode_entry_mode_callback_t            *p_app_enter_pincode_entry_mode;
    hidd_link_app_enter_passcode_entry_mode_callback_t           *p_app_enter_passcode_entry_mode;
    hidd_link_app_exit_pin_and_passcode_entry_mode_callback_t    *p_app_exit_pin_and_passcode_entry_mode;

    hidd_link_app_get_idle_callback_t      *p_app_get_idle;
    hidd_link_app_set_idle_callback_t      *p_app_set_idle;
    hidd_link_app_get_protocol_callback_t  *p_app_get_protocol;
    hidd_link_app_set_protocol_callback_t  *p_app_set_protocol;
    hidd_link_app_get_report_callback_t    *p_app_get_report;
    hidd_link_app_set_report_callback_t    *p_app_set_report;
    hidd_link_app_rx_data_callback_t       *p_app_rx_data;

}hidd_link_callback_t;

typedef struct
{
    hidd_link_callback_t * callbacks;
    wiced_timer_t reconnect_timer;
    BD_ADDR hidd_link_passkeyreq_bdaddr;
    uint8_t bredr_pollEnabled:1;
    uint8_t ble_pollEnabled:1;
} hidd_link_t;
extern hidd_link_t link;

/******************************************************************************************/
/******************************************************************************************
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
 *
 ******************************************************************************************/
wiced_bt_hidd_status_t hidd_link_send_data(uint8_t ch, uint8_t type, uint8_t *ptr, uint16_t len);

/******************************************************************************************/
/******************************************************************************************/
wiced_bt_hidd_status_t hidd_link_send_report(void *ptr, uint16_t len);

/******************************************************************************************/
// Host functions //////////////////////////////////////////////////////////////////////////
/******************************************************************************************/

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void hidd_link_virtual_cable_unplug(void);

///////////////////////////////////////////////////////////////////////////////
/// hidd_link_connect()
///////////////////////////////////////////////////////////////////////////////
void hidd_link_connect();

#ifdef AUTO_RECONNECT
#define AUTO_RECONNECT_DELAY 3000
///////////////////////////////////////////////////////////////////////////////
/// hidd_link_delayed_reconnect()
///////////////////////////////////////////////////////////////////////////////
void hidd_link_delayed_reconnect(uint32_t delay);
#endif

///////////////////////////////////////////////////////////////////////////////
/// hidd_link_stop_reconnect_timer()
///////////////////////////////////////////////////////////////////////////////
void hidd_link_stop_reconnect_timer();

///////////////////////////////////////////////////////////////////////////////
/// BOOL hidd_link_is_reconnect_timer_running()
///////////////////////////////////////////////////////////////////////////////
BOOL8 hidd_link_is_reconnect_timer_running();

///////////////////////////////////////////////////////////////////////////////
/// hidd_link_disconnect()
///////////////////////////////////////////////////////////////////////////////
void hidd_link_disconnect();

///////////////////////////////////////////////////////////////////////////////
/// returns corrent link state
///////////////////////////////////////////////////////////////////////////////
uint8_t hidd_link_state();

///////////////////////////////////////////////////////////////////////////////
/// returns if HID link is connected
///////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_link_is_connected();

///////////////////////////////////////////////////////////////////////////////
/// returns if HID link is disconnected
///////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_link_is_disconnected();

///////////////////////////////////////////////////////////////////////////////
/// returns if HID link is connected
///////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_link_is_encrypted();

///////////////////////////////////////////////////////////////////////////////
/// hidd_link_enable_poll_callback()
///////////////////////////////////////////////////////////////////////////////
void hidd_link_enable_poll_callback(uint8_t transport, wiced_bool_t enable);

///////////////////////////////////////////////////////////////////////////////
/// hidd_link_init()
///////////////////////////////////////////////////////////////////////////////
void hidd_link_init();

///////////////////////////////////////////////////////////////////////////////
/// hidd_link_init()
///////////////////////////////////////////////////////////////////////////////
const char * hidd_link_state_str(uint8_t state);

////////////////////////////////////////////////////////////////////////////////
// hidd_link_aon_action_handler
////////////////////////////////////////////////////////////////////////////////
void hidd_link_aon_action_handler(uint8_t  type);

#ifdef SUPPORT_CODE_ENTRY
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
void hidd_link_pinCodeRequest(wiced_bt_dev_name_and_class_t *p_event_data);

////////////////////////////////////////////////////////////////////////////////
/// Provide a key press indication to the peer. If the subState is DISCOVERABLE,
/// uses BTM to send out a notification to the peer. Else ignored.
////////////////////////////////////////////////////////////////////////////////
void hidd_link_passCodeKeyPressReport(uint8_t key);

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
void hidd_link_pinCode(uint8_t pinCodeSize, uint8_t *pinCodeBuffer);

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
void hidd_link_passCode(uint8_t pinCodeSize, uint8_t *pinCodeBuffer);
#endif

#endif
