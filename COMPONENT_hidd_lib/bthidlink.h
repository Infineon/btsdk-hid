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
* File Name: bthidlink.h
*
* Abstract: Bluetooth(BT) Classic HID link definitions and functions
*******************************************************************************/

#ifndef _BT_HIDLINK_H_
#define _BT_HIDLINK_H_

#include "bt_types.h"
#include "wiced.h"
#include "wiced_sleep.h"
#include "wiced_bt_hidd.h"
#include "wiced_timer.h"

#define LINK_SUPERVISION_TIMEOUT_IN_SLOTS   3200
#define DISCOVERY_TIMEOUT                  60000                  // One minite discovery
#define IDLE_TIMEOUT                      180000                  // idle time

#pragma pack(1)
typedef struct
{
    /// When non-zero, indicates that we can initiate a connection
    uint8_t reconnectInitiate;

    /// When non-zero, indicates that we are nrmally connectable assuming
    /// we have one or more bonded hosts
    uint8_t normallyConnectable;

    /// When non-zero, enables Inquiry and page scans when disconnected.
    uint8_t becomeDiscoverableWhenNotConnected;

    /// Flag indicating whether we should exit discoverable on an authentication failure
    uint8_t exitDiscoverableOnAuthFailure;

    /// Link Supervision Timeout in slots
    uint16_t linkSupervisionTimeout;

    /// Packet types. Valid ones are HCI_PKT_TYPES_MASK_*
    uint16_t packetTypes;

    /// Page timeout in slot used for reconnecting
    uint16_t reconnectPageTimeout;

    /// Maximum number of time an attempt will be made to reconnect to one host
    /// before deciding that it is not connectable and moving on to the next
    uint8_t maxReconnectRetryCount;
}tBtHidLinkCfg;
#pragma pack()

typedef struct
{
    /// bd address of peer device
    BD_ADDR  bdAddr;

    /// indicates if the link is encrypted: 0 - not encrypted; 1-encrypted
    wiced_bool_t encrypted;

}tBtHidLinkEncryptStatus;

typedef struct
{
    uint8_t   bthidlink_state;
} bthid_aon_save_content_t;

enum bthidlink_state_e
{
    /// The bthidlink is initialized but inactive. This is the initial state.
    /// No events should be issued to the bthidlink in this state.
    BTHIDLINK_INITIALIZED,

    /// Initialized, but idle (disconnected, not discoverable, not connectable)
    BTHIDLINK_DISCONNECTED,

    /// Discoverable, connectable, and pairable (if applicable).
    /// We do inquiry and page scans in this state
    BTHIDLINK_DISCOVERABLE,

    /// Connectable with host(s) we have been cabled with. We do page scans
    /// in this state and only accept connections from hosts we know
    BTHIDLINK_CONNECTABLE,

    /// We have a connection with a host over which we can pass reports. In this state
    /// both interrupt and control channels are open
    BTHIDLINK_CONNECTED,

    /// We are in the process of disconnecting
    BTHIDLINK_DISCONNECTING,

    /// Reconnecting to previously bonded host(s)
    BTHIDLINK_RECONNECTING,
};

enum
{
    /// No activity
    BTHIDLINK_ACTIVITY_NONE                 = 0x00,

    /// Reportable activity, e.g. motion, scroll, key or button press, etc.
    BTHIDLINK_ACTIVITY_REPORTABLE           = 0x01,

    /// Non-reportable activity, e.g. key or button held down
    BTHIDLINK_ACTIVITY_NON_REPORTABLE       = 0x02,

    /// Activities that require a transport to be connected, reportable
    BTHIDLINK_ACTIVITY_CONNECTABLE          = BTHIDLINK_ACTIVITY_REPORTABLE,

    /// Connect button pressed
    BTHIDLINK_ACTIVITY_CONNECT_BUTTON_DOWN  = 0x80
};

enum
{
    BTHIDLINK_SAVE_TO_AON = 0,     /* Save context from SRAM to AON */
    BTHIDLINK_RESTORE_FROM_AON,    /* Restore context from AON to SRAM */
};

typedef void (wiced_bt_hidd_state_change_callback_t)(uint32_t);

typedef struct _btLinkStateObserver
{
    struct _btLinkStateObserver* next;

    wiced_bt_hidd_state_change_callback_t* callback;
} tBtLinkStateObserver;

typedef void wiced_bt_hidd_link_app_write_eir_callback_t(void);
typedef void wiced_bt_hidd_link_app_poll_callback_t(void);
typedef void wiced_bt_hidd_link_app_connection_failed_callback_t(void);
typedef void wiced_bt_hidd_link_app_enter_pincode_entry_mode_callback_t(void);
typedef void wiced_bt_hidd_link_app_enter_passcode_entry_mode_callback_t(void);
typedef void wiced_bt_hidd_link_app_exit_pin_and_passcode_entry_mode_callback_t(void);
typedef uint8_t wiced_bt_hidd_link_app_get_idle_callback_t(void);
typedef uint8_t wiced_bt_hidd_link_app_set_idle_callback_t(uint8_t idleRateIn4msUnits);
typedef uint8_t wiced_bt_hidd_link_app_get_protocol_callback_t(void);
typedef uint8_t wiced_bt_hidd_link_app_set_protocol_callback_t(uint8_t);
typedef uint8_t wiced_bt_hidd_link_app_get_report_callback_t( uint8_t reportType, uint8_t reportId);
typedef uint8_t wiced_bt_hidd_link_app_set_report_callback_t(uint8_t reportType, uint8_t *payload, uint16_t payloadSize);
typedef void wiced_bt_hidd_link_app_rx_data_callback_t(uint8_t reportType, uint8_t *payload, uint16_t payloadSize);

typedef struct
{
    wiced_bt_hidd_link_app_write_eir_callback_t                           *p_app_write_eir_data;
    wiced_bt_hidd_link_app_poll_callback_t                                *p_app_poll_user_activities;
    wiced_bt_hidd_link_app_connection_failed_callback_t                   *p_app_connection_failed_notification;

    wiced_bt_hidd_link_app_enter_pincode_entry_mode_callback_t            *p_app_enter_pincode_entry_mode;
    wiced_bt_hidd_link_app_enter_passcode_entry_mode_callback_t           *p_app_enter_passcode_entry_mode;
    wiced_bt_hidd_link_app_exit_pin_and_passcode_entry_mode_callback_t    *p_app_exit_pin_and_passcode_entry_mode;

    wiced_bt_hidd_link_app_get_idle_callback_t      *p_app_get_idle;
    wiced_bt_hidd_link_app_set_idle_callback_t      *p_app_set_idle;
    wiced_bt_hidd_link_app_get_protocol_callback_t  *p_app_get_protocol;
    wiced_bt_hidd_link_app_set_protocol_callback_t  *p_app_set_protocol;
    wiced_bt_hidd_link_app_get_report_callback_t    *p_app_get_report;
    wiced_bt_hidd_link_app_set_report_callback_t    *p_app_set_report;
    wiced_bt_hidd_link_app_rx_data_callback_t       *p_app_rx_data;
}wiced_bt_hidd_link_app_callback_t;

/////////////////////////////////////////////////////////////////////////////////////////////
typedef struct
{
    /// state (enum bthidlink_state_e)
    uint8_t subState;

    /// is application polling enabled?
    uint8_t appPoll_enabled;

    /// is pending to become discoverable ?
    uint8_t becomeDiscoverablePending;

    /// index of the host to reconnect
    uint8_t reconnectHostIndex;

    /// reconnect retry count
    uint8_t reconnectRetryCount;

    /// is security failed?
    uint8_t security_failed;

    /// observer for state changed
    tBtLinkStateObserver* firstStateObserver;

    /// timer for state transition
    wiced_timer_t stateTimer;

    /// bt link encrypted status
    tBtHidLinkEncryptStatus encrypt_status;

    /// subState before we entering SDS. When we exist SDS, we resume from this subState
    uint8_t resumeState;

} tBtHidLink;

extern tBtHidLink bt_hidd_link;

/////////////////////////////////////////////////////////////////////////////////////////////
/// Abstract link layer initialize
/////////////////////////////////////////////////////////////////////////////////////////////
wiced_bt_hidd_status_t wiced_bt_hidd_init(void);

/////////////////////////////////////////////////////////////////////////////////
/// register application callback functions
///
/// \param cb - pointer to application callback functions
/////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_link_register_app_callback(wiced_bt_hidd_link_app_callback_t *cb);

/////////////////////////////////////////////////////////////////////////////////
/// Add new observer for link state changed.
/// Whenever link state changed, the observer will be notified.
///
/// \param observer - pointer to the callback function
///
/////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_link_add_state_observer(wiced_bt_hidd_state_change_callback_t* observer);

/////////////////////////////////////////////////////////////////////////////////
 /// Enable application poll
///
/// \param enable - WICED_TRUE/WICED_FALSE
/////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_link_enable_poll_callback(wiced_bool_t enable);

/////////////////////////////////////////////////////////////////////////////////
/// Check if the current link state is in the requested state
///
/// \return WICED_TRUE/WICED_FALSE
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_bt_hidd_link_state_is(uint8_t state);

/////////////////////////////////////////////////////////////////////////////////
/// Check if the current bt link state is in the requested state
///
/// \return WICED_TRUE/WICED_FALSE
/////////////////////////////////////////////////////////////////////////////////
#define wiced_bt_hidd_link_is_connected() wiced_bt_hidd_link_state_is(BTHIDLINK_CONNECTED)

/////////////////////////////////////////////////////////////////////////////////
/// Check if it is currently disconnected.
///
/// \return WICED_TRUE/WICED_FALSE
/////////////////////////////////////////////////////////////////////////////////
#define wiced_bt_hidd_link_is_disconnected() wiced_bt_hidd_link_state_is(BTHIDLINK_DISCONNECTED)

/////////////////////////////////////////////////////////////////////////////////
/// Return whether we are Discoverable.
///
/// \return WICED_TRUE if substate is discoverable, WICED_FALSE otherwise
/////////////////////////////////////////////////////////////////////////////////
#define wiced_bt_hidd_link_is_discoverable() wiced_bt_hidd_link_state_is(BTHIDLINK_DISCOVERABLE)

/////////////////////////////////////////////////////////////////////////////////
/// Connect
/////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_link_connect(void);

/////////////////////////////////////////////////////////////////////////////////
/// Disconnect
/////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_link_disconnect(void);

/////////////////////////////////////////////////////////////////////////////////
/// virtual cable unplug.
/// It immediately clears the host list, removes bonded device info from bt stack, and
/// request VC unplug.
///
/// \return WICED_TRUE if sent VIRTUAL CABLE UNPLUG message, WICED_FALSE otherwise
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_bt_hidd_link_virtual_cable_unplug(void);

////////////////////////////////////////////////////////////////////////////////
/// Disables page and inquiry scans.
////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_link_disable_page_and_inquiry_scans(void);

////////////////////////////////////////////////////////////////////////////////
/// Enters discoverable state.
////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_link_enter_discoverable(void);

////////////////////////////////////////////////////////////////////////////////
/// Enters disconnected state.
////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_link_enter_disconnected(void);

////////////////////////////////////////////////////////////////////////////////////
/// save/restore contents to/from Always On Memory when entering/exiting SDS
///
/// \param type - BTHIDLINK_SAVE_TO_AON or BTHIDLINK_RESTORE_FROM_AON
////////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_link_aon_action_handler(uint8_t  type);

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
void bthidlink_pinCodeRequest(wiced_bt_dev_name_and_class_t *p_event_data);

////////////////////////////////////////////////////////////////////////////////
/// Provide a key press indication to the peer. If the subState is DISCOVERABLE,
/// uses BTM to send out a notification to the peer. Else ignored.
////////////////////////////////////////////////////////////////////////////////
void bthidlink_passCodeKeyPressReport(uint8_t key);

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
void bthidlink_pinCode(uint8_t pinCodeSize, uint8_t *pinCodeBuffer);

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
void bthidlink_passCode(uint8_t pinCodeSize, uint8_t *pinCodeBuffer);

////////////////////////////////////////////////////////////////////////////////////
/// bthidlink_setHostAddr
/////////////////////////////////////////////////////////////////////////////////////////////
void bthidlink_setHostAddr(const wiced_bt_device_address_t addr);


#endif
