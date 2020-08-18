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
* File Name: bt_link.h
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
    uint8_t   hidd_btlink_state;
} bthid_aon_save_content_t;

typedef void (wiced_bt_hidd_state_change_callback_t)(uint32_t);

typedef struct _btLinkStateObserver
{
    struct _btLinkStateObserver* next;

    wiced_bt_hidd_state_change_callback_t* callback;
} tBtLinkStateObserver;

typedef void wiced_bt_hidd_link_app_write_eir_callback_t(void);

/////////////////////////////////////////////////////////////////////////////////////////////
typedef struct
{
    /// state (enum hidd_btlink_state_e)
    uint8_t subState;

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
/// Add new observer for link state changed.
/// Whenever link state changed, the observer will be notified.
///
/// \param observer - pointer to the callback function
///
/////////////////////////////////////////////////////////////////////////////////
void hidd_btlink_add_state_observer(wiced_bt_hidd_state_change_callback_t* observer);

/////////////////////////////////////////////////////////////////////////////////
 /// Enable application poll
///
/// \param enable - WICED_TRUE/WICED_FALSE
/////////////////////////////////////////////////////////////////////////////////
void hidd_btlink_enable_poll_callback(wiced_bool_t enable);

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_btlink_determine_next_state(void);

/////////////////////////////////////////////////////////////////////////////////
/// Return whether link state is in the requested state
///
/// \return WICED_TRUE if yes, WICED_FALSE otherwise
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_btlink_state_is(uint8_t state);

/////////////////////////////////////////////////////////////////////////////////
/// Check if it is currently connected. Connected is defined as ACL, control, and
/// interrupt channel all being open.
///
/// \return WICED_TRUE/WICED_FALSE
/////////////////////////////////////////////////////////////////////////////////
#define hidd_btlink_is_connected() hidd_btlink_state_is(HIDLINK_CONNECTED)

/////////////////////////////////////////////////////////////////////////////////
/// Check if it is currently disconnected.
///
/// \return WICED_TRUE/WICED_FALSE
/////////////////////////////////////////////////////////////////////////////////
#define hidd_btlink_is_disconnected() hidd_btlink_state_is(HIDLINK_DISCONNECTED)

/////////////////////////////////////////////////////////////////////////////////
/// Return whether we are Discoverable.
///
/// \return WICED_TRUE if substate is discoverable, WICED_FALSE otherwise
/////////////////////////////////////////////////////////////////////////////////
#define hidd_btlink_is_discoverable() hidd_btlink_state_is(HIDLINK_DISCOVERABLE)

/////////////////////////////////////////////////////////////////////////////////
/// Connect
/////////////////////////////////////////////////////////////////////////////////
void hidd_btink_connect(void);

/////////////////////////////////////////////////////////////////////////////////
/// Disconnect
/////////////////////////////////////////////////////////////////////////////////
void hidd_btlink_disconnect(void);

/////////////////////////////////////////////////////////////////////////////////
/// virtual cable unplug.
/// It immediately clears the host list, removes bonded device info from bt stack, and
/// request VC unplug.
///
/// \return WICED_TRUE if sent VIRTUAL CABLE UNPLUG message, WICED_FALSE otherwise
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_btlink_virtual_cable_unplug(void);

////////////////////////////////////////////////////////////////////////////////
/// Disables page and inquiry scans.
////////////////////////////////////////////////////////////////////////////////
void hidd_btlink_disable_page_and_inquiry_scans(void);

////////////////////////////////////////////////////////////////////////////////
/// Enters discoverable state.
////////////////////////////////////////////////////////////////////////////////
void hidd_btlink_enter_discoverable(void);

////////////////////////////////////////////////////////////////////////////////
/// Enters disconnected state.
////////////////////////////////////////////////////////////////////////////////
void hidd_btlink_enter_disconnected(void);

////////////////////////////////////////////////////////////////////////////////////
/// save/restore contents to/from Always On Memory when entering/exiting SDS
///
/// \param type - HIDD_LINK_SAVE_TO_AON or HIDD_LINK_RESTORE_FROM_AON
////////////////////////////////////////////////////////////////////////////////////
void hidd_btlink_aon_action_handler(uint8_t  type);

////////////////////////////////////////////////////////////////////////////////////
/// hidd_btlink_setHostAddr
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_btlink_setHostAddr(const wiced_bt_device_address_t addr);

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
void hidd_btlink_init(void);


#endif
