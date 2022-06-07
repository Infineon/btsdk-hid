/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
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
* File Name: hidd_blelink.h
*
* Abstract: This file implements the BLE HID application transport
*
* Functions:
*
*******************************************************************************/
#ifdef BLE_SUPPORT

#ifndef __HIDD_BLELINK__
#define __HIDD_BLELINK__

#include "wiced_bt_dev.h"
#include "wiced_sleep.h"
#include "wiced_timer.h"
#include "clock_timer.h"
#if BTSTACK_VER < 0x03000001
#include "hidd_blelink_v1.h"
#else
#include "hidd_blelink_v3.h"
#endif

#ifdef EASY_PAIR
#define INVALID_RSSI            -1000
#define INVALID_INDEX      0xFF
#define MAX_DEVICES             5
#define EASY_PAIR_SCAN_TIMEOUT  5               //timeout in seconds

#define INVALID_ID              0
#define VALID_ID                1

typedef struct
{
    uint8_t                 valid;
    uint8_t                 addressType;
    uint8_t                 wd_addr[BD_ADDR_LEN];
    int32_t                 rssi_total;
    uint8_t                 rssi_count;
} EASY_PAIR_CANDIDATE;

typedef struct
{
    uint8_t                 availableSlots;        //which device index is available.
    EASY_PAIR_CANDIDATE     device[MAX_DEVICES];
} EASY_PAIR_INFO;
#endif

enum
{
    /// do not allow 2nd connection (default)
    BLEHIDLINK_2ND_CONNECTION_NOT_ALLOWED                  = 0x00,

    /// Allow 2nd connection, i.e. connectable undirected LE advertising is on
    BLEHIDLINK_2ND_CONNECTION_ALLOWED                      = 0x01,

    /// 2nd connection connected, pairing/bonding process pending
    BLEHIDLINK_2ND_CONNECTION_PENDING                      = 0x02
};

//application timer type activated in SDS
enum
{
    //connectable undirected ADV timer
    BLEHIDLINK_ADV_CONNECTABLE_UNDIRECTED_TIMER             = 0x02,  //bit 1 is used for this timer

    //connection idle timer
    BLEHIDLINK_CONNECTION_IDLE_TIMER                        = 0x04  //bit 2 is used for this timer
};

typedef void (wiced_ble_hidd_state_change_callback_t)(uint32_t);
typedef void (hidd_blelink_one_param_fp)(uint32_t arg);
typedef struct _LinkStateObserver
{
    struct _LinkStateObserver* next;

    wiced_ble_hidd_state_change_callback_t* callback;
} LinkStateObserver;

typedef struct
{
    EMCONINFO_DEVINFO   emconinfo;
    uint8_t   hidd_blelink_state;
    uint8_t   gatts_peer_addr_type;
    uint8_t   gatts_peer_addr[BD_ADDR_LEN];
    uint16_t  gatts_conn_id;
    uint64_t  osapi_app_timer_start_instant;
    uint8_t   osapi_app_timer_running;
#ifdef FILTER_ACCEPT_LIST_FOR_ADVERTISING
    uint8_t   adv_filter_accept_list_enabled;
#endif
} blehid_aon_save_content_t;

#pragma pack(1)
typedef struct
{
   /// osapi timer start instant (we used it to keep track of remaining timeout period when wake from uBCS mode)
    uint64_t  osapi_app_timer_start_instant;

     ///connection idle timer (osapi timer that can be supported in uBCS mode)
    hidd_blelink_timer_t conn_idle_timer;

    /// observer for state changed
    LinkStateObserver* firstStateObserver;

#ifdef ALLOW_SDS_IN_DISCOVERABLE
     ///DISCOVERABLE timer (osapi timer that can be supported in uBCS mode)
    hidd_blelink_timer_t discoverable_timer;

    /// timer to switch from DISCOVERABLE to HIDLINK_LE_ADVERTISING_IN_uBCS_UNDIRECTED
    wiced_timer_t state_switch_timer;

    /// timeout value of state_switch_timer in mili seconds
    uint32_t state_switch_timeout_in_ms;
#endif

    /// pointer to the pending State transiting function
    hidd_blelink_one_param_fp* stateTransitingFunc;

    ///embedded controller info for the LE link before we entering SDS. When we exist SDS, we resume w/ it.
    EMCONINFO_DEVINFO   resume_emconinfo;

    ///embedded controller info for the existing LE link before we enter connected-advertising. If new connection failed, we need to recover with it
    EMCONINFO_DEVINFO   existing_emconinfo;

#ifdef AUTO_RECONNECT
    /// indicate if we will try "auto reconnect", i.e. start LE advertising,  when disconnected
    wiced_bool_t auto_reconnect;
#endif

    /// connection_param_updated;
    wiced_bool_t connection_param_updated;

    // adv_mode;
    wiced_bt_ble_advert_mode_t adv_mode;

    /// GATT connection ID
    uint16_t gatts_conn_id;

    /// connection idle timer timeout value. Timeout value in seconds. 0 for infinity (default)
    uint16_t conn_idle_timeout;

    /// the existing connection GATT connetion ID when 2nd LE connection is up
    uint16_t existing_connection_gatts_conn_id;

    /// state (enum hidd_blelink_state_e)
    uint8_t subState;

    /// is State transition pending?
    uint8_t pendingStateTransiting;

    /// subState before we entering SDS. When we exist SDS, we resume from this subState
    uint8_t resumeState;

    /// peer addr type in GATT (the peer address and peer address type used in GATT can be different when peer used random address)
    uint8_t gatts_peer_addr_type;

    /// peer addr in GATT (the peer address and peer address type used in GATT can be different when peer used random address)
    uint8_t gatts_peer_addr[BD_ADDR_LEN];

     /// indicate if we have application osapi timer running when entering uBCS mode
    uint8_t   osapi_app_timer_running;

    /// 2nd LE connection state (not allowed/allowed/pending)
    uint8_t second_conn_state;

    /// wake_from_SDS_timer_timeout_flag
    uint8_t wake_from_SDS_timer_timeout_flag;

#ifdef FILTER_ACCEPT_LIST_FOR_ADVERTISING
    uint8_t   adv_filter_accept_list_enabled;
#endif

#ifdef EASY_PAIR
    /// the index in the easy pair candidate array
    uint8_t easyPair_deviceIndex;

    /// easy pair candidates
    EASY_PAIR_INFO easyPair;

    /// easy pair timer
    wiced_timer_t easyPair_timer;
#endif

} tBleHidLink;
#pragma pack()

typedef void (hidd_blelink_no_param_fp)(void);

/*************************************************************************
* externs
**************************************************************************/
extern tBleHidLink blelink;
extern blehid_aon_save_content_t   ble_aon_data;

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_set_adv_mode(wiced_bt_ble_advert_mode_t newMode);

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_restart_idle_timer();

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_blelink_conn_param_updated();

/////////////////////////////////////////////////////////////////////////////////
/// send HID report as GATT notification
///
/// \param reportID - report ID
/// \param reportType - report type.
/// \param data - pointer to report data
/// \param length - length of the report data
///
/// \return 0 - successful
///         1 - failed
/////////////////////////////////////////////////////////////////////////////////
uint8_t hidd_blelink_send_report(uint8_t reportID, wiced_hidd_report_type_t reportType, uint8_t *data, uint8_t length);

/////////////////////////////////////////////////////////////////////////////////
/// Add new observer for link state changed.
/// Whenever link state changed, the observer will be notified.
///
/// \param observer - pointer to the callback function
///
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_add_state_observer(wiced_ble_hidd_state_change_callback_t *observer);

/////////////////////////////////////////////////////////////////////////////////
/// check if the link state is in requested state
///
/// \return WICED_TRUE/WICED_FALSE
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t  hidd_blelink_state_is(uint8_t state);

/////////////////////////////////////////////////////////////////////////////////
/// check if it is currently connected
///
/// \return WICED_TRUE/WICED_FALSE
/////////////////////////////////////////////////////////////////////////////////
#define hidd_blelink_is_connected() hidd_blelink_state_is(HIDLINK_LE_CONNECTED)

/////////////////////////////////////////////////////////////////////////////////
/// check if it is currently in disconnected state
///
/// \return WICED_TRUE/WICED_FALSE
/////////////////////////////////////////////////////////////////////////////////
#define hidd_blelink_is_disconnected() hidd_blelink_state_is(HIDLINK_LE_DISCONNECTED)

/////////////////////////////////////////////////////////////////////////////////
/// Check if it is discoverable (i.e. connetable undirected advertising)
///
/// \return WICED_TRUE/WICED_FALSE
/////////////////////////////////////////////////////////////////////////////////
#define hidd_blelink_is_discoverable() hidd_blelink_state_is(HIDLINK_LE_DISCOVERABLE)

/////////////////////////////////////////////////////////////////////////////////
/// Connect
/// As LE peripheral, it means start LE advertising
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_connect(void);

/////////////////////////////////////////////////////////////////////////////////
/// Disconnect
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_disconnect(void);

/////////////////////////////////////////////////////////////////////////////////
 /// Enable application poll
///
/// \param enable - WICED_TRUE/WICED_FALSE
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_enable_poll_callback(wiced_bool_t enable);

/////////////////////////////////////////////////////////////////////////////////
/// virtual cable unplug.
/// This function will remove all HID host information and start connectable undirected advertising
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_virtual_cable_unplug(void);

////////////////////////////////////////////////////////////////////////////////////
/// save/restore contents to/from Always On Memory when entering/exiting SDS
///
/// \param type - HIDD_LINK_SAVE_TO_AON or HIDD_LINK_RESTORE_FROM_AON
////////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_aon_action_handler(uint8_t  type);

/////////////////////////////////////////////////////////////////////////////////
/// request Connection Parameter Update
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_conn_param_update(void);

/////////////////////////////////////////////////////////////////////////////////
/// handler for LE Connection Update Complete event
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_conn_update_complete(void);

/////////////////////////////////////////////////////////////////////////////////
/// set ble HID link connection Idle timer timeout value in seconds (default is 0, i.e. no timeout)
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_set_connection_Idle_timeout_value(uint16_t value);

/////////////////////////////////////////////////////////////////////////////////////////////
/// Notified ble hidd link that high duty cycle directed advertising stops
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_directed_adv_stop(void);

/////////////////////////////////////////////////////////////////////////////////////////////
/// ble hidd link get current advertizing mode
/////////////////////////////////////////////////////////////////////////////////////////////
wiced_bt_ble_advert_mode_t hidd_blelink_get_adv_mode(void);
#define hidd_blelink_is_advertizing() (hidd_blelink_get_adv_mode() != BTM_BLE_ADVERT_OFF)

/////////////////////////////////////////////////////////////////////////////////////////////
/// Notified ble hidd link that LE advertising (except high duty cycle directed adv) stops
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_adv_stop(void);

/////////////////////////////////////////////////////////////////////////////////////////////
/// Notified ble hidd link that LE connection up
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_connected(void);

/////////////////////////////////////////////////////////////////////////////////////////////
/// Notified ble hidd link that LE connection down
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_disconnected(void);

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_init();

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_determine_next_state();

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_set_state(uint8_t newState);

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_enterDiscoverable(uint32_t SDS_allow);

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_enterDisconnected(void);

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_allowDiscoverable(void);
#endif
#endif
