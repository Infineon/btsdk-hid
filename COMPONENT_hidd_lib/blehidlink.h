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
* File Name: blehidlink.h
*
* Abstract: This file implements the BLE HID application transport
*
* Functions:
*
*******************************************************************************/
#ifdef BLE_SUPPORT
#ifndef __BLE_HID_TRANSPORT__
#define __BLE_HID_TRANSPORT__

#include "wiced_hidd_lib.h"
#include "wiced_bt_dev.h"
#include "wiced_sleep.h"
#include "wiced_timer.h"
#include "emconinfo.h"
#include "clock_timer.h"

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

typedef struct
{
    EMCONINFO_DEVINFO   emconinfo;
    uint8_t   blehidlink_state;
    uint8_t   gatts_peer_addr_type;
    uint8_t   gatts_peer_addr[BD_ADDR_LEN];
    uint16_t  gatts_conn_id;
    uint16_t  blehostlist_flags;
    uint64_t  osapi_app_timer_start_instant;
    uint8_t   osapi_app_timer_running;
#ifdef WHITE_LIST_FOR_ADVERTISING
    uint8_t   adv_white_list_enabled;
#endif
} blehid_aon_save_content_t;

extern uint16_t  blehostlist_flags;
extern wiced_bt_ble_advert_mode_t app_adv_mode;

enum blehidlink_state_e
{
        /// 0 The blehidlink is initialized but inactive. This is the initial state.
        BLEHIDLINK_INITIALIZED,

        /// 1 Initialized, but idle (disconnected, not discoverable, not connectable)
        BLEHIDLINK_DISCONNECTED,

        /// 2 Advertising with a discoverable or connectable advertising event
        ///   with a discoverable flag set in LE.
        BLEHIDLINK_DISCOVERABLE,

        /// 3 We have a connection with a host over which we can pass reports. In this state
        ///   both interrupt and control channels are open
        BLEHIDLINK_CONNECTED,

        /// 4 Reconnecting to previously bonded host(s)
        BLEHIDLINK_RECONNECTING,

        /// 5 directed Advertising in uBCS mode
        BLEHIDLINK_ADVERTISING_IN_uBCS_DIRECTED,

        /// 6 undirected Advertising in uBCS mode
        BLEHIDLINK_ADVERTISING_IN_uBCS_UNDIRECTED

};

enum
{
    /// No activity
    BLEHIDLINK_ACTIVITY_NONE                   = 0x00,

    /// Reportable activity, e.g. motion, scroll, key or button press, etc.
    BLEHIDLINK_ACTIVITY_REPORTABLE             = 0x01,

    /// Non-reportable activity, e.g. key or button held down
    BLEHIDLINK_ACTIVITY_NON_REPORTABLE         = 0x02,

    /// Activities that require a transport to be connected, reportable
    BLEHIDLINK_ACTIVITY_CONNECTABLE            = BLEHIDLINK_ACTIVITY_REPORTABLE,

    /// Connect button pressed
    BLEHIDLINK_ACTIVITY_CONNECT_BUTTON_DOWN    = 0x80
};

enum blehidlink_conn_param_index_e
{
        BLEHIDLINK_CONN_INTERVAL_MIN = 0,       //minimum connection interval
        BLEHIDLINK_CONN_INTERVAL_MAX,           //maximum connection interval
        BLEHIDLINK_CONN_SLAVE_LATENCY,          //slave latency
        BLEHIDLINK_CONN_TIMEOUT                 //timeout
};

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

enum
{
    BLEHIDLINK_SAVE_TO_AON = 0,     /* Save context from SRAM to AON */
    BLEHIDLINK_RESTORE_FROM_AON,    /* Restore context from AON to SRAM */
};

typedef void (wiced_ble_hidd_state_change_callback_t)(uint32_t);

typedef struct _LinkStateObserver
{
    struct _LinkStateObserver* next;

    wiced_ble_hidd_state_change_callback_t* callback;
} LinkStateObserver;

typedef void (blehidlink_no_param_fp)(void);
typedef void (blehidlink_one_param_fp)(uint32_t arg);

typedef struct
{
    ///prefered connection parameters. Index see enum blehidlink_conn_param_index_e
    uint16_t prefered_conn_params[4];

    /// state (enum blehidlink_state_e)
    uint8_t subState;

    /// is State transition pending?
    uint8_t pendingStateTransiting;

    /// is application polling enabled?
    uint8_t appPoll_enabled;

    /// GATT connection ID
    uint16_t gatts_conn_id;

    /// connection idle timer timeout value. Timeout value in seconds. 0 for infinity (default)
    uint16_t conn_idle_timeout;

     ///connection idle timer (osapi timer that can be supported in uBCS mode)
    OSAPI_TIMER conn_idle_timer;

    /// auto reconnect timer
    wiced_timer_t reconnect_timer;

    /// observer for state changed
    LinkStateObserver* firstStateObserver;

#ifdef ALLOW_SDS_IN_DISCOVERABLE
     ///DISCOVERABLE timer (osapi timer that can be supported in uBCS mode)
    OSAPI_TIMER discoverable_timer;

    /// timer to switch from DISCOVERABLE to BLEHIDLINK_ADVERTISING_IN_uBCS_UNDIRECTED
    wiced_timer_t state_switch_timer;

    /// timeout value of state_switch_timer in mili seconds
    uint32_t state_switch_timeout_in_ms;
#endif

    /// pointer to the pending State transiting function
    blehidlink_one_param_fp* stateTransitingFunc;

    /// pointer to the callback function to poll user activity
    blehidlink_no_param_fp* pollReportUserActivityCallback;

    ///embedded controller info for the LE link before we entering SDS. When we exist SDS, we resume w/ it.
    EMCONINFO_DEVINFO   resume_emconinfo;

    ///embedded controller info for the existing LE link before we enter connected-advertising. If new connection failed, we need to recover with it
    EMCONINFO_DEVINFO   existing_emconinfo;

    /// subState before we entering SDS. When we exist SDS, we resume from this subState
    uint8_t resumeState;

    /// peer addr type in GATT (the peer address and peer address type used in GATT can be different when peer used random address)
    uint8_t gatts_peer_addr_type;

    /// peer addr in GATT (the peer address and peer address type used in GATT can be different when peer used random address)
    uint8_t gatts_peer_addr[BD_ADDR_LEN];

    /// osapi timer start instant (we used it to keep track of remaining timeout period when wake from uBCS mode)
    uint64_t  osapi_app_timer_start_instant;

    /// indicate if we have application osapi timer running when entering uBCS mode
    uint8_t   osapi_app_timer_running;

    /// 2nd LE connection state (not allowed/allowed/pending)
    uint8_t second_conn_state;

    /// the existing connection GATT connetion ID when 2nd LE connection is up
    uint16_t existing_connection_gatts_conn_id;

#ifdef AUTO_RECONNECT
    /// indicate if we will try "auto reconnect", i.e. start LE advertising,  when disconnected
    wiced_bool_t auto_reconnect;
#endif

#ifdef WHITE_LIST_FOR_ADVERTISING
    uint8_t   adv_white_list_enabled;
#endif

#ifdef EASY_PAIR
    /// the index in the easy pair candidate array
    uint8_t easyPair_deviceIndex;

    /// easy pair candidates
    EASY_PAIR_INFO easyPair;

    ///easy pair timer
    wiced_timer_t easyPair_timer;
#endif
} tBleHidLink;

extern tBleHidLink ble_hidd_link;

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_ble_hidd_link_conn_param_updated();

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
uint8_t wiced_ble_hidd_link_send_report(uint8_t reportID, wiced_hidd_report_type_t reportType, uint8_t *data, uint8_t length);

/////////////////////////////////////////////////////////////////////////////////
/// Add new observer for link state changed.
/// Whenever link state changed, the observer will be notified.
///
/// \param observer - pointer to the callback function
///
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_add_state_observer(wiced_ble_hidd_state_change_callback_t *observer);

/////////////////////////////////////////////////////////////////////////////////
/// check if it is currently connected
///
/// \return WICED_TRUE/WICED_FALSE
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t  wiced_ble_hidd_link_is_connected(void);

/////////////////////////////////////////////////////////////////////////////////
/// Check if it is discoverable (i.e. connetable undirected advertising)
///
/// \return WICED_TRUE/WICED_FALSE
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t  wiced_ble_hidd_link_is_discoverable(void);

/////////////////////////////////////////////////////////////////////////////////
/// Connect
/// As LE slave, it means start LE advertising
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_connect(void);

/////////////////////////////////////////////////////////////////////////////////
/// Disconnect
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_disconnect(void);

/////////////////////////////////////////////////////////////////////////////////
 /// Enable application poll
///
/// \param enable - WICED_TRUE/WICED_FALSE
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_enable_poll_callback(wiced_bool_t enable);

/////////////////////////////////////////////////////////////////////////////////
/// register application callback function when application is polled
///
/// \param cb - pointer to application callback function
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_register_poll_callback(blehidlink_no_param_fp *cb);

/////////////////////////////////////////////////////////////////////////////////
/// virtual cable unplug.
/// This function will remove all HID host information and start connectable undirected advertising
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_virtual_cable_unplug(void);

////////////////////////////////////////////////////////////////////////////////////
/// save/restore contents to/from Always On Memory when entering/exiting SDS
///
/// \param type - BLEHIDLINK_SAVE_TO_AON or BLEHIDLINK_RESTORE_FROM_AON
////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_aon_action_handler(uint8_t  type);

/////////////////////////////////////////////////////////////////////////////////
/// request Connection Parameter Update
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_conn_param_update(void);

/////////////////////////////////////////////////////////////////////////////////
/// handler for LE Connection Update Complete event
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_conn_update_complete(void);

/////////////////////////////////////////////////////////////////////////////////
/// request assymmetric slave latency.
/// when master doesn't accept slave's connection parameter update request,
/// slave can enable assymmetric slave latency to lower power consumption
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_set_slave_latency(uint16_t slaveLatencyinmS);

/////////////////////////////////////////////////////////////////////////////////
/// set ble HID link prefered conneciton parameters
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_set_preferred_conn_params(uint16_t conn_min_interval, uint16_t conn_max_interval, uint16_t slavelatency, uint16_t timeout);

/////////////////////////////////////////////////////////////////////////////////
/// set ble HID link connection Idle timer timeout value in seconds (default is 0, i.e. no timeout)
/////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_set_connection_Idle_timeout_value(uint16_t value);

/////////////////////////////////////////////////////////////////////////////////////////////
/// Notified ble hidd link that high duty cycle directed advertising stops
/////////////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_directed_adv_stop(void);

/////////////////////////////////////////////////////////////////////////////////////////////
/// ble hidd link get current advertizing mode
/////////////////////////////////////////////////////////////////////////////////////////////
wiced_bt_ble_advert_mode_t wiced_ble_hidd_link_get_adv_mode(void);
#define wiced_ble_hidd_link_is_advertizing() (wiced_ble_hidd_link_get_adv_mode() != BTM_BLE_ADVERT_OFF)

/////////////////////////////////////////////////////////////////////////////////////////////
/// Notified ble hidd link that LE advertising (except high duty cycle directed adv) stops
/////////////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_adv_stop(void);

/////////////////////////////////////////////////////////////////////////////////////////////
/// Notified ble hidd link that LE connection up
/////////////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_connected(void);

/////////////////////////////////////////////////////////////////////////////////////////////
/// Notified ble hidd link that LE connection down
/////////////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_link_disconnected(void);

#endif
#endif
