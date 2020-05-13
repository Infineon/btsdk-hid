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
* File Name: bthidlink.c
*
* Abstract: This file implements the Bluetooth(BT) Classic HID application transport
*
* Functions:
*
*******************************************************************************/
#ifdef BR_EDR_SUPPORT
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_event.h"
#include "wiced_hal_batmon.h"
#include "wiced_hidd_lib.h"
#include "wiced_memory.h"
#include "wiced_transport.h"

#include "spar_utils.h"
#include "bthidlink.h"
#include "hidd_lib.h"

#ifdef TESTING_USING_HCI
#include "hci_control_api.h"
#endif

tBtHidLinkCfg wiced_bt_hidlinkcfg =
{
    /// When non-zero, indicates that we can initiate a connection
    /// uint8_t reconnectInitiate;
    HID_DEV_RECONN_INITIATE,

    /// When non-zero, indicates that we are normally connectable assuming
    /// we have one or more bonded hosts
    /// uint8_t normallyConnectable;
    HID_DEV_NORMALLY_CONN,

    /// When non-zero, enables Inquiry and page scans when disconnected.
    /// uint8_t  becomeDiscoverableWhenNotConnected;
    0,

    /// Flag indicating whether we should exit discoverable on an authentication failure
    ///uint8_t exitDiscoverableOnAuthFailure;
    0,

    /// Link Supervision Timeout in slots (625us)
    /// uint16_t linkSupervisionTimeout;
    LINK_SUPERVISION_TIMEOUT_IN_SLOTS,

    // Page parameters

    /// Packet types. Valid ones are HCI_PKT_TYPES_MASK_*
    /// uint16_t packetTypes;
    (HCI_PKT_TYPES_MASK_DM1 | HCI_PKT_TYPES_MASK_DH1 |
     HCI_PKT_TYPES_MASK_NO_2_DH1 | HCI_PKT_TYPES_MASK_NO_3_DH1 |
     HCI_PKT_TYPES_MASK_NO_2_DH3 | HCI_PKT_TYPES_MASK_NO_3_DH3 |
     HCI_PKT_TYPES_MASK_NO_2_DH5 | HCI_PKT_TYPES_MASK_NO_3_DH5),

     /// Page timeout in slot used for reconnecting
     /// uint16_t reconnectPageTimeout;
     8192,

    /// Maximum number of time an attempt will be made to reconnect to one host
     /// before deciding that it is not connectable and moving on to the next
     /// uint8_t maxReconnectRetryCount;
     4
};

tBtHidLink bt_hidd_link = {};

wiced_bt_hidd_link_app_callback_t *bthidlink_app_callback = NULL;


//wiced_bool_t bthidlink_firstConnAfterDiscoverable = WICED_FALSE;
//wiced_bool_t bthidlink_discoverableStateWhileReconnect = WICED_FALSE;
//wiced_bool_t bthidlink_pinCodeRequested = WICED_FALSE;

BD_ADDR bthidlink_passkeyreq_bdaddr = {0, };

typedef UINT8 tBTM_STATUS;

void bthidlink_enterConnectable(void);
void bthidlink_enterReconnecting(void);
void bthidlink_statetimerTimeoutCb(uint32_t args);

uint8_t bthidlink_earlyWakeNotification(void* unused);
void bthidlink_bthidd_evtHandler(wiced_bt_hidd_cback_event_t  event, uint32_t data, wiced_bt_hidd_event_data_t *p_event_data );
BTM_API extern tBTM_STATUS BTM_WritePageTimeout(UINT16 timeout);
BTM_API extern tBTM_STATUS BTM_SetPacketTypes (BD_ADDR remote_bda, UINT16 pkt_types);

PLACE_DATA_IN_RETENTION_RAM bthid_aon_save_content_t   bthid_aon_data;

////////////////////////////////////////////////////////////////////////////////////
/// bthidlink_setHostAddr
/////////////////////////////////////////////////////////////////////////////////////////////
void bthidlink_setHostAddr(const wiced_bt_device_address_t addr)
{
    wiced_bt_hidd_reg_info_t bthidlink_reg_info;

    bthidlink_reg_info.p_app_cback = bthidlink_bthidd_evtHandler;
    memcpy(bthidlink_reg_info.host_addr, addr, BD_ADDR_LEN);

    // clear registered host
    wiced_bt_hidd_deregister();

    //register with hidd
    wiced_bt_hidd_register(&bthidlink_reg_info);
}

////////////////////////////////////////////////////////////////////////////////////
/// bt classic hid link init
/////////////////////////////////////////////////////////////////////////////////////////////
void bthidlink_init()
{
    // app to write EIR data
    if (bthidlink_app_callback && bthidlink_app_callback->p_app_write_eir_data)
    {
        bthidlink_app_callback->p_app_write_eir_data();
    }

    // Initialize Link Supervision Timerout value
    wiced_bthidd_setDefaultLinkSupervisionTimeout(wiced_bt_hidlinkcfg.linkSupervisionTimeout);

    wiced_init_timer( &bt_hidd_link.stateTimer, bthidlink_statetimerTimeoutCb, 0, WICED_MILLI_SECONDS_TIMER );

}

/////////////////////////////////////////////////////////////////////////////////
/// Add new observer for link state changed.
/// Whenever link state changed, the observer will be notified.
///
/// \param observer - pointer to the callback function
///
/////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_link_add_state_observer(wiced_bt_hidd_state_change_callback_t* observer)
{
    tBtLinkStateObserver* ob = (tBtLinkStateObserver*)wiced_memory_permanent_allocate(sizeof(tBtLinkStateObserver));

    // If allocation was OK, put this registration in the SL
    if(ob)
    {
        ob->callback = observer;
        ob->next = bt_hidd_link.firstStateObserver;
        bt_hidd_link.firstStateObserver = ob;
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// set new link state and notify observers.
///
/// \param newState - the new link state
/////////////////////////////////////////////////////////////////////////////////
void bthidlink_setState(uint8_t newState)
{
    tBtLinkStateObserver* tmpObs = bt_hidd_link.firstStateObserver;

    if(newState != bt_hidd_link.subState)
    {
        WICED_BT_TRACE("\nBT state changed from %d to %d", bt_hidd_link.subState, newState);
        bt_hidd_link.subState = newState;

        hci_control_send_state_change(BT_TRANSPORT_BR_EDR, newState);

        //if current active transport is LE, do not notify observer
        if (wiced_hidd_host_transport() == BT_TRANSPORT_LE)
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

/////////////////////////////////////////////////////////////////////////////////////////////
/// determine next action
/////////////////////////////////////////////////////////////////////////////////////////////
void bthidlink_determineNextState(void)
{
    // Have we been asked to become discoverable
    if (bt_hidd_link.becomeDiscoverablePending)
    {
        // Become discoverable
        wiced_bt_hidd_link_enter_discoverable();
    }
    // Do we have any bonded hosts
    else if (wiced_hidd_is_paired())
    {
        WICED_BT_TRACE("\nbonded info in NVRAM");
#if 0
        // Do we have user activity and are allowed to reconnect?
        if (connectRequestPending && wiced_bt_hidlinkcfg.reconnectInitiate)
        {
            bthidlink_enterReconnecting();
        }
        // Do we want to respond to inquiry/pages
        else
#endif
        if(wiced_bt_hidlinkcfg.becomeDiscoverableWhenNotConnected)
        {
            // Become discoverable
            wiced_bt_hidd_link_enter_discoverable();
        }
        // Are we connectable
        else if (wiced_bt_hidlinkcfg.normallyConnectable)
        {
            bthidlink_enterConnectable();
        }
        else
        {
            // Nothing to do. Enter DISCONNECTED state
            wiced_bt_hidd_link_enter_disconnected();
        }
    }
    else
    {
        if(wiced_bt_hidlinkcfg.becomeDiscoverableWhenNotConnected)
        {
            // Become discoverable
            wiced_bt_hidd_link_enter_discoverable();
        }
        else
        {
            // We have no hosts. Enter DISCONNECTED state
            wiced_bt_hidd_link_enter_disconnected();
        }
    }
}

#if 0
/////////////////////////////////////////////////////////////////////////////////////////////
/// determine next action when wake from SDS
/////////////////////////////////////////////////////////////////////////////////////////////
void bthidlink_determineNextState_on_wake_from_SDS(void)
{
    //set subState to resumeState
    bthidlink_setState(bt_hidd_link.resumeState);

    if ((BTHIDLINK_DISCONNECTED == bt_hidd_link.subState) && !wiced_hal_batmon_is_low_battery_shutdown())
    {
        //poll user activity and act accordingly
        if (bthidlink_app_callback && bthidlink_app_callback->p_app_poll_user_activities)
        {
            bthidlink_app_callback->p_app_poll_user_activities();
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// determine next action on power uo (cold boot or wake from SDS)
/////////////////////////////////////////////////////////////////////////////////////////////
void bthidlink_determineNextState_on_powerup(void)
{
    if(!wiced_hal_mia_is_reset_reason_por())
    {
        WICED_BT_TRACE("\nwake from shutdown");
        bthidlink_determineNextState_on_wake_from_SDS();
    }
    else
    {
        WICED_BT_TRACE("\ncold boot");
        bthidlink_determineNextState();
    }

    //always reset to 0
    wake_from_SDS_timer_timeout = 0;
}
#endif



/////////////////////////////////////////////////////////////////////////////////
/// register application callback functions
///
/// \param cb - pointer to application callback functions
/////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_link_register_app_callback(wiced_bt_hidd_link_app_callback_t *cb)
{
    bthidlink_app_callback = cb;
}

////////////////////////////////////////////////////////////////////////////////
/// Enables page scans. This is done by calling BTM methods to set
/// connectability
////////////////////////////////////////////////////////////////////////////////
void bthidlink_enablePageScans(void)
{
    WICED_BT_TRACE("\nenablePageScans");
#if 0
    if (BTM_SetPageScanType(wiced_hidd_cfg()->br_edr_scan_cfg.page_scan_type))
        WICED_BT_TRACE("\n Failed to set page scan type!");
#endif
    if (wiced_bt_dev_set_connectability(WICED_TRUE,
                                        wiced_hidd_cfg()->br_edr_scan_cfg.page_scan_window,
                                        wiced_hidd_cfg()->br_edr_scan_cfg.page_scan_interval))
        WICED_BT_TRACE("\n Failed to set Connectability");
    else
        WICED_BT_TRACE("\n wiced_bt_dev_set_connectability %d %d", wiced_hidd_cfg()->br_edr_scan_cfg.page_scan_window, wiced_hidd_cfg()->br_edr_scan_cfg.page_scan_interval);
}

////////////////////////////////////////////////////////////////////////////////
/// Enables page and inquiry scans. This is done by calling BTM methods to set
/// discoverability and connectability
////////////////////////////////////////////////////////////////////////////////
void bthidlink_enablePageAndInquiryScans(void)
{
    WICED_BT_TRACE("\nenablePageAndInquiryScans");
#if 0
    if (BTM_SetInquiryScanType(wiced_hidd_cfg()->br_edr_scan_cfg.inquiry_scan_type))
        WICED_BT_TRACE("\n Failed to set inquiry scan type!");
#endif
    if (wiced_bt_dev_set_discoverability(BTM_GENERAL_DISCOVERABLE, //BTM_LIMITED_DISCOVERABLE doesn't work
                                    wiced_hidd_cfg()->br_edr_scan_cfg.inquiry_scan_window,
                                    wiced_hidd_cfg()->br_edr_scan_cfg.inquiry_scan_interval))
        WICED_BT_TRACE("\n Failed to set Discoverability");
    else
        WICED_BT_TRACE("\n wiced_bt_dev_set_discoverability inq_scan_win:%d inq_scan_int:%d", wiced_hidd_cfg()->br_edr_scan_cfg.inquiry_scan_window, wiced_hidd_cfg()->br_edr_scan_cfg.inquiry_scan_interval);

    bthidlink_enablePageScans();

}

////////////////////////////////////////////////////////////////////////////////
/// Disables page and inquiry scans. This is done by calling BTM methods to set
/// discoverability and connectability
////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_link_disable_page_and_inquiry_scans(void)
{
    WICED_BT_TRACE("\ndisablePageAndInquiryScans");
    if (wiced_bt_dev_set_discoverability( BTM_NON_DISCOVERABLE,
                                        wiced_hidd_cfg()->br_edr_scan_cfg.inquiry_scan_window,
                                        wiced_hidd_cfg()->br_edr_scan_cfg.inquiry_scan_interval))
        WICED_BT_TRACE("\nFailed to set Discoverability to none");

    if (wiced_bt_dev_set_connectability( WICED_FALSE,
                                        wiced_hidd_cfg()->br_edr_scan_cfg.page_scan_window,
                                        wiced_hidd_cfg()->br_edr_scan_cfg.page_scan_interval))
        WICED_BT_TRACE("\nFailed to set Connectability to none");
}

////////////////////////////////////////////////////////////////////////////////
/// Enters disconnected state. Upon entering this state we perform the following
/// actions:
///   - change state to DISCONNECTED
///   - disable scans
///   - stop the timer (in case it is running)
///   - note that for us to enter this state implies no queued connect or
///     become discoverable events
///   - Notify application of our state change
////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_link_enter_disconnected(void)
{
//    WICED_BT_TRACE("\nBT: enterDisconnected from state %d", bt_hidd_link.subState );
    wiced_bt_hidd_link_disable_page_and_inquiry_scans();

    wiced_stop_timer(&bt_hidd_link.stateTimer);
    bthidlink_setState(BTHIDLINK_DISCONNECTED);
}

////////////////////////////////////////////////////////////////////////////////
/// Enters discoverable state. Upon entering this state we perform the following
/// actions:
///   - change state to DISCOVERABLE
///   - discard any host/link key saved temporarily in previous discoverable state
///   - enable page and inquiry scans
///   - start the timer for the discoverable period
///   - clear any pending "become discoverable" or "connect" requests
///   - Notify application of our state change
////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_link_enter_discoverable(void)
{
    wiced_bt_device_address_t anyHost = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

//    WICED_BT_TRACE("\n>>enterDiscoverable");
    //savedDiscoveryInfoValid = FALSE;
    bthidlink_enablePageAndInquiryScans();

    bthidlink_setHostAddr(anyHost);

    bt_hidd_link.becomeDiscoverablePending = 0;
    wiced_start_timer( &bt_hidd_link.stateTimer, DISCOVERY_TIMEOUT);
    bthidlink_setState(BTHIDLINK_DISCOVERABLE);
}

////////////////////////////////////////////////////////////////////////////////
/// Enters connectable state. Upon entering this state we perform the following
/// actions:
///   - change state to CONNECTABLE
///   - stop the timer (for safety)
///   - enable page scans
///   - note that for us to enter this state implies no queued connect or
///     become discoverable events
///   - Notify application of our state change
////////////////////////////////////////////////////////////////////////////////
void bthidlink_enterConnectable(void)
{
//    WICED_BT_TRACE("\n>>enterConnectable from %d", bt_hidd_link.subState );
    wiced_stop_timer(&bt_hidd_link.stateTimer);
    // Disable all scans first and then enabe page scans alone
    wiced_bt_hidd_link_disable_page_and_inquiry_scans();
    bthidlink_enablePageScans();
    bthidlink_setState(BTHIDLINK_CONNECTABLE);
}

////////////////////////////////////////////////////////////////////////////////
/// Enters disconnecting state. Upon entering this state we perform the following
/// actions:
///   - change state to DISCONNECTING
///   - stop the timer in case it was running
///   - disable page/inquiry scans (in case they were enabled)
///   - tell the BT hid connection to disconnect
///   - Notify application of our state change
////////////////////////////////////////////////////////////////////////////////
void bthidlink_enterDisconnecting(void)
{
//    WICED_BT_TRACE("\n>>enterDisconnecting from %d", bt_hidd_link.subState );
    switch(bt_hidd_link.subState)
    {
        case BTHIDLINK_CONNECTED:
        case BTHIDLINK_RECONNECTING:
            bt_hidd_link.subState = BTHIDLINK_DISCONNECTING;

            // The disconnect is not for the multicast link
            // Start a timer for 1 ms. Note that enterDisconnecting is called when timer expires
            // and thats when btHidConn->disconnect() will be called.
            wiced_start_timer( &bt_hidd_link.stateTimer, 1);
            break;

        case BTHIDLINK_DISCONNECTED:
            wiced_bt_hidd_link_enter_discoverable();
            break;

        case BTHIDLINK_DISCONNECTING:
        default:
            // Stop the timer if it was running
            wiced_stop_timer(&bt_hidd_link.stateTimer);

            // We don't need to scan. Will do so when we determineNextState
            wiced_bt_hidd_link_disable_page_and_inquiry_scans();

            // disconnect hidd connection
            if (wiced_bt_hidd_disconnect() == WICED_BT_HIDD_SUCCESS)
            {
                bthidlink_setState(BTHIDLINK_DISCONNECTING);
            }
            // Have we been asked to become discoverable
            else if (bt_hidd_link.becomeDiscoverablePending)
            {
                // Become discoverable
                wiced_bt_hidd_link_enter_discoverable();
            }
            else
            {
                bthidlink_setState(BTHIDLINK_DISCONNECTED);
            }
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Page the next host in the reconnect process. Note that we assume that we do
/// not have a pending become discoverable request as we would not be in the
/// reconnecting state otherwise. This is done as follows:
/// - Check if we have exceeded the number of retries for any host. If we have
///   move to the next host.
/// - Check that we have not exceeded the number of bonded hosts. If we have
///   determines what to do by calling determineNextState()
/// - Otherwise page the current host and start the reconnect timer.
////////////////////////////////////////////////////////////////////////////////
void bthidlink_pageNextHost(void)
{
    // If we will exceed the reconnect retry count for the current host, move to the next
    if (++bt_hidd_link.reconnectRetryCount > wiced_bt_hidlinkcfg.maxReconnectRetryCount)
    {
            // Move to the next host
            bt_hidd_link.reconnectHostIndex++;

            // Reset retry count so we can try the next host
            bt_hidd_link.reconnectRetryCount = 0;
    }

    // Ensure that the host count is in range
    if (bt_hidd_link.reconnectHostIndex < wiced_hidd_host_count())
    {
        // Setup a timer since we are not sure if we have to start connecting immediately or
        // wait for sometime until the stack closes a previous attempt.
        // Lets wait for 10 ms
        wiced_start_timer( &bt_hidd_link.stateTimer, 10);
    }
    else
    {
        // Tell the app that we ran out of hosts
        if (bthidlink_app_callback && bthidlink_app_callback->p_app_connection_failed_notification)
        {
            bthidlink_app_callback->p_app_connection_failed_notification();
        }

        // Out of hosts. Figure out what to do
        bthidlink_determineNextState();
    }
}


////////////////////////////////////////////////////////////////////////////////
/// Enters reconnecting state. Upon entering this state we perform the following
/// actions:
///   - change state to RECONNECTING
///   - disable page/inquiry scans (in case they were enabled)
///   - stop the timer in case it was running
///   - set host index and reconnect retry count to 0 to mark the start of the
///     reconnect process
///   - Set page timeout to reconnectPageTimeout
///   - page the first host
///   - Notify application of our state change
////////////////////////////////////////////////////////////////////////////////
void bthidlink_enterReconnecting(void)
{
    // If a connect request was pending, reset it - we are trying to connect
    WICED_BT_TRACE("\nenterReconnecting :%d", bt_hidd_link.subState );

    bthidlink_setHostAddr(wiced_hidd_host_addr());
    wiced_bt_hidd_link_disable_page_and_inquiry_scans();

#if 0
    if(bt_hidd_link.subState == BTHIDLINK_DISCOVERABLE)
    {
        bthidlink_discoverableStateWhileReconnect = WICED_TRUE;
    }
#endif
    wiced_stop_timer(&bt_hidd_link.stateTimer);

    bt_hidd_link.reconnectHostIndex = 0;
    bt_hidd_link.reconnectRetryCount = 0;

    // Update page timeout as we don't know what it is set to
    if (BTM_WritePageTimeout(wiced_bt_hidlinkcfg.reconnectPageTimeout))
    {
        WICED_BT_TRACE("\nFailed to write page timeout");
    }


    bthidlink_setState(BTHIDLINK_RECONNECTING);

    // Page next host will now page the first host because we set the parameters
    // above correctly. If multicast, that will also be handled correctly
    bthidlink_pageNextHost();
}

////////////////////////////////////////////////////////////////////////////////
/// This method informs the transport that our state timer has expired.
/// \param data provided by timer. Ignored
////////////////////////////////////////////////////////////////////////////////
void bthidlink_activityDetected()
{
    if (bt_hidd_link.subState == BTHIDLINK_CONNECTED)
    {
//        WICED_BT_TRACE("\nstart idle timer");
        if (wiced_is_timer_in_use(&bt_hidd_link.stateTimer))
        {
            wiced_stop_timer(&bt_hidd_link.stateTimer);
        }
        wiced_start_timer( &bt_hidd_link.stateTimer, IDLE_TIMEOUT);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Enters connected state. Upon entering this state we perform the following
/// actions:
///   - change state to CONNECTED
///   - stop the timer in case it was running
///   - disable page/inquiry scans
///   - save the BD address of this host as the last connected host
///     as we may want to reconnect to it in case of an abnormal disconnect
///   - move this host to the top of the host list if configured to do so
///   - clear any pending connect request
///   - notify application of our state change
///   - if a become discoverable request is pending, enters DISCONNECTING state
////////////////////////////////////////////////////////////////////////////////
void bthidlink_enterConnected(const BD_ADDR host_bd_addr)
{
//    WICED_BT_TRACE("\n>>bthidlink_enterConnected from %d", bt_hidd_link.subState );

    // Stop state timer in case it is running
    //stateTimer->stop();

    // Set as an active hsot
    hidd_host_setTransport(host_bd_addr, BT_TRANSPORT_BR_EDR);

    // Ensure that we are not doing any scans. We no longer want to
    // be discoverable/connectable
    wiced_bt_hidd_link_disable_page_and_inquiry_scans();

    // Set packet types we want to use
    BTM_SetPacketTypes((uint8_t*) host_bd_addr, wiced_bt_hidlinkcfg.packetTypes);

    // Inform the app of our current state.
    bthidlink_setState(BTHIDLINK_CONNECTED);

    // start idle timer
    bthidlink_activityDetected();
}

////////////////////////////////////////////////////////////////////////////////
/// This method tells the transport to connect. If we don't have any hosts
/// or if we are not allowed to reconnect, it immediately calls the application
/// notification method with the current state of the transport.
/// Otherwise, we enter reconnect if we don't have any connections. If we have a partial
/// connection, we queue the request and wait for it to complete or terminate before
/// deciding what to do.
/// If the connect request is deferred, it will be lower priority than any
/// "become discoverable" request that was pending from before or comes after
/// the connect but before the connect processing starts. In this situation
/// the connect request will get ignored.
/// The behavior per state (based on the above) is:
///  - INITIALIZED: Must not be called.
///  - DISCONNECTED: enter reconnect state
///  - DISCOVERABLE: if we have a partial connection we queue this request waiting
///        for either the current connection to complete (at which point we
///        will discard the request) or for the connection to fail (at which point we will attempt
///        to connect). If we don't have a partial connection we enter reconnect
///  - CONNECTABLE: if we have a partial connection we queue this request waiting
///        for either the current connection to complete (at which point we
///        will discard the request) or for the connection to fail (at which point we will attempt
///        to connect). If we don't have a partial connection we enter reconnect
///  - CONNECTED: Ignored. Should not happen.
///  - DISCONNECTING: connect request saved for processing after the disconnect completes
///  - RECONNECTING: Ignored. We are already trying to connect
////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_link_connect(void)
{
//    WICED_BT_TRACE("\n>>wiced_bt_hidd_link_connect %d", bt_hidd_link.subState);
    // Not legal in INITIALIZED and CONNECTED states
    if ((bt_hidd_link.subState != BTHIDLINK_INITIALIZED) && (bt_hidd_link.subState != BTHIDLINK_CONNECTED))
    {
        if (wiced_hidd_is_paired() && (wiced_bt_hidlinkcfg.reconnectInitiate))
        {
             switch(bt_hidd_link.subState)
            {
                case BTHIDLINK_DISCONNECTED:
                case BTHIDLINK_DISCOVERABLE:
                case BTHIDLINK_CONNECTABLE:
                case BTHIDLINK_DISCONNECTING:
                    // Enter reconnecting state
                    bthidlink_enterReconnecting();
                    break;
                case BTHIDLINK_CONNECTED:
                case BTHIDLINK_RECONNECTING:
                    // In these states we are either already reconnecting or already connected. Ignore the request.
                    break;
            }
        }
        else if (BTHIDLINK_DISCONNECTED == bt_hidd_link.subState)
        {
            wiced_bt_hidd_link_enter_discoverable();
        }

    }
}

////////////////////////////////////////////////////////////////////////////////
/// This method tells the tranport to initiate a disconnect. Note that
/// this is illegal in INITIALIZED states. For other states
/// this causes us to enter DISCONNECTING state
////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_link_disconnect(void)
{
//    WICED_BT_TRACE("\n>>disconnect from %d", bt_hidd_link.subState );

    // Ignore in INITIALIZEDstates
    if (bt_hidd_link.subState != BTHIDLINK_INITIALIZED)
    {
        // For all other states enter disconnecting.
        bthidlink_enterDisconnecting();
    }
}

////////////////////////////////////////////////////////////////////////////////
/// This method informs the transport that the HID layer was just connected, i.e.
/// the interrupt channel just came up. As we just connected, this clears
/// any pending connect requests.
/// The behavior per state is:
///  - INITIALIZED: Impossible. Ignore.
///  - DISCONNECTED: Impossible. Enter disconnecting to get back to known good state
///  - DISCOVERABLE: We were just virtually cabled. Add the host to the host list
///        if it is not already in it. Flag that this is the first connection after
///        discoverable. Move to connected state.
///  - CONNECTABLE: One of our hosts just connected to us. Move to connected state.
///  - CONNECTED: Impossible. Issue disconnect to get back to known good state
///  - DISCONNECTING: May happen because of race conditions. Do nothing
///        as we have already initiated a disconnect
///  - RECONNECTING: We just connected. Move to connected state.
////////////////////////////////////////////////////////////////////////////////
void bthidlink_connectInd(const BD_ADDR host_bd_addr)
{
    WICED_BT_TRACE("\nbthidlink_connectInd from %d", bt_hidd_link.subState );

    // Ignore in INITIALIZED/RESET states
    if (bt_hidd_link.subState != BTHIDLINK_INITIALIZED)
    {
        // Rest pf the processing depends on the state.
        switch(bt_hidd_link.subState)
        {
            case BTHIDLINK_CONNECTED:
                // Cant get a connection in either of these states. Initiate disconnect
                bthidlink_enterDisconnecting();
                break;
            case BTHIDLINK_DISCONNECTING:
                // This may happen legitimately if we initiated a disconnect and immediately
                // get back a connected indicator queued just before. Ignore as we have already
                // initiated a disconnect.
                break;
            case BTHIDLINK_DISCONNECTED:
            case BTHIDLINK_DISCOVERABLE:
                // Flag that this is the first connection out of DISCOVERABLE
                //bthidlink_firstConnAfterDiscoverable = WICED_TRUE;

                // Enter connected state
                bthidlink_enterConnected(host_bd_addr);
                break;
            case BTHIDLINK_RECONNECTING:
            case BTHIDLINK_CONNECTABLE:
                // Flag that we did not become CONNECTED from DISCOVERABLE
                //bthidlink_firstConnAfterDiscoverable = WICED_FALSE;

                // Enter connected state. Note that host must be in the host list at this time.
                bthidlink_enterConnected(host_bd_addr);
                break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// Check if the current bt link state is in the requested state
///
/// \return TRUE/FALSE
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_bt_hidd_link_state_is(uint8_t state)
{
    return bt_hidd_link.subState == state;
}

////////////////////////////////////////////////////////////////////////////////
/// This method informs the transport that an ACL was just disconnected.
/// If the disconnect reason is authentication failure, it clears any temporary
/// key associated with this host. It also unconditionally clear auto-pairing
/// mode. Additional per state behavior is described below:
///  - INITIALIZED: Impossible. Ignore.
///  - DISCONNECTED: Impossible. Reenter disconnected to ensure
///    we disable scans.
///  - DISCOVERABLE: Normal behavior. If the disconnect reason is
///        authentication failure and we are configured to
///        exit discoverable on authentication failure, do so
///        by calling determineNextState(). Otherwise, do nothing
///  - CONNECTABLE: Someone tried to connect but failed. Ignore.
///  - CONNECTED: If the reason is authentication failure or we were in pin
///        code entry mode and this is the first connection after discoverable
///        and we are configured to become discoverable in such a situation, do so.
///        Otherwise reassess next state via call to determineNextState()
///  - DISCONNECTING: Reasses next state via call to determineNextState()
///  - RECONNECTING: Connection came up partially but failed.
///        Treat it like a page failure
/// \param reason HCI disconnect reason
////////////////////////////////////////////////////////////////////////////////
void bthidlink_disconnectInd(uint16_t reason)
{
    WICED_BT_TRACE("\nbthidlink_disconnectInd %d", bt_hidd_link.subState);
#if 0
    // Clear auto-pairing flag. It does not last across connecttions
    autoPairingRequested = FALSE;
#endif

    // Inform application to exit pin/pass code entry mode
    if (bthidlink_app_callback && bthidlink_app_callback->p_app_exit_pin_and_passcode_entry_mode)
    {
        bthidlink_app_callback->p_app_exit_pin_and_passcode_entry_mode();
    }

    switch(bt_hidd_link.subState)
    {
        case BTHIDLINK_DISCONNECTED:
            wiced_bt_hidd_link_enter_disconnected();
            break;
        case BTHIDLINK_INITIALIZED:
        case BTHIDLINK_CONNECTABLE:
            // Not possible
            break;
        case BTHIDLINK_DISCOVERABLE:
            WICED_BT_TRACE("\nBTHIDLINK_DISCOVERABLE: %d",reason );
            if ((reason == HCI_ERR_AUTH_FAILURE) && wiced_bt_hidlinkcfg.exitDiscoverableOnAuthFailure)
            {
                bthidlink_determineNextState();
            }
            break;
        case BTHIDLINK_CONNECTED:
            bthidlink_determineNextState();
            break;

        case BTHIDLINK_DISCONNECTING:
            bthidlink_determineNextState();
            break;
        case BTHIDLINK_RECONNECTING:
            // If host lost/removed the link key
            if (bt_hidd_link.security_failed == HCI_ERR_KEY_MISSING)
            {
                WICED_BT_TRACE("\nremove paired host: %B", wiced_hidd_host_addr());
                wiced_bt_dev_delete_bonded_device(wiced_hidd_host_addr());
                wiced_hidd_host_remove();
            }

            // see if we can connect to the next host..
            bthidlink_pageNextHost();
            break;
    }

    //reset link encrypted flag
    if (!memcmp(wiced_hidd_host_addr(), bt_hidd_link.encrypt_status.bdAddr, BD_ADDR_LEN))
    {
        bt_hidd_link.encrypt_status.encrypted = WICED_FALSE;
    }

    //reset security failed flag
    bt_hidd_link.security_failed = 0;

    // We are not requesting a pin code at this time. Clear it unconditionally.
    // Note that this must be done at the end as we use the value above
    //bthidlink_pinCodeRequested = WICED_FALSE;
}

////////////////////////////////////////////////////////////////////////////////
/// This method tells the tranport to initiate a device VC unplug.
/// It immediately clears the host list, removes bonded device info from btstack, and
/// request VC unplug.
///
/// \return WICED_TRUE if sent VIRTUAL CABLE UNPLUG message, WICED_FALSE otherwise
////////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_bt_hidd_link_virtual_cable_unplug(void)
{
    wiced_bool_t sentVcUnplug = WICED_FALSE;

    WICED_BT_TRACE("\nwiced_bt_hidd_link_virtual_cable_unplug");

    //must pause power managment. i.e. stop transition between active, sniff, sniff subrate etc.
    //so that it won't' interfere with device vc unplug
//    wiced_bt_hidd_power_management_pause();

    // Remove current/last connected host from the pairing list and WICED btstack
    if (wiced_hidd_is_paired())
    {
        WICED_BT_TRACE("\nwiced_bt_hidd_link_virtual_cable_unplug from %B", wiced_hidd_host_addr());

        wiced_bt_dev_delete_bonded_device(wiced_hidd_host_addr());
        wiced_hidd_host_remove();

        // virtual cable unplug
        sentVcUnplug = wiced_bt_hidd_virtual_unplug() ? WICED_FALSE : WICED_TRUE;
    }
    else
    {
        WICED_BT_TRACE("\nnot bt virtual cable connected");
    }

    // disable page scan and inquiry scan anyway */
    wiced_bt_hidd_link_disable_page_and_inquiry_scans();

    /* disconnect any connections if active */
    wiced_bt_hidd_disconnect( );

    //We will deregister HIDD without waiting the result of the HIDD Disconnection above.
    //Call wiced_bt_hidd_init to reset HIDD state.
    wiced_bt_hidd_init();

    bthidlink_setState(BTHIDLINK_DISCONNECTED);

    return sentVcUnplug;

}


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
void bthidlink_pinCode(uint8_t pinCodeSize, uint8_t *pinCodeBuffer)
{
    // Flag that we are no longer requesting pin code from the application
    //bthidlink_pinCodeRequested = WICED_FALSE;

    // We are only interested in DISCOVERABLE and CONNECTED states
#if 0
    if ((subState == DISCOVERABLE || subState == CONNECTED) ||
        (hostList->isWaitForSdpEnabled(reconnectHostIndex) &&
         BT_MEMCMP(tmpAddr, &(*hostList)[reconnectHostIndex], sizeof(BD_ADDR)) == 0))
#else
    if ((bt_hidd_link.subState == BTHIDLINK_DISCOVERABLE || bt_hidd_link.subState == BTHIDLINK_CONNECTED))
#endif
    {
        wiced_bt_dev_pin_code_reply(bthidlink_passkeyreq_bdaddr, WICED_BT_SUCCESS, pinCodeSize, pinCodeBuffer);
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
void bthidlink_pinCodeRequest(wiced_bt_dev_name_and_class_t *p_event_data)
{
    switch(bt_hidd_link.subState)
    {
        case BTHIDLINK_INITIALIZED:
        case BTHIDLINK_DISCONNECTED:
        case BTHIDLINK_DISCONNECTING:
            break;
        case BTHIDLINK_RECONNECTING:
        case BTHIDLINK_CONNECTABLE:
#if 0
            if(!(hostList->isWaitForSdpEnabled(reconnectHostIndex) &&
             BT_MEMCMP(bdAddr, &(*hostList)[reconnectHostIndex], sizeof(BD_ADDR)) == 0))
            {
                // If we were not waiting for the host to complete the connection, end it
                btHidConn->disconnect();
                break;
            }
#endif
        case BTHIDLINK_DISCOVERABLE:
        case BTHIDLINK_CONNECTED:
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
                if (bthidlink_app_callback && bthidlink_app_callback->p_app_enter_pincode_entry_mode)
                {
                    bthidlink_app_callback->p_app_enter_pincode_entry_mode();
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
void bthidlink_passCode(uint8_t pinCodeSize, uint8_t *pinCodeBuffer)
{
    uint32_t passkey = 0;

    // Flag that we are no longer requesting pin code from the application
    //bthidlink_pinCodeRequested = WICED_FALSE;

    if(pinCodeSize && pinCodeBuffer)
    {
        if(bt_hidd_link.subState == BTHIDLINK_DISCOVERABLE)
        {
            uint8_t i;
            // Convert the char string to an unsigned int, base10
            //passkey = (UINT32) utl_strtoul((const char*)pinCodeBuffer, NULL, 10);
            for (i=0; i<pinCodeSize; i++)
            {
              passkey = passkey * 10 + pinCodeBuffer[i] - '0';
            }
            WICED_BT_TRACE("\npasskey: %d",passkey);

            // Now pass the pass key to BTM.
            wiced_bt_dev_pass_key_req_reply(WICED_BT_SUCCESS, bthidlink_passkeyreq_bdaddr, passkey);
        }
    }
    else
    {
        // If app is not capable of responding to pass key, reject it
        wiced_bt_dev_pass_key_req_reply(WICED_BT_UNSUPPORTED, bthidlink_passkeyreq_bdaddr, 0);
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
void bthidlink_passKeyRequest(wiced_bt_dev_user_key_req_t* passKeyReq)
{
    memcpy(bthidlink_passkeyreq_bdaddr, passKeyReq->bd_addr, BD_ADDR_LEN);

    switch(bt_hidd_link.subState)
    {
    case BTHIDLINK_INITIALIZED:
    case BTHIDLINK_DISCONNECTED:
    case BTHIDLINK_DISCONNECTING:
    case BTHIDLINK_CONNECTED:
        // Ignore and reject.
        break;

    case BTHIDLINK_CONNECTABLE:
#if 0
        if(!(hostList->isWaitForSdpEnabled(reconnectHostIndex) &&
             BT_MEMCMP(passKeyReq->bd_addr, &(*hostList)[reconnectHostIndex], sizeof(BD_ADDR)) == 0))
        {
            // reject and tell btHidConn to disconnect
            btHidConn->disconnect();
            break;
        }
#endif
        // else deliberate fallthrough
    case BTHIDLINK_DISCOVERABLE:
    case BTHIDLINK_RECONNECTING:
        //call back to application to enter Pass Key
        if (bthidlink_app_callback && bthidlink_app_callback->p_app_enter_passcode_entry_mode)
        {
            bthidlink_app_callback->p_app_enter_passcode_entry_mode();
        }
        break;
    }

}

////////////////////////////////////////////////////////////////////////////////
/// Provide a key press indication to the peer. If the subState is DISCOVERABLE,
/// uses BTM to send out a notification to the peer. Else ignored.
////////////////////////////////////////////////////////////////////////////////
void bthidlink_passCodeKeyPressReport(uint8_t key)
{
    switch(bt_hidd_link.subState)
    {
    case BTHIDLINK_CONNECTABLE:
    case BTHIDLINK_RECONNECTING:
#if 0
        if(!(hostList->isWaitForSdpEnabled(reconnectHostIndex) &&
             BT_MEMCMP(btHidConn->getBdAddr(), &(*hostList)[reconnectHostIndex], sizeof(BD_ADDR)) == 0))
        {
            break;
        }
#endif
        // else deliberate fallthrugh
    case BTHIDLINK_DISCOVERABLE:
        // Send the noti out
        wiced_bt_dev_send_key_press_notif(bthidlink_passkeyreq_bdaddr, (UINT8)key);
        break;

    default:
        break;
    }
}

////////////////////////////////////////////////////////////////////////////////
/// This method informs the transport that our state timer has expired.
/// \param data provided by timer. Ignored
////////////////////////////////////////////////////////////////////////////////
void bthidlink_statetimerTimeoutCb(uint32_t args)
{
    // This timer is for initiating a connect, initiating a disconnect and time for which we are going to be discoverable.
    // Since we will never be in DISCOVERABLE state and RECONNECTING states at the same time,  it is safe to use the same timer.
    // If our substate is RECONNECTING, this timer MUST have been started by one of connect and friends.
    WICED_BT_TRACE("\nbthidlink_statetimerTimeoutCb %d",bt_hidd_link.subState );
    switch(bt_hidd_link.subState)
    {
        case BTHIDLINK_CONNECTED:
            wiced_bt_hidd_link_disconnect();
            break;

        case BTHIDLINK_RECONNECTING:
            wiced_bt_hidd_connect();
            break;

        case BTHIDLINK_DISCOVERABLE:
            wiced_hidd_pairing_stopped(BT_TRANSPORT_BR_EDR);
            wiced_bt_hidd_link_disconnect();
            break;

        case BTHIDLINK_DISCONNECTING:
            // When disconnecting issue the real disconnect now.
            wiced_bt_hidd_link_disconnect();
            break;

        default:
            // We dont expect the timer to fire in any other state!
            //ASSERT_PANIC(0, subState, NULL);
            break;
    }

}

////////////////////////////////////////////////////////////////////////////////
/// This method is called when it's time to call the application's
/// pollReportUserActivity().
////////////////////////////////////////////////////////////////////////////////
int bthidlink_pollTimerExpiryAction(void *data)
{
     // Tell application to poll
     if (bthidlink_app_callback && bthidlink_app_callback->p_app_poll_user_activities && bt_hidd_link.appPoll_enabled)
     {
        bthidlink_app_callback->p_app_poll_user_activities();
     }

     return 0;
}

/////////////////////////////////////////////////////////////////////////////
/// Called by BCS (from ISR) at the sniff notification instant
///
/// \param task - don't care
/// \param context - don't care
/////////////////////////////////////////////////////////////////////////////
void bthidlink_sniffNotification(void* task, uint32_t context)
{
   wiced_app_event_serialize(bthidlink_pollTimerExpiryAction, NULL);
}

////////////////////////////////////////////////////////////////////////////////
/// Enable or disable application polling
///
/// \param enable If TRUE, will enable application polling, else, polling
/// will be disabled.
////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_link_enable_poll_callback(wiced_bool_t enable)
{
  if (bt_hidd_link.appPoll_enabled == enable)
  {
    return;
  }

  WICED_BT_TRACE("\nenableAppPoll:%d", enable);
  bt_hidd_link.appPoll_enabled = enable;

  wiced_hidd_register_callback_for_poll_event(BT_TRANSPORT_BR_EDR, wiced_hidd_host_addr(), enable, bthidlink_sniffNotification);
}

////////////////////////////////////////////////////////////////////////////////////
/// save/restore contents to/from Always On Memory when entering/exiting SDS
///
/// \param type - BTHIDLINK_SAVE_TO_AON or BTHIDLINK_RESTORE_FROM_AON
////////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_link_aon_action_handler(uint8_t  type)
{
    if (type == BTHIDLINK_RESTORE_FROM_AON)
    {
        WICED_BT_TRACE("\nWICED_BT_AON_DRIVER_RESTORE");

        bt_hidd_link.resumeState = bthid_aon_data.bthidlink_state;
    }
    else
    {
        // save all output GPIO values in the saved cfgs before entering uBCS mode
        //wiced_hal_gpio_slimboot_reenforce_outputpin_value();

        bthid_aon_data.bthidlink_state = bt_hidd_link.subState;
    }
}


void bthidlink_bthidd_evtHandler(wiced_bt_hidd_cback_event_t  event, uint32_t data, wiced_bt_hidd_event_data_t *p_event_data )
{
    uint8_t status = HID_PAR_HANDSHAKE_RSP_ERR_UNSUPPORTED_REQ;
    WICED_BT_TRACE("\nEVENT: WICED_BT_HIDD_EVT_");
    switch (event)
    {
    case WICED_BT_HIDD_EVT_OPEN:
        WICED_BT_TRACE("OPEN: %B", p_event_data->host_bdaddr);
#ifdef BLE_SUPPORT
        wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
#endif
        bthidlink_connectInd(p_event_data->host_bdaddr);
        hci_control_send_data( HCI_CONTROL_HIDD_EVENT_OPENED, p_event_data->host_bdaddr, BD_ADDR_LEN );
        break;

    case WICED_BT_HIDD_EVT_CLOSE:
        WICED_BT_TRACE("CLOSE: reason:%d",data);
        bthidlink_disconnectInd(data);
        status = (uint8_t) data;
        hci_control_send_data( HCI_CONTROL_HIDD_EVENT_CLOSED, &status, 1 );
        break;

    case WICED_BT_HIDD_EVT_MODE_CHG:
        WICED_BT_TRACE("MODE_CHG: mode=%d, interval=%d", data, p_event_data->pm_interval);
        break;

    case WICED_BT_HIDD_EVT_PM_FAILED:
        WICED_BT_TRACE("PM_FAILED: mode=%d, error_code=%d", data, p_event_data->pm_err_code);
        break;

    case WICED_BT_HIDD_EVT_CONTROL:
        WICED_BT_TRACE("CONTROL: param=%d", data);
        if (data == HID_PAR_CONTROL_VIRTUAL_CABLE_UNPLUG)
        {
            // Remove current/last connected host from the list
            WICED_BT_TRACE("\nremove bonded device : %B", wiced_hidd_host_addr());
            wiced_bt_dev_delete_bonded_device(wiced_hidd_host_addr());
            wiced_hidd_host_remove();

            //if vc unplug from host, it will eventually disconnect.
            //if we don't want to enter discoverable right afterwards, set this flag to 0.
            //otherwise, remove this line
            wiced_bt_hidlinkcfg.becomeDiscoverableWhenNotConnected = 0;
        }
        break;

    case WICED_BT_HIDD_EVT_GET_REPORT:
        WICED_BT_TRACE("GET_REPORT: data=%d\n, reportID=%d, reportType=%d", data, p_event_data->get_rep.rep_id, p_event_data->get_rep.rep_type);
        if (bthidlink_app_callback && bthidlink_app_callback->p_app_get_report)
        {
            status = bthidlink_app_callback->p_app_get_report(p_event_data->get_rep.rep_type, p_event_data->get_rep.rep_id);
        }

        if (status)
        {
            wiced_bt_hidd_hand_shake(status);
        }
        break;

    case WICED_BT_HIDD_EVT_SET_REPORT:
        WICED_BT_TRACE("SET_REPORT: data=%d, len=%d \n", data, p_event_data->data.len);
        TRACE_ARRAY(p_event_data->data.p_data, p_event_data->data.len);

        if (bthidlink_app_callback && bthidlink_app_callback->p_app_set_report)
        {
            status = bthidlink_app_callback->p_app_set_report(data, p_event_data->data.p_data, p_event_data->data.len);
        }
        wiced_bt_hidd_hand_shake(status);
        break;

    case WICED_BT_HIDD_EVT_GET_PROTO:
        WICED_BT_TRACE("GET_PROTO:");
        if (bthidlink_app_callback && bthidlink_app_callback->p_app_get_protocol)
        {
            uint8_t protocol = bthidlink_app_callback->p_app_get_protocol();
            wiced_bt_hidd_send_data(WICED_TRUE, HID_PAR_REP_TYPE_INPUT, &protocol, 1);
        }
        else
        {
            wiced_bt_hidd_hand_shake(status);
        }

        break;

    case WICED_BT_HIDD_EVT_SET_PROTO:
        WICED_BT_TRACE("SET_PROTO: %d", data);
        if (bthidlink_app_callback && bthidlink_app_callback->p_app_set_protocol)
        {
            status = bthidlink_app_callback->p_app_set_protocol(data);
        }
        wiced_bt_hidd_hand_shake(status);
        break;

    case WICED_BT_HIDD_EVT_GET_IDLE:
        WICED_BT_TRACE("GET_IDLE");
        if (bthidlink_app_callback && bthidlink_app_callback->p_app_get_idle)
        {
            uint8_t idlerate = bthidlink_app_callback->p_app_get_idle();
            wiced_bt_hidd_send_data(WICED_TRUE, HID_PAR_REP_TYPE_INPUT, &idlerate, 1);
        }
        else
        {
            wiced_bt_hidd_hand_shake(status);
        }
        break;

    case WICED_BT_HIDD_EVT_SET_IDLE:
        WICED_BT_TRACE("SET_IDLE: data=%d", data);
        if (bthidlink_app_callback && bthidlink_app_callback->p_app_set_idle)
        {
            status = bthidlink_app_callback->p_app_set_idle(data);
        }
        wiced_bt_hidd_hand_shake(status);
        break;

    case WICED_BT_HIDD_EVT_DATA:
        WICED_BT_TRACE("DATA: len=%d data=", p_event_data->data.len);
        TRACE_ARRAY(p_event_data->data.p_data, p_event_data->data.len);
        if (bthidlink_app_callback && bthidlink_app_callback->p_app_rx_data)
        {
            bthidlink_app_callback->p_app_rx_data(data, p_event_data->data.p_data, p_event_data->data.len);
        }
        break;

    case WICED_BT_HIDD_EVT_RETRYING:
        WICED_BT_TRACE("RETRYING: Repage timeout, retrial=%d", data);
        break;

    default:
        WICED_BT_TRACE(": unsupported evt:%d data=%d", event, data);
        break;
    }
}

#endif //#ifdef BR_EDR_SUPPORT
