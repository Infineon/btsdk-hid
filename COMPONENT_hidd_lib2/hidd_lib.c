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

/********************************************************************************
*
* File Name: hidd_lib.c
*
* Abstract: This file implements HID application transport that supports both the Bluetooth(BT) Classic
*               and LE
* Functions:
*
*******************************************************************************/
#include "wiced_hal_batmon.h"
#include "wiced_hal_nvram.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_pwm.h"
#include "wiced_hal_aclk.h"
#include "wiced_hal_mia.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_ota_firmware_upgrade.h"
#include "wiced_bt_stack.h"
#include "wiced_hidd_lib.h"
#include "hci_control_api.h"
#include "hidd_lib.h"

#define PMU_CONFIG_FLAGS_ENABLE_SDS           0x00002000
extern UINT32 g_foundation_config_PMUflags;

//Local identity key ID
#define  VS_LOCAL_IDENTITY_ID WICED_NVRAM_VSID_START

#define STOP_PAIRING  0
#define START_PAIRING 1

//#if !is_208xxFamily
#if 0
 #define SFI_DEEP_SLEEP 1
#else
 #define SFI_DEEP_SLEEP 0
#endif

#if SFI_DEEP_SLEEP
extern uint8_t pmu_attemptSleepState;
extern void sfi_enter_deep_power_down(void);
extern void sfi_exit_deep_power_down(BOOL8 forceExitDeepPowerDown);
extern void sfi_allow_deep_sleep(void);
#else
 #define sfi_allow_deep_sleep()
#endif

typedef struct
{
    // is shutdown sleep (SDS) allowed?
    uint8_t allowDeepSleep:1;
    uint8_t allowHIDOFF:1;


    /// allow SDS timer
    wiced_timer_t allowDeepSleepTimer;

    wiced_sleep_allow_check_callback  p_app_sleep_handler;

    const wiced_bt_cfg_settings_t * bt_cfg_ptr;

    uint8_t pairing_type;

} tHiddLink; tHiddLink hidd = {};



/////////////////////////////////////////////////////////////////////////////////
/// register application callback functions
///
/// \param cb - pointer to application callback functions
/////////////////////////////////////////////////////////////////////////////////
void hidd_register_app_callback(hidd_link_callback_t *link_cb)
{
    link.callbacks = link_cb;
}

////////////////////////////////////////////////////////////////////////////////
// returns chip number
////////////////////////////////////////////////////////////////////////////////
uint32_t hidd_chip_id()
{
#if is_208xxFamily
    #define RADIO_ID    0x006007c0
    #define RADIO_20820 0x80
    static uint32_t chip = 0;

    // the radio id register become not accessible after ePDS; thus, read it only once at power up. Return the saved value thereafter.
    if (!chip)
    {
        chip = (*(UINT32*) RADIO_ID & RADIO_20820) ? 20820 : 20819;
    }
    return chip;
#else
    return CHIP;
#endif
}

////////////////////////////////////////////////////////////////////////////////
/// This function is the timeout handler for allowsleep_timer
////////////////////////////////////////////////////////////////////////////////
void hidd_allowsleeptimerCb( uint32_t arg )
{
    hidd_deep_sleep_not_allowed(0); // sleep is allowed now
}

////////////////////////////////////////////////////////////////////////////////
/// This function returns if sleep is allowed
////////////////////////////////////////////////////////////////////////////////
uint8_t hidd_is_deep_sleep_allowed()
{
    return hidd.allowDeepSleep;
}

////////////////////////////////////////////////////////////////////////////////
/// This function returns if allow to sleep timer is running
////////////////////////////////////////////////////////////////////////////////
uint8_t hidd_is_deep_sleep_timer_running()
{
    return wiced_is_timer_in_use(&hidd.allowDeepSleepTimer);
}

////////////////////////////////////////////////////////////////////////////////
/// This function allows device to sleep
////////////////////////////////////////////////////////////////////////////////
void hidd_set_deep_sleep_allowed(uint8_t allowed)
{
    hidd.allowDeepSleep = allowed;
    if (hidd_is_deep_sleep_timer_running())
    {
        wiced_stop_timer(&hidd.allowDeepSleepTimer);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// This function not allowing the device to sleep for period of time
////////////////////////////////////////////////////////////////////////////////
void hidd_deep_sleep_not_allowed( uint32_t milliseconds )
{
    hidd_set_deep_sleep_allowed(milliseconds ? WICED_FALSE : WICED_TRUE);
    if (!hidd_is_deep_sleep_allowed())
    {
        wiced_start_timer(&hidd.allowDeepSleepTimer, milliseconds);
//        WICED_BT_TRACE("\ndeepSleep not allowed for %d milliseconds", milliseconds);
    }
    else
    {
//        WICED_BT_TRACE("\ndeepSleep allowed");
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Sleep permit query to check if sleep (normal or SDS) is allowed and sleep time
///
/// \param type - sleep poll type
///
/// \return   sleep permission or sleep time, depending on input param
////////////////////////////////////////////////////////////////////////////////////////////////////////////
static uint32_t HIDD_sleep_handler(wiced_sleep_poll_type_t type )
{
    uint32_t ret = WICED_SLEEP_NOT_ALLOWED;

#if SLEEP_ALLOWED
    switch(type)
    {
    case WICED_SLEEP_POLL_TIME_TO_SLEEP:
        //query application for sleep time
        ret = hidd.p_app_sleep_handler ? hidd.p_app_sleep_handler(type) : WICED_SLEEP_MAX_TIME_TO_SLEEP;

 #if SFI_DEEP_SLEEP
        // In 20735, sfi CS may contains glitch that wakes up Flash result in high current. Apply workaround to put sfi into powerdown
        if ((ret == WICED_SLEEP_MAX_TIME_TO_SLEEP) && ( pmu_attemptSleepState == 5 ))
        {
            sfi_exit_deep_power_down(FALSE);
            sfi_enter_deep_power_down();
        }
 #endif
        break;

    case WICED_SLEEP_POLL_SLEEP_PERMISSION:
 #if SLEEP_ALLOWED > 1
        //query application for sleep permit
        ret = (hidd.p_app_sleep_handler) ? hidd.p_app_sleep_handler(type) : WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;

        // if we are allowed to shutdown, check further if we really should shutdown
        if ( (ret == WICED_SLEEP_ALLOWED_WITH_SHUTDOWN)
  #ifdef FATORY_TEST_SUPPORT
              && !force_sleep_in_HID_mode
  #endif
           )
        {
            // any of the following is true, we don't allow shutdown
            if ( !hidd_is_deep_sleep_allowed()
                 || wiced_hidd_is_transport_detection_polling_on()
  #ifdef BLE_SUPPORT
                 || (hidd_blelink.second_conn_state == BLEHIDLINK_2ND_CONNECTION_PENDING)
  #endif
  #if defined(BR_EDR_SUPPORT) && is_20735Family
                 //due to sniff+SDS is not supported in core FW, at this time, only allow SDS when disconnected
                 || (bt_hidd_link.subState != HIDLINK_DISCONNECTED)
  #endif
  #ifdef OTA_FIRMWARE_UPGRADE
                 || wiced_ota_fw_upgrade_is_active()
  #endif
              )
           {
               // change to no shutdown
               ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
           }
       }

       // if we are shutting down, prepare for shutdown
       if (ret == WICED_SLEEP_ALLOWED_WITH_SHUTDOWN)
       {
// if support epds, we need to determine if it should enter epds or hidoff
  #ifdef SUPPORT_EPDS
            if (!wiced_hidd_link_is_disconnected() || !hidd.allowHIDOFF)
            {
                // enter ePDS
                g_foundation_config_PMUflags &= ~PMU_CONFIG_FLAGS_ENABLE_SDS;
            }
            else
            {
                static uint8_t showHIDOFF = 1;
                if (showHIDOFF)
                {
                    showHIDOFF = 0;
                    WICED_BT_TRACE("\nHIDOFF");
                }
                /* allow ePDS */
                g_foundation_config_PMUflags |= PMU_CONFIG_FLAGS_ENABLE_SDS;
            }
  #endif
  #if is_SDS_capable
            hidd_link_aon_action_handler(HIDD_LINK_SAVE_TO_AON);
  #endif
        }
 #else // SLEEP_ALLOWED == 1
        ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
 #endif
        break;
    }
#endif // SLEEP_ALLOWED
    return ret;
}

////////////////////////////////////////////////////////////////////////////////////////////
/// Enter pairing
/////////////////////////////////////////////////////////////////////////////////////////////

#ifdef BLE_SUPPORT
/*
 * Handle host command to set device pairable.  This is typically a HID device button push.
 */
void ble_accept_pairing( BOOLEAN enable )
{
    WICED_BT_TRACE("\n%s LE pairing", enable ? "Start" : "Stop");

    if ( !enable )
    {
        hidd.pairing_type = 0;
        wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );
        hidd_blelink_enterDisconnected();
        return;
    }

    hidd.pairing_type = BT_TRANSPORT_LE;
    hidd_blelink_enterDiscoverable(WICED_TRUE);
 #if 0
    /* disconnect any connections if active */
    if (hidd_blelink_is_connected())
    {
        hidd_blelink_disconnect();
    }

    // start advertisements so that a host can connect
    wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
    hidd_blelink_set_state(HIDLINK_LE_DISCOVERABLE);
 #endif
}
#endif

#ifdef BR_EDR_SUPPORT
void bt_accept_pairing( BOOLEAN enable )
{
    WICED_BT_TRACE("\n%s BR/EDR pairing", enable ? "Enter" : "Exit");
    if (enable)
    {
        hidd.pairing_type = BT_TRANSPORT_BR_EDR;
        hidd_btlink_enter_discoverable();
    }
    else
    {
        hidd.pairing_type = 0;
        if (hidd_btlink_is_discoverable())
            hidd_btlink_enter_disconnected();
        else
            hidd_btlink_disable_page_and_inquiry_scans();
    }
}
#endif

/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_enter_pairing()
{
    if (hidd_link_is_connected())
    {
        hidd_link_disconnect();
    }

#ifdef BR_EDR_SUPPORT
    if (hidd_btlink_is_discoverable()) // if we are in BT discovery state
    {
        bt_accept_pairing(STOP_PAIRING); // stop BT pairing
 #ifdef BLE_SUPPORT
        ble_accept_pairing(START_PAIRING); // start LE pairing
 #endif
        return;
    }
#endif
#ifdef BLE_SUPPORT
    if (hidd_blelink_is_discoverable())
    {
        ble_accept_pairing(STOP_PAIRING);
        return;
    }
#endif
#ifdef BR_EDR_SUPPORT
    bt_accept_pairing(START_PAIRING); // start BT pairing
#elif defined(BLE_SUPPORT)
    ble_accept_pairing(START_PAIRING); // start LE pairing
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// Abstract link layer initialize
/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_sleep_configure(wiced_sleep_config_t * hidd_link_sleep_config)
{
    // Take over sleep handler
    hidd.p_app_sleep_handler = hidd_link_sleep_config->sleep_permit_handler;
    hidd_link_sleep_config->sleep_permit_handler = HIDD_sleep_handler;

    //configure sleep
    wiced_sleep_configure( hidd_link_sleep_config );
}

/*
 * hidd lib link default management callback
 */
wiced_bt_management_cback_t *app_management_cback_ptr = NULL;
wiced_result_t (*app_init_ptr)(void) = NULL;
wiced_result_t hidd_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_device_link_keys_t *pLinkKeys=NULL;
    uint8_t *p_keys;
    wiced_bt_device_address_t         bda = { 0 };

    WICED_BT_TRACE("\n=== BT stack cback event %d", event);

    if (app_management_cback_ptr)
    {
        result = app_management_cback_ptr(event, p_event_data);
        if (result != WICED_RESUME_HIDD_LIB_HANDLER) // if application has handled the event
        {
            return result;
        }
        // Not handled by app or app requests to resume.
        // Default result back to success and continue with default handler.
        result = WICED_BT_SUCCESS;
    }

    // hidd_lib default handler
    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            if ( p_event_data->enabled.status == WICED_BT_SUCCESS )
            {
                hci_control_enable_trace();
                wiced_bt_dev_read_local_addr(bda);
                WICED_BT_TRACE("\nAddress: [ %B]", bda);
                if (app_init_ptr)
                {
                    result = app_init_ptr();
                }
            }
            else
            {
                WICED_BT_TRACE("\nBT Enable status: 0x%02x", p_event_data->enabled.status);
            }
            hci_control_send_paired_host_info();
            break;

        case BTM_PAIRING_COMPLETE_EVT:
#ifdef BR_EDR_SUPPORT
            if ((p_event_data->pairing_complete.transport == BT_TRANSPORT_BR_EDR) &&
                ((hidd.pairing_type == BT_TRANSPORT_BR_EDR) || hidd_host_transport() == BT_TRANSPORT_BR_EDR))
            {
                result = p_event_data->pairing_complete.pairing_complete_info.br_edr.status;
                WICED_BT_TRACE("\nBR/EDR Pairing Complete: Status:%d Addr:%B", result, p_event_data->pairing_complete.bd_addr);

                //bonding successful
                if (!result)
                {
                    WICED_BT_TRACE("\nBONDED successful");
                    hidd_host_setTransport(p_event_data->pairing_complete.bd_addr, BT_TRANSPORT_BR_EDR);
                    hci_control_send_pairing_complete_evt( result, p_event_data->pairing_complete.bd_addr, BT_DEVICE_TYPE_BREDR );
                }
            }
#endif
#ifdef BLE_SUPPORT
            if((p_event_data->pairing_complete.transport == BT_TRANSPORT_LE) &&
                ((hidd.pairing_type == BT_TRANSPORT_LE) || hidd_host_transport() == BT_TRANSPORT_LE))
            {
                wiced_bt_dev_ble_pairing_info_t * p_info;
                p_info =  &p_event_data->pairing_complete.pairing_complete_info.ble;

                WICED_BT_TRACE("\nLE Pairing Complete:");
                //bonding successful
                if (!p_info->reason )
                {
                    WICED_BT_TRACE("\n BONDED successful");
                    hidd_host_setBonded(TRUE);
                    if (!wiced_blehidd_is_device_bonded())
                    {
                        WICED_BT_TRACE("\n set device bonded flag");
                        wiced_blehidd_set_device_bonded_flag(WICED_TRUE);
                    }

                    //SMP result callback: successful
                    hidd_host_setTransport(hidd_blelink.gatts_peer_addr, BT_TRANSPORT_LE);
                    hci_control_send_pairing_complete_evt( p_info->reason, p_event_data->pairing_complete.bd_addr, BT_DEVICE_TYPE_BLE );
                }
                else
                {
                    //SMP result callback: failed
                    WICED_BT_TRACE("\n BONDED failed reason:%d", p_info->reason);
                    hidd_host_remove();
                }
            }
#endif
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            pLinkKeys = &p_event_data->paired_device_link_keys_update;
            WICED_BT_TRACE("\nBTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT   BdAddr:%B", pLinkKeys->bd_addr );

            // make sure if address is valid
            if (memcmp(pLinkKeys->bd_addr, bda, BD_ADDR_LEN))
            {
                hidd_host_setLinkKey(pLinkKeys->bd_addr, pLinkKeys);
#ifdef BLE_SUPPORT
                if (hidd_host_transport() == BT_TRANSPORT_LE)
                {
                    WICED_BT_TRACE("\nmask           :  x%02X", pLinkKeys->key_data.le_keys_available_mask);
                    WICED_BT_TRACE("\nBLE AddrType   :  %d", pLinkKeys->key_data.ble_addr_type);
                    WICED_BT_TRACE("\nStatic AddrType:  %d", pLinkKeys->key_data.static_addr_type);
                    WICED_BT_TRACE("\nStatic Addr    :  %B", pLinkKeys->key_data.static_addr);
                    STRACE_ARRAY  ("\n  irk: ", &(pLinkKeys->key_data.le_keys.irk), LINK_KEY_LEN);
 #if SMP_INCLUDED == TRUE && SMP_LE_SC_INCLUDED == TRUE
                    STRACE_ARRAY  ("\n pltk: ", &(pLinkKeys->key_data.le_keys.pltk), LINK_KEY_LEN);
                    STRACE_ARRAY  ("\npcsrk: ", &(pLinkKeys->key_data.le_keys.pcsrk), LINK_KEY_LEN);
                    STRACE_ARRAY  ("\n lltk: ", &(pLinkKeys->key_data.le_keys.lltk), LINK_KEY_LEN);
                    STRACE_ARRAY  ("\nlcsrk: ", &(pLinkKeys->key_data.le_keys.lcsrk), LINK_KEY_LEN);
 #else
                    STRACE_ARRAY  ("\n ltk: ", &(pLinkKeys->key_data.le_keys.ltk), LINK_KEY_LEN);
                    STRACE_ARRAY  ("\ncsrk: ", &(pLinkKeys->key_data.le_keys.csrk), LINK_KEY_LEN);
 #endif
                }
#endif
#ifdef BR_EDR_SUPPORT
                if (hidd_host_transport() == BT_TRANSPORT_BR_EDR)
                {
                    STRACE_ARRAY("\nBR/EDR Link key:", pLinkKeys->key_data.br_edr_key, LINK_KEY_LEN);
                }
#endif
            }
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            WICED_BT_TRACE("\nBTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT");
            if (!hidd_host_getLinkKey(p_event_data->paired_device_link_keys_request.bd_addr, &p_event_data->paired_device_link_keys_request))
            {
                WICED_BT_TRACE("\n link_key not available");
                result = WICED_BT_ERROR;
            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            WICED_BT_TRACE("\nBTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT");
            /* save keys to NVRAM */
            p_keys = (uint8_t *)&p_event_data->local_identity_keys_update;
            wiced_hal_write_nvram ( VS_LOCAL_IDENTITY_ID, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
#if 0
            WICED_BT_TRACE("\n local keys save to NVRAM result: %d", result);
            TRACE_ARRAY(p_event_data->local_identity_keys_update.local_key_data, BTM_SECURITY_LOCAL_KEY_DATA_LEN);
#endif
            break;

        case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            WICED_BT_TRACE("\nBTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT");
            /* read keys from NVRAM */
            p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
            wiced_hal_read_nvram( VS_LOCAL_IDENTITY_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
#if 0
            WICED_BT_TRACE("\n local keys read from NVRAM result: %d",  result);
            if (!result)
            {
                STRACE_ARRAY("\n", p_keys, BTM_SECURITY_LOCAL_KEY_DATA_LEN);
            }
#endif
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            WICED_BT_TRACE("\nBTM_ENCRYPTION_STATUS_EVT, result=%d", p_event_data->encryption_status.result);
            //ble
#ifdef BLE_SUPPORT
            if (hidd_host_transport() == BT_TRANSPORT_LE)
            {
                if (p_event_data->encryption_status.result == WICED_SUCCESS)
                {
                    WICED_BT_TRACE("\n link encrypted");
                    wiced_blehidd_set_link_encrypted_flag(WICED_TRUE);
                }
                else
                {
                    WICED_BT_TRACE("\n Encryption failed:%d", p_event_data->encryption_status.result);
                }
            }
#endif
#ifdef BR_EDR_SUPPORT
            if (hidd_host_transport() == BT_TRANSPORT_BR_EDR)
            {
                if (p_event_data->encryption_status.result == WICED_SUCCESS)
                {
                    bt_hidd_link.encrypt_status.encrypted = WICED_TRUE;
                }
                else
                {
                    bt_hidd_link.encrypt_status.encrypted = WICED_FALSE;

                }
                memcpy(bt_hidd_link.encrypt_status.bdAddr, p_event_data->encryption_status.bd_addr, BD_ADDR_LEN);
            }
#endif
            break;

#ifdef SUPPORT_CODE_ENTRY
        case BTM_PIN_REQUEST_EVT:
            WICED_BT_TRACE("\nBTM_PIN_REQUEST_EVT");
            hidd_link_pinCodeRequest((wiced_bt_dev_name_and_class_t *)p_event_data);
            break;

        case BTM_PASSKEY_REQUEST_EVT:
            WICED_BT_TRACE("\nBTM_PASSKEY_REQUEST_EVT");
 #ifdef USE_KEYBOARD_IO_CAPABILITIES
            hidd_link_passKeyRequest(p_event_data);
 #else
            wiced_bt_dev_pass_key_req_reply(WICED_BT_SUCCESS,p_event_data->user_passkey_request.bd_addr, 0);
 #endif
            break;
#endif

#ifdef BR_EDR_SUPPORT
        case BTM_POWER_MANAGEMENT_STATUS_EVT:
            WICED_BT_TRACE("\nBTM_POWER_MANAGEMENT_STATUS_EVT");
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            WICED_BT_TRACE("\nBTM_USER_CONFIRMATION_REQUEST_EVT");
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            WICED_BT_TRACE("\nBTM_PAIRING_IO_CAPABILITIES_REQUEST_EVT bda %B",
                        p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
 #ifdef USE_KEYBOARD_IO_CAPABILITIES
            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_KEYBOARD_ONLY;
 #else
            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
 #endif
            p_event_data->pairing_io_capabilities_br_edr_request.oob_data = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
            p_event_data->pairing_io_capabilities_br_edr_request.is_orig = FALSE;
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT:
            WICED_BT_TRACE("\nBTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT");
            WICED_BT_TRACE("\n peer_bd_addr: %B, peer_io_cap: %d, peer_oob_data: %d, peer_auth_req: %d",
                                p_event_data->pairing_io_capabilities_br_edr_response.bd_addr,
                                p_event_data->pairing_io_capabilities_br_edr_response.io_cap,
                                p_event_data->pairing_io_capabilities_br_edr_response.oob_data,
                                p_event_data->pairing_io_capabilities_br_edr_response.auth_req);
            break;

        case BTM_SECURITY_FAILED_EVT:
            WICED_BT_TRACE("\nBTM_SECURITY_FAILED_EVT. hci_status:%d", p_event_data->security_failed.hci_status);
            bt_hidd_link.security_failed = p_event_data->security_failed.hci_status;
            hidd_host_setLinkKey(hidd_host_addr(), NULL);
            break;
#endif
#ifdef BLE_SUPPORT
        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            WICED_BT_TRACE("\nBTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT");
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_SC_ONLY|BTM_LE_AUTH_REQ_BOND;              /* LE sec bonding */
            p_event_data->pairing_io_capabilities_ble_request.max_key_size = 16;
            p_event_data->pairing_io_capabilities_ble_request.init_keys = 0x0F; //(BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_PLK);
            p_event_data->pairing_io_capabilities_ble_request.resp_keys = 0x0F; //(BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_PLK);
            break;

        case BTM_SECURITY_REQUEST_EVT:
            WICED_BT_TRACE("\nBTM_SECURITY_REQUEST_EVT");
            if (hidd_host_transport() == BT_TRANSPORT_LE)
            {
                WICED_BT_TRACE("\nClear CCCD's");
                hidd_host_set_flags(p_event_data->security_request.bd_addr, 0, 0xFFFF);
            }
             /* Use the default security */
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr,  WICED_BT_SUCCESS);
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            {
                wiced_bt_ble_advert_mode_t curr_adv_mode = hidd_blelink_get_adv_mode();
                wiced_bt_ble_advert_mode_t new_adv_mode = p_event_data->ble_advert_state_changed;
                hidd_blelink_set_adv_mode(new_adv_mode);
                WICED_BT_TRACE("\nAdvertisement State Change: %d -> %d", curr_adv_mode, new_adv_mode);
                hci_control_send_advertisement_state_evt( new_adv_mode );
                if (!new_adv_mode)
                {
//                    WICED_BT_TRACE("\nsubstate: %d", hidd_blelink.subState);
                    if (hidd_blelink.subState==HIDLINK_LE_DISCOVERABLE)
                    {
//                        WICED_BT_TRACE("\ndisconnecting..");
                        hidd_blelink_enterDisconnected();
                    }
                }

                //if high duty cycle directed advertising stops
                if ( (curr_adv_mode == BTM_BLE_ADVERT_DIRECTED_HIGH) &&
                         (new_adv_mode == BTM_BLE_ADVERT_DIRECTED_LOW))
                {
                    hidd_blelink_directed_adv_stop();
                }
//#if !defined(ENDLESS_LE_ADVERTISING_WHILE_DISCONNECTED) || !is_208xxFamily
#if 0
                // btstack will switch to low adv mode automatically when high adv mode timeout,
                // for HIDD, we want to stop adv instead
                else if ((curr_adv_mode == BTM_BLE_ADVERT_UNDIRECTED_HIGH) &&
                             (new_adv_mode == BTM_BLE_ADVERT_UNDIRECTED_LOW))
                {
                    wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
                }
#endif
                // if we are reconnecting and adv stops, we enter disconnected state
                if (hidd_blelink.subState == HIDLINK_LE_RECONNECTING && !new_adv_mode)
                {
                    hidd_blelink_set_state(HIDLINK_LE_DISCONNECTED);
#ifdef AUTO_RECONNECT
                    hidd_link_delayed_reconnect(AUTO_RECONNECT_DELAY);
#endif
                }
            }
            break;

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
            WICED_BT_TRACE("\nScan State Change: %d", p_event_data->ble_scan_state_changed );
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            WICED_BT_TRACE("\nBTM_BLE_CONNECTION_PARAM_UPDATE status:%d", p_event_data->ble_connection_param_update.status);
            if (!p_event_data->ble_connection_param_update.status)
            {
                hidd_blelink_conn_update_complete();
            }
            break;
#endif
        default:
            WICED_BT_TRACE("\nUnhandled management_cback event: %d!!!", event );
            break;
    }
    return result;
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
const wiced_bt_cfg_settings_t * hidd_cfg()
{
    return hidd.bt_cfg_ptr;
}

/////////////////////////////////////////////////////////////////////////////////
/// wiced_ble_hidd_start
///
/// \param p_bt_management_cback  - application bt_management callback function
///        p_bt_cfg_settings      - bt configuration setting
///        wiced_bt_cfg_buf_pools - buffer pool configuration
/////////////////////////////////////////////////////////////////////////////////
void hidd_start(wiced_result_t (*p_bt_app_init)(),
                wiced_bt_management_cback_t   * p_bt_management_cback,
                const wiced_bt_cfg_settings_t * p_bt_cfg_settings,
                const wiced_bt_cfg_buf_pool_t * p_bt_cfg_buf_pools)
{
#if is_SDS_capable
    if (!wiced_hal_mia_is_reset_reason_por())
    {
        hidd_link_aon_action_handler(HIDD_LINK_RESTORE_FROM_AON);
    }
#endif

#if is_208xxFamily
    // For 208xx, the chip id is identified by Radio id register; however, the register may get disabled after entering ePDS;
    // therefore, we read it once at power up and save the id.
    hidd_chip_id();
#endif

    sfi_allow_deep_sleep();

    app_management_cback_ptr = p_bt_management_cback;
    app_init_ptr = p_bt_app_init;
    if (!p_bt_cfg_settings || !p_bt_cfg_buf_pools)
    {
        WICED_BT_TRACE("\nbt or buff_pool configration is undefined!!"); while(1);
    }
    else
    {
        hidd.bt_cfg_ptr = p_bt_cfg_settings;
        wiced_bt_stack_init (hidd_management_cback, p_bt_cfg_settings, p_bt_cfg_buf_pools);
        WICED_BT_TRACE("\n\n<<%s start>>",p_bt_cfg_settings->device_name);
    }

    //timer to allow shut down sleep (SDS)
    wiced_init_timer( &hidd.allowDeepSleepTimer, hidd_allowsleeptimerCb, 0, WICED_MILLI_SECONDS_TIMER );

    hci_control_init();
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void hidd_allowed_hidoff(wiced_bool_t en)
{
    hidd.allowHIDOFF = en ? 1 : 0;
}

////////////////////////////////////////////////////////////////////////////////
// hidd_activity_detected
////////////////////////////////////////////////////////////////////////////////
void hidd_activity_detected()
{
#ifdef BR_EDR_SUPPORT
    extern void hidd_btlink_activity_detected();
    hidd_btlink_activity_detected();
#endif
}

#ifdef WICED_BT_TRACE_ENABLE
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void trace_array(void * ptr, uint32_t len)
{
    uint8_t * cPtr = (uint8_t *) ptr;
    int cnt=0;
    while (len--)
    {
        WICED_BT_TRACE("%02X ",*cPtr++);
        if (len && !(++cnt & 0xf))
            WICED_BT_TRACE("\n");
    }
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void strace_array(char * str, void * ptr, uint32_t len)
{
    WICED_BT_TRACE("%s",str);
    trace_array(ptr, len);
}
#endif
