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
* File Name: hidd_lib.c
*
* Abstract: This file implements HID application transport that supports both the Bluetooth(BT) Classic
*               and LE
* Functions:
*
*******************************************************************************/
#include "hidd_lib.h"

////////////////////////////////////////////////////////////////////////////////
// defines
////////////////////////////////////////////////////////////////////////////////
#define STOP_PAIRING  0
#define START_PAIRING 1

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

////////////////////////////////////////////////////////////////////////////////
// returns chip number
////////////////////////////////////////////////////////////////////////////////
uint32_t hidd_chip_id()
{
#if is_20819Family
    #define RADIO_ID    0x006007c0
    #define RADIO_20820 0x80
    static uint32_t chip = 0;

    // the radio id register become not accessible after ePDS; thus, read it only once at power up. Return the saved value thereafter.
    if (!chip)
    {
        chip = (*(uint32_t*) RADIO_ID & RADIO_20820) ? 20820 : 20819;
    }
    return chip;
#else
    return CHIP;
#endif
}

////////////////////////////////////////////////////////////////////////////////////////////
/// Enter pairing
/////////////////////////////////////////////////////////////////////////////////////////////

#ifdef BLE_SUPPORT
/*
 * Handle host command to set device pairablelink.  This is typically a HID device button push.
 */
void ble_accept_pairing( wiced_bool_t enable )
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
void bt_accept_pairing( wiced_bool_t enable )
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
#ifdef BR_EDR_SUPPORT
    bt_accept_pairing(START_PAIRING); // start BT pairing
#elif defined(BLE_SUPPORT)
    ble_accept_pairing(START_PAIRING); // start LE pairing
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////////
void hidd_pairing()
{
    if (hidd_link_is_connected())
    {
        hidd_link_disconnect();
    }

#ifdef BR_EDR_SUPPORT
    if (hidd_btlink_is_discoverable()) // if we are in BT discovery state
    {
        bt_accept_pairing(STOP_PAIRING); // stop BT pairing
 #if defined(BLE_SUPPORT) && !defined(PTS)
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
#if defined(BR_EDR_SUPPORT) && is_20819Family
    /* For 20819, CoD can be cleared, we reinforce to correct value by re-writing the CoD */
    WICED_BT_TRACE("\nCoD %02x%02x%02x",hidd_cfg()->device_class[0],hidd_cfg()->device_class[1],hidd_cfg()->device_class[2]);
    wiced_bt_set_device_class(hidd_cfg()->device_class);
#endif
    hidd_enter_pairing();
}

/*
 * hidd lib link default management callback
 */
wiced_result_t hidd_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_device_link_keys_t *p_link_keys=NULL;
    uint8_t *p_keys;
    wiced_bt_device_address_t         bda = { 0 };

    WICED_BT_TRACE("\n=== BT stack cback event %d", event);

    if (hidd.app_management_cback_ptr)
    {
        result = hidd.app_management_cback_ptr(event, p_event_data);
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
                hidd_hci_control_enable_trace();
                wiced_bt_dev_read_local_addr(bda);
                WICED_BT_TRACE("\nAddress: [ %B]", bda);
                if (hidd.app_init_ptr)
                {
                    result = hidd.app_init_ptr();
                }
            }
            else
            {
                WICED_BT_TRACE("\nBT Enable status: 0x%02x", p_event_data->enabled.status);
            }
            hidd_hci_control_send_paired_host_info();
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
                    hidd_btlink_connectInd(p_event_data->pairing_complete.bd_addr);
                    hidd_hci_control_send_pairing_complete_evt( result, p_event_data->pairing_complete.bd_addr, BT_DEVICE_TYPE_BREDR );
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
                    hidd_host_setTransport(blelink.gatts_peer_addr, BT_TRANSPORT_LE);
                    hidd_hci_control_send_pairing_complete_evt( p_info->reason, p_event_data->pairing_complete.bd_addr, BT_DEVICE_TYPE_BLE );
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
            p_link_keys = &p_event_data->paired_device_link_keys_update;
            WICED_BT_TRACE("\nBTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT   BdAddr:%B", p_link_keys->bd_addr );

            // make sure if address is valid
            if (memcmp(p_link_keys->bd_addr, bda, BD_ADDR_LEN))
            {
                hidd_host_setLinkKey(p_link_keys->bd_addr, p_link_keys);
#ifdef BLE_SUPPORT
                if (hidd_host_transport() == BT_TRANSPORT_LE)
                {
                    hidd_blelink_pr_link_key(p_link_keys);
                }
#endif
#ifdef BR_EDR_SUPPORT
                if (hidd_host_transport() == BT_TRANSPORT_BR_EDR)
                {
                    STRACE_ARRAY("\nBR/EDR Link key:", p_link_keys->key_data.br_edr_key, LINK_KEY_LEN);
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
            hidd_write_nvram ( VS_ID_LOCAL_IDENTITY, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
#if 0
            WICED_BT_TRACE("\n local keys save to NVRAM result: %d", result);
            TRACE_ARRAY(p_event_data->local_identity_keys_update.local_key_data, BTM_SECURITY_LOCAL_KEY_DATA_LEN);
#endif
            break;

        case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            WICED_BT_TRACE("\nBTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT");
            /* read keys from NVRAM */
            p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
            hidd_read_nvram( VS_ID_LOCAL_IDENTITY, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
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
 #ifdef FASTPAIR_ENABLE
            wiced_bt_gfps_provider_seeker_passkey_set(p_event_data->user_confirmation_request.numeric_value);
 #endif
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            WICED_BT_TRACE("\nBTM_PAIRING_IO_CAPABILITIES_REQUEST_EVT bda %B",
                        p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
 #ifdef FASTPAIR_ENABLE
            if (wiced_bt_gfps_provider_pairing_state_get())
            {   // Google Fast Pair service Seeker triggers this pairing process.
                /* Set local capability to Display/YesNo to identify local device is not a
                 * man-in-middle device.
                 * Otherwise, the Google Fast Pair Service Seeker will terminate this pairing
                 * process. */
                p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
            }
            else
 #endif
            {
 #ifdef USE_KEYBOARD_IO_CAPABILITIES
                p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_KEYBOARD_ONLY;
 #else
                p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
 #endif
            }
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

 #ifdef FASTPAIR_ENABLE
            if (wiced_bt_gfps_provider_pairing_state_get())
            {   // Google Fast Pair service Seeker triggers this pairing process.
                /* If the device capability is set to NoInput/NoOutput, end pairing, to avoid using
                 * Just Works pairing method. todo*/
                if (p_event_data->pairing_io_capabilities_br_edr_response.io_cap == BTM_IO_CAPABILITIES_NONE)
                {
                    WICED_BT_TRACE("Terminate the pairing process\n");
                }
            }
 #endif
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
            p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_SC | BTM_LE_AUTH_REQ_BOND;              /* LE sec bonding */
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
                hidd_hci_control_send_advertisement_state_evt( new_adv_mode );
                if (new_adv_mode != BTM_BLE_ADVERT_OFF)
                {
#ifdef LE_LOCAL_PRIVACY_SUPPORT
                    WICED_BT_TRACE(" RPA: %B", wiced_btm_get_private_bda());
#else
                    wiced_bt_device_address_t  bda;
                    wiced_bt_dev_read_local_addr(bda);
                    WICED_BT_TRACE(" BDA: %B", bda);
#endif
//                    WICED_BT_TRACE("\nsubstate: %d", blelink.subState);
                    if (blelink.subState==HIDLINK_LE_DISCOVERABLE)
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
//#if !defined(ENDLESS_LE_ADVERTISING) || !is_20819Family
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
                if (blelink.subState == HIDLINK_LE_RECONNECTING && !new_adv_mode)
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
wiced_bt_cfg_settings_t * hidd_cfg()
{
    return hidd.bt_cfg_ptr;
}

/////////////////////////////////////////////////////////////////////////////////
/// hidd_start_v
///
/// \param p_bt_app_init          - pointer to application init function
///        p_bt_management_cback  - poniter to application bt_management callback function
///        p_bt_cfg_settings      - bt configuration setting
/////////////////////////////////////////////////////////////////////////////////
void hidd_start_v(app_start_callback_t * p_bt_app_init,
                wiced_bt_management_cback_t   * p_bt_management_cback,
                wiced_bt_cfg_settings_t * p_bt_cfg_settings)
{
    if (p_bt_cfg_settings == NULL)
    {
        WICED_BT_TRACE("\nInvalid BT configuration");
        return;
    }

    hidd.app_management_cback_ptr = p_bt_management_cback;
    hidd.app_init_ptr = p_bt_app_init;
    hidd.bt_cfg_ptr = p_bt_cfg_settings;

    hidd_stack_init(hidd_management_cback);
    WICED_BT_TRACE("\n\n<< %s >>",p_bt_cfg_settings->device_name);

    hidd_sleep_init();
    hidd_hci_control_init();
    hidd_nvram_deep_sleep();
}

#ifdef FASTPAIR_ENABLE
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
wiced_result_t hidd_gfps_discoverablility_set(wiced_bt_ble_advert_mode_t advert_mode)
{
    wiced_bt_gfps_provider_discoverablility_set(advert_mode!=BTM_BLE_ADVERT_OFF);
    return WICED_SUCCESS;
}
#endif

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
