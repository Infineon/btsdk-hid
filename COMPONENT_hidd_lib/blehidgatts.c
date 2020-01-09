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

/** @file
 *
 * GATT callback function and handlers
 *
 */
#ifdef BLE_SUPPORT
#include "wiced_bt_trace.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_nvram.h"
#include "wiced_result.h"
#include "blehidlink.h"
#include "blehidgatts.h"
#include "hidd_lib.h"

#ifdef OTA_FIRMWARE_UPGRADE
#include "wiced_bt_ota_firmware_upgrade.h"

#ifdef OTA_SECURE_FIRMWARE_UPGRADE
#include "bt_types.h"
#include "p_256_multprecision.h"
#include "p_256_ecc_pp.h"

// If secure version of the OTA firmware upgrade is used, the app should be linked with the ecdsa256_pub.c
// which exports the public key
extern Point    ecdsa256_public_key;
#endif
static uint8_t  ota_fw_upgrade_initialized = WICED_FALSE;

extern void     bleremoteapp_ota_fw_upgrade_status(uint8_t status);
#endif

static attribute_t * gattAttributes = NULL;
static uint16_t gattAttributes_size = 0;
static blehid_gatts_req_read_callback_t blehid_gatts_req_read_callback = NULL;
static blehid_gatts_req_write_callback_t blehid_gatts_req_write_callback = NULL;
#ifdef CONNECTED_ADVERTISING_SUPPORTED
wiced_bt_gatt_2nd_link_up_handler_t wiced_bt_gatt_2nd_link_up_handler = NULL;
#endif

/*
 *
 */
const attribute_t * blehid_gatts_get_attribute(uint16_t handle)
{
    attribute_t * puAttributes = gattAttributes;
    uint16_t limit = gattAttributes_size;

    while(limit--)
    {
        if(puAttributes->handle == handle)
        {
            return puAttributes;
        }

        puAttributes++;
    }

    WICED_BT_TRACE("\nRequested attribute 0x%04x not found!!!", handle);
    return NULL;
}

/*
 * Process Read request or command from peer device
 */
wiced_bt_gatt_status_t blehid_gatts_req_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data )
{
    const attribute_t * puAttribute;
    wiced_bt_gatt_status_t result;
    uint16_t attr_len_to_copy;
    uint8_t *attr_val_ptr = NULL;

    if(!p_read_data)
    {
        return WICED_BT_GATT_ERROR;
    }

#ifdef OTA_FIRMWARE_UPGRADE
    // if read request is for the OTA FW upgrade service, pass it to the library to process
    if (p_read_data->handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
    {
        return wiced_ota_fw_upgrade_read_handler(conn_id, p_read_data);
    }
#endif

    // check of application wants to handle it
    if (blehid_gatts_req_read_callback)
    {
        result = blehid_gatts_req_read_callback(conn_id, p_read_data);

        if (result != WICED_BT_GATT_NOT_FOUND)
        {
            // already handled by application
            return result;
        }
    }

    // default to invalid handle
    result = WICED_BT_GATT_INVALID_HANDLE;

    puAttribute = blehid_gatts_get_attribute(p_read_data->handle);
    if(puAttribute)
    {
        uint16_t mtu;

        //check if this is read request is for a long attribute value, if so take care of the offset as well
        if(p_read_data->is_long)
        {
            attr_val_ptr = (uint8_t *) puAttribute->p_attr + p_read_data->offset;
            attr_len_to_copy = puAttribute->attr_len - p_read_data->offset;
        }
        else
        {
            attr_val_ptr = (uint8_t *) puAttribute->p_attr;
            attr_len_to_copy = puAttribute->attr_len;
        }

        if(attr_len_to_copy<*p_read_data->p_val_len)
        {
            // report back our length
            *p_read_data->p_val_len = attr_len_to_copy;
        }

        mtu = wiced_blehidd_get_att_mtu_size(ble_hidd_link.gatts_peer_addr);

        //make sure copying buff is large enough so it won't corrupt memory
        if(attr_len_to_copy >= mtu)
        {
            WICED_BT_TRACE("\nsize(%d) > mtu(%d)", attr_len_to_copy, mtu);
            attr_len_to_copy = mtu - 1;
        }

        // copy over the value to the supplied buffer(entirely if it fits, data worth of MTU size)
        // if we have only sent partial value of an attribute we expect the peer to issue a read blob request to get the
        // rest of the attribute value.
        memcpy( p_read_data->p_val, attr_val_ptr, attr_len_to_copy );
//        WICED_BT_TRACE("\nSending %d bytes from offset %d for attrib handle 0x%04x", attr_len_to_copy, p_read_data->offset, p_read_data->handle);

        result = WICED_BT_GATT_SUCCESS;
    }
    return result;
}

#ifdef OTA_FIRMWARE_UPGRADE
blehid_ota_fw_upgrade_status_callback_t blehid_ota_fw_upgrade_status_callback = NULL;
/*
 *
 */
void blehid_register_ota_fw_upgrade_status_callback(blehid_ota_fw_upgrade_status_callback_t cb)
{
    blehid_ota_fw_upgrade_status_callback = cb;
}

/*
 * Process write request or command from peer device
 */
void blehid_ota_fw_upgrade_status(uint8_t status)
{
    if (blehid_ota_fw_upgrade_status_callback)
    {
        blehid_ota_fw_upgrade_status_callback(status);
    }
}
#endif

/*
 * Process write request or command from peer device
 */
wiced_bt_gatt_status_t blehid_gatts_req_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data )
{
    attribute_t *puAttribute;

    wiced_bt_gatt_status_t result;

    // NOTE: the gatt connection id is not the connection handler in the controller.
    if( conn_id != ble_hidd_link.gatts_conn_id)
    {
        return WICED_BT_GATT_ERROR;
    }

#ifdef OTA_FIRMWARE_UPGRADE
    // if write request is for the OTA FW upgrade service, pass it to the library to process
    if (p_data->handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
    {
        if (!ota_fw_upgrade_initialized)
        {
            /* OTA Firmware upgrade Initialization */
 #ifdef OTA_SECURE_FIRMWARE_UPGRADE
            if (wiced_ota_fw_upgrade_init(&ecdsa256_public_key, blehid_ota_fw_upgrade_status, NULL) == WICED_FALSE)
 #else
            if (wiced_ota_fw_upgrade_init(NULL, blehid_ota_fw_upgrade_status, NULL) == WICED_FALSE)
 #endif
            {
                WICED_BT_TRACE("\nOTA upgrade Init failure!!!");
                return WICED_BT_GATT_ERR_UNLIKELY;
            }
            ota_fw_upgrade_initialized = WICED_TRUE;
        }
        result = wiced_ota_fw_upgrade_write_handler(conn_id, p_data);
    }
    else
#endif
    {
        // check of application wants to handle it
        if (blehid_gatts_req_write_callback)
        {
            result = blehid_gatts_req_write_callback(conn_id, p_data);

            if (result != WICED_BT_GATT_NOT_FOUND)
            {
                // already handled by application
                return result;
            }
        }

        // default to success
        result = WICED_BT_GATT_SUCCESS;

        //WICED_BT_TRACE("\nwrite_handler: conn %d hdl %x prep %d off %d len %d", conn_id, p_data->handle, p_data->is_prep, p_data->offset,p_data->val_len );

        puAttribute = (attribute_t *)blehid_gatts_get_attribute(p_data->handle);

        if(puAttribute)
        {
            if(p_data->offset > puAttribute->attr_len)
            {
                result = WICED_BT_GATT_INVALID_OFFSET;
            }
            else if((p_data->val_len + p_data->offset) > puAttribute->attr_len)
            {
                result = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            else
            {
                result = wiced_blehidd_write_handler(p_data);
            }
#if 0
            if (result)
            {
                result = wiced_bt_gatt_legattdb_dispatchWriteCb(p_data);
            }
#endif
        }
        else
        {
            //result = wiced_bt_gatt_legattdb_dispatchWriteCb(p_data);
            result = WICED_BT_GATT_INVALID_HANDLE;
        }
    }

    // Whenever there is an activity, restart the idle timer
    if (ble_hidd_link.conn_idle_timeout)
    {
        osapi_activateTimer( &ble_hidd_link.conn_idle_timer, ble_hidd_link.conn_idle_timeout * 1000000UL); //timout in micro seconds.
        ble_hidd_link.osapi_app_timer_start_instant = clock_SystemTimeMicroseconds64();
        ble_hidd_link.osapi_app_timer_running |= BLEHIDLINK_CONNECTION_IDLE_TIMER;
        ble_hidd_link.osapi_app_timer_running |= 1;
    }
    return result;
}

/*
 * This function is invoked when connection is established
 */
wiced_bt_gatt_status_t blehid_gatts_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    WICED_BT_TRACE("\nLink up, id: %d, peer_addr_type: %d, peer_addr: %B, second_conn_state: %d",  p_status->conn_id, p_status->addr_type, p_status->bd_addr, ble_hidd_link.second_conn_state);

    //if 2nd connection is not allowed, disconnect right away
    if (wiced_ble_hidd_link_is_connected() && !ble_hidd_link.second_conn_state)
    {
        //disconnect the new connection
        wiced_bt_gatt_disconnect(p_status->conn_id);
        return WICED_BT_GATT_SUCCESS;
    }

    //configure ATT MTU size with peer device
    wiced_bt_gatt_configure_mtu(p_status->conn_id, wiced_hidd_cfg()->gatt_cfg.max_mtu_size);

#ifdef CONNECTED_ADVERTISING_SUPPORTED
    //get new connection while connected with existing host
    if (wiced_ble_hidd_link_is_connected())
    {
        // save the existing connection gatt id
        ble_hidd_link.existing_connection_gatts_conn_id = ble_hidd_link.gatts_conn_id;
        ble_hidd_link.second_conn_state = BLEHIDLINK_2ND_CONNECTION_PENDING;

        if (wiced_bt_gatt_2nd_link_up_handler)
        {
            wiced_bt_gatt_2nd_link_up_handler(p_status);
        }
    }
#endif

    ble_hidd_link.gatts_conn_id = p_status->conn_id;
    ble_hidd_link.gatts_peer_addr_type = p_status->addr_type;
    memcpy(ble_hidd_link.gatts_peer_addr, p_status->bd_addr, BD_ADDR_LEN);

    wiced_ble_hidd_link_connected();

#ifdef OTA_FIRMWARE_UPGRADE
    // Pass connection up/down event to the OTA FW upgrade library
    wiced_ota_fw_upgrade_connection_status_event(p_status);
#endif

    hci_control_send_connect_evt( p_status->addr_type, p_status->bd_addr, p_status->conn_id, p_status->link_role );

    return WICED_BT_GATT_SUCCESS;
}

/*
 * This function is invoked when connection is lost
 * */
wiced_bt_gatt_status_t blehid_gatts_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    WICED_BT_TRACE("\nLink down, id: %d reason: %d",  p_status->conn_id, p_status->reason );
    //connection is disconnected, and 2nd connection not allowed
    if (ble_hidd_link.gatts_conn_id == p_status->conn_id && !ble_hidd_link.second_conn_state)
    {
        ble_hidd_link.gatts_conn_id = 0;
        wiced_ble_hidd_link_disconnected();
    }
#ifdef CONNECTED_ADVERTISING_SUPPORTED
    // connection is disconnected during connected-advertising
    else if (ble_hidd_link.gatts_conn_id == p_status->conn_id && (BLEHIDLINK_2ND_CONNECTION_ALLOWED == ble_hidd_link.second_conn_state))
    {
        ble_hidd_link.gatts_conn_id = 0;
        blehidlink_setState(BLEHIDLINK_DISCOVERABLE);

        //stop the connection idle timer
        osapi_deactivateTimer(&ble_hidd_link.conn_idle_timer);

        ble_hidd_link.osapi_app_timer_running &= ~BLEHIDLINK_CONNECTION_IDLE_TIMER;
        if ((ble_hidd_link.osapi_app_timer_running >> 1) == 0)
        {
            ble_hidd_link.osapi_app_timer_running = 0; // no more application osapi timer is running
        }

        ble_hidd_link.gatts_conn_id = 0;

#ifdef ALLOW_SDS_IN_DISCOVERABLE
        blehidlink_setState(BLEHIDLINK_ADVERTISING_IN_uBCS_UNDIRECTED);
#endif
    }
#endif
    hci_control_send_disconnect_evt( p_status->reason, p_status->conn_id );
    return WICED_BT_GATT_SUCCESS;
}

/*
 *
 */
wiced_bt_gatt_status_t blehid_gatts_conn_status_cb( wiced_bt_gatt_connection_status_t *p_status )
{
    if(p_status->connected)
    {
        return blehid_gatts_connection_up( p_status );
    }
    else
    {
        return blehid_gatts_connection_down( p_status );
    }
}

/*
 * Process indication confirm. If client wanted us to use indication instead of
 * notifications we have to wait for confirmation after every message sent.
 * For example if user pushed button twice very fast
 * we will send first message, then
 * wait for confirmation, then
 * send second message, then
 * wait for confirmation and
 * if configured start idle timer only after that.
 */
wiced_bt_gatt_status_t blehid_gatts_req_conf_handler( uint16_t conn_id, uint16_t handle )
{
    WICED_BT_TRACE("\nblehid_gatts_req_conf_handler, conn %d hdl %d", conn_id, handle );

#ifdef OTA_FIRMWARE_UPGRADE
    // if indication confirmation is for the OTA FW upgrade service, pass it to the library to process
    if (handle > HANDLE_OTA_FW_UPGRADE_SERVICE)
    {
        return wiced_ota_fw_upgrade_indication_cfm_handler(conn_id, handle);
    }
#endif

    return WICED_BT_GATT_SUCCESS;
}

/*
 *
 */
wiced_bt_gatt_status_t blehid_gatts_req_cb( wiced_bt_gatt_attribute_request_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

//    WICED_BT_TRACE("\nblehid_gatts_req_cb conn %d, type %d", p_data->conn_id, p_data->request_type );
    switch ( p_data->request_type )
    {
        case GATTS_REQ_TYPE_READ:
            result = blehid_gatts_req_read_handler(p_data->conn_id, &(p_data->data.read_req));
            break;

        case GATTS_REQ_TYPE_WRITE:
        case GATTS_REQ_TYPE_PREP_WRITE:
            result = blehid_gatts_req_write_handler(p_data->conn_id, &(p_data->data.write_req));
            break;

        case GATTS_REQ_TYPE_MTU:
            WICED_BT_TRACE("\nGATTS_REQ_TYPE_MTU to %d bytes", p_data->data.mtu);
            break;

        case GATTS_REQ_TYPE_CONF:
            result = blehid_gatts_req_conf_handler( p_data->conn_id, p_data->data.handle );
            break;

        default:
            WICED_BT_TRACE("\nPlease check this blehid_gatts_req_cb!!!");
            break;
    }

    wiced_hidd_deep_sleep_not_allowed(1000);// No deep sleep for 1 second.

    return result;
}

/*
 *
 */
wiced_bt_gatt_status_t blehid_gatts_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

//    WICED_BT_TRACE("\nblehid_gatts_callback event: %d", event);

    switch(event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            result = blehid_gatts_conn_status_cb(&p_data->connection_status);
            break;

        case GATT_OPERATION_CPLT_EVT:
        case GATT_DISCOVERY_CPLT_EVT:
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            result = blehid_gatts_req_cb(&p_data->attribute_request);
            break;

        case GATT_CONGESTION_EVT:
            WICED_BT_TRACE("\ncongested:%d", p_data->congestion.congested);
            break;

        default:
            WICED_BT_TRACE("\ngatts_callback: unhandled event!!!:0x%x", event);
            break;
    }

    return result;
}

/*
 *
 */
wiced_bt_gatt_status_t wiced_hidd_gatts_init(const uint8_t * gatt_db, uint16_t len, const attribute_t * gAttrib, uint16_t gAttrib_len, blehid_gatts_req_read_callback_t rd_cb,  blehid_gatts_req_write_callback_t wr_cb)
{
    wiced_bt_gatt_status_t gatt_status;

    blehid_gatts_req_read_callback = rd_cb;
    blehid_gatts_req_write_callback = wr_cb;
    gattAttributes = (attribute_t *) gAttrib;
    gattAttributes_size = gAttrib_len;

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register( blehid_gatts_callback );
    if (gatt_status != WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("\nFAILED: wiced_bt_gatt_register status %d", gatt_status );
    }
    else
    {
        /* GATT DB Initialization */
        gatt_status = wiced_bt_gatt_db_init( gatt_db, len );
        if (gatt_status != WICED_BT_SUCCESS)
        {
            WICED_BT_TRACE("\nFAILD: wiced_bt_gatt_db_init %d", gatt_status );
        }
    }
    return gatt_status;
}

#endif //#ifdef BLE_SUPPORT
