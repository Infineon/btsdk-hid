/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
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

/** @file
 *
 * GATT callback function and handlers
 *
 */
#ifdef BLE_SUPPORT

#include "hidd_lib.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_gpio.h"
#include "wiced_result.h"

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
#endif

gatt_t gatt = {0};

/*
 *
 */
const attribute_t * hidd_gatt_get_attribute(uint16_t handle)
{
    attribute_t * p_attribute = gatt.gattAttributes;
    uint16_t limit = gatt.gattAttributes_size;

    while(limit--)
    {
        if(p_attribute->handle == handle)
        {
            return p_attribute;
        }

        p_attribute++;
    }
    WICED_BT_TRACE("\nRequested attribute 0x%04x not found!!!", handle);
    return NULL;
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
wiced_bt_gatt_status_t hidd_gatt_write_handler( uint16_t conn_id, hidd_gatt_write_t * p_data )
{
    attribute_t *p_attribute;
    wiced_bt_gatt_status_t result;

    // NOTE: the gatt connection id is not the connection handler in the controller.
    if( conn_id != blelink.gatts_conn_id)
    {
        return WICED_BT_GATT_ERROR;
    }

#ifdef OTA_FIRMWARE_UPGRADE
    // if write request is for the OTA FW upgrade service, pass it to the library to process
    if (wiced_ota_fw_upgrade_is_gatt_handle(p_data->handle))
    {
//        WICED_BT_TRACE("\nOTA hidd_gatt_write_handler %04x", p_data->handle );
        if (!ota_fw_upgrade_initialized)
        {
            WICED_BT_TRACE("\nOTA upgrade Init");
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
        if (gatt.hidd_gatt_req_write_callback)
        {
            result = gatt.hidd_gatt_req_write_callback(conn_id, p_data);

            if (result != WICED_BT_GATT_NOT_FOUND)
            {
                // already handled by application
                return result;
            }
        }

        // default to success
        result = WICED_BT_GATT_SUCCESS;

        // WICED_BT_TRACE("\nwrite_handler: conn %d hdl %x prep %d off %d len %d", conn_id, p_data->handle, p_data->is_prep, p_data->offset,p_data->val_len );

        p_attribute = (attribute_t *)hidd_gatt_get_attribute(p_data->handle);

        if(p_attribute)
        {
            if(p_data->offset > p_attribute->attr_len)
            {
                result = WICED_BT_GATT_INVALID_OFFSET;
            }
            else if((p_data->val_len + p_data->offset) > p_attribute->attr_len)
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
            if ((result == WICED_BT_GATT_SUCCESS) && !wiced_blehidd_is_device_bonded())
            {
                result = WICED_BT_GATT_INSUF_AUTHENTICATION;
            }
        }
        else
        {
            //result = wiced_bt_gatt_legattdb_dispatchWriteCb(p_data);
            result = WICED_BT_GATT_INVALID_HANDLE;
        }
    }

    // Whenever there is an activity, restart the idle timer
    hidd_blelink_restart_idle_timer();

    return result;
}

/*
 * This function is invoked when connection is established
 */
wiced_bt_gatt_status_t hidd_gatt_connection_up( wiced_bt_gatt_connection_status_t *p_status )
{
    WICED_BT_TRACE("\nLink up, id: %d, peer_addr_type: %d, peer_addr: %B, second_conn_state: %d",  p_status->conn_id, p_status->addr_type, p_status->bd_addr, blelink.second_conn_state);

    //if 2nd connection is not allowed, disconnect right away
    if (hidd_blelink_is_connected() && !blelink.second_conn_state)
    {
        //disconnect the new connection
        wiced_bt_gatt_disconnect(p_status->conn_id);
        return WICED_BT_GATT_SUCCESS;
    }

    //configure ATT MTU size with peer device
    wiced_bt_gatt_configure_mtu(p_status->conn_id, hidd_cfg_mtu());

    blelink.gatts_conn_id = p_status->conn_id;
    blelink.gatts_peer_addr_type = p_status->addr_type;
    memcpy(blelink.gatts_peer_addr, p_status->bd_addr, BD_ADDR_LEN);

    hidd_blelink_connected();

#ifdef OTA_FIRMWARE_UPGRADE
    // Pass connection up event to the OTA FW upgrade library
    wiced_ota_fw_upgrade_connection_status_event(p_status);
#endif

    hidd_hci_control_send_connect_evt( p_status->addr_type, p_status->bd_addr, p_status->conn_id, p_status->link_role );

    return WICED_BT_GATT_SUCCESS;
}

/*
 * This function is invoked when connection is lost
 * */
wiced_bt_gatt_status_t hidd_gatt_connection_down( wiced_bt_gatt_connection_status_t *p_status )
{
    WICED_BT_TRACE("\nLink down, id: %d reason: %d",  p_status->conn_id, p_status->reason );
    //connection is disconnected, and 2nd connection not allowed
    if (blelink.gatts_conn_id == p_status->conn_id && !blelink.second_conn_state)
    {
        blelink.gatts_conn_id = 0;
        hidd_blelink_disconnected();
    }

#ifdef OTA_FIRMWARE_UPGRADE
    // Pass connection down event to the OTA FW upgrade library
    wiced_ota_fw_upgrade_connection_status_event(p_status);
#endif

    hidd_hci_control_send_disconnect_evt( p_status->reason, p_status->conn_id );

    return WICED_BT_GATT_SUCCESS;
}

/*
 *
 */
wiced_bt_gatt_status_t hidd_gatt_conn_status_cb( wiced_bt_gatt_connection_status_t *p_status )
{
    if(p_status->connected)
    {
        return hidd_gatt_connection_up( p_status );
    }
    else
    {
        return hidd_gatt_connection_down( p_status );
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
wiced_bt_gatt_status_t hidd_gatt_req_conf_handler( uint16_t conn_id, uint16_t handle )
{
    WICED_BT_TRACE("\nhidd_gatt_req_conf_handler, conn %d hdl %d", conn_id, handle );

#ifdef OTA_FIRMWARE_UPGRADE
    // if indication confirmation is for the OTA FW upgrade service, pass it to the library to process
    if (wiced_ota_fw_upgrade_is_gatt_handle(handle))
    {
//        WICED_BT_TRACE("\nOTA req_conf_handler %04x", handle );
        return wiced_ota_fw_upgrade_indication_cfm_handler(conn_id, handle);
    }
#endif

    return WICED_BT_GATT_SUCCESS;
}

/*
 *
 */
wiced_bt_gatt_status_t hidd_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

//    WICED_BT_TRACE("\nhidd_gatt_callback event: %d", event);

    switch(event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            result = hidd_gatt_conn_status_cb(&p_data->connection_status);
            break;

        case GATT_OPERATION_CPLT_EVT:
        case GATT_DISCOVERY_CPLT_EVT:
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            result = hidd_gatt_req_cb(&p_data->attribute_request);
            break;

        case GATT_CONGESTION_EVT:
//            WICED_BT_TRACE("\ncongested:%d", p_data->congestion.congested);
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
wiced_bt_gatt_status_t hidd_gatt_init(wiced_blehidd_report_gatt_characteristic_t* rptTable, uint16_t rptTableNum,
                                       uint8_t * gatt_db, uint16_t gatt_db_len,
                                       attribute_t * gAttrib, uint16_t gAttrib_len,
                                       hidd_gatt_req_read_callback_t rd_cb,  hidd_gatt_req_write_callback_t wr_cb)
{
    gatt.rptTable = rptTable;
    gatt.rptTableNum = rptTableNum;
    gatt.hidd_gatt_req_read_callback = rd_cb;
    gatt.hidd_gatt_req_write_callback = wr_cb;
    gatt.gattAttributes = (attribute_t *) gAttrib;
    gatt.gattAttributes_size = gAttrib_len;

    /* GATT DB Initialization */
    if (hidd_gatt_db_init( gatt_db, gatt_db_len ) == WICED_BT_GATT_SUCCESS)
    {
        /* Register with stack to receive GATT callback */
        if (wiced_bt_gatt_register( hidd_gatt_callback ) == WICED_BT_GATT_SUCCESS)
        {
            wiced_blehidd_register_report_table(rptTable, rptTableNum);
            return WICED_BT_GATT_SUCCESS;
        }
    }
    return WICED_BT_GATT_ERROR;
}

#ifdef FASTPAIR_ENABLE
wiced_bool_t hidd_gatt_gfps_init(wiced_bt_gfps_provider_conf_t * fastpair_conf)
{
    fastpair_conf->p_gatt_cb = hidd_gatt_callback;
    return wiced_bt_gfps_provider_init(fastpair_conf);
}
#endif

/*
 *
 */
wiced_bool_t hidd_gatt_set_data(uint8_t * ptr, uint16_t len)
{
    uint8_t i,j;
    uint8_t rpt_id = *ptr++;
    wiced_blehidd_report_gatt_characteristic_t* map = gatt.rptTable;

    // based on report ID, find handle
    for(i = 0; i < gatt.rptTableNum; i++)
    {
        if((map->reportType == WICED_HID_REPORT_TYPE_INPUT) && (map->reportId == rpt_id))
        {
            // we found the entry
            attribute_t * attrib = gatt.gattAttributes;

            // now we find the handle from attrib_t to get the size and pointer
            for (j=0; j<gatt.gattAttributes_size; j++)
            {
                if (attrib->handle == map->handle)
                {
                    // if not pointing to the same place, we copy the data
                    if (attrib->p_attr != ptr)
                    {
                        // The length should be enough, but to prevent memory corruption, we check to make sure anyway.
                        if (attrib->attr_len < --len)
                            len = attrib->attr_len;
                        memcpy(attrib->p_attr, ptr, len);
                    }
                    return TRUE;
                }
                attrib++;
            }
        }
        map++;
    }
    return FALSE;
}

#endif //#ifdef BLE_SUPPORT
