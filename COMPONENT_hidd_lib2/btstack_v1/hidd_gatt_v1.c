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

/** @file
 *
 * GATT callback function and handlers
 *
 */
#if BTSTACK_VER < 0x03000001
#ifdef BLE_SUPPORT

#include "hidd_lib.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_gpio.h"
#include "wiced_result.h"
#include "wiced_bt_ota_firmware_upgrade.h"

/*
 * Process Read request or command from peer device
 */
static wiced_bt_gatt_status_t hidd_gatt_req_read_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data )
{
    const attribute_t * p_attribute;
    wiced_bt_gatt_status_t result;
    uint16_t attr_len_to_copy;
    uint8_t *attr_val_ptr = NULL;

    if(!p_read_data)
    {
        return WICED_BT_GATT_ERROR;
    }

#ifdef OTA_FIRMWARE_UPGRADE
    // if read request is for the OTA FW upgrade service, pass it to the library to process
    if (wiced_ota_fw_upgrade_is_gatt_handle(p_read_data->handle))
    {
//        WICED_BT_TRACE("\nOTA req_read_handler %04x", p_read_data->handle );
        return wiced_ota_fw_upgrade_read_handler(conn_id, p_read_data);
    }
#endif

    // check of application wants to handle it
    if (gatt.hidd_gatt_req_read_callback)
    {
        result = gatt.hidd_gatt_req_read_callback(conn_id, p_read_data);

        if (result != WICED_BT_GATT_NOT_FOUND)
        {
            // already handled by application
            return result;
        }
    }

    // default to invalid handle
    result = WICED_BT_GATT_INVALID_HANDLE;

//    WICED_BT_TRACE("\nread_hndlr conn %d hdl 0x%x", conn_id, p_read_data->handle );

    p_attribute = hidd_gatt_get_attribute(p_read_data->handle);
    if(p_attribute)
    {
        uint16_t mtu;

        //check if this is read request is for a long attribute value, if so take care of the offset as well
        if(p_read_data->is_long)
        {
            attr_val_ptr = (uint8_t *) p_attribute->p_attr + p_read_data->offset;
            attr_len_to_copy = p_attribute->attr_len - p_read_data->offset;
        }
        else
        {
            attr_val_ptr = (uint8_t *) p_attribute->p_attr;
            attr_len_to_copy = p_attribute->attr_len;
        }

//        WICED_BT_TRACE("\nattr_len_to_copy: %d offset: %d", attr_len_to_copy, p_read_data->offset);

        if(attr_len_to_copy<*p_read_data->p_val_len)
        {
            // report back our length
            *p_read_data->p_val_len = attr_len_to_copy;
        }

        mtu = wiced_blehidd_get_att_mtu_size(blelink.gatts_peer_addr);

        //make sure copying buff is large enough so it won't corrupt memory
        if(attr_len_to_copy >= mtu)
        {
//            WICED_BT_TRACE("\nsize(%d) > mtu(%d)", attr_len_to_copy, mtu);
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

/*
 *
 */
wiced_bt_gatt_status_t hidd_gatt_req_cb( wiced_bt_gatt_attribute_request_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

//    WICED_BT_TRACE("\nhidd_gatt_req_cb conn %d, type %d", p_data->conn_id, p_data->request_type );
    switch ( p_data->request_type )
    {
        case GATTS_REQ_TYPE_READ:
            result = hidd_gatt_req_read_handler(p_data->conn_id, &(p_data->data.read_req));
            break;

        case GATTS_REQ_TYPE_WRITE:
        case GATTS_REQ_TYPE_PREP_WRITE:
            result = hidd_gatt_write_handler(p_data->conn_id, &(p_data->data.write_req));
            break;

        case GATTS_REQ_TYPE_MTU:
            WICED_BT_TRACE("\nGATTS_REQ_TYPE_MTU to %d bytes", p_data->data.mtu);
            break;

        case GATTS_REQ_TYPE_CONF:
            result = hidd_gatt_req_conf_handler( p_data->conn_id, p_data->data.handle );
            break;

        default:
//            WICED_BT_TRACE("\nPlease check %d hidd_gatt_req type!!!", p_data->request_type);
            break;
    }

    hidd_deep_sleep_not_allowed(1000);// No deep sleep for 1 second.

    return result;
}


#endif //#ifdef BLE_SUPPORT

#endif // #if BTSTACK_VER < 0x03000001
