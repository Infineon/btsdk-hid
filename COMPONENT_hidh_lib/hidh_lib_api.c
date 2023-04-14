/*
 *  Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
 *  an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * 
 *  This software, including source code, documentation and related
 *  materials ("Software") is owned by Cypress Semiconductor Corporation
 *  or one of its affiliates ("Cypress") and is protected by and subject to
 *  worldwide patent protection (United States and foreign),
 *  United States copyright laws and international treaty provisions.
 *  Therefore, you may use this Software only as provided in the license
 *  agreement accompanying the software package from which you
 *  obtained this Software ("EULA").
 *  If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 *  non-transferable license to copy, modify, and compile the Software
 *  source code solely for use in connection with Cypress's
 *  integrated circuit products.  Any reproduction, modification, translation,
 *  compilation, or representation of this Software except as specified
 *  above is prohibited without the express written permission of Cypress.
 * 
 *  Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 *  EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 *  reserves the right to make changes to the Software without notice. Cypress
 *  does not assume any liability arising out of the application or use of the
 *  Software or any product or circuit described in the Software. Cypress does
 *  not authorize its products for use in any products where a malfunction or
 *  failure of the Cypress product may reasonably be expected to result in
 *  significant property damage, injury or death ("High Risk Product"). By
 *  including Cypress's product in a High Risk Product, the manufacturer
 *  of such system or application assumes all risk of such use and in doing
 *  so agrees to indemnify Cypress against all liability.
 */

/**
 * hidh_lib_api.c
 *
 * The Human Interface Device Host Role (HIDH) LE library of the SDK provide a simple method
 * for an application to integrate HIDH functionality.
 * This library is typically used to connect to LE HID Devices such as Bluetooth (classic) Mice,
 * Keyboards or Remote Control.
 */

#include "hidh_lib_int.h"
#include "hidh_lib_core.h"
#include "hidh_lib_gattc.h"
#include "hidh_lib_wakeup.h"
#include "cyhal_gpio.h"

/**
 * To initialize/start LE HID library
 */
hidh_le_status_t hidh_le_init(hidh_le_cback_t *p_callback)
{
    if (p_callback == NULL)
    {
        LE_HIDH_TRACE_ERR("no callback");
        return HIDH_STATUS_ERROR;
    }

    memset(&hidh_le_cb, 0, sizeof(hidh_le_cb));
    hidh_le_cb.p_callback = p_callback;

    return HIDH_STATUS_SUCCESS;
}

/**
 * This function is called to initiate a connection a device
 */
hidh_le_status_t hidh_le_connect(wiced_bt_device_address_t bdaddr,
        wiced_bt_ble_address_type_t addr_type)
{
    hidh_le_dev_t *p_dev;
    wiced_bool_t status;

    LE_HIDH_TRACE("dev:%B type:%d", bdaddr, addr_type);

    if (hidh_le_cb.p_callback == NULL)
    {
        LE_HIDH_TRACE_ERR("no callback", bdaddr);
        return HIDH_STATUS_ERROR;
    }

    p_dev = hidh_le_core_dev_from_bdaddr(bdaddr);
    if (p_dev == NULL)
    {
        p_dev = hidh_le_core_dev_alloc(bdaddr);
        if (p_dev == NULL)
        {
            LE_HIDH_TRACE_ERR("Mem full dev:%B Mem Full", bdaddr);
            return HIDH_STATUS_MEM_FULL;
        }
    }
    else
    {
        if (p_dev->conn_id != 0)
        {
            LE_HIDH_TRACE_ERR("dev:%B already connected", bdaddr);
            return HIDH_STATUS_ERROR;
        }
    }

    LE_HIDH_TRACE("connect LE dev:%B AddrType:%d", bdaddr, addr_type);

    status = wiced_bt_gatt_le_connect(bdaddr, addr_type, BLE_CONN_MODE_HIGH_DUTY, WICED_TRUE);
    if (status != WICED_TRUE)
    {
        LE_HIDH_TRACE_ERR("dev:%B wiced_bt_gatt_le_connect failed", bdaddr);
        return HIDH_STATUS_ERROR;
    }

    return HIDH_STATUS_SUCCESS;
}

/**
 * This function is called to disconnect a link
 */
hidh_le_status_t hidh_le_disconnect(uint16_t handle)
{
    hidh_le_dev_t *p_dev;
    wiced_bt_gatt_status_t status;

    LE_HIDH_TRACE("handle:%d", handle);

    if (hidh_le_cb.p_callback == NULL)
    {
        LE_HIDH_TRACE_ERR("no callback");
        return HIDH_STATUS_ERROR;
    }

    p_dev = hidh_le_core_dev_from_conn_id(handle - HIDH_HANDLE_OFFSET);
    if (p_dev == NULL)
    {
        LE_HIDH_TRACE_ERR("Unknown conn_id:%04x", handle);
        return HIDH_STATUS_INVALID_HANDLE;
    }

    if (p_dev->conn_id == 0)
    {
        LE_HIDH_TRACE_ERR("not connected for conn_id:%04x", handle);
        return HIDH_STATUS_NOT_CONNECTED;
    }

    status = wiced_bt_gatt_disconnect(handle - HIDH_HANDLE_OFFSET);
    if (status != WICED_BT_GATT_SUCCESS)
    {
        LE_HIDH_TRACE_ERR("wiced_bt_gatt_le_disconnect failed status:%d", status);
        return HIDH_STATUS_ERROR;
    }

    return HIDH_STATUS_SUCCESS;
}

/**
 * This function is called to add a LE HID Device (to allow it to reconnect)
 */
hidh_le_status_t hidh_le_add(wiced_bt_device_address_t bdaddr,
        wiced_bt_ble_address_type_t addr_type, hidh_le_gatt_cache_t *p_gatt_cache)
{
    hidh_le_dev_t *p_dev;
    wiced_bool_t status;

    LE_HIDH_TRACE("address:%B type:%d", bdaddr, addr_type);

    if (hidh_le_cb.p_callback == NULL)
    {
        LE_HIDH_TRACE_ERR("no callback");
        return HIDH_STATUS_ERROR;
    }

    /* check if this device is already allocated */
    p_dev = hidh_le_core_dev_from_bdaddr(bdaddr);
    if (p_dev == NULL)
    {
        p_dev = hidh_le_core_dev_alloc(bdaddr);
        if (p_dev == NULL)
        {
            LE_HIDH_TRACE_ERR("Mem full dev:%B", bdaddr);
            return HIDH_STATUS_MEM_FULL;
        }
    }

    /* Save the Address Type */
    p_dev->addr_type = addr_type;

    /* Restore the GATT Cache information (if provided) */
    if(p_gatt_cache)
    {
        if ((p_gatt_cache->characteristics_nb <= HIDH_DEV_CHAR_MAX) &&
            (p_gatt_cache->report_descs_nb <= HIDH_REPORT_DESC_MAX))
        {
            p_dev->database.characteristics_nb = p_gatt_cache->characteristics_nb;
            memcpy(p_dev->database.characteristics, p_gatt_cache->characteristics,
                    sizeof(p_dev->database.characteristics));
            p_dev->database.report_descs_nb = p_gatt_cache->report_descs_nb;
            memcpy(p_dev->database.report_descs, p_gatt_cache->report_descs,
                    sizeof(p_dev->database.report_descs));
            p_dev->database.db_state = HIDH_GATTC_STATE_DONE;

            hidh_le_gattc_dump(p_dev);
        }
        else
        {
            LE_HIDH_TRACE_ERR("Wrong GATT Cache nb_char:%d nb_report:%d",
                    p_gatt_cache->characteristics_nb, p_gatt_cache->report_descs_nb);
        }
    }

    if (p_dev->dev_state_msk & HIDH_DEV_STATE_ADDED)
    {
        /* Device already Added. nothing to do */
        LE_HIDH_TRACE("already added dev:%B", bdaddr);
        return HIDH_STATUS_SUCCESS;
    }

    status = wiced_bt_ble_set_background_connection_type(BTM_BLE_CONN_AUTO, NULL);
    if (status == WICED_FALSE)
    {
        LE_HIDH_TRACE_ERR("wiced_bt_ble_set_background_connection_type failed");
        return HIDH_STATUS_ERROR;
    }

    /* Allow Background connection for this device */
    status = wiced_bt_gatt_le_connect(bdaddr, addr_type, BLE_CONN_MODE_LOW_DUTY, WICED_FALSE);
    if (status == WICED_FALSE)
    {
        LE_HIDH_TRACE_ERR("wiced_bt_gatt_le_connect failed");
        return HIDH_STATUS_GATT_ERROR;
    }

    p_dev->dev_state_msk |= HIDH_DEV_STATE_ADDED;

    return HIDH_STATUS_SUCCESS;
}

/**
 * To remove a LE HID Device (to do not allow it to reconnect).
 */
hidh_le_status_t hidh_le_remove(wiced_bt_device_address_t bdaddr)
{
    hidh_le_dev_t *p_dev;
    wiced_bool_t status;

    LE_HIDH_TRACE("address:%B", bdaddr);

    if (hidh_le_cb.p_callback == NULL)
    {
        LE_HIDH_TRACE_ERR("no callback", bdaddr);
        return HIDH_STATUS_ERROR;
    }

    p_dev = hidh_le_core_dev_from_bdaddr(bdaddr);
    if (p_dev == NULL)
    {
        LE_HIDH_TRACE_ERR("dev:%B unknown", bdaddr);
        return HIDH_STATUS_INVALID_DEV;
    }

    if (p_dev->conn_id != 0)
    {
        LE_HIDH_TRACE_ERR("dev:%B connected", bdaddr);
        return HIDH_STATUS_INVALID_CONN_ID;
    }

    if ((p_dev->dev_state_msk & HIDH_DEV_STATE_ADDED) == 0)
    {
        /* Device was not Added */
        LE_HIDH_TRACE_ERR("was not added dev:%B", bdaddr);
        return HIDH_STATUS_INVALID_BDADDR;
    }

    /* Do not allow this device to Reconnect (using Background connection) */
    status = wiced_bt_gatt_cancel_connect(bdaddr, WICED_FALSE);
    if (status == WICED_FALSE)
    {
        LE_HIDH_TRACE_ERR("wiced_bt_gatt_cancel_connect failed");
        /* Continue (to free the device) even if it fails */
    }

    hidh_le_core_dev_free(p_dev);

    return HIDH_STATUS_SUCCESS;
}

/**
 * Set (send) a Report to a peer LE HID Device
 */
hidh_le_status_t hidh_le_set_report(uint16_t handle,
        hidh_le_report_type_t report_type, uint8_t report_id, uint8_t *p_data,
        uint16_t length)
{
    hidh_le_dev_t *p_dev;

    LE_HIDH_TRACE("handle:%d type:%d id:%02x len:%d%", handle, report_type, report_id,
            length);

    if (hidh_le_cb.p_callback == NULL)
    {
        LE_HIDH_TRACE_ERR("no callback");
        return HIDH_STATUS_ERROR;
    }

    p_dev = hidh_le_core_dev_from_conn_id(handle - HIDH_HANDLE_OFFSET);
    if (p_dev == NULL)
    {
        LE_HIDH_TRACE_ERR("Unknown conn_id:%B", handle);
        return HIDH_STATUS_INVALID_DEV;
    }

    if (p_dev->conn_id == 0)
    {
        LE_HIDH_TRACE_ERR("not connected conn_id:%d", handle);
        return HIDH_STATUS_INVALID_CONN_ID;
    }

    /* Set the Report */
    return hidh_le_gattc_set_report(p_dev, report_type, report_id, p_data, length);
}

/**
 * This function is called to Get (receive) an LE HID Report from a connected LE HID Device.
 * This function can be used, for example, to read the last HID Report received.
 */
hidh_le_status_t hidh_le_get_report(uint16_t handle,
        hidh_le_report_type_t report_type, uint8_t report_id, uint16_t length)
{
    hidh_le_dev_t *p_dev;

    LE_HIDH_TRACE("handle:%d type:%d id:%02x len:%d%", handle, report_type, report_id,
            length);

    if (hidh_le_cb.p_callback == NULL)
    {
        LE_HIDH_TRACE_ERR("no callback");
        return HIDH_STATUS_NOT_INITIALIZED;
    }

    p_dev = hidh_le_core_dev_from_conn_id(handle - HIDH_HANDLE_OFFSET);
    if (p_dev == NULL)
    {
        LE_HIDH_TRACE_ERR("Unknown conn_id:%B", handle);
        return HIDH_STATUS_INVALID_DEV;
    }

    if (p_dev->conn_id == 0)
    {
        LE_HIDH_TRACE_ERR("not connected conn_id:%d", handle);
        return HIDH_STATUS_NOT_CONNECTED;
    }

    /* Get the Report */
    return hidh_le_gattc_get_report(p_dev, report_type, report_id, length);
}

/**
 * Sends Set HID Protocol to a peer HID Device.
 * This function is called to change the HID Protocol of a connected HID Device.
 */
hidh_le_status_t hidh_le_set_protocol(uint16_t handle,
        hidh_le_protocol_t protocol)
{
    hidh_le_dev_t *p_dev;

    LE_HIDH_TRACE("handle:%d protocol:%d%", handle, protocol);

    if (hidh_le_cb.p_callback == NULL)
    {
        LE_HIDH_TRACE_ERR("no callback");
        return HIDH_STATUS_ERROR;
    }

    p_dev = hidh_le_core_dev_from_conn_id(handle - HIDH_HANDLE_OFFSET);
    if (p_dev == NULL)
    {
        LE_HIDH_TRACE_ERR("Unknown conn_id:%B", handle);
        return HIDH_STATUS_INVALID_DEV;
    }

    if (p_dev->conn_id == 0)
    {
        LE_HIDH_TRACE_ERR("not connected conn_id:%d", handle);
        return HIDH_STATUS_NOT_CONNECTED;
    }

    /* Set the Protocol */
    return hidh_le_gattc_set_protocol(p_dev, protocol);
}

/**
 * When a LE connection is established, application calls this function.
 */
void hidh_le_up(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    hidh_le_event_data_t hidh_evt;
    hidh_le_dev_t *p_dev;
    hidh_le_status_t status;
#ifndef DISABLE_ENCRYPTION
    wiced_result_t result;
    wiced_bt_ble_sec_action_type_t encryption_type = BTM_BLE_SEC_ENCRYPT;
#endif

    LE_HIDH_TRACE("address:%B conn_id:%04x", p_conn_status->bd_addr, p_conn_status->conn_id);

    if (hidh_le_cb.p_callback == NULL)
    {
        LE_HIDH_TRACE_ERR("no callback");
        return;
    }

    if (p_conn_status->transport != BT_TRANSPORT_LE)
    {
        LE_HIDH_TRACE_ERR("not LE dev:%B", p_conn_status->bd_addr);
        return;
    }

    p_dev = hidh_le_core_dev_from_bdaddr(p_conn_status->bd_addr);
    if (p_dev == NULL)
    {
        LE_HIDH_TRACE_ERR("dev:%B unknown", p_conn_status->bd_addr);
        return;
    }

    /* Connection success, Save Connection Id */
    p_dev->conn_id = p_conn_status->conn_id;
    p_dev->dev_state_msk |= HIDH_DEV_STATE_CONNECTED;

#ifndef DISABLE_ENCRYPTION
    /* If the link is not yet Encrypted */
    if ((p_dev->dev_state_msk & HIDH_DEV_STATE_ENCRYPTED) == 0)
    {
        if ((p_dev->dev_state_msk & HIDH_DEV_STATE_ADDED) == 0)
        {
            LE_HIDH_TRACE("wiced_bt_dev_sec_bond dev:%B", p_conn_status->bd_addr);
            result = wiced_bt_dev_sec_bond (p_dev->bdaddr, p_dev->addr_type, BT_TRANSPORT_LE, 0, NULL);
            if (result == WICED_BT_PENDING)
            {
                /* Pairing Started. The Stack will automatically start Encryption */
                LE_HIDH_TRACE("wiced_bt_dev_sec_bond pending");
                return;
            }
            else if (result == WICED_BT_SUCCESS)
            {
                /* Already Paired???. */
                LE_HIDH_TRACE("already Paired???");
            }
            else
            {
                /* Pairing failed to start. Disconnect link */
                LE_HIDH_TRACE_ERR("wiced_bt_dev_sec_bond failed");
                wiced_bt_gatt_disconnect(p_dev->conn_id);
                return;
            }
        }

        /* Already Paired. Let's Start Encryption */
        LE_HIDH_TRACE("wiced_bt_dev_set_encryption");
        result = wiced_bt_dev_set_encryption (p_dev->bdaddr, BT_TRANSPORT_LE, &encryption_type);
        if (result == WICED_BT_PENDING)
        {
            /* Encryption Started */
            LE_HIDH_TRACE("wiced_bt_dev_set_encryption pending");
            return;
        }
        else if (result == WICED_BT_SUCCESS)
        {
            /* Already Encrypted ???. Let's continue */
            p_dev->dev_state_msk |= HIDH_DEV_STATE_ENCRYPTED;
        }
        else
        {
            /* Encryption failed to start. Disconnect link */
            LE_HIDH_TRACE_ERR("wiced_bt_dev_set_encryption failed");
            wiced_bt_gatt_disconnect(p_dev->conn_id);
            return;
        }
    }
#endif
    /*
     * The link is already Encrypted
     */

    /* If the database is empty, discover it */
    if (p_dev->database.db_state == HIDH_GATTC_STATE_EMPTY)
    {
        status =  hidh_le_gattc_search(p_dev);
        if (status != HIDH_STATUS_SUCCESS)
        {
            LE_HIDH_TRACE_ERR("hidh_le_gatt_search failed");
            /* Disconnect the link */
            wiced_bt_gatt_disconnect(p_dev->conn_id);
        }
        return;
    }

    LE_HIDH_TRACE("connected dev:%B", p_conn_status->bd_addr);
    memcpy(&hidh_evt.connected.bdaddr, p_conn_status->bd_addr, BD_ADDR_LEN);
    hidh_evt.connected.handle = p_conn_status->conn_id + HIDH_HANDLE_OFFSET;
    hidh_evt.connected.status = HIDH_STATUS_SUCCESS;
    hidh_le_core_callback(HIDH_OPEN_EVT, &hidh_evt);
}

/**
 * This function is called when the LE connection is lost
 */
void hidh_le_down(wiced_bt_gatt_connection_status_t *p_conn_status)
{
    hidh_le_event_data_t hidh_evt;
    hidh_le_dev_t *p_dev;
    hidh_le_dev_state_t dev_state_msk;

    LE_HIDH_TRACE("BdAddr:%B reason:%d",
            p_conn_status->bd_addr, p_conn_status->reason);

    if (hidh_le_cb.p_callback == NULL)
    {
        LE_HIDH_TRACE_ERR("no callback");
        return;
    }

    if (p_conn_status->transport != BT_TRANSPORT_LE)
    {
        LE_HIDH_TRACE_ERR("not LE dev:%B", p_conn_status->bd_addr);
        return;
    }

    p_dev = hidh_le_core_dev_from_bdaddr(p_conn_status->bd_addr);
    if (p_dev == NULL)
    {
        LE_HIDH_TRACE_ERR("unknown BdAddr:%B", p_conn_status->bd_addr);
        return;
    }

    /* Save the device's state */
    dev_state_msk = p_dev->dev_state_msk;

    /* If this device has not been 'added', free it */
    if ((dev_state_msk & HIDH_DEV_STATE_ADDED) == 0)
    {
        hidh_le_core_dev_free(p_dev);
    }
    else
    {
        /* Reset any state information excepted the Allocated and Added */
        p_dev->dev_state_msk = HIDH_DEV_STATE_ALLOCATED |
                               HIDH_DEV_STATE_ADDED;
        p_dev->conn_id = 0;
    }

    /* If the device was connected, it's a disconnection */
    if (dev_state_msk & HIDH_DEV_STATE_CONNECTED)
    {
        hidh_evt.disconnected.handle = p_conn_status->conn_id + HIDH_HANDLE_OFFSET;
        hidh_evt.disconnected.reason = p_conn_status->reason;
        hidh_le_core_callback(HIDH_CLOSE_EVT, &hidh_evt);
    }
    /* Else, the device was not yet connected, it's an outgoing connection failure */
    else
    {
        LE_HIDH_TRACE_ERR("connection failed dev:%B", p_conn_status->bd_addr);
        memcpy(&hidh_evt.connected.bdaddr, p_conn_status->bd_addr, BD_ADDR_LEN);
        hidh_evt.connected.handle = 0;
        hidh_evt.connected.status = HIDH_STATUS_CONNECTION_FAILED;
        hidh_le_core_callback(HIDH_OPEN_EVT, &hidh_evt);
    }
}

/**
 * This function is called when encryption status changes
 */
void hidh_le_encryption_changed(wiced_bt_dev_encryption_status_t *p_encryption_changed)
{
    hidh_le_dev_t *p_dev;
    wiced_bt_gatt_status_t status;
    hidh_le_event_data_t hidh_evt;

    /* Ignore BR/EDR Encryption Changed Event */
    if (p_encryption_changed->transport != BT_TRANSPORT_LE)
    {
        return;
    }

    if (hidh_le_cb.p_callback == NULL)
    {
        LE_HIDH_TRACE_ERR("no callback");
        return;
    }

    p_dev = hidh_le_core_dev_from_bdaddr(p_encryption_changed->bd_addr);
    if (p_dev == NULL)
    {
        LE_HIDH_TRACE_ERR("BdAddr:%B",
                p_encryption_changed->bd_addr);
        return;
    }

    if (p_encryption_changed->result != WICED_BT_SUCCESS)
    {
        LE_HIDH_TRACE_ERR("Failed result:%d", p_encryption_changed->result);
        /* Disconnect the link */
        wiced_bt_gatt_disconnect(p_dev->conn_id);

        // remove it from HIDH database
        hidh_le_remove(p_encryption_changed->bd_addr);
        return;
    }

    p_dev->dev_state_msk |= HIDH_DEV_STATE_ENCRYPTED;

    LE_HIDH_TRACE("Success");

    /* If the database is empty, discover it */
    if (p_dev->database.db_state == HIDH_GATTC_STATE_EMPTY)
    {
        status =  hidh_le_gattc_search(p_dev);
        if (status != HIDH_STATUS_SUCCESS)
        {
            LE_HIDH_TRACE_ERR("hidh_le_gatt_search failed");
            /* Disconnect the link */
            wiced_bt_gatt_disconnect(p_dev->conn_id);
        }
        return;
    }

    LE_HIDH_TRACE("connected dev:%B", p_dev->bdaddr);
    memcpy(&hidh_evt.connected.bdaddr, p_dev->bdaddr, BD_ADDR_LEN);
    hidh_evt.connected.handle = p_dev->conn_id + HIDH_HANDLE_OFFSET;
    hidh_evt.connected.status = HIDH_STATUS_SUCCESS;
    hidh_le_core_callback(HIDH_OPEN_EVT, &hidh_evt);
}

/**
 * Application must call this function when GATT_DISCOVERY_RESULT_EVT event is received
 */
wiced_bt_gatt_status_t hidh_le_gatt_discovery_result(wiced_bt_gatt_discovery_result_t *p_discovery_result)
{
    hidh_le_dev_t *p_dev;

    if (hidh_le_cb.p_callback == NULL)
    {
        LE_HIDH_TRACE_ERR("no callback");
        return WICED_BT_GATT_WRONG_STATE;
    }

    p_dev = hidh_le_core_dev_from_conn_id(p_discovery_result->conn_id);
    if (p_dev == NULL)
    {
        LE_HIDH_TRACE_ERR("Unknown conn_id:%d",
                p_discovery_result->conn_id);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    return hidh_le_gattc_discovery_result(p_dev, p_discovery_result);
}

/**
 * Application must call this function when GATT_DISCOVERY_CPLT_EVT event is received
 */
wiced_bt_gatt_status_t hidh_le_gatt_discovery_complete(wiced_bt_gatt_discovery_complete_t *p_discovery_complete)
{
    hidh_le_dev_t *p_dev;

    if (hidh_le_cb.p_callback == NULL)
    {
        LE_HIDH_TRACE_ERR("no callback");
        return WICED_BT_GATT_WRONG_STATE;
    }

    p_dev = hidh_le_core_dev_from_conn_id(p_discovery_complete->conn_id);
    if (p_dev == NULL)
    {
        LE_HIDH_TRACE_ERR("Unknown conn_id:%d",
                p_discovery_complete->conn_id);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    return hidh_le_gattc_discovery_complete(p_dev, p_discovery_complete);
}

/**
 * Application must call this function when GATT_OPERATION_CPLT_EVT event is received
 */
wiced_bt_gatt_status_t hidh_le_gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_operation_complete)
{
    hidh_le_dev_t *p_dev;

    LE_HIDH_TRACE("conn_id:%04x, op:%d, status:%d", p_operation_complete->conn_id, p_operation_complete->op, p_operation_complete->status);
    if (hidh_le_cb.p_callback == NULL)
    {
        LE_HIDH_TRACE_ERR("no callback");
        return WICED_BT_GATT_WRONG_STATE;
    }

    p_dev = hidh_le_core_dev_from_conn_id(p_operation_complete->conn_id);
    if (p_dev == NULL)
    {
        LE_HIDH_TRACE_ERR("Unknown conn_id:%04x", p_operation_complete->conn_id);
        return WICED_BT_GATT_INVALID_HANDLE;
    }

    return hidh_le_gattc_operation_complete(p_dev, p_operation_complete);
}

/**
 * LE HIDH libraries (e.g. LE HIDH Audio) can use this function to register a 'Filter Callback'
 */
hidh_le_status_t hidh_le_filter_register(
        hidh_le_filter_cback_t *p_filter_callback)
{
    int filter_idx;

    LE_HIDH_TRACE("");

    if (hidh_le_cb.p_callback == NULL)
    {
        LE_HIDH_TRACE_ERR("no callback");
        return HIDH_STATUS_ERROR;
    }

    /* Search for a free Event Filter entry */
    for (filter_idx = 0 ; filter_idx < HIDH_EVENT_FILTER_NB_MAX; filter_idx++)
    {
        if (hidh_le_cb.p_filter_callbacks[filter_idx] == NULL)
        {
            /* Register the Filter callback */
            hidh_le_cb.p_filter_callbacks[filter_idx] = p_filter_callback;
            return HIDH_STATUS_SUCCESS;
        }
    }
    LE_HIDH_TRACE_ERR("Mem Full (max:%d)", HIDH_EVENT_FILTER_NB_MAX);
    return HIDH_STATUS_MEM_FULL;
}

/**
 * To set the WakeUp pattern.
 * This function must be called after the hidh_le_add function is called
 */
hidh_le_status_t hidh_le_wakeup_pattern_set(wiced_bt_device_address_t bdaddr,
        hidh_le_wakeup_pattern_cmd_t command, uint16_t report_id,
        uint8_t *p_pattern, uint16_t pattern_len)
{
    hidh_le_dev_t *p_dev;
    wiced_bt_gatt_status_t status;

    LE_HIDH_TRACE("dev:%B cmd:%d report_id:%04x", bdaddr, command, report_id);

    if (hidh_le_cb.p_callback == NULL)
    {
        LE_HIDH_TRACE_ERR("no callback", bdaddr);
        return HIDH_STATUS_ERROR;
    }

    p_dev = hidh_le_core_dev_from_bdaddr(bdaddr);
    if (p_dev == NULL)
    {
        LE_HIDH_TRACE_ERR("Unknown device:%B", bdaddr);
        return HIDH_STATUS_INVALID_DEV;
    }

    if ((p_dev->dev_state_msk & HIDH_DEV_STATE_ADDED) == 0)
    {
        LE_HIDH_TRACE_ERR("device:%B not added", bdaddr);
        return HIDH_STATUS_INVALID_BDADDR;
    }

    if (command == HIDH_WAKEUP_PATTERN_CMD_ADD)
    {
        status = hidh_le_wakeup_pattern_add(p_dev, report_id, p_pattern, pattern_len);
    }
    else if (command == HIDH_WAKEUP_PATTERN_CMD_DEL)
    {
        status =HIDH_STATUS_NOT_YET_IMPLEMENTED;
    }
    else if (command == HIDH_WAKEUP_PATTERN_CMD_LIST)
    {
        status =HIDH_STATUS_NOT_YET_IMPLEMENTED;
    }
    else
    {
        LE_HIDH_TRACE_ERR("Unknown command:%d", command);
        status = HIDH_STATUS_INVALID_PARAM;
    }

    return status;
}

/**
 * to enable/disable the WakeUp
 *
 * This function is typically called after hidh_le_wakeup_pattern_set
 * The p_data and data_len, could be used (later) to add additional GPIO Control (duration,
 * pattern, etc.).
 */
hidh_le_status_t hidh_le_wakeup_pattern_control(wiced_bool_t enable,
        cyhal_gpio_t gpio_num, uint8_t polarity, uint8_t *p_data, uint8_t data_len)
{
    LE_HIDH_TRACE("enable:%d gpio:%d polarity:%x", enable, gpio_num, polarity);

    if (hidh_le_cb.p_callback == NULL)
    {
        LE_HIDH_TRACE_ERR("no callback");
        return HIDH_STATUS_ERROR;
    }

    hidh_le_cb.wakeup_control.enable = enable;

    if (enable)
    {
        /* Save GPIO information */
        hidh_le_cb.wakeup_control.gpio_num = gpio_num;
        hidh_le_cb.wakeup_control.polarity = polarity;
    }

    /* Configure the GPIO as Output and desassert it */
    cyhal_gpio_init(gpio_num, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, polarity ? false : true);

    return HIDH_STATUS_SUCCESS;
}

/**
 * to retrieve the HID descriptor of a connected LE HID Device
 */
wiced_bt_gatt_status_t hidh_le_get_descriptor(uint16_t handle)
{
    hidh_le_dev_t *p_dev;

    LE_HIDH_TRACE("handle:%d", handle);

    if (hidh_le_cb.p_callback == NULL)
    {
        LE_HIDH_TRACE_ERR("no callback");
        return HIDH_STATUS_ERROR;
    }

    p_dev = hidh_le_core_dev_from_conn_id(handle - HIDH_HANDLE_OFFSET);
    if (p_dev == NULL)
    {
        LE_HIDH_TRACE_ERR("Unknown conn_id:%B", handle);
        return HIDH_STATUS_INVALID_DEV;
    }

    if (p_dev->conn_id == 0)
    {
        LE_HIDH_TRACE_ERR("not connected conn_id:%d", handle);
        return HIDH_STATUS_NOT_CONNECTED;
    }

    /* Read the Descriptor */
    return hidh_le_gattc_read_descriptor(p_dev);
}
