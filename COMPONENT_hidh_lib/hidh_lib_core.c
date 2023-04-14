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
 * file   hidh_lib_core.c
 *
 * hidh_lib core functions
 */

#include "hidh_lib_core.h"

/******************************************************
 *  Global Variables
 ******************************************************/
hidh_le_cb_t hidh_le_cb;

/*
 * Allocate a structure to save information about a LE HID device
 * This function is called for LE HID Host Initialization.
 * This function must be called, once, before any other LE HIDH functions.
 */
hidh_le_dev_t *hidh_le_core_dev_alloc(wiced_bt_device_address_t bdaddr)
{
    hidh_le_dev_t *p_dev;
    int i;

    p_dev = &hidh_le_cb.devices[0];
    for (i = 0 ; i < HIDH_DEV_MAX ; i++, p_dev++)
    {
        if ((p_dev->dev_state_msk & HIDH_DEV_STATE_ALLOCATED) == 0)
        {
            LE_HIDH_TRACE("i:%d", i);
            memset(p_dev, 0, sizeof(*p_dev));
            memcpy(p_dev->bdaddr, bdaddr, BD_ADDR_LEN);
            p_dev->dev_state_msk |= HIDH_DEV_STATE_ALLOCATED;
            return p_dev;
        }
    }
    LE_HIDH_TRACE_ERR("mem full", i);
    return NULL;
}

/*
 * Free a structure to save information about a LE HID device
 */
void hidh_le_core_dev_free(hidh_le_dev_t *p_dev)
{
    LE_HIDH_TRACE("i:%d", p_dev - &hidh_le_cb.devices[0]);

    memset(p_dev, 0, sizeof(*p_dev));
}

/*
 * Retrieve a structure containing information about a LE HID device from it's BdAddr
 */
hidh_le_dev_t *hidh_le_core_dev_from_bdaddr(wiced_bt_device_address_t bdaddr)
{
    hidh_le_dev_t *p_dev;
    int i;

    p_dev = &hidh_le_cb.devices[0];
    for (i = 0 ; i < HIDH_DEV_MAX ; i++, p_dev++)
    {
        if ((p_dev->dev_state_msk & HIDH_DEV_STATE_ALLOCATED) &&
            (memcmp(p_dev->bdaddr, bdaddr, BD_ADDR_LEN) == 0))
        {
            LE_HIDH_TRACE("bdaddr:%B i:%d", bdaddr, i);
            return p_dev;
        }
    }
    LE_HIDH_TRACE("unknown dev:%B", bdaddr);
    return NULL;
}

/*
 * Retrieve a structure containing information about a LE HID device from it's Connection Handle
 */
hidh_le_dev_t *hidh_le_core_dev_from_conn_id(uint16_t conn_id)
{
    hidh_le_dev_t *p_dev;
    int i;

    p_dev = &hidh_le_cb.devices[0];
    for (i = 0; i < HIDH_DEV_MAX; i++, p_dev++)
    {
        if ((p_dev->dev_state_msk & HIDH_DEV_STATE_ALLOCATED)
                && (p_dev->conn_id == conn_id))
        {
            LE_HIDH_TRACE("conn_id:%04x i:%d", conn_id, i);
            return p_dev;
        }
    }
    LE_HIDH_TRACE_ERR("unknown conn_id:%04x", conn_id);
    return NULL;
}

/*
 * This function is called when the LE HID GATT operation is complete
 */
void hidh_le_core_gatt_complete(hidh_le_dev_t *p_dev,
        hidh_le_status_t status)
{
    hidh_le_event_data_t hidh_evt;

    LE_HIDH_TRACE("conn_id:%d status:%d",
            p_dev->conn_id, status);

    if (status == HIDH_STATUS_SUCCESS)
    {
        LE_HIDH_TRACE("connected dev:%B", p_dev->bdaddr);
        /* Send a LE HID Open Event */
        memcpy(&hidh_evt.connected.bdaddr, p_dev->bdaddr, BD_ADDR_LEN);
        hidh_evt.connected.handle = p_dev->conn_id + HIDH_HANDLE_OFFSET;
        hidh_evt.connected.status = HIDH_STATUS_SUCCESS;
        hidh_le_core_callback(HIDH_OPEN_EVT, &hidh_evt);

        /* Send a LE HID GATT Cache Event */
        memset(&hidh_evt.gatt_cache, 0, sizeof(hidh_le_gatt_cache_t));
        memcpy(&hidh_evt.gatt_cache.bdaddr, p_dev->bdaddr, BD_ADDR_LEN);
        hidh_evt.gatt_cache.characteristics_nb = p_dev->database.characteristics_nb;
        memcpy(&hidh_evt.gatt_cache.characteristics, p_dev->database.characteristics,
                p_dev->database.characteristics_nb * sizeof(hidh_le_gatt_char_t));
        hidh_evt.gatt_cache.report_descs_nb = p_dev->database.report_descs_nb;
        memcpy(&hidh_evt.gatt_cache.report_descs, p_dev->database.report_descs,
                p_dev->database.report_descs_nb * sizeof(hidh_le_gatt_report_t));
        hidh_le_core_callback(HIDH_GATT_CACHE_EVT, &hidh_evt);
    }
    else
    {
        LE_HIDH_TRACE("disconnect");
        status = wiced_bt_gatt_disconnect(p_dev->conn_id);
        if (status != HIDH_STATUS_SUCCESS)
            LE_HIDH_TRACE_ERR("wiced_bt_gatt_disconnect failed:%s", status);
    }
}

/*
 * This function will check (one by one) all the event filter registered.
 * If none 'catch' it, the event will be sent to the application
 */
void hidh_le_core_callback(hidh_le_event_t event,
        hidh_le_event_data_t *p_event_data)
{
    int filter_idx;
    wiced_bool_t event_filtered;

    LE_HIDH_TRACE("event:%d", event);

    /* Call every event filter installed */
    for (filter_idx = 0 ; filter_idx < HIDH_EVENT_FILTER_NB_MAX; filter_idx++)
    {
        if (hidh_le_cb.p_filter_callbacks[filter_idx] != NULL)
        {
            /* If this event filter is registered */
            event_filtered = hidh_le_cb.p_filter_callbacks[filter_idx](event, p_event_data);
            /* Return as soon as an event filter 'catches' it (return TRUE) */
            if (event_filtered)
            {
                return;
            }
        }
    }

    /* If no Event filter 'caught' the event, send it to the app */
    if (hidh_le_cb.p_callback != NULL)
    {
        hidh_le_cb.p_callback(event, p_event_data);
    }
}
