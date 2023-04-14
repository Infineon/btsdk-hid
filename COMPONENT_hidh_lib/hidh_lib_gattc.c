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
 * file hidh_lib_gattc.c
 *
 * hidh_lib gatt functions
 */

#include "hidh_lib_gattc.h"
#include "hidh_lib_wakeup.h"
#include "wiced_bt_uuid.h"
#include "gatt_utils_lib.h"
#include "wiced_memory.h"

/***********************************************************************
 * Private functions
 ***********************************************************************/

/**
 * Add a GATT attribute in the database
 */
static hidh_le_status_t hidh_le_gattc_add_char(hidh_le_dev_t *p_dev,
        wiced_bt_gatt_char_declaration_t *p_char)
{
    hidh_le_gatt_char_t *p_db_char;

    if (p_dev->database.characteristics_nb >= HIDH_DEV_CHAR_MAX)
    {
        LE_HIDH_TRACE_ERR("Mem Full !!!");
        LE_HIDH_TRACE_ERR("Increase HIDH_DEV_CHAR_MAX (%d)",
                HIDH_DEV_CHAR_MAX);
        return HIDH_STATUS_MEM_FULL;
    }

    if (p_char->char_uuid.len != LEN_UUID_16)
    {
        LE_HIDH_TRACE_ERR("UUID Length:%d unsupported. UUID Ignored");
        return HIDH_STATUS_SUCCESS;

    }

    p_db_char = &p_dev->database.characteristics[p_dev->database.characteristics_nb];
    p_db_char->handle = p_char->handle;
    p_db_char->val_handle = p_char->val_handle;
    p_db_char->properties = p_char->characteristic_properties;
    p_db_char->uuid16 = p_char->char_uuid.uu.uuid16;

    p_dev->database.characteristics_nb++;

    return HIDH_STATUS_SUCCESS;
}

/**
 * Discover next GATT Attribute
 */
static wiced_bt_gatt_status_t hidh_le_gattc_discover_next_char_desc( hidh_le_dev_t *p_dev )
{
    wiced_bt_gatt_status_t status;
    uint16_t start_handle = 0;
    uint16_t end_handle;
    int char_idx;
    hidh_le_gatt_char_t *p_db_char;

    LE_HIDH_TRACE("idx:%d", p_dev->database.search_char_desc_index);

    if (p_dev->database.search_char_desc_index >= p_dev->database.characteristics_nb)
    {
        LE_HIDH_TRACE("conn_id:%d last:Done");
        return WICED_BT_GATT_SUCCESS;
    }

    p_db_char = &p_dev->database.characteristics[p_dev->database.search_char_desc_index];
    for (char_idx = p_dev->database.search_char_desc_index ;
            char_idx < p_dev->database.characteristics_nb ; char_idx++, p_db_char++)
    {
        /* If this is an HID Report Descriptor */
        if (p_db_char->uuid16 == UUID_CHARACTERISTIC_HID_REPORT)
        {
            start_handle = p_db_char->handle;
            /* If this is not the last Characteristic stored, the last Handle is the Next one minus 1 */
            if (char_idx < (p_dev->database.characteristics_nb - 1))
            {
                end_handle = p_db_char[1].handle - 1;
            }
            /* If it's the last Characteristic stored, use the Global End Handle */
            else
            {
                end_handle = p_dev->database.end_handle;
            }
            p_dev->database.search_char_desc_index = char_idx;
            break;
        }
    }

    if (start_handle == 0)
    {
        LE_HIDH_TRACE("conn_id:%d no more:Done");
        return WICED_BT_GATT_SUCCESS;
    }

    LE_HIDH_TRACE("conn_id:%d idx:%d start:%04X end:%04X",
            p_dev->database.search_char_desc_index, p_dev->conn_id, start_handle, end_handle);

    status = wiced_bt_util_send_gatt_discover(p_dev->conn_id,
            GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS, 0, start_handle, end_handle);
    if (status == WICED_BT_GATT_SUCCESS)
        status = WICED_BT_GATT_PENDING;

    return status;
}

/**
 * Add a GATT Characteristic Attribute in the GATT database
 */
static hidh_le_status_t hidh_le_gattc_add_char_desc(hidh_le_dev_t *p_dev,
        wiced_bt_gatt_char_descr_info_t *p_char_desc_info)
{
    wiced_bt_uuid_t ccc_uuid = {LEN_UUID_16, {UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION}};
    wiced_bt_uuid_t rpt_ref_uuid = {LEN_UUID_16, {UUID_DESCRIPTOR_REPORT_REFERENCE}};
    wiced_bt_uuid_t hid_report_uuid = {LEN_UUID_16, {UUID_CHARACTERISTIC_HID_REPORT}};

    /* If this is an Client Characteristic Configuration UUID */
    if (wiced_bt_util_uuid_cmp(&p_char_desc_info->type, &ccc_uuid) == 0)
    {
        /* Save it to configure Notification (later) */
        LE_HIDH_TRACE("Save CCC");
        if (p_dev->database.client_char_configs_nb >= HIDH_REPORT_DESC_MAX)
        {
            LE_HIDH_TRACE_ERR("CCC Mem Full !!!");
            return HIDH_STATUS_MEM_FULL;
        }
        /* Save the Client Characteristic Config Handle */
        p_dev->database.client_char_configs[p_dev->database.client_char_configs_nb] = p_char_desc_info->handle;
        p_dev->database.client_char_configs_nb++;
    }
    /* Else If this is a Report Reference UUID */
    else if (wiced_bt_util_uuid_cmp(&p_char_desc_info->type, &rpt_ref_uuid) == 0)
    {
        /*
         * Save it. we will need it to retrieve the associated reportId to convert received
         * Handles to/from ReportId */
        LE_HIDH_TRACE("Save Report Reference");
        if (p_dev->database.report_descs_nb >= HIDH_REPORT_DESC_MAX)
        {
            LE_HIDH_TRACE_ERR("Report Mem Full !!!");
            return HIDH_STATUS_MEM_FULL;
        }
        /* Save the Report Handle */
        p_dev->database.report_descs[p_dev->database.report_descs_nb].handle = p_char_desc_info->handle;
        p_dev->database.report_descs_nb++;
    }
    /* Else If this is an HID Report Handle  UUID */
    else if (wiced_bt_util_uuid_cmp(&p_char_desc_info->type, &hid_report_uuid) == 0)
    {
        /*
         * Save it. we will need it to retrieve the associated reportId to convert received
         * Handles to/from ReportId */
        LE_HIDH_TRACE("Save HID Report Handle");
        if (p_dev->database.report_val_hdl_nb >= HIDH_REPORT_DESC_MAX)
        {
            LE_HIDH_TRACE_ERR("HID Report Mem Full !!!");
            return HIDH_STATUS_MEM_FULL;
        }
        /* Save the Report Val Handle */
        p_dev->database.report_val_hdl[p_dev->database.report_val_hdl_nb] = p_char_desc_info->handle;
        p_dev->database.report_val_hdl_nb++;
    }
    else
    {
        LE_HIDH_TRACE("ignore UUID");
    }
    return HIDH_STATUS_SUCCESS;
}

/**
 * Get Report Information from Handle
 */
static hidh_le_gatt_report_t *hidh_le_gattc_get_report_info_from_handle(
        hidh_le_dev_t *p_dev, uint16_t handle)
{
    int report_idx;
    hidh_le_gatt_report_t *p_report;

    LE_HIDH_TRACE("conn_id:%d handle:0x%04X", p_dev->conn_id, handle);
    p_report = &p_dev->database.report_descs[0];
    for (report_idx = 0 ; report_idx < p_dev->database.report_descs_nb ; report_idx++, p_report++)
    {
        if (p_report->val_handle == handle)
        {
            LE_HIDH_TRACE("found ReportId:%02x", p_report->rpt_id);
            return p_report;
        }
    }
    LE_HIDH_TRACE_ERR("handle:%04X unknown", handle);
    return NULL;
}

/**
 * GATT Notification received for LE HID Report
 */
static void hidh_le_gattc_notification(hidh_le_dev_t *p_dev,
        wiced_bt_gatt_operation_complete_t *p_operation_complete)
{
    hidh_le_gatt_report_t *p_report;
    hidh_le_event_data_t report_evt;
    wiced_bool_t wakeup_status;

    /* Get the Report Information based on the Received Handle */
    p_report = hidh_le_gattc_get_report_info_from_handle(p_dev,
            p_operation_complete->response_data.att_value.handle);
    if (p_report == NULL)
    {
        LE_HIDH_TRACE("unknown handle:%04X",
                p_operation_complete->response_data.att_value.handle);
        return;
    }

    /* Check if this Report matches a WakeUp pattern */
    wakeup_status =  hidh_le_wakeup_pattern_check(p_dev, p_report->rpt_id,
            p_operation_complete->response_data.att_value.p_data,
            p_operation_complete->response_data.att_value.len);
    if (wakeup_status != WICED_FALSE)
    {
        /*
         * The Report match a WakeUp pattern. The WakeUp GPIO has been asserted.
         * We could add a return statement here to do not send the report to Host.
         * But, it may prevent the Host to receive the Power button press (asking to enter StandBy
         * mode) if the Host enabled the WakeUp
         */
    }

    /* Sent the Receive Report to the Application */
    report_evt.report.handle = p_dev->conn_id + HIDH_HANDLE_OFFSET;
    report_evt.report.report_id = p_report->rpt_id;
    report_evt.report.p_data = p_operation_complete->response_data.att_value.p_data;
    report_evt.report.length = p_operation_complete->response_data.att_value.len;
    hidh_le_core_callback(HIDH_REPORT_EVT, &report_evt);
}

/**
 * Read he Next Report Reference attribute
 */
static wiced_bt_gatt_status_t hidh_le_gattc_read_next_report_ref(hidh_le_dev_t *p_dev)
{
    wiced_bt_gatt_status_t status;

    LE_HIDH_TRACE("conn_id:%d idx:%d", p_dev->conn_id,
            p_dev->database.search_char_desc_index);

    if (p_dev->database.search_char_desc_index >= p_dev->database.report_descs_nb)
    {
        LE_HIDH_TRACE("conn_id:%d last:Done");
        return WICED_BT_GATT_SUCCESS;
    }

    status = wiced_bt_util_send_gatt_read_by_handle(p_dev->conn_id, p_dev->database.report_descs[p_dev->database.search_char_desc_index].handle);
    if (status == WICED_BT_GATT_SUCCESS)
        status = WICED_BT_GATT_PENDING;

    return status;
}

/**
 * Configure Notifications for the next Report (writes Client Control Configuration attribute)
 */
static wiced_bt_gatt_status_t hidh_le_gattc_configure_next_report(hidh_le_dev_t *p_dev)
{
    wiced_bt_gatt_status_t status;

    LE_HIDH_TRACE("conn_id:%d idx:%d", p_dev->conn_id,
            p_dev->database.search_char_desc_index);

    if (p_dev->database.search_char_desc_index >= p_dev->database.client_char_configs_nb)
    {
        LE_HIDH_TRACE("conn_id:%d last:Done");
        return WICED_BT_GATT_SUCCESS;
    }

    /* Write the Configuration Attribute */
    status = wiced_bt_util_set_gatt_client_config_descriptor(p_dev->conn_id,
            p_dev->database.client_char_configs[p_dev->database.search_char_desc_index],
            GATT_CLIENT_CONFIG_NOTIFICATION);
    if (status == WICED_BT_GATT_SUCCESS)
        status = WICED_BT_GATT_PENDING;
    return status;
}

/**
 * This functin is called for READ gatt event
 */
static void hidh_le_gattc_read_complete(hidh_le_dev_t *p_dev,
        wiced_bt_gatt_operation_complete_t *p_operation_complete)
{
    uint8_t *p_data;
    wiced_bt_gatt_status_t status;
    hidh_le_event_data_t hidh_evt;

    /* If we are searching for Report Reference */
    if (p_dev->database.db_state == HIDH_GATTC_STATE_SEARCHING_REPORT_REF)
    {
        if ((p_operation_complete->status == WICED_BT_GATT_SUCCESS) &&
            (p_operation_complete->response_data.att_value.len == 2))
        {
            /* The received data contains the ReportId and the ReportType */
            p_data = p_operation_complete->response_data.att_value.p_data;
            STREAM_TO_UINT8(p_dev->database.report_descs[p_dev->database.search_char_desc_index].rpt_id, p_data);
            STREAM_TO_UINT8(p_dev->database.report_descs[p_dev->database.search_char_desc_index].rpt_type, p_data);

            /* Copy the Report Val Handle (received earlier) in the Report Descriptor database */
            p_dev->database.report_descs[p_dev->database.search_char_desc_index].val_handle =
                    p_dev->database.report_val_hdl[p_dev->database.search_char_desc_index];

            /* Continue to Read HID Report References */
            p_dev->database.search_char_desc_index++;
            LE_HIDH_TRACE("Continuing HID Report Ref search...");
            status = hidh_le_gattc_read_next_report_ref(p_dev);
            if (status == WICED_BT_GATT_PENDING)
            {
                /* Gatt Read Pending */
            }
            else if (status == WICED_BT_GATT_SUCCESS)
            {
                /* This is the last Report Reference. We can, now, configure Notification for
                 * every Input Report */
                hidh_le_gattc_dump(p_dev);

                /* Start to Configure Notifications */
                p_dev->database.search_char_desc_index = 0;
                LE_HIDH_TRACE("Starting CCC...");
                status = hidh_le_gattc_configure_next_report(p_dev);
                if (status == WICED_BT_GATT_PENDING)
                {
                    p_dev->database.db_state = HIDH_GATTC_STATE_CONFIGURE_NOTIFICATION;
                }
                else if (status == WICED_BT_GATT_SUCCESS)
                {
                    /* If there is not Client Configuration, we are done */
                    p_dev->database.db_state = HIDH_GATTC_STATE_DONE;
                    hidh_le_core_gatt_complete(p_dev, HIDH_STATUS_SUCCESS);
                }
                else
                {
                    hidh_le_core_gatt_complete(p_dev, HIDH_STATUS_GATT_ERROR);
                }
            }
            else
            {
                hidh_le_core_gatt_complete(p_dev, HIDH_STATUS_GATT_ERROR);
            }
        }
        else
        {
            hidh_le_core_gatt_complete(p_dev, HIDH_STATUS_GATT_ERROR);
        }
    }
    else if (p_dev->dev_state_msk & HIDH_DEV_STATE_OP_READ_DESC)
    {
        LE_HIDH_TRACE("ReadDesc conn_id:%d status:%d len:%d offset:%d",
                p_dev->conn_id, p_operation_complete->status,
                p_operation_complete->response_data.att_value.len,
                p_operation_complete->response_data.att_value.offset);
        p_dev->dev_state_msk &= ~HIDH_DEV_STATE_OP_READ_DESC;
        if (p_operation_complete->status == WICED_BT_GATT_SUCCESS)
        {
            hidh_evt.descriptor.status = HIDH_STATUS_SUCCESS;
            hidh_evt.descriptor.length = p_operation_complete->response_data.att_value.len;
            hidh_evt.descriptor.p_descriptor =
                    p_operation_complete->response_data.att_value.p_data +
                    p_operation_complete->response_data.att_value.offset;
        }
        else
        {
            hidh_evt.descriptor.status = HIDH_STATUS_GATT_ERROR;
            hidh_evt.descriptor.length = 0;
            hidh_evt.descriptor.p_descriptor = NULL;
        }
        hidh_evt.descriptor.handle = p_dev->conn_id + HIDH_HANDLE_OFFSET;
        hidh_le_core_callback(HIDH_DESCRIPTOR_EVT, &hidh_evt);
    }
    else if (p_dev->dev_state_msk & HIDH_DEV_STATE_OP_GET_REPORT)
    {
        LE_HIDH_TRACE("GetReport conn_id:%d status:%d len:%d offset:%d",
                p_dev->conn_id, p_operation_complete->status,
                p_operation_complete->response_data.att_value.len,
                p_operation_complete->response_data.att_value.offset);
        p_dev->dev_state_msk &= ~HIDH_DEV_STATE_OP_GET_REPORT;
        if (p_operation_complete->status == WICED_BT_GATT_SUCCESS)
        {
            hidh_evt.get_report.status = HIDH_STATUS_SUCCESS;
        }
        else
        {
            hidh_evt.get_report.status = HIDH_STATUS_GATT_ERROR;
        }
        hidh_evt.get_report.handle = p_dev->conn_id + HIDH_HANDLE_OFFSET;
        hidh_evt.get_report.length = p_operation_complete->response_data.att_value.len;
        hidh_evt.get_report.p_data = p_operation_complete->response_data.att_value.p_data;
        hidh_le_core_callback(HIDH_GET_REPORT_EVT, &hidh_evt);
    }
    else
    {
        LE_HIDH_TRACE_ERR("no pending op!!!");
    }
}

/**
 * This functin is called for READ gatt event
 */
static void hidh_le_gattc_write_complete(hidh_le_dev_t *p_dev,
        wiced_bt_gatt_operation_complete_t *p_operation_complete)
{
    wiced_bt_gatt_status_t status;
    hidh_le_event_data_t hidh_evt;

    /* If we are searching for Report Reference */
    if (p_dev->database.db_state == HIDH_GATTC_STATE_CONFIGURE_NOTIFICATION)
    {
        if (p_operation_complete->status == WICED_BT_GATT_SUCCESS)
        {
            /* Continue to Configure HID Report Notification */
            p_dev->database.search_char_desc_index++;
            LE_HIDH_TRACE("Continuing CCC...");
            status = hidh_le_gattc_configure_next_report(p_dev);
            if (status == WICED_BT_GATT_PENDING)
            {
                /* Operation Pending */
            }
            else if (status == WICED_BT_GATT_SUCCESS)
            {
                /* No more Notification to configure */
                p_dev->database.db_state = HIDH_GATTC_STATE_DONE;
                hidh_le_core_gatt_complete(p_dev, HIDH_STATUS_SUCCESS);
            }
            else
            {
                hidh_le_core_gatt_complete(p_dev, HIDH_STATUS_GATT_ERROR);
            }
        }
        else
        {
            hidh_le_core_gatt_complete(p_dev, HIDH_STATUS_GATT_ERROR);
        }
    }
    else if (p_dev->dev_state_msk & HIDH_DEV_STATE_OP_SET_REPORT)
    {
        LE_HIDH_TRACE("SetReport conn_id:%d status:%d len:%d offset:%d",
                p_dev->conn_id, p_operation_complete->status,
                p_operation_complete->response_data.att_value.len,
                p_operation_complete->response_data.att_value.offset);
        p_dev->dev_state_msk &= ~HIDH_DEV_STATE_OP_SET_REPORT;
        if (p_operation_complete->status == WICED_BT_GATT_SUCCESS)
        {
            hidh_evt.set_report.status = HIDH_STATUS_SUCCESS;
        }
        else
        {
            hidh_evt.set_report.status = HIDH_STATUS_GATT_ERROR;
        }
        hidh_evt.set_report.handle = p_dev->conn_id + HIDH_HANDLE_OFFSET;
        hidh_le_core_callback(HIDH_SET_REPORT_EVT, &hidh_evt);
    }
    else if (p_dev->dev_state_msk & HIDH_DEV_STATE_OP_SET_PROTOCOL)
    {
        LE_HIDH_TRACE("SetProtocol conn_id:%d status:%d len:%d offset:%d",
                p_dev->conn_id, p_operation_complete->status,
                p_operation_complete->response_data.att_value.len,
                p_operation_complete->response_data.att_value.offset);
        p_dev->dev_state_msk &= ~HIDH_DEV_STATE_OP_SET_PROTOCOL;
        if (p_operation_complete->status == WICED_BT_GATT_SUCCESS)
        {
            hidh_evt.set_protocol.status = HIDH_STATUS_SUCCESS;
        }
        else
        {
            hidh_evt.set_protocol.status = HIDH_STATUS_GATT_ERROR;
        }
        hidh_evt.set_protocol.handle = p_dev->conn_id + HIDH_HANDLE_OFFSET;
        hidh_le_core_callback(HIDH_SET_PROTOCOL_EVT, &hidh_evt);
    }
    else
    {
        LE_HIDH_TRACE_ERR("no pending op!!!");
    }
}

/**
 * Get Report Information from Report Id and Type
 */
static hidh_le_gatt_report_t *hidh_le_gattc_get_report_info_from_id_type(
        hidh_le_dev_t *p_dev, uint8_t report_id,
        hidh_le_report_type_t report_type)
{
    int report_idx;
    hidh_le_gatt_report_t *p_report;

    LE_HIDH_TRACE("conn_id:%d id:%02x type:%d", p_dev->conn_id, report_id, report_type);
    p_report = &p_dev->database.report_descs[0];
    for (report_idx = 0 ; report_idx < p_dev->database.report_descs_nb ; report_idx++, p_report++)
    {
        if ((p_report->rpt_id == report_id) &&
            (p_report->rpt_type == report_type))
        {
            return p_report;
        }
    }
    LE_HIDH_TRACE_ERR("id:%02x type:%d unknown", report_id, report_type);
    return NULL;
}

/**
 * Get Report Information from Type
 */
static hidh_le_gatt_report_t *hidh_le_gattc_get_report_info_from_type(
        hidh_le_dev_t *p_dev, hidh_le_report_type_t report_type)
{
    int report_idx;
    hidh_le_gatt_report_t *p_report;

    LE_HIDH_TRACE("conn_id:%d type:%d", p_dev->conn_id, report_type);
    p_report = &p_dev->database.report_descs[0];
    for (report_idx = 0 ; report_idx < p_dev->database.report_descs_nb ; report_idx++, p_report++)
    {
        /* Stop when the first Report matching ReportType is found */
        if (p_report->rpt_type == report_type)
        {
            return p_report;
        }
    }
    LE_HIDH_TRACE_ERR("no report type:%d found", report_type);
    return NULL;
}

/**
 * Get a Characteristic from its (16 bits) UUID
 */
static hidh_le_gatt_char_t *hidh_le_gattc_get_char_from_uuid16(
        hidh_le_dev_t *p_dev, uint16_t uuid16)
{
    int char_idx;
    hidh_le_gatt_char_t *p_char;

    LE_HIDH_TRACE("conn_id:%d uuid16:%04X", p_dev->conn_id, uuid16);
    p_char = &p_dev->database.characteristics[0];

    for (char_idx = 0 ; char_idx < p_dev->database.characteristics_nb ; char_idx++, p_char++)
    {
        /* Stop when the first Report matching ReportType is found */
        if (p_char->uuid16 == uuid16)
        {
            return p_char;
        }
    }
    LE_HIDH_TRACE_ERR("uuid16:%04X not found", uuid16);
    return NULL;
}

/***********************************************************************
 * Public functions
 ***********************************************************************/

/**
 * Start to search/discover all GATT LE HID from a peer device
 */
hidh_le_status_t hidh_le_gattc_search(hidh_le_dev_t *p_dev)
{
    wiced_bt_gatt_status_t status;

    LE_HIDH_TRACE("conn_id:%d", p_dev->conn_id);

    /* Perform HID primary service search */
    status = wiced_bt_util_send_gatt_discover(p_dev->conn_id, GATT_DISCOVER_SERVICES_BY_UUID,
            UUID_SERVICE_HID, 1, 0xffff);
    if (status != WICED_BT_GATT_SUCCESS)
    {
        LE_HIDH_TRACE_ERR("Gatt discover failed status:%d", status);
        return HIDH_STATUS_ERROR;
    }
    p_dev->database.db_state = HIDH_GATTC_STATE_SEARCHING_SRV;
    return HIDH_STATUS_SUCCESS;
}

/**
 * Read the descriptor
 */
hidh_le_status_t hidh_le_gattc_read_descriptor(hidh_le_dev_t *p_dev)
{
    wiced_bt_gatt_status_t status;
    int char_idx;
    hidh_le_gatt_char_t *p_char;

    LE_HIDH_TRACE("conn_id:%d", p_dev->conn_id);

    if (p_dev->database.db_state != HIDH_GATTC_STATE_DONE)
    {
        LE_HIDH_TRACE_ERR("no GATT DB conn_id:%d", p_dev->conn_id);
        return HIDH_STATUS_GATT_ERROR;
    }

    /* Check if a GATT Operation is pending */
    if (p_dev->dev_state_msk & HIDH_DEV_STATE_OP_PENDING_MSK)
    {
        LE_HIDH_TRACE_ERR("GATT Op Pending conn_id:%d", p_dev->conn_id);
        return HIDH_STATUS_ERROR;
    }

    /* Search the UUID_CHARACTERISTIC_REPORT_MAP Attribute in the Database */
    p_char = &p_dev->database.characteristics[0];
    for (char_idx = 0 ; char_idx < p_dev->database.characteristics_nb ; char_idx++, p_char++ )
    {
        /* If this is an HID Descriptor UUID (aka ReportMap) */
        if (p_char->uuid16 == UUID_CHARACTERISTIC_REPORT_MAP)
        {
            status = wiced_bt_util_send_gatt_read_by_handle(p_dev->conn_id, p_char->val_handle);
            if (status == WICED_BT_GATT_SUCCESS)
            {
                p_dev->dev_state_msk |= HIDH_DEV_STATE_OP_READ_DESC;
                return HIDH_STATUS_SUCCESS;
            }
            else
                return HIDH_STATUS_ERROR;
        }
    }
    return HIDH_STATUS_GATT_ERROR;
}

/**
 * Application calls this function to Set an HID Report
 */
hidh_le_status_t hidh_le_gattc_set_report(hidh_le_dev_t *p_dev,
        hidh_le_report_type_t report_type, uint8_t report_id, uint8_t *p_data,
        uint16_t length)
{
    hidh_le_gatt_report_t *p_report;
    wiced_bt_gatt_write_hdr_t write_data;
    uint8_t * p_data_buff;

    LE_HIDH_TRACE("conn_id:%d type:%d id:%02x len:%d%", p_dev->conn_id, report_type, report_id, length);

    if (p_dev->database.db_state != HIDH_GATTC_STATE_DONE)
    {
        LE_HIDH_TRACE_ERR("no GATT DB conn_id:%d", p_dev->conn_id);
        return HIDH_STATUS_GATT_ERROR;
    }

    /* Check if a GATT Operation is pending */
    if (p_dev->dev_state_msk & HIDH_DEV_STATE_OP_PENDING_MSK)
    {
        LE_HIDH_TRACE_ERR("GATT Op Pending conn_id:%d", p_dev->conn_id);
        return HIDH_STATUS_ERROR;
    }

    /* Get the Report Information based on the Report Id and type */
    p_report = hidh_le_gattc_get_report_info_from_id_type(p_dev, report_id, report_type);
    if (p_report == NULL)
    {
        LE_HIDH_TRACE_ERR("unknown id:%02x type:%d", report_id, report_type);
        return HIDH_STATUS_INVALID_PARAM;
    }

    /* Allocate a temporary buffer to send the GATT Value */
    p_data_buff = (uint8_t *) wiced_bt_get_buffer(length);
    if (p_data_buff == NULL)
    {
        LE_HIDH_TRACE_ERR("Mem Full", report_id, report_type);
        return HIDH_STATUS_MEM_FULL;
    }

    write_data.handle = p_report->val_handle;
    write_data.len = length;
    write_data.offset = 0;
    write_data.auth_req = GATT_AUTH_REQ_NONE;
    memcpy(p_data_buff, p_data, length);

    /* Send the Report */
    if (wiced_bt_gatt_client_send_write (p_dev->conn_id, GATT_REQ_WRITE, &write_data, p_data_buff, NULL) == WICED_BT_GATT_SUCCESS)
    {
        p_dev->dev_state_msk |= HIDH_DEV_STATE_OP_SET_REPORT;
        return HIDH_STATUS_SUCCESS;
    }
    else
    {
        /* Free the temporary buffer */
        wiced_bt_free_buffer(p_data_buff);
        return HIDH_STATUS_ERROR;
    }
}

/**
 * Application calls this function to Set an HID Report
 */
hidh_le_status_t hidh_le_gattc_get_report(hidh_le_dev_t *p_dev,
        hidh_le_report_type_t report_type, uint8_t report_id, uint16_t length)
{
    hidh_le_gatt_report_t *p_report;

    LE_HIDH_TRACE("conn_id:%d type:%d id:%02x len:%d%", p_dev->conn_id, report_type,
            report_id, length);

    if (p_dev->database.db_state != HIDH_GATTC_STATE_DONE)
    {
        LE_HIDH_TRACE_ERR("no GATT DB conn_id:%d", p_dev->conn_id);
        return HIDH_STATUS_GATT_ERROR;
    }

    /* Check if a GATT Operation is pending */
    if (p_dev->dev_state_msk & HIDH_DEV_STATE_OP_PENDING_MSK)
    {
        LE_HIDH_TRACE_ERR("GATT Op Pending conn_id:%d", p_dev->conn_id);
        return HIDH_STATUS_ERROR;
    }

    /* If the application does not indicate Report Id */
    if (report_id == 0)
    {
        /* Get the Report Information based on the type (only) */
        p_report = hidh_le_gattc_get_report_info_from_type(p_dev, report_type);
    }
    else
    {
        /* Get the Report Information based on both the Report Id and type */
        p_report = hidh_le_gattc_get_report_info_from_id_type(p_dev, report_id, report_type);
    }
    if (p_report == NULL)
    {
        LE_HIDH_TRACE_ERR("unknown id:%02x type:%d", report_id, report_type);
        return HIDH_STATUS_INVALID_PARAM;
    }

    if (wiced_bt_gatt_client_send_read_handle(p_dev->conn_id, p_report->val_handle, 0, NULL, 0, GATT_AUTH_REQ_NONE) == WICED_BT_GATT_SUCCESS)
    {
        p_dev->dev_state_msk |= HIDH_DEV_STATE_OP_GET_REPORT;
        return HIDH_STATUS_SUCCESS;
    }
    else
    {
        return HIDH_STATUS_ERROR;
    }
}

/**
 * Application calls this function to Set Protocol
 */
hidh_le_status_t hidh_le_gattc_set_protocol(hidh_le_dev_t *p_dev,
        hidh_le_protocol_t protocol)
{
    hidh_le_gatt_char_t *p_db_char;
    wiced_bt_gatt_write_hdr_t write_data;
    uint8_t * p_data_buff;

    LE_HIDH_TRACE("conn_id:%d protocol:%d%", p_dev->conn_id, protocol);

    if (p_dev->database.db_state != HIDH_GATTC_STATE_DONE)
    {
        LE_HIDH_TRACE_ERR("no GATT DB conn_id:%d", p_dev->conn_id);
        return HIDH_STATUS_GATT_ERROR;
    }

    /* Check if a GATT Operation is pending */
    if (p_dev->dev_state_msk & HIDH_DEV_STATE_OP_PENDING_MSK)
    {
        LE_HIDH_TRACE_ERR("GATT Op Pending conn_id:%d", p_dev->conn_id);
        return HIDH_STATUS_ERROR;
    }

    /* Search the HID Protocol characteristic in the GATT database */
    p_db_char = hidh_le_gattc_get_char_from_uuid16(p_dev,
            UUID_CHARACTERISTIC_HID_PROTOCOL_MODE);
    if (p_db_char == NULL)
    {
        LE_HIDH_TRACE_ERR("HID Protocol characteristic not found");
        return HIDH_STATUS_UNSUPPORTED;
    }

    write_data.handle = p_db_char->val_handle;
    write_data.len = sizeof(uint8_t);
    write_data.offset = 0;
    write_data.auth_req = GATT_AUTH_REQ_NONE;

    p_data_buff = (uint8_t *) wiced_bt_get_buffer(1);
    if (p_data_buff == NULL)
    {
        LE_HIDH_TRACE_ERR("Mem Full");
        return HIDH_STATUS_MEM_FULL;
    }
    *p_data_buff = (uint8_t)protocol;

    /* Send the Report */
    if (wiced_bt_gatt_client_send_write (p_dev->conn_id, GATT_REQ_WRITE, &write_data, p_data_buff, NULL) == WICED_BT_GATT_SUCCESS)
    {
        p_dev->dev_state_msk |= HIDH_DEV_STATE_OP_SET_PROTOCOL;
        return HIDH_STATUS_SUCCESS;
    }
    else
    {
        /* Free the temporary buffer */
        wiced_bt_free_buffer(p_data_buff);
        return HIDH_STATUS_ERROR;
    }
}

/**
 * Gatt discovery result
 */
wiced_bt_gatt_status_t hidh_le_gattc_discovery_result(hidh_le_dev_t *p_dev,
        wiced_bt_gatt_discovery_result_t *p_result)
{
    hidh_le_status_t result;
    wiced_bt_gatt_char_declaration_t *p_char;
    wiced_bt_gatt_char_descr_info_t *p_char_desc_info;

    switch (p_result->discovery_type)
    {
    case GATT_DISCOVER_SERVICES_BY_UUID:
        /* Service Discovery */
        LE_HIDH_TRACE("LE HIDH Service s_handle:%04x e_handle:%04x uuid:%04x",
                p_result->discovery_data.group_value.s_handle,
                p_result->discovery_data.group_value.e_handle,
                p_result->discovery_data.group_value.service_type.uu.uuid16);

        /* Check if the UUID matches the request (HID) */
        if ((p_result->discovery_data.group_value.service_type.len == LEN_UUID_16) &&
             (p_result->discovery_data.group_value.service_type.uu.uuid16 == UUID_SERVICE_HID))
        {
            LE_HIDH_TRACE( "HID Service found");
            p_dev->database.start_handle = p_result->discovery_data.group_value.s_handle;
            p_dev->database.end_handle = p_result->discovery_data.group_value.e_handle;
        }
        break;

    case GATT_DISCOVER_CHARACTERISTICS:
        /* Result for characteristic discovery.  Save appropriate handle based on the UUID. */
        p_char = &p_result->discovery_data.characteristic_declaration;

        LE_HIDH_TRACE("UUID:%04X hdl:%04x val_hdl:%04x, prop:%x",
                p_char->char_uuid.uu.uuid16, p_char->handle,  p_char->val_handle,
                p_char->characteristic_properties);
        /* Add this Characteristic in the Database */
        result = hidh_le_gattc_add_char(p_dev, p_char);
        if (result != HIDH_STATUS_SUCCESS)
        {
            /* No more Memory to save Characteristic. Disconnect link to prevent GATT Attribute mess */
            LE_HIDH_TRACE_ERR("No more Memory to save Characteristic. Disconnect link to prevent GATT Attribute mess" );
            wiced_bt_gatt_disconnect(p_dev->conn_id);
        }
        break;

    case GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS:
        p_char_desc_info = &p_result->discovery_data.char_descr_info;
        LE_HIDH_TRACE("Char Desc idx:%d UUID:%04X Handle:%04X",
                p_dev->database.search_char_desc_index,
                p_char_desc_info->type.uu.uuid16,
                p_char_desc_info->handle);
        result = hidh_le_gattc_add_char_desc(p_dev, p_char_desc_info);
        if (result != HIDH_STATUS_SUCCESS)
        {
            /* No more Memory to save Characteristic Descriptor.
             * Disconnect link to prevent GATT Attribute mess.
             */
            LE_HIDH_TRACE_ERR("No more Memory to save Characteristic discriptor. Disconnect link to prevent GATT Attribute mess" );
            wiced_bt_gatt_disconnect(p_dev->conn_id);
        }
        break;

    case GATT_DISCOVER_INCLUDED_SERVICES:
    case GATT_DISCOVER_SERVICES_ALL:
        /* Unused by this application */
        LE_HIDH_TRACE("type:%d ignored", p_result->discovery_type);
        break;

    default:
        LE_HIDH_TRACE("type:%d unknown", p_result->discovery_type);
        break;
    }
    return WICED_BT_GATT_SUCCESS;
}

/**
 * Gatt discovery complete
 */
wiced_bt_gatt_status_t hidh_le_gattc_discovery_complete(hidh_le_dev_t *p_dev,
        wiced_bt_gatt_discovery_complete_t *p_discovery_complete)
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_SUCCESS;

    LE_HIDH_TRACE("type:%d status:%d", p_discovery_complete->discovery_type,
            p_discovery_complete->status);

    // Tak: work around for now... TBD
    if (p_discovery_complete->status == 10)
    {
        WICED_BT_TRACE("*** WORKAROUND *** Forced p_discovery_complete->status 10 to 0 (success)");
        p_discovery_complete->status = WICED_BT_GATT_SUCCESS;
    }

    switch(p_discovery_complete->discovery_type)
    {
    case GATT_DISCOVER_SERVICES_BY_UUID:
        LE_HIDH_TRACE("GATT_DISCOVER_SERVICES_BY_UUID");
        /* Service Discovery */
        if ((p_discovery_complete->status != WICED_BT_GATT_SUCCESS) ||
            (p_dev->database.start_handle == 0) ||
            (p_dev->database.end_handle == 0))
        {
            hidh_le_core_gatt_complete(p_dev, HIDH_STATUS_GATT_ERROR);
        }
        else
        {
            /* Perform HID Characteristics search */
            status = wiced_bt_util_send_gatt_discover(p_dev->conn_id, GATT_DISCOVER_CHARACTERISTICS,
                    0, p_dev->database.start_handle, p_dev->database.end_handle);
            if (status != WICED_BT_GATT_SUCCESS)
            {
                LE_HIDH_TRACE_ERR("Gatt Disc Char failed status:%d", status);
                hidh_le_core_gatt_complete(p_dev, HIDH_STATUS_GATT_ERROR);
            }
            else
            {
                LE_HIDH_TRACE("Searching char...");
                p_dev->database.db_state = HIDH_GATTC_STATE_SEARCHING_CHAR;
            }
        }
        break;

    case GATT_DISCOVER_CHARACTERISTICS:
        LE_HIDH_TRACE("GATT_DISCOVER_CHARACTERISTICS");
        /* Characteristic Discovery */
        if ((p_discovery_complete->status != WICED_BT_GATT_SUCCESS) ||
            (p_dev->database.characteristics_nb == 0))
        {
            hidh_le_core_gatt_complete(p_dev, HIDH_STATUS_GATT_ERROR);
        }
        else
        {
            hidh_le_gattc_dump(p_dev);

            /* Start to search HID Report Descriptors */
            p_dev->database.search_char_desc_index = 0;
            LE_HIDH_TRACE("Starting HID Report search...");
            status = hidh_le_gattc_discover_next_char_desc(p_dev);
            /* If the First HID Report search does not start (no HID Report), this is an error */
            if (status == WICED_BT_GATT_PENDING)
            {
                p_dev->database.db_state = HIDH_GATTC_STATE_SEARCHING_CHAR_DESC;
            }
            else
            {
                LE_HIDH_TRACE("hidh_le_gattc_discover_next_char_desc didn't return WICED_BT_GATT_PENDING but %d", status);
                hidh_le_core_gatt_complete(p_dev, HIDH_STATUS_GATT_ERROR);
            }
        }
        break;

    case GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS:
        LE_HIDH_TRACE("GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS");
        /* Characteristic Descriptor Discovery */
        if ( p_discovery_complete->status != WICED_BT_GATT_SUCCESS )
        {
            hidh_le_core_gatt_complete(p_dev, HIDH_STATUS_GATT_ERROR);
        }
        else
        {
            /* Continue to search HID Report Descriptors */
            LE_HIDH_TRACE("Continuing HID Report search...");
            p_dev->database.search_char_desc_index++;
            status = hidh_le_gattc_discover_next_char_desc(p_dev);
            if (status == WICED_BT_GATT_SUCCESS)
            {
                /* No more HID Report to search */
                LE_HIDH_TRACE("CharDesc Complete");
                hidh_le_gattc_dump(p_dev);
                /* Start to search HID Report References */
                p_dev->database.search_char_desc_index = 0;
                LE_HIDH_TRACE("Starting HID Report Ref search...");
                status = hidh_le_gattc_read_next_report_ref(p_dev);
                /* If the First HID Report search does not start (no HID Report), this is an error */
                if (status == WICED_BT_GATT_PENDING)
                {
                    p_dev->database.db_state = HIDH_GATTC_STATE_SEARCHING_REPORT_REF;
                }
                else
                {
                    LE_HIDH_TRACE("hidh_le_gattc_read_next_report_ref() did not return pending status but %d", status);
                    hidh_le_core_gatt_complete(p_dev, HIDH_STATUS_GATT_ERROR);
                }
            }
            else if (status == WICED_BT_GATT_PENDING)
            {
                /* In Progress */
            }
            else
            {
                hidh_le_core_gatt_complete(p_dev, HIDH_STATUS_GATT_ERROR);
            }
        }
        break;

    case GATT_DISCOVER_INCLUDED_SERVICES:
    case GATT_DISCOVER_SERVICES_ALL:
        /* Unused for this application */
        LE_HIDH_TRACE("type:%d ignored", p_discovery_complete->discovery_type);
        break;

    default:
        LE_HIDH_TRACE("type:%d unknown", p_discovery_complete->discovery_type);
        break;
    }
    return status;
}

/**
 * Gatt operation complete
 */
wiced_bt_gatt_status_t hidh_le_gattc_operation_complete(hidh_le_dev_t *p_dev,
        wiced_bt_gatt_operation_complete_t *p_operation_complete)
{
    LE_HIDH_TRACE("conn_id:%02x type:%d status:%d", p_operation_complete->conn_id,
            p_operation_complete->op, p_operation_complete->status);

    switch (p_operation_complete->op)
    {
    case GATTC_OPTYPE_WRITE_WITH_RSP:
    case GATTC_OPTYPE_WRITE_NO_RSP:
    case GATTC_OPTYPE_PREPARE_WRITE:
        LE_HIDH_TRACE("GATTC_OPTYPE_WRITE operation_complete");
        hidh_le_gattc_write_complete(p_dev, p_operation_complete);
        break;

    case GATTC_OPTYPE_CONFIG_MTU:
        LE_HIDH_TRACE("GATTC_OPTYPE_CONFIG_MTU operation_complete, peer mtu:%d", p_operation_complete->response_data.mtu);
        break;

    case GATTC_OPTYPE_READ_HANDLE:
    case GATTC_OPTYPE_READ_BY_TYPE:
    case GATTC_OPTYPE_READ_MULTIPLE:
        LE_HIDH_TRACE("GATTC_OPTYPE_READ operation_complete\n");
        hidh_le_gattc_read_complete(p_dev, p_operation_complete);
        break;

    case GATTC_OPTYPE_NOTIFICATION:
        LE_HIDH_TRACE("GATTC_OPTYPE_NOTIFICATION operation_complete\n");
        hidh_le_gattc_notification(p_dev, p_operation_complete);
        break;

    case GATTC_OPTYPE_INDICATION:
        LE_HIDH_TRACE("GATTC_OPTYPE_INDICATION operation_complete, unsupported\n");
#if 0
        // process data with indication confirm
        hidh_le_gattc_notification(p_dev, p_operation_complete);
        wiced_bt_gatt_client_send_indication_confirm(p_data->conn_id, p_data->response_data.handle);
#endif
        break;

    default:
        LE_HIDH_TRACE_ERR("unhandled gatt op", p_operation_complete->op);
        break;
    }
    return WICED_BT_GATT_SUCCESS;
}

/**
 * Debug function which dump all the GATT information retrieved from a LE HID Device
 */
void hidh_le_gattc_dump(hidh_le_dev_t *p_dev)
{
    int char_idx;
    hidh_le_gatt_char_t *p_db_char;
    hidh_le_gatt_report_t *p_report;

    p_db_char = &p_dev->database.characteristics[0];
    for (char_idx = 0 ; char_idx < p_dev->database.characteristics_nb ; char_idx++, p_db_char++)
    {
        LE_HIDH_TRACE("Char Idx:%d UUID:%04X hdl:%04X val_hdl:%04X",
                char_idx, p_db_char->uuid16, p_db_char->handle, p_db_char->val_handle);
    }
    p_report = &p_dev->database.report_descs[0];
    for (char_idx = 0 ; char_idx < p_dev->database.report_descs_nb ; char_idx++, p_report++)
    {
        LE_HIDH_TRACE("Report Idx:%d hdl:%04X id:%02X type:%x val_hdl:%04X",
                char_idx, p_report->handle, p_report->rpt_id, p_report->rpt_type, p_report->val_handle);
    }
    for (char_idx = 0 ; char_idx < p_dev->database.report_val_hdl_nb ; char_idx++)
    {
        LE_HIDH_TRACE("ValHdl Idx:%d hdl:%04X",
                char_idx, p_dev->database.report_val_hdl[char_idx]);
    }
    for (char_idx = 0 ; char_idx < p_dev->database.client_char_configs_nb ; char_idx++)
    {
        LE_HIDH_TRACE("CCC Idx:%d hdl:%04X",
                char_idx, p_dev->database.client_char_configs[char_idx]);
    }
}
