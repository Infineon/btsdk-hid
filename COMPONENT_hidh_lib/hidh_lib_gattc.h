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
 * @file hidh_lib_gattc.h
 *
 * @brief   hidh_lib gatt functions header file
 */
#ifndef __HIDH_LIB_GATTC_H_
#define __HIDH_LIB_GATTC_H_

#include "hidh_lib_core.h"

/**
 * @brief     Start to search/discover all GATT LE HID from a peer device
 *
 * @param[in] p_dev   : pointer to the device
 *
 * @return    @hidh_le_status_t HIDH_STATUS_SUCCESS or HIDH_STATUS_ERROR @endlink
 */
hidh_le_status_t hidh_le_gattc_search(hidh_le_dev_t *p_dev);

/**
 * @brief     Read the descriptor
 *
 * @param[in] p_dev : pointer to the device
 *
 * @return    @hidh_le_status_t HIDH_STATUS_SUCCESS or HIDH_STATUS_ERROR @endlink
 */
hidh_le_status_t hidh_le_gattc_read_descriptor(hidh_le_dev_t *p_dev);

/**
 * @brief     Application calls this function to Set an HID Report
 *
 * @param[in] p_dev       : pointer to the device
 * @param[in] report_type : report type
 * @param[in] report_id   : report id
 * @param[in] p_data      : data pointer
 *
 * @return    @hidh_le_status_t HIDH_STATUS_SUCCESS or HIDH_STATUS_ERROR @endlink
 */
hidh_le_status_t hidh_le_gattc_set_report(hidh_le_dev_t *p_dev,
        hidh_le_report_type_t report_type, uint8_t report_id, uint8_t *p_data,
        uint16_t length);

/**
 * @brief     Application calls this function to Set an HID Report
 *
 * @param[in] p_dev         : pointer to the device
 * @param[in] report_type   : report type
 * @param[in] report_id     : report id
 * @param[in] p_data        : data pointer
 *
 * @return    @hidh_le_status_t HIDH_STATUS_SUCCESS or HIDH_STATUS_ERROR @endlink
 */
hidh_le_status_t hidh_le_gattc_get_report(hidh_le_dev_t *p_dev,
        hidh_le_report_type_t report_type, uint8_t report_id, uint16_t length);

/**
 * @brief     Application calls this function to Set Protocol
 *
 * @param[in] p_dev     : pointer to the device
 * @param[in] protocol  : protocol
 *
 * @return    @hidh_le_status_t HIDH_STATUS_SUCCESS or HIDH_STATUS_ERROR @endlink
 */
hidh_le_status_t hidh_le_gattc_set_protocol(hidh_le_dev_t *p_dev,
        hidh_le_protocol_t protocol);

/**
 * @brief     Gatt discovery result
 *
 * @param[in] hidh_le_dev_t *p_dev                       : pointer to the device
 * @param[in] wiced_bt_gatt_discovery_result_t *p_result : discovery result data
 *
 * @return    @wiced_bt_gatt_status_t WICED_BT_GATT_SUCCESS or GATT_ERROR @endlink
 */
wiced_bt_gatt_status_t hidh_le_gattc_discovery_result(hidh_le_dev_t *p_dev,
        wiced_bt_gatt_discovery_result_t *p_discovery_result);

/**
 * @brief     Gatt discovery complete
 *
 * @param[in] p_dev                : pointer to the device
 * @param[in] p_discovery_complete : discovery complete data
 *
 * @return    @wiced_bt_gatt_status_t WICED_BT_GATT_SUCCESS or GATT_ERROR @endlink
 */
wiced_bt_gatt_status_t hidh_le_gattc_discovery_complete(hidh_le_dev_t *p_dev,
        wiced_bt_gatt_discovery_complete_t *p_discovery_complete);

/**
 * @brief     Gatt operation complete
 *
 * @param[in] p_dev                              : pointer to the device
 * @param[in] wiced_bt_gatt_operation_complete_t : operation complete data
 *
 * @return    @wiced_bt_gatt_status_t WICED_BT_GATT_SUCCESS or GATT_ERROR @endlink
 */
wiced_bt_gatt_status_t hidh_le_gattc_operation_complete(hidh_le_dev_t *p_dev,
        wiced_bt_gatt_operation_complete_t *p_operation_complete);

/**
 * @brief     Debug function which dump all the GATT information retrieved from a LE HID Device
 *
 * @param[in] p_dev : pointer to the device
 *
 * @return    Nothing.
 */
void hidh_le_gattc_dump(hidh_le_dev_t *p_dev);

#endif // __HIDH_LIB_GATTC_H_

/* end of file */
