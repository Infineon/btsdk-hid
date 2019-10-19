/*
 * Copyright 2019, Cypress Semiconductor Corporation or a subsidiary of
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

#ifndef _BLE_HID_GATTS_H__
#define _BLE_HID_GATTS_H__

#include "wiced_bt_gatt.h"

/******************************************************************************
 *                         Type Definitions
 ******************************************************************************/

typedef struct{
    uint16_t     handle;
    uint16_t     attr_len;
    const void * p_attr;
}attribute_t;

#ifdef OTA_FIRMWARE_UPGRADE
typedef void (*blehid_ota_fw_upgrade_status_callback_t)(uint8_t status);
void blehid_register_ota_fw_upgrade_status_callback(blehid_ota_fw_upgrade_status_callback_t);
#endif

typedef wiced_bt_gatt_status_t (*blehid_gatts_req_read_callback_t)( uint16_t conn_id, wiced_bt_gatt_read_t * p_data );
typedef wiced_bt_gatt_status_t (*blehid_gatts_req_write_callback_t)( uint16_t conn_id, wiced_bt_gatt_write_t * p_data );
#ifdef CONNECTED_ADVERTISING_SUPPORTED
typedef void (*wiced_bt_gatt_2nd_link_up_handler_t)( wiced_bt_gatt_connection_status_t *p_status );
extern wiced_bt_gatt_2nd_link_up_handler_t wiced_bt_gatt_2nd_link_up_handler;
#endif

wiced_bt_gatt_status_t blehid_gatts_init(const uint8_t * gatt_db, uint16_t len, blehid_gatts_req_read_callback_t rd_cb, blehid_gatts_req_write_callback_t wr_cb);

#endif //_BLE_HID_GATTS_H__
