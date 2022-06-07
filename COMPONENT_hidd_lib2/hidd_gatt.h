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

#ifndef _HIDD_GATTS_H__
#define _HIDD_GATTS_H__

#include "wiced_bt_gatt.h"
#ifdef FASTPAIR_ENABLE
#include "wiced_bt_gfps.h"
#endif
#if BTSTACK_VER < 0x03000001
#include "hidd_gatt_v1.h"
#else
#include "hidd_gatt_v3.h"
#endif
/******************************************************************************
 *                         Type Definitions
 ******************************************************************************/
typedef struct{
    uint16_t    handle;
    uint16_t    attr_len;
    void *      p_attr;
}attribute_t;

typedef struct {
    attribute_t * gattAttributes;
    uint16_t gattAttributes_size;
    hidd_gatt_req_read_callback_t hidd_gatt_req_read_callback;
    hidd_gatt_req_write_callback_t hidd_gatt_req_write_callback;
    wiced_blehidd_report_gatt_characteristic_t* rptTable;
    uint16_t rptTableNum;
} gatt_t;

#ifdef OTA_FIRMWARE_UPGRADE
typedef void (*blehid_ota_fw_upgrade_status_callback_t)(uint8_t status);
void blehid_register_ota_fw_upgrade_status_callback(blehid_ota_fw_upgrade_status_callback_t);
#endif

extern gatt_t gatt;

/******************************************************************************************/
// Gatts functions /////////////////////////////////////////////////////////////////////////
/******************************************************************************************/
wiced_bt_gatt_status_t hidd_gatt_req_conf_handler( uint16_t conn_id, uint16_t handle );
wiced_bt_gatt_status_t hidd_gatt_write_handler( uint16_t conn_id, hidd_gatt_write_t * p_data );
const attribute_t * hidd_gatt_get_attribute( uint16_t handle );
wiced_bool_t hidd_gatt_set_data( uint8_t * ptr, uint16_t len );
wiced_bt_gatt_status_t hidd_gatt_init( wiced_blehidd_report_gatt_characteristic_t* rptTable, uint16_t rptTableNum,
                                        uint8_t * gatt_db, uint16_t gatt_db_len,
                                        attribute_t * gAttrib, uint16_t gAttrib_len,
                                        hidd_gatt_req_read_callback_t rd_cb, hidd_gatt_req_write_callback_t wr_cb );

/******************************************************************************************/
/******************************************************************************************/
#ifdef FASTPAIR_ENABLE
wiced_bool_t hidd_gatt_gfps_init(wiced_bt_gfps_provider_conf_t * fastpair_conf);
#endif

#endif //_BLE_HID_GATTS_H__
