/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
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
#include "wiced_hidd_lib.h"
#ifdef FASTPAIR_ENABLE
#include "wiced_bt_gfps.h"
#endif

typedef struct{
    uint16_t    handle;
    uint16_t    attr_len;
    void *      p_attr;
}attribute_t;

/******************************************************************************
 *                         Type Definitions
 ******************************************************************************/

#ifdef OTA_FIRMWARE_UPGRADE
typedef void (*blehid_ota_fw_upgrade_status_callback_t)(uint8_t status);
void blehid_register_ota_fw_upgrade_status_callback(blehid_ota_fw_upgrade_status_callback_t);
#endif

typedef wiced_bt_gatt_status_t (*hidd_gatts_req_read_callback_t)( uint16_t conn_id, wiced_bt_gatt_read_t * p_data );
typedef wiced_bt_gatt_status_t (*hidd_gatts_req_write_callback_t)( uint16_t conn_id, wiced_bt_gatt_write_t * p_data );

wiced_bool_t hidd_gatts_set_data(uint8_t * ptr, uint16_t len);

/******************************************************************************************/
// Gatts functions /////////////////////////////////////////////////////////////////////////
/******************************************************************************************/
wiced_bt_gatt_status_t hidd_gatts_init(wiced_blehidd_report_gatt_characteristic_t* rptTable, uint16_t rptTableNum,
                                       uint8_t * gatt_db, uint16_t gatt_db_len,
                                       attribute_t * gAttrib, uint16_t gAttrib_len,
                                       hidd_gatts_req_read_callback_t rd_cb, hidd_gatts_req_write_callback_t wr_cb);

/******************************************************************************************/
/******************************************************************************************/
#ifdef FASTPAIR_ENABLE
wiced_bool_t hidd_gatts_gfps_init(wiced_bt_gfps_provider_conf_t * fastpair_conf);
#endif

#endif //_BLE_HID_GATTS_H__
