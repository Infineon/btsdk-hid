/*
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
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

/**
 * @file hidd_lib.h
 *
 * @brief Bluetooth Low Energy (LE) HID Device (HIDD) Functions
 */

#ifndef __HIDD_LIB_H_
#define __HIDD_LIB_H_

#include "wiced_bt_types.h"
#include "wiced_bt_hidd.h"
#include "wiced_bt_gatt.h"
#include "cycfg_gatt_db.h"
#include "gatt_utils_lib.h"

/*********************************************************************************
 * typedefs
 ********************************************************************************/

/**
 *  @brief:  Gatt HIDD Report Table
 */
typedef struct
{
    uint8_t     rpt_id;         /**< HID Report ID */
    uint8_t     rpt_type;       /**< HID Report type. (HID_PAR_REP_TYPE_INPUT, HID_PAR_REP_TYPE_OUTPUT, or HID_PAR_REP_TYPE_FEATURE) */
    uint16_t    handle_cccd;    /**< The handle for CCCD flags */
    uint16_t    handle_val;     /**< The handle for report value */

} hidd_rpt_t;

/**
 *  @brief:  hidd report callback function pointer type
 */
typedef wiced_bt_gatt_status_t (* hidd_rpt_cb_t)( uint8_t rpt_id, uint8_t type, uint16_t conn_id, wiced_bt_gatt_write_req_t * p_wr_data );

/**
 *  @brief:  hidd report CCCD write callback function pointer type
 */
typedef wiced_bt_gatt_status_t (* hidd_cccd_write_cb_t)( uint16_t conn_id, wiced_bt_gatt_write_req_t * p_wr_data );

/**
 *  @brief:  hidd configuration type.
 */
typedef struct
{
    uint8_t                     rpt_table_size;     /**< The number of HID Report                   */
    hidd_rpt_t *                rpt_table;          /**< HID Report table                           */
    hidd_rpt_cb_t               rpt_cb;             /**< get_Report/set_report callback function    */
    hidd_cccd_write_cb_t        cccd_cb;            /**< cccd flag change callback function         */
    gatt_db_lookup_table_t *    gatt_lookup_table;  /**< GATT attribute lookup table                */

} hidd_cfg_t;

/*********************************************************************************
 * functions
 ********************************************************************************/

/**
 * @brief     Send HIDD input report by passing the report pointer and report length.
 *
 * @param[in] p_data  : A pointer points to a HID report. The first byte of HID report contains report id.
 * @param[in] len     : HID report length
 *
 * @retval    WICED_BT_GATT_SUCCESS: OK
 * @retval    Other values, see wiced_bt_gatt_status_e for the error code.
 */
wiced_bt_gatt_status_t  hidd_send_report(void * rpt, uint16_t len);

/**
 * @brief     Send HIDD report by providing report id, report type, report data pointer, and data length.
 *
 * @param[in] rpt_id   : report id
 * @param[in] rpt_type : HID report type
 * @param[in] p_data   : a pointer to data after report id
 * @param[in] len      : data length (not including report id)
 *
 * @retval    WICED_BT_GATT_SUCCESS: OK
 * @retval    Other values, see wiced_bt_gatt_status_e for the error code.
 */
wiced_bt_gatt_status_t  hidd_send_data(uint8_t rpt_id, uint8_t rpt_type, uint8_t * p_data, uint16_t len);

/**
 * @brief     This function is called when GATT write handle is HID handle
 *
 * @param[in] conn_id   : Connection ID
 * @param[in] p_wr_data : Pointer to the gatt_write data
 *
 * @retval    WICED_BT_GATT_SUCCESS: OK
 * @retval    Other values, see wiced_bt_gatt_status_e for the error code.
 */
wiced_bt_gatt_status_t  hidd_gatt_write_handler( uint16_t conn_id, wiced_bt_gatt_write_req_t * p_wr_data );

/**
 * @brief      This function gathers all report's CCCD flags saved in given data pointers
 *
 * @param[out] nflags  : Pointer to Notification flags variable
 * @param[out] iflags  : Pointer to Indication flags variable
 *
 * @return     Nothing.
 */
void hidd_get_cccd_flags(uint16_t * nflags, uint16_t * iflags);

/**
 * @brief     This function go through report table to set notification and indications flags to each handle
 *
 * @param[in] uint16_t nflags : Notification flags
 *            uint16_t iflags : Indication flags
 *
 * @return    Nothing.
 */
void hidd_set_cccd_flags(uint16_t notif_flags, uint16_t indicate_flags);

/**
 * @brief  This function clears all report's CCCD flags in each handle
 *
 * @param  Nothing.
 *
 * @return Nothing.
 */
void hidd_clear_cccd_flags();

/**
 * @brief     When link state changes, this function should be called to set the connection id.
 *            A value 0 for connection id indicates the link is down.
 *
 * @param[in] conn_id  : connection id
 *
 * @return    Nothing.
 */
void hidd_set_conn_id( uint16_t conn_id );

/**
 * @brief     This is the first function to initialize this lib
 *
 * @param[in] hidd_cfg_t* cfg  : hidd configuration
 *
 * @return    none
 */
void hidd_init(hidd_cfg_t * cfg);

#endif // __HIDD_LIB_H_

/* end of file */
