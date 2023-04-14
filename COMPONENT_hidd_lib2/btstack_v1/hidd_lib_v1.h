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

/*******************************************************************************
*
* File Name: btstack_lib.h
*
* Abstract: btstack_lib API functions
*
* Functions:
*
*******************************************************************************/
#ifndef __HIDD_LIB_V1_H__
#define __HIDD_LIB_V1_H__

#include "wiced_timer.h"
#include "wiced_sleep.h"
#include "wiced_bt_cfg.h"
#include "wiced_hidd_lib.h"
#include "wiced_hal_mia.h"
#include "emconinfo.h"

/******************************************************
 *               Macro Function Definitions
 ******************************************************/
#define hidd_cfg_sec_mask() ( hidd_cfg()->security_requirement_mask )
#define hidd_cfg_p_scan()  ( &hidd_cfg()->ble_scan_cfg )
#define hidd_cfg_mtu()      ( hidd_cfg()->gatt_cfg.max_mtu_size )
#define hidd_transport_send_hci_trace( type, data, len ) wiced_transport_send_hci_trace( NULL, type, len, data )
#define hidd_buf_pool_is_sufficient() (wiced_bt_buffer_poolutilization (HCI_ACL_POOL_ID) < 80)
#define BTM_LE_AUTH_REQ_SC BTM_LE_AUTH_REQ_SC_ONLY


/******************************************************
 * typedef
 ******************************************************/
typedef wiced_result_t (app_start_callback_t)(void);
typedef struct
{
    uint8_t pairing_type;

    wiced_bt_cfg_settings_t * bt_cfg_ptr;
    wiced_bt_management_cback_t * app_management_cback_ptr;
    app_start_callback_t * app_init_ptr;
    const wiced_bt_cfg_buf_pool_t * bt_cfg_buf_pools_ptr;

} hidd_lib_t;

/******************************************************
 * extern variables
 ******************************************************/
extern hidd_lib_t hidd;

/******************************************************
 * functions
 ******************************************************/
void hidd_register_cfg_buf_pools( const wiced_bt_cfg_buf_pool_t * pool_cfg_p );
wiced_result_t hidd_stack_init( wiced_bt_management_cback_t * hidd_bt_management_ptr );
void hidd_enable_interrupt(wiced_bool_t en);

#endif // __HIDD_LIB_V1_H__
