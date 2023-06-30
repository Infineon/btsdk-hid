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

/********************************************************************************
*
* File Name: hidd_lib.c
*
* Abstract: This file implements HID application transport that supports both the Bluetooth(BT) Classic
*               and LE
* Functions:
*
*******************************************************************************/
#if BTSTACK_VER < 0x03000001
#include "hidd_lib.h"
#include "wiced_bt_stack.h"
#include "wiced_hal_mia.h"
#include "wiced_bt_trace.h"

hidd_lib_t hidd = {0};

////////////////////////////////////////////////////////////////////////////////
/// is_cfg_buf_pools_valid()
///
/// returns true if cfg_buf is assigned
///
////////////////////////////////////////////////////////////////////////////////
static wiced_bool_t buf_pools_cfg_valid(void)
{
    return hidd.bt_cfg_buf_pools_ptr != NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// hidd_register_cfg_buf_pools
///
/// \param pool_cfg_p -- pool configuration pointer
///
////////////////////////////////////////////////////////////////////////////////
void hidd_register_cfg_buf_pools(const wiced_bt_cfg_buf_pool_t * pool_cfg_p)
{
    hidd.bt_cfg_buf_pools_ptr = pool_cfg_p;
}

////////////////////////////////////////////////////////////////////////////////
/// hidd_stack_init
////////////////////////////////////////////////////////////////////////////////
wiced_result_t hidd_stack_init(wiced_bt_management_cback_t * hidd_bt_management_ptr)
{
#if is_SDS_capable
    if (!wiced_hal_mia_is_reset_reason_por())
    {
        hidd_link_aon_action_handler(HIDD_LINK_RESTORE_FROM_AON);
    }
#endif

#if is_20819Family
    // For 208xx, the chip id is identified by Radio id register; however, the register may get disabled after entering ePDS;
    // therefore, we read it once at power up and save the id.
    hidd_chip_id();
#endif

    if (buf_pools_cfg_valid())
    {
        return wiced_bt_stack_init(hidd_bt_management_ptr, hidd.bt_cfg_ptr, hidd.bt_cfg_buf_pools_ptr);
    }

    WICED_BT_TRACE("\nBuffer pool is not configured\n");
    return WICED_ERROR;
}

void hidd_enable_interrupt(wiced_bool_t en)
{
    wiced_hal_mia_enable_mia_interrupt(en);
    wiced_hal_mia_enable_lhl_interrupt(en);
}

#endif // #if BTSTACK_VER < 0x03000001
