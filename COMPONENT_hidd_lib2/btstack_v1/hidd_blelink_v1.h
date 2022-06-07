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
/********************************************************************************
*
* File Name: hidd_blelink.h
*
* Abstract: This file implements the BLE HID application transport
*
* Functions:
*
*******************************************************************************/
#ifndef __HIDD_BLELINK_V1__
#define __HIDD_BLELINK_V1__

#include "spar_utils.h"
#include "wiced_hal_mia.h"
#include "wiced_timer.h"
#include "clock_timer.h"

/*************************************************************************
* typedefs
**************************************************************************/
typedef OSAPI_TIMER hidd_blelink_timer_t;

/*************************************************************************
* defines
**************************************************************************/
#define hidd_blelink_stop_timer(t) osapi_deactivateTimer(t)
#define hidd_blelink_start_timer(t, d) osapi_activateTimer(t, d * 1000UL)
#define hidd_blelink_is_timer_in_use(t)osapi_is_timer_running(t)
#define hidd_blelink_init_timer(t, cb, p) osapi_createTimer(t, cb, p)

/*************************************************************************
* functions
**************************************************************************/
void hidd_blelink_determine_next_state_on_wake_from_SDS(void);
void hidd_blelink_connectionIdle_timerCb(INT32 args, UINT32 overTimeInUs);
void hidd_blelink_pr_link_key(wiced_bt_device_link_keys_t * p_link_key);

#endif // __HIDD_BLELINK_V1__
