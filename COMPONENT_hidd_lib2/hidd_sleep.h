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
* File Name: hidd_lib.h
*
* Abstract: hidd_lib API functions
*
* Functions:
*
*******************************************************************************/
#ifndef __HIDD_SLEEP_H__
#define __HIDD_SLEEP_H__

#include "wiced_sleep.h"

/////////////////////////////////////////////////////////////////////////////////
// defines
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
// Functions
/////////////////////////////////////////////////////////////////////////////////

/******************************************************************************************/
/******************************************************************************************/
void hidd_sleep_configure(wiced_sleep_config_t * hidd_link_sleep_config);

/******************************************************************************************/
// Sleep functions /////////////////////////////////////////////////////////////////////////
/******************************************************************************************/

/**
 * @param[in] en  : TRUE to allow device to power off
 */
void hidd_allowed_hidoff(wiced_bool_t en);

///////////////////////////////////////////////////////////////////////////////
/// This function disallow sleep for given period of time in milliseconds.
///////////////////////////////////////////////////////////////////////////////
void hidd_deep_sleep_not_allowed(uint32_t milliseconds);

////////////////////////////////////////////////////////////////////////////////
/// This function allows device to sleep
////////////////////////////////////////////////////////////////////////////////
void hidd_set_deep_sleep_allowed(uint8_t allowed);

////////////////////////////////////////////////////////////////////////////////
/// This function returns if allow to sleep timer is running
////////////////////////////////////////////////////////////////////////////////
uint8_t hidd_is_deep_sleep_timer_running();

////////////////////////////////////////////////////////////////////////////////
/// This function returns if sleep is allowed
////////////////////////////////////////////////////////////////////////////////
uint8_t hidd_is_deep_sleep_allowed();

void hidd_sleep_init();

#endif
