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

/*******************************************************************************
*
* File Name: hidd_lib.h
*
* Abstract: hidd_lib API functions
*
* Functions:
*
*******************************************************************************/
#ifndef __HIDD_LIB_H__
#define __HIDD_LIB_H__

#include "wiced_bt_types.h"
#include "wiced_hal_nvram.h"
#include "wiced_bt_trace.h"

/////////////////////////////////////////////////////////////////////////////////
// defines
/////////////////////////////////////////////////////////////////////////////////

#define is_20819Family ((CHIP==20819) || (CHIP==20820))
#define is_20739Family ((CHIP==20739) || (CHIP==20719) || (CHIP==20721) || (CHIP==30739))
#define is_20735Family ((CHIP==20735) || (CHIP==20835))
#define is_SDS_capable (is_20735Family || is_20739Family)
#define is_ePDS_capable is_20819Family
#define is_newFamily (is_20735Family || is_20739Family || is_20819Family)

#define BT_TRANSPORT_DUAL (BT_TRANSPORT_BR_EDR | BT_TRANSPORT_LE)

#ifdef WICED_BT_TRACE_ENABLE
 void trace_array(void * ptr, uint32_t len);
 void strace_array(char * str, void * ptr, uint32_t len);
 #define TRACE_ARRAY(ptr, len) trace_array(ptr, len)
 #define STRACE_ARRAY(str, ptr, len) strace_array(str, ptr, len)
#else
 #define TRACE_ARRAY(ptr, len)
 #define STRACE_ARRAY(str, ptr, len)
#endif

#define WICED_RESUME_HIDD_LIB_HANDLER WICED_NOT_FOUND

#if is_20735Family
 #define SFI_DEEP_SLEEP 1
#else
 #define SFI_DEEP_SLEEP 0
#endif

#if SFI_DEEP_SLEEP
 extern uint8_t pmu_attemptSleepState;
 extern void sfi_enter_deep_power_down(void);
 extern void sfi_exit_deep_power_down(BOOL8 forceExitDeepPowerDown);
 extern void sfi_allow_deep_sleep(void);
 #define hidd_nvram_allow_deep_sleep() sfi_allow_deep_sleep();
 #define hidd_nvram_enter_deep_sleep() sfi_enter_deep_power_down()
 #define hidd_nvram_exit_deep_sleep() sfi_exit_deep_power_down(TRUE)
 #define hidd_nvram_deep_sleep() {sfi_allow_deep_sleep(); sfi_enter_deep_power_down();}
#else
 #define hidd_nvram_allow_deep_sleep()
 #define hidd_nvram_enter_deep_sleep()
 #define hidd_nvram_exit_deep_sleep()
 #define hidd_nvram_deep_sleep()
#endif

/////////////////////////////////////////////////////////////////////////////////
// NVRAM ID defines
/////////////////////////////////////////////////////////////////////////////////
enum {
    VS_ID_LOCAL_IDENTITY = WICED_NVRAM_VSID_START,
    VS_ID_HIDD_HOST_LIST,
    VS_ID_GFPS_ACCOUNT_KEY,
};

/////////////////////////////////////////////////////////////////////////////////
// Include libraries submodules
/////////////////////////////////////////////////////////////////////////////////
#if BTSTACK_VER < 0x03000001
#include "hidd_lib_v1.h"
#else
#include "hidd_lib_v3.h"
#endif
#include "hidd_gatt.h"
#include "hidd_link.h"
#include "hidd_hci.h"
#include "hidd_host.h"
#include "hidd_audio.h"
#include "hidd_led.h"
#include "hidd_sleep.h"

/////////////////////////////////////////////////////////////////////////////////
// Functions
/////////////////////////////////////////////////////////////////////////////////

/******************************************************************************************
 * hidd_chip_id
 *
 * returns chip number
 ******************************************************************************************/
uint32_t hidd_chip_id();

///////////////////////////////////////////////////////////////////////////////
/// hidd_enter_pairing() -- force to enter paging
///////////////////////////////////////////////////////////////////////////////
void hidd_enter_pairing();

///////////////////////////////////////////////////////////////////////////////
/// hidd_pairing
///  If link is connected, it disables link and enter pairing.
///  If link is already in discovery:
///     If in BR/EDR discovery --> enters LE discovery
///     If in LE discovery --> stops pairing.
///  If link is disconnected --> do BR/EDR pairing if supported. Otherwise, do LE pairing
///////////////////////////////////////////////////////////////////////////////
void hidd_pairing();

////////////////////////////////////////////////////////////////////////////////
// returns hidd configuraion pointer
////////////////////////////////////////////////////////////////////////////////
wiced_bt_cfg_settings_t * hidd_cfg();

////////////////////////////////////////////////////////////////////////////////
/// hidd_start_v
///
/// \param p_bt_app_init          - pointer to application init function
///        p_bt_management_cback  - poniter to application bt_management callback function
///        p_bt_cfg_settings      - bt configuration setting
////////////////////////////////////////////////////////////////////////////////
void hidd_start_v(app_start_callback_t * p_bt_app_init,
                wiced_bt_management_cback_t   * p_bt_management_cback,
                wiced_bt_cfg_settings_t * p_bt_cfg_settings);

/**
 * @return : true if any activity is detected
 */
void hidd_activity_detected();

/**
 * Writes the data to NVRAM,
 * Application can write up to 255 bytes in one VS  id section
 *
 * @param[in] vs_id        : Volatile Section Identifier. Application can use
 *                           the VS ids from WICED_NVRAM_VSID_START to
 *                           WICED_NVRAM_VSID_END
 *
 * @param[in] data_length  : Length of the data to be written to the NVRAM,
 *
 * @param[in] p_data       : Pointer to the data to be written to the NVRAM
 *
 * @param[out] p_status    : Pointer to location where status of the call
 *                           is returned
 *
 *
 * @return : number of bytes written, 0 on error
 */
uint16_t hidd_write_nvram( uint16_t vs_id, uint16_t data_length, uint8_t * p_data, wiced_result_t * p_status);

/** Reads the data from NVRAM
 *
 * @param[in]  vs_id       : Volatile Section Identifier. Application can use
 *                           the VS ids from WICED_NVRAM_VSID_START to
 *                           WICED_NVRAM_VSID_END
 *
 * @param[in]  data_length : Length of the data to be read from NVRAM
 *
 * @param[out] p_data      : Pointer to the buffer to which data will be copied
 *
 * @param[out] p_status    : Pointer to location where status of the call
 *                           is returned
 *
 * @return  the number of bytes read, 0 on failure
 */
uint16_t hidd_read_nvram( uint16_t vs_id, uint16_t data_length, uint8_t * p_data, wiced_result_t * p_status);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
#ifdef FASTPAIR_ENABLE
 #define hidd_start_advertisements(ad,type,adr) hidd_gfps_discoverablility_set(ad)
#else
 #define hidd_start_advertisements(ad,type,adr) wiced_bt_start_advertisements(ad,type,adr)
#endif


////////////////////////////////////////////////////////////////////////////////
// For backward compatiblity
#define app_poll_callback_t hidd_link_app_poll_callback_t
#define hidd_register_app_callback hidd_link_register_callbacks
#define hci_control_register_key_handler hidd_hci_control_register_key_handler
#define hidd_start(init, cb, cfg, pool) {hidd_register_cfg_buf_pools(pool); hidd_start_v(init, cb, cfg);}
#define hidd_gatts_init hidd_gatt_init

#endif
