/*
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
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

/*******************************************************************************
*
* File Name: dualhidlink.h
*
* Abstract: DUAL (BT/BLE) Host List definitions and API functions
*
* Functions:
*
*******************************************************************************/
#ifndef __HIDD_LIB_H__
#define __HIDD_LIB_H__

#include "wiced_bt_cfg.h"
#include "blehidgatts.h"
#include "blehidlink.h"
#include "bthidlink.h"
#include "hidd_hci.h"
#include "hidd_host.h"

#ifdef WICED_BT_TRACE_ENABLE
 void trace_array(void * ptr, uint32_t len);
 void strace_array(char * str, void * ptr, uint32_t len);
 #define TRACE_ARRAY(ptr, len) trace_array(ptr, len)
 #define STRACE_ARRAY(str, ptr, len) strace_array(str, ptr, len)
#else
 #define TRACE_ARRAY(ptr, len)
 #define STRACE_ARRAY(str, ptr, len)
#endif

#define is_208xxFamily ((CHIP==20819) || (CHIP==20820))
#define is_20739Family ((CHIP==20739) || (CHIP==20719))
#define is_20735Family (CHIP==20735)
#define is_SDS_capable (is_20735Family || is_20739Family)
#define is_ePDS_capable is_208xxFamily
#define is_newFamily (is_20735Family || is_20739Family || is_208xxFamily)

typedef struct{
    uint16_t     handle;
    uint16_t     attr_len;
    const void * p_attr;
}attribute_t;

void blehidlink_allowDiscoverable(void);
void blehidlink_setState(uint8_t newState);

#define WICED_RESUME_HIDD_LIB_HANDLER WICED_NOT_FOUND
/******************************************************************************************/
// Gatts functions /////////////////////////////////////////////////////////////////////////
/******************************************************************************************/
wiced_bt_gatt_status_t wiced_hidd_gatts_init(const uint8_t * gatt_db, uint16_t len, const attribute_t * gAttrib, uint16_t gAttrib_len, blehid_gatts_req_read_callback_t rd_cb, blehid_gatts_req_write_callback_t wr_cb);

/******************************************************************************************/
// Host functions //////////////////////////////////////////////////////////////////////////
/******************************************************************************************/

///////////////////////////////////////////////////////////////////////////////
/// Return current host count
///        returns WICED_TRUE if a host is paired
///////////////////////////////////////////////////////////////////////////////
uint8_t wiced_hidd_host_count();
#define wiced_hidd_is_paired() wiced_hidd_host_count()

///////////////////////////////////////////////////////////////////////////////
/// Return true if host already in the list
///////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_hidd_host_exist(const BD_ADDR host_bd_addr);

///////////////////////////////////////////////////////////////////////////////
/// Return current transport type
///
///        returns BT_TRANSPORT_LE
///                BT_TRANSPORT_BR_EDR
///                or 0 if not paired
///
///////////////////////////////////////////////////////////////////////////////
wiced_bt_transport_t wiced_hidd_host_transport();

///////////////////////////////////////////////////////////////////////////////
/// Return wiced_hidd_host_addr
///
///        returns host address, NULL if not paired
///
///////////////////////////////////////////////////////////////////////////////
uint8_t * wiced_hidd_host_addr();

///////////////////////////////////////////////////////////////////////////////
/// wiced_hidd_host_remove()
///////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_hidd_host_remove(void);

///////////////////////////////////////////////////////////////////////////////
/// wiced_hidd_host_remove()
///////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_hidd_host_remove_addr(wiced_bt_device_address_t host_bd_addr);

///////////////////////////////////////////////////////////////////////////////
/// wiced_hidd_host_set_flags set host flags
///
/// \param flgas
///
///////////////////////////////////////////////////////////////////////////////
uint16_t wiced_hidd_host_set_flags(const BD_ADDR bdAddr, uint16_t enable, uint16_t flags);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
uint16_t wiced_hidd_host_get_flags();

///////////////////////////////////////////////////////////////////////////////
/// wiced_hidd_host_remove()
///////////////////////////////////////////////////////////////////////////////
void wiced_hidd_host_remove_all(void);

///////////////////////////////////////////////////////////////////////////////
/// Returns host addr type
///
///    returns If paired host is LE, return address type, otherwise return 0
///
///////////////////////////////////////////////////////////////////////////////
uint8_t wiced_hidd_host_addr_type();

/******************************************************************************************/
// Link functions //////////////////////////////////////////////////////////////////////////
/******************************************************************************************/

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void wiced_hidd_link_virtual_cable_unplug(void);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void wiced_hidd_pairing_stopped(wiced_bt_transport_t transport);

///////////////////////////////////////////////////////////////////////////////
/// wiced_hidd_enter_pairing(BOOLEAN)
///////////////////////////////////////////////////////////////////////////////
void wiced_hidd_enter_pairing();

///////////////////////////////////////////////////////////////////////////////
/// wiced_hidd_link_connect()
///////////////////////////////////////////////////////////////////////////////
void wiced_hidd_link_connect();

///////////////////////////////////////////////////////////////////////////////
/// wiced_hidd_disconnect()
///////////////////////////////////////////////////////////////////////////////
void wiced_hidd_disconnect();

///////////////////////////////////////////////////////////////////////////////
/// returns corrent link state
///////////////////////////////////////////////////////////////////////////////
uint8_t wiced_hidd_link_state();

///////////////////////////////////////////////////////////////////////////////
/// returns if HID link is connected
///////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_hidd_link_is_connected();

///////////////////////////////////////////////////////////////////////////////
/// returns if HID link is disconnected
///////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_hidd_link_is_disconnected();

///////////////////////////////////////////////////////////////////////////////
/// returns if HID link is connected
///////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_hidd_link_is_encrypted();

///////////////////////////////////////////////////////////////////////////////
/// wiced_hidd_link_init()
///////////////////////////////////////////////////////////////////////////////
void wiced_hidd_link_init();


/******************************************************************************************/
// Sleep functions /////////////////////////////////////////////////////////////////////////
/******************************************************************************************/

///////////////////////////////////////////////////////////////////////////////
/// This function disallow sleep for given period of time in milliseconds.
///////////////////////////////////////////////////////////////////////////////
void wiced_hidd_deep_sleep_not_allowed(uint32_t milliseconds);

////////////////////////////////////////////////////////////////////////////////
/// This function allows device to sleep
////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_set_deep_sleep_allowed(uint8_t allowed);

////////////////////////////////////////////////////////////////////////////////
/// This function returns if allow to sleep timer is running
////////////////////////////////////////////////////////////////////////////////
uint8_t wiced_hidd_is_deep_sleep_timer_running();

////////////////////////////////////////////////////////////////////////////////
/// This function returns if sleep is allowed
////////////////////////////////////////////////////////////////////////////////
uint8_t wiced_hidd_is_deep_sleep_allowed();

////////////////////////////////////////////////////////////////////////////////
/// register application sleep permit handler
///
/// \param cb - pointer to application callback function
////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_link_register_sleep_permit_handler(wiced_sleep_allow_check_callback sleep_handler);

/******************************************************************************************/
// LED functions ///////////////////////////////////////////////////////////////////////////
/******************************************************************************************/
enum {
    LED_ERROR_CODE_NO_ERROR,   // 0
    LED_ERROR_CODE_GATTS,      // 1 LE GATTS database init error
    LED_ERROR_CODE_SDP_DB,     // 2 BR/EDR SDP Database init error
};

#if LED_SUPPORT
 #if LED_SUPPORT==2
  #define LED_USE_PWM
 #endif
void wiced_hidd_led_init(uint8_t gpio, uint8_t off_level);
void wiced_hidd_led_on(uint8_t gpio);
void wiced_hidd_led_off(uint8_t gpio);

// The current design only allow one LED to blink at one time. (They shares the same timer)
void wiced_hidd_led_blink(uint8_t gpio, uint32_t count, uint32_t how_fast_in_ms);
void wiced_hidd_led_blink_stop();
void wiced_hidd_led_blink_error(uint8_t gpio, uint8_t error_code);

 #ifdef LED_USE_PWM
void wiced_hidd_pwm_led_init(uint8_t gpio, uint8_t off_level);
void wiced_hidd_pwm_led_on(uint8_t gpio, uint8_t percent);
void wiced_hidd_pwm_led_off(uint8_t gpio);
 #endif
#else
 #define wiced_hidd_led_init(gpio,  off_level)
 #define wiced_hidd_led_on(gpio)
 #define wiced_hidd_led_off(gpio)
 #define wiced_hidd_led_blink(gpio, count, how_fast_in_ms)
 #define wiced_hidd_led_blink_stop()
 #define wiced_hidd_led_blink_error(gpio, error_code)
 #define wiced_hidd_pwm_led_init(gpio, off_level)
 #define wiced_hidd_pwm_led_on(gpio, percent)
 #define wiced_hidd_pwm_led_off(gpio)
#endif
/******************************************************************************************/
// Generic functions ///////////////////////////////////////////////////////////////////////
/******************************************************************************************/

////////////////////////////////////////////////////////////////////////////////
// returns chip number
////////////////////////////////////////////////////////////////////////////////
uint32_t wiced_hidd_chip();

////////////////////////////////////////////////////////////////////////////////
// returns hidd configuraion pointer
////////////////////////////////////////////////////////////////////////////////
const wiced_bt_cfg_settings_t * wiced_hidd_cfg();

////////////////////////////////////////////////////////////////////////////////
/// wiced_ble_hidd_start
///
/// \param p_bt_management_cback  - application bt_management callback function
///        p_bt_cfg_settings      - bt configuration setting
///        wiced_bt_cfg_buf_pools - buffer pool configuration
////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_start(wiced_result_t (*p_bt_app_init)(),
                      wiced_bt_management_cback_t   * p_bt_management_cback,
                      const wiced_bt_cfg_settings_t * p_bt_cfg_settings,
                      const wiced_bt_cfg_buf_pool_t * p_bt_cfg_buf_pools);


////////////////////////////////////////////////////////////////////////////////
// wiced_hidd_allowed_hidoff
////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_allowed_hidoff(wiced_bool_t en);

////////////////////////////////////////////////////////////////////////////////
// wiced_hidd_activity_detected
////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_activity_detected();


#endif
