/*
 * Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
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

/********************************************************************************
*
* File Name: dualhidlink.c
*
* Abstract: This file implements HID application transport that supports both the Bluetooth(BT) Classic
*               and LE
* Functions:
*
*******************************************************************************/
#include "hidd_lib.h"
#include "wiced_hal_batmon.h"
#include "wiced_hal_nvram.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_pwm.h"
#include "wiced_hal_aclk.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_ota_firmware_upgrade.h"
#include "wiced_bt_stack.h"
#include "wiced_hidd_lib.h"
#include "hci_control_api.h"
#include "bthidlink.h"

#define PMU_CONFIG_FLAGS_ENABLE_SDS           0x00002000
extern UINT32 g_foundation_config_PMUflags;

//Local identity key ID
#define  VS_LOCAL_IDENTITY_ID WICED_NVRAM_VSID_START

#define STOP_PAIRING  0
#define START_PAIRING 1

typedef struct
{
    // is shutdown sleep (SDS) allowed?
    uint8_t allowDeepSleep:1;
    uint8_t allowHIDOFF:1;

    /// allow SDS timer
    wiced_timer_t allowDeepSleepTimer;

    // application sleep handler pointer
    wiced_sleep_allow_check_callback registered_app_sleep_handler;

    const wiced_bt_cfg_settings_t * wiced_bt_hid_cfg_settings_ptr;

    wiced_bt_transport_t pairingMode;

} tHiddLink; tHiddLink hidd = {};

void blehidlink_init(void);
void blehidlink_determineNextState(void);
void blehidlink_enterDisconnected();
void blehidlink_enterDiscoverable(uint32_t);
void bthidlink_init(void);
void bthidlink_determineNextState(void);
void bthidlink_setState(uint8_t newState);

#if LED_SUPPORT
 #define ERROR_CODE_BLINK_SPEED_SLOW 500
 #define ERROR_CODE_BLINK_SPEED_FAST 250
 #define ERROR_CODE_BLINK_BREAK      4000
////////////////////////////////////////////////////////////////////////////////
// wiced_hidd_led_init initialize
////////////////////////////////////////////////////////////////////////////////
typedef struct
{
    uint64_t      off_level;
    uint64_t      state;

    wiced_timer_t blinking_timer;
    uint32_t      blinking_duration;
    uint32_t      blinking_count;
    uint8_t       blinking_error_code;

    uint8_t       blinking_gpio:6;
    uint8_t       blinking_state:1;
    uint8_t       blinking_off_level:1;

} tLED; tLED led={};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void led_blink_handler(uint32_t arg)
{
    // LED is on, we turn it off
    if (led.blinking_state)
    {
        led.blinking_state = 0;
        wiced_hal_gpio_set_pin_output(led.blinking_gpio, led.blinking_off_level);
        if (led.blinking_count)
        {
            // if counted to 0, don't start again
            if (!--led.blinking_count)
            {
                if (led.blinking_error_code)
                {
                    // blinking error code, reload counter
                    led.blinking_count = led.blinking_error_code;
                    // give a long break before start another error code blinking
                    wiced_start_timer(&led.blinking_timer, ERROR_CODE_BLINK_BREAK);
                }
                else // we are done with blinking, put LED back to what it suppose to be
                {
                    if (led.state & (1 << led.blinking_gpio))
                    {
                        wiced_hal_gpio_set_pin_output(led.blinking_gpio, !led.blinking_off_level);
                    }
                    else
                    {
                        wiced_hal_gpio_set_pin_output(led.blinking_gpio, led.blinking_off_level);
                    }
                }
                return;
            }
        }
    }
    else
    {
        wiced_hal_gpio_set_pin_output(led.blinking_gpio, !led.blinking_off_level);
        led.blinking_state = 1;
    }
    wiced_start_timer(&led.blinking_timer, led.blinking_duration);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_led_blink_stop()
{
    led.blinking_error_code = 0; // clear error code
    led.blinking_count = 1;      // get ready to stop
    led.blinking_state = 1;
}

////////////////////////////////////////////////////////////////////////////////
// pass count 0 to blink forever
////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_led_blink(uint8_t gpio, uint32_t count, uint32_t how_fast_in_ms)
{
    // if any LED is blinking, stop it
    if (wiced_is_timer_in_use(&led.blinking_timer))
    {
        wiced_hidd_led_blink_stop();
        led_blink_handler(0);
    }

    led.blinking_duration = how_fast_in_ms;
    led.blinking_count = count;
    led.blinking_gpio = gpio;
    led.blinking_state = 0;
    led.blinking_off_level = led.off_level & (1<<gpio) ? 1 : 0;
    led_blink_handler(0);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_led_blink_error(uint8_t gpio, uint8_t code)
{
    led.blinking_error_code = code;
    wiced_hidd_led_blink(gpio, code, ERROR_CODE_BLINK_SPEED_SLOW);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_led_init(uint8_t gpio, uint8_t off_level)
{
    uint64_t ledbit = ((uint64_t)1 << gpio);
    wiced_hal_gpio_configure_pin(gpio, GPIO_OUTPUT_ENABLE, off_level);
    wiced_hal_gpio_slimboot_reenforce_cfg(gpio, GPIO_OUTPUT_ENABLE);
    if (off_level)
        led.off_level |= ledbit;
    else
        led.off_level &= ~ledbit;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_led_on(uint8_t gpio)
{
    uint64_t ledbit = ((uint64_t)1 << gpio);
    uint8_t on = led.off_level & ledbit ? 0 : 1;
    led.state |= ledbit;
//    WICED_BT_TRACE("\nLED_on %d %d",gpio, on);
    wiced_hal_gpio_set_pin_output(gpio, on);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_led_off(uint8_t gpio)
{
    uint64_t ledbit = ((uint64_t)1 << gpio);
    uint8_t off = led.off_level & ledbit ? 1 : 0;
    led.state &= ~ledbit;
//    WICED_BT_TRACE("\nLED_off %d %d",gpio, off);
    wiced_hal_gpio_set_pin_output(gpio, off);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
 #ifdef LED_USE_PWM
  #define PWM_BASE 26
  #if is_newFamily
   #define PWM_MAX_COUNTER 0xffff   // 16 bit counter
  #else
   #define PWM_MAX_COUNTER 0x3ff   // 10 bit counter
  #endif

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_pwm_led_init(uint8_t gpio, uint8_t off_level)
{
    wiced_hidd_led_init(gpio, off_level);
    //wiced_hal_pwm_configure_pin(gpio, gpio - PWM_BASE); --- should be done in platform
    wiced_hal_aclk_enable(256000, ACLK1, ACLK_FREQ_24_MHZ);
    wiced_hidd_pwm_led_off(gpio);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// gpio needs to be changed to PWM specified in platform.c
void wiced_hidd_pwm_led_on(uint8_t gpio, uint8_t percent)
{
    uint64_t mask = ((uint64_t)1 << gpio);
    uint8_t off = led.off_level & mask ? 1 : 0;
    uint8_t pwm = gpio - PWM_BASE;
    uint16_t init_value = PWM_MAX_COUNTER;
    uint16_t toggle_val = PWM_MAX_COUNTER / 100 * percent;

    // if already on
    if (led.state & mask)
    {
        wiced_hal_pwm_change_values(pwm, toggle_val, init_value);
    }
    else
    {
        // new start
        wiced_hal_pwm_enable(pwm);
        wiced_hal_pwm_start(pwm, PMU_CLK, toggle_val, init_value, off);
        led.state |= mask;
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_pwm_led_off(uint8_t gpio)
{
    wiced_hal_pwm_disable(gpio - PWM_BASE);
    wiced_hidd_led_off(gpio);
}
 #endif // LED_USE_PWM
#endif // LED_SUPPORT

////////////////////////////////////////////////////////////////////////////////
/// This function is the timeout handler for allowsleep_timer
////////////////////////////////////////////////////////////////////////////////
void hidd_allowsleeptimerCb( uint32_t arg )
{
    wiced_hidd_deep_sleep_not_allowed(0); // sleep is allowed now
}

////////////////////////////////////////////////////////////////////////////////
/// This function returns if sleep is allowed
////////////////////////////////////////////////////////////////////////////////
uint8_t wiced_hidd_is_deep_sleep_allowed()
{
    return hidd.allowDeepSleep;
}

////////////////////////////////////////////////////////////////////////////////
/// This function returns if allow to sleep timer is running
////////////////////////////////////////////////////////////////////////////////
uint8_t wiced_hidd_is_deep_sleep_timer_running()
{
    return wiced_is_timer_in_use(&hidd.allowDeepSleepTimer);
}

////////////////////////////////////////////////////////////////////////////////
/// This function allows device to sleep
////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_set_deep_sleep_allowed(uint8_t allowed)
{
    hidd.allowDeepSleep = allowed;
    if (wiced_hidd_is_deep_sleep_timer_running())
    {
        wiced_stop_timer(&hidd.allowDeepSleepTimer);
    }
}

////////////////////////////////////////////////////////////////////////////////
/// This function not allowing the device to sleep for period of time
////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_deep_sleep_not_allowed( uint32_t milliseconds )
{
    wiced_hidd_set_deep_sleep_allowed(milliseconds ? WICED_FALSE : WICED_TRUE);
    if (!wiced_hidd_is_deep_sleep_allowed())
    {
        wiced_start_timer(&hidd.allowDeepSleepTimer, milliseconds);
//        WICED_BT_TRACE("\ndeepSleep not allowed for %d milliseconds", milliseconds);
    }
    else
    {
//        WICED_BT_TRACE("\ndeepSleep allowed");
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Sleep permit query to check if sleep (normal or SDS) is allowed and sleep time
///
/// \param type - sleep poll type
///
/// \return   sleep permission or sleep time, depending on input param
////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t hidd_link_sleep_handler(wiced_sleep_poll_type_t type )
{
    uint32_t ret = WICED_SLEEP_NOT_ALLOWED;

    switch(type)
    {
        case WICED_SLEEP_POLL_TIME_TO_SLEEP:
            //query application for sleep time
            if (hidd.registered_app_sleep_handler)
                ret = hidd.registered_app_sleep_handler(type);
            else
                ret = WICED_SLEEP_MAX_TIME_TO_SLEEP;
            break;

        case WICED_SLEEP_POLL_SLEEP_PERMISSION:
            ret = WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;

            //query application for sleep permit first
            if (hidd.registered_app_sleep_handler)
                ret = hidd.registered_app_sleep_handler(type);

            if (  (ret == WICED_SLEEP_ALLOWED_WITH_SHUTDOWN) && ( // We want to check for WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN only if we are WICED_SLEEP_ALLOWED_WITH_SHUTDOWN
                                                                  // Otherwise, we want to honor applilcation's decision.
               !wiced_hidd_is_deep_sleep_allowed()
               || ( wiced_hidd_is_transport_detection_polling_on()
#ifdef BLE_SUPPORT
               || (ble_hidd_link.second_conn_state == BLEHIDLINK_2ND_CONNECTION_PENDING)
#endif
#if defined(BR_EDR_SUPPORT) && is_20735Family
               //due to sniff+SDS is not supported in core FW, at this time, only allow SDS when disconnected
               || (bt_hidd_link.subState != BTHIDLINK_DISCONNECTED)
#endif
#ifdef FATORY_TEST_SUPPORT
                    && !force_sleep_in_HID_mode
#endif
                  )
#ifdef OTA_FIRMWARE_UPGRADE
               || wiced_ota_fw_upgrade_is_active()
#endif
               ))
            {
                ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
            }

            //save to AON before entering SDS
            if (ret == WICED_SLEEP_ALLOWED_WITH_SHUTDOWN)
            {
// if support epds, we need to determine if it should enter epds or hidoff
#ifdef SUPPORT_EPDS
                if (!wiced_hidd_link_is_disconnected() || !hidd.allowHIDOFF)
                {
                    // enter ePDS
                    g_foundation_config_PMUflags &= ~PMU_CONFIG_FLAGS_ENABLE_SDS;
                }
                else
                {
                    static uint8_t showHIDOFF = 1;
                    if (showHIDOFF)
                    {
                        showHIDOFF = 0;
                        WICED_BT_TRACE("\nHIDOFF");
                    }
                    /* allow ePDS */
                    g_foundation_config_PMUflags |= PMU_CONFIG_FLAGS_ENABLE_SDS;
                }
#endif

#ifdef BLE_SUPPORT
                wiced_ble_hidd_link_aon_action_handler(BLEHIDLINK_SAVE_TO_AON);
#endif
#ifdef BR_EDR_SUPPORT
                wiced_bt_hidd_link_aon_action_handler(BTHIDLINK_SAVE_TO_AON);
#endif
            }
            break;
    }

    return ret;
}

wiced_sleep_config_t    hidd_link_sleep_config = {
    WICED_SLEEP_MODE_NO_TRANSPORT,  //sleep_mode
    0,                              //host_wake_mode
    0,                              //device_wake_mode
    WICED_SLEEP_WAKE_SOURCE_GPIO | WICED_SLEEP_WAKE_SOURCE_KEYSCAN | WICED_SLEEP_WAKE_SOURCE_QUAD,  //device_wake_source
    255,                            //must set device_wake_gpio_num to 255 for WICED_SLEEP_MODE_NO_TRANSPORT
    hidd_link_sleep_handler,       //sleep_permit_handler
#if is_208xxFamily
    NULL,                           //post_sleep_handler
#endif
};

////////////////////////////////////////////////////////////////////////////////
// returns chip number
////////////////////////////////////////////////////////////////////////////////
uint32_t wiced_hidd_chip()
{
#if is_208xxFamily
    #define RADIO_ID    0x006007c0
    #define RADIO_20820 0x80
    static uint32_t chip = 0;

    // the radio id register become not accessible after ePDS; thus, read it only once at power up. Return the saved value thereafter.
    if (!chip)
    {
        chip = (*(UINT32*) RADIO_ID & RADIO_20820) ? 20820 : 20819;
    }
    return chip;
#else
    return CHIP;
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// wiced_hidd_link_connect
/////////////////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_link_connect()
{
#ifdef BLE_SUPPORT
    if (wiced_hidd_host_transport()==BT_TRANSPORT_LE)
    {
        // for BLE there is no channel, skip p_data[0]
        wiced_ble_hidd_link_connect();
    }
#endif
#ifdef BR_EDR_SUPPORT
    if (wiced_hidd_host_transport() == BT_TRANSPORT_BR_EDR)
    {
        wiced_bt_hidd_link_connect();
    }
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// wiced_hidd_disconnect
/////////////////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_disconnect()
{
#ifdef BLE_SUPPORT
    if (wiced_hidd_host_transport()==BT_TRANSPORT_LE)
    {
        // for BLE there is no channel, skip p_data[0]
        wiced_ble_hidd_link_disconnect();
    }
#endif
#ifdef BR_EDR_SUPPORT
    if (wiced_hidd_host_transport() == BT_TRANSPORT_BR_EDR)
    {
        wiced_bt_hidd_link_disconnect();
    }
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_hidd_link_is_connected()
{
#ifdef BLE_SUPPORT
    if (wiced_hidd_host_transport() == BT_TRANSPORT_LE)
    {
        return wiced_ble_hidd_link_is_connected();
    }
#endif
#ifdef BR_EDR_SUPPORT
    if (wiced_hidd_host_transport() == BT_TRANSPORT_BR_EDR)
    {
        return wiced_bt_hidd_link_is_connected();
    }
#endif
    return FALSE;
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_hidd_link_is_disconnected()
{
    wiced_bool_t state =
#ifdef BLE_SUPPORT
        wiced_ble_hidd_link_is_disconnected()
 #if BR_EDR_SUPPORT
        &&
 #endif
#endif
#ifdef BR_EDR_SUPPORT
        wiced_bt_hidd_link_is_disconnected()
#endif
        ;
//    WICED_BT_TRACE("\nis_disconnected %d %d(%d) %d(%d)", state,  wiced_ble_hidd_link_is_disconnected(), ble_hidd_link.subState, wiced_bt_hidd_link_is_disconnected(), bt_hidd_link.subState);
    return state;
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_hidd_link_is_encrypted()
{
#ifdef BLE_SUPPORT
    if (wiced_hidd_host_transport() == BT_TRANSPORT_LE)
    {
        return wiced_blehidd_is_link_encrypted();
    } else
#endif
#ifdef BR_EDR_SUPPORT
    if (wiced_hidd_host_transport() == BT_TRANSPORT_BR_EDR)
    {
        return bt_hidd_link.encrypt_status.encrypted;
    } else
#endif
    return FALSE;
}

////////////////////////////////////////////////////////////////////////////////////////////
/// Pairing had ended
/////////////////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_pairing_stopped(wiced_bt_transport_t transport)
{
//    WICED_BT_TRACE("\nwiced_hidd_pairing_stopped %d %d", transport, hidd.pairingMode);
    if (hidd.pairingMode == transport)
    {
//        WICED_BT_TRACE("\nPairing mode is 0");
        hidd.pairingMode=0;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////
/// Enter pairing
/////////////////////////////////////////////////////////////////////////////////////////////


#ifdef BLE_SUPPORT
/*
 * Handle host command to set device pairable.  This is typically a HID device button push.
 */
void ble_accept_pairing( BOOLEAN enable )
{
    WICED_BT_TRACE("\n%s LE pairing", enable ? "Start" : "Stop");
//    wiced_bt_set_pairable_mode(enable, 0);

    if ( !enable )
    {
        wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );
        wiced_hidd_pairing_stopped(BT_TRANSPORT_LE);
        blehidlink_enterDisconnected();
        return;
    }

    blehidlink_enterDiscoverable(WICED_TRUE);
 #if 0
    /* disconnect any connections if active */
    if (wiced_ble_hidd_link_is_connected())
    {
        wiced_ble_hidd_link_disconnect();
    }

    // start advertisements so that a host can connect
    wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
    blehidlink_setState(BLEHIDLINK_DISCOVERABLE);
 #endif
}
#endif

#ifdef BR_EDR_SUPPORT
void bt_accept_pairing( BOOLEAN enable )
{
    WICED_BT_TRACE("\n%s BR/EDR pairing", enable ? "Enter" : "Exit");
    if (enable)
    {
        wiced_bt_hidd_link_enter_discoverable();
    }
    else
    {
        if (wiced_bt_hidd_link_is_discoverable())
            wiced_bt_hidd_link_enter_disconnected();
        else
            wiced_bt_hidd_link_disable_page_and_inquiry_scans();
    }
}
#endif

void wiced_hidd_link_virtual_cable_unplug(void)
{
#ifdef BR_EDR_SUPPORT
    if (wiced_hidd_host_transport()==BT_TRANSPORT_BR_EDR)
    {
        wiced_bt_hidd_link_virtual_cable_unplug();
    }
#endif
#ifdef BLE_SUPPORT
    if (wiced_hidd_host_transport()==BT_TRANSPORT_LE)
    {
        wiced_ble_hidd_link_virtual_cable_unplug();
    }
#endif
}

void wiced_hidd_enter_pairing()
{
    WICED_BT_TRACE("\nwwiced_hidd_enter_pairing before: %d", hidd.pairingMode);
    switch (hidd.pairingMode)
    {
#ifdef BR_EDR_SUPPORT
        // we are in BT mode
        case BT_TRANSPORT_BR_EDR:
            bt_accept_pairing(STOP_PAIRING); // stop BT pairing
            hidd.pairingMode = 0;
 #ifdef BLE_SUPPORT
            hidd.pairingMode = BT_TRANSPORT_LE;
            ble_accept_pairing(START_PAIRING); // start LE pairing
 #endif
            break;
#endif

#ifdef BLE_SUPPORT
        case BT_TRANSPORT_LE:
            ble_accept_pairing(STOP_PAIRING);
            hidd.pairingMode = 0;
            break;
#endif

        default:
#ifdef BR_EDR_SUPPORT
            bt_accept_pairing(START_PAIRING); // start BT pairing
            hidd.pairingMode = BT_TRANSPORT_BR_EDR;
#elif defined(BLE_SUPPORT)
            ble_accept_pairing(START_PAIRING); // start LE pairing
            hidd.pairingMode = BT_TRANSPORT_LE;
#endif
            break;
    }
//    WICED_BT_TRACE("\nwwiced_hidd_enter_pairing after: %d", hidd.pairingMode);
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// Abstract link layer initialize
/////////////////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_link_init()
{
    //Setup Battery Service
    wiced_hal_batmon_init();

    //configure sleep
    wiced_sleep_configure( &hidd_link_sleep_config );

    //timer to allow shut down sleep (SDS)
    wiced_init_timer( &hidd.allowDeepSleepTimer, hidd_allowsleeptimerCb, 0, WICED_MILLI_SECONDS_TIMER );

    // initialize host
    hidd_host_init();

    //hid link init
#ifdef BLE_SUPPORT
    blehidlink_init();
    blehidlink_determineNextState();
#endif
#ifdef BR_EDR_SUPPORT
    bthidlink_init();
    bthidlink_determineNextState();
#endif
}

/////////////////////////////////////////////////////////////////////////////////
/// register application sleep permit handler
///
/// \param cb - pointer to application callback function
/////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_link_register_sleep_permit_handler(wiced_sleep_allow_check_callback sleep_handler)
{
    hidd.registered_app_sleep_handler = sleep_handler;
}

/*
 * hidd lib link default management callback
 */
wiced_bt_management_cback_t *app_management_cback_ptr = NULL;
wiced_result_t (*app_init_ptr)(void) = NULL;
wiced_result_t hidd_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_device_link_keys_t *pLinkKeys=NULL;
    wiced_bt_ble_advert_mode_t curr_adv_mode;
    uint8_t *p_keys;
    wiced_bt_device_address_t         bda = { 0 };

    WICED_BT_TRACE("\n=== BT stack cback event %d", event);

    if (app_management_cback_ptr)
    {
        result = app_management_cback_ptr(event, p_event_data);
        if (result != WICED_RESUME_HIDD_LIB_HANDLER) // if application has handled the event
        {
            return result;
        }
        // Not handled by app or app requests to resume.
        // Default result back to success and continue with default handler.
        result = WICED_BT_SUCCESS;
    }

    // hidd_lib default handler
    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            if ( p_event_data->enabled.status == WICED_BT_SUCCESS )
            {
                hci_control_enable_trace();
                wiced_bt_dev_read_local_addr(bda);
                WICED_BT_TRACE("\nAddress: [ %B]", bda);
                if (app_init_ptr)
                {
                    result = app_init_ptr();
                }
            }
            else
            {
                WICED_BT_TRACE("\nBT Enable status: 0x%02x", p_event_data->enabled.status);
            }
            hci_control_send_paired_host_info();
            break;

        case BTM_PAIRING_COMPLETE_EVT:
#ifdef BR_EDR_SUPPORT
            if ((p_event_data->pairing_complete.transport == BT_TRANSPORT_BR_EDR) && (hidd.pairingMode == BT_TRANSPORT_BR_EDR))
            {
                result = p_event_data->pairing_complete.pairing_complete_info.br_edr.status;
                WICED_BT_TRACE("\nBR/EDR Pairing Complete: Status:%d Addr:%B", result, p_event_data->pairing_complete.bd_addr);

                //bonding successful
                if (!result)
                {
                    WICED_BT_TRACE("\nBONDED successful");
                    hidd_host_setTransport(p_event_data->pairing_complete.bd_addr, BT_TRANSPORT_BR_EDR);
                    hci_control_send_pairing_complete_evt( result, p_event_data->pairing_complete.bd_addr, BT_DEVICE_TYPE_BREDR );
                }
                wiced_hidd_pairing_stopped(BT_TRANSPORT_BR_EDR);
            }
#endif
#ifdef BLE_SUPPORT
            if((p_event_data->pairing_complete.transport == BT_TRANSPORT_LE) && (hidd.pairingMode == BT_TRANSPORT_LE))
            {
                wiced_bt_dev_ble_pairing_info_t * p_info;
                p_info =  &p_event_data->pairing_complete.pairing_complete_info.ble;

                WICED_BT_TRACE("\nLE Pairing Complete:");
                //bonding successful
                if (!p_info->reason )
                {
                    WICED_BT_TRACE("\n BONDED successful");
                    hidd_host_setBonded(TRUE);
                    if (!wiced_blehidd_is_device_bonded())
                    {
                        WICED_BT_TRACE("\n set device bonded flag");
                        wiced_blehidd_set_device_bonded_flag(WICED_TRUE);
                    }

 #ifdef CONNECTED_ADVERTISING_SUPPORTED
                    //If there is a connection existing, delete pairing information and disconnect existing connection
                    if (ble_hidd_link.second_conn_state == BLEHIDLINK_2ND_CONNECTION_PENDING)
                    {
                        ble_hidd_link.second_conn_state = BLEHIDLINK_2ND_CONNECTION_NOT_ALLOWED;

                        if (hidd_host_isBonded())
                        {
                            uint8_t *bonded_bdadr = wiced_hidd_host_addr();

                            WICED_BT_TRACE("\n remove bonded device : %B", bonded_bdadr);
                            wiced_bt_dev_delete_bonded_device(bonded_bdadr);
                        }

                        WICED_BT_TRACE("\n Removing all bonded info");
                        wiced_hidd_host_remove_all();

                        //disconnect existing connection
                        wiced_bt_gatt_disconnect(ble_hidd_link.existing_connection_gatts_conn_id);
                    }
 #endif
                    //SMP result callback: successful
                    hidd_host_setTransport(ble_hidd_link.gatts_peer_addr, BT_TRANSPORT_LE);
                    hci_control_send_pairing_complete_evt( p_info->reason, p_event_data->pairing_complete.bd_addr, BT_DEVICE_TYPE_BLE );
                }
                else
                {
                    //SMP result callback: failed
                    WICED_BT_TRACE("\n BONDED failed reason:%d", p_info->reason);
 #ifdef CONNECTED_ADVERTISING_SUPPORTED
                    //If this is from the new connection
                    if (ble_hidd_link.second_conn_state == BLEHIDLINK_2ND_CONNECTION_PENDING)
                    {
                        uint16_t temp_gatts_conn_id = ble_hidd_link.gatts_conn_id;

                        WICED_BT_TRACE("\n delete the new connection: %d", temp_gatts_conn_id);

                        ble_hidd_link.second_conn_state = BLEHIDLINK_2ND_CONNECTION_NOT_ALLOWED;

                        //recover current connection gatt connection id
                        ble_hidd_link.gatts_conn_id = ble_hidd_link.existing_connection_gatts_conn_id;

                        //disconnect new connection
                        wiced_bt_gatt_disconnect(temp_gatts_conn_id);

                        //restore embeded controller info for the LE link (peer device info, bonded, encrypted, connection parameters etc.)
                        memcpy(&emConInfo_devInfo, &ble_hidd_link.existing_emconinfo, sizeof(EMCONINFO_DEVINFO));
                    }
                    else
 #endif
                    wiced_hidd_host_remove();
                }
                wiced_hidd_pairing_stopped(BT_TRANSPORT_LE);
            }
#endif
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            pLinkKeys = &p_event_data->paired_device_link_keys_update;
            WICED_BT_TRACE("\nBTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT   BdAddr:%B", pLinkKeys->bd_addr );

            // make sure if address is valid
            if (memcmp(pLinkKeys->bd_addr, bda, BD_ADDR_LEN))
            {
                hidd_host_setLinkKey(pLinkKeys->bd_addr, pLinkKeys);
#ifdef BLE_SUPPORT
                if (wiced_hidd_host_transport() == BT_TRANSPORT_LE)
                {
                    WICED_BT_TRACE("\nmask           :  x%02X", pLinkKeys->key_data.le_keys_available_mask);
                    WICED_BT_TRACE("\nBLE AddrType   :  %d", pLinkKeys->key_data.ble_addr_type);
                    WICED_BT_TRACE("\nStatic AddrType:  %d", pLinkKeys->key_data.static_addr_type);
                    WICED_BT_TRACE("\nStatic Addr    :  %B", pLinkKeys->key_data.static_addr);
                    STRACE_ARRAY  ("\n  irk: ", &(pLinkKeys->key_data.le_keys.irk), LINK_KEY_LEN);
 #if SMP_INCLUDED == TRUE && SMP_LE_SC_INCLUDED == TRUE
                    STRACE_ARRAY  ("\n pltk: ", &(pLinkKeys->key_data.le_keys.pltk), LINK_KEY_LEN);
                    STRACE_ARRAY  ("\npcsrk: ", &(pLinkKeys->key_data.le_keys.pcsrk), LINK_KEY_LEN);
                    STRACE_ARRAY  ("\n lltk: ", &(pLinkKeys->key_data.le_keys.lltk), LINK_KEY_LEN);
                    STRACE_ARRAY  ("\nlcsrk: ", &(pLinkKeys->key_data.le_keys.lcsrk), LINK_KEY_LEN);
 #else
                    STRACE_ARRAY  ("\n ltk: ", &(pLinkKeys->key_data.le_keys.ltk), LINK_KEY_LEN);
                    STRACE_ARRAY  ("\ncsrk: ", &(pLinkKeys->key_data.le_keys.csrk), LINK_KEY_LEN);
 #endif
                }
#endif
#ifdef BR_EDR_SUPPORT
                if (wiced_hidd_host_transport() == BT_TRANSPORT_BR_EDR)
                {
                    STRACE_ARRAY("\nBR/EDR Link key:", pLinkKeys->key_data.br_edr_key, LINK_KEY_LEN);
                }
#endif
            }
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            WICED_BT_TRACE("\nBTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT");
            if (!hidd_host_getLinkKey(p_event_data->paired_device_link_keys_request.bd_addr, &p_event_data->paired_device_link_keys_request))
            {
                WICED_BT_TRACE("\n link_key not available");
                result = WICED_BT_ERROR;
            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            WICED_BT_TRACE("\nBTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT");
            /* save keys to NVRAM */
            p_keys = (uint8_t *)&p_event_data->local_identity_keys_update;
            wiced_hal_write_nvram ( VS_LOCAL_IDENTITY_ID, sizeof( wiced_bt_local_identity_keys_t ), p_keys ,&result );
#if 0
            WICED_BT_TRACE("\n local keys save to NVRAM result: %d", result);
            TRACE_ARRAY(p_event_data->local_identity_keys_update.local_key_data, BTM_SECURITY_LOCAL_KEY_DATA_LEN);
#endif
            break;

        case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            WICED_BT_TRACE("\nBTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT");
            /* read keys from NVRAM */
            p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
            wiced_hal_read_nvram( VS_LOCAL_IDENTITY_ID, sizeof(wiced_bt_local_identity_keys_t), p_keys, &result );
#if 0
            WICED_BT_TRACE("\n local keys read from NVRAM result: %d",  result);
            if (!result)
            {
                STRACE_ARRAY("\n", p_keys, BTM_SECURITY_LOCAL_KEY_DATA_LEN);
            }
#endif
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            WICED_BT_TRACE("\nBTM_ENCRYPTION_STATUS_EVT, result=%d", p_event_data->encryption_status.result);
            //ble
#ifdef BLE_SUPPORT
            if (wiced_hidd_host_transport() == BT_TRANSPORT_LE)
            {
                if (p_event_data->encryption_status.result == WICED_SUCCESS)
                {
                    WICED_BT_TRACE("\n link encrypted");
                    wiced_blehidd_set_link_encrypted_flag(WICED_TRUE);
                }
                else
                {
                    WICED_BT_TRACE("\n Encryption failed:%d", p_event_data->encryption_status.result);
                }
            }
#endif
#ifdef BR_EDR_SUPPORT
            if (wiced_hidd_host_transport() == BT_TRANSPORT_BR_EDR)
            {
                if (p_event_data->encryption_status.result == WICED_SUCCESS)
                {
                    bt_hidd_link.encrypt_status.encrypted = WICED_TRUE;
                }
                else
                {
                    bt_hidd_link.encrypt_status.encrypted = WICED_FALSE;

                }
                memcpy(bt_hidd_link.encrypt_status.bdAddr, p_event_data->encryption_status.bd_addr, BD_ADDR_LEN);
            }
#endif
            break;

#ifdef BR_EDR_SUPPORT
        case BTM_POWER_MANAGEMENT_STATUS_EVT:
            WICED_BT_TRACE("\nBTM_POWER_MANAGEMENT_STATUS_EVT");
            break;

        case BTM_PIN_REQUEST_EVT:
            WICED_BT_TRACE("\nBTM_PIN_REQUEST_EVT");
            bthidlink_pinCodeRequest((wiced_bt_dev_name_and_class_t *)p_event_data);
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            WICED_BT_TRACE("\nBTM_USER_CONFIRMATION_REQUEST_EVT");
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PASSKEY_REQUEST_EVT:
            WICED_BT_TRACE("\nBTM_PASSKEY_REQUEST_EVT");
 #ifdef USE_KEYBOARD_IO_CAPABILITIES
            bthidlink_passKeyRequest(p_event_data);
 #else
            wiced_bt_dev_pass_key_req_reply(WICED_BT_SUCCESS,p_event_data->user_passkey_request.bd_addr, 0);
 #endif
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            WICED_BT_TRACE("\nBTM_PAIRING_IO_CAPABILITIES_REQUEST_EVT bda %B",
                        p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
 #ifdef USE_KEYBOARD_IO_CAPABILITIES
            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_KEYBOARD_ONLY;
 #else
            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
 #endif
            p_event_data->pairing_io_capabilities_br_edr_request.oob_data = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
            p_event_data->pairing_io_capabilities_br_edr_request.is_orig = FALSE;
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT:
            WICED_BT_TRACE("\nBTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT");
            WICED_BT_TRACE("\n peer_bd_addr: %B, peer_io_cap: %d, peer_oob_data: %d, peer_auth_req: %d",
                                p_event_data->pairing_io_capabilities_br_edr_response.bd_addr,
                                p_event_data->pairing_io_capabilities_br_edr_response.io_cap,
                                p_event_data->pairing_io_capabilities_br_edr_response.oob_data,
                                p_event_data->pairing_io_capabilities_br_edr_response.auth_req);
            break;

        case BTM_SECURITY_FAILED_EVT:
            WICED_BT_TRACE("\nBTM_SECURITY_FAILED_EVT. hci_status:%d", p_event_data->security_failed.hci_status);
            bt_hidd_link.security_failed = p_event_data->security_failed.hci_status;
            hidd_host_setLinkKey(wiced_hidd_host_addr(), NULL);
            break;
#endif
#ifdef BLE_SUPPORT
        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            WICED_BT_TRACE("\nBTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT");
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_SC_ONLY|BTM_LE_AUTH_REQ_BOND;              /* LE sec bonding */
            p_event_data->pairing_io_capabilities_ble_request.max_key_size = 16;
            p_event_data->pairing_io_capabilities_ble_request.init_keys = 0x0F; //(BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_PLK);
            p_event_data->pairing_io_capabilities_ble_request.resp_keys = 0x0F; //(BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_PLK);
            break;

        case BTM_SECURITY_REQUEST_EVT:
            WICED_BT_TRACE("\nBTM_SECURITY_REQUEST_EVT");
            if (wiced_hidd_host_transport() == BT_TRANSPORT_LE)
            {
                WICED_BT_TRACE("\nClear CCCD's");
                wiced_hidd_host_set_flags(p_event_data->security_request.bd_addr, 0, 0xFFFF);
            }
             /* Use the default security */
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr,  WICED_BT_SUCCESS);
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            curr_adv_mode = app_adv_mode;
            app_adv_mode = p_event_data->ble_advert_state_changed;

            WICED_BT_TRACE("\nAdvertisement State Change: %d -> %d", curr_adv_mode, app_adv_mode);
            hci_control_send_advertisement_state_evt( app_adv_mode );
            if (!app_adv_mode)
            {
                WICED_BT_TRACE("\nsubstate: %d", ble_hidd_link.subState);
                if (ble_hidd_link.subState==BLEHIDLINK_DISCOVERABLE)
                {
                    wiced_hidd_pairing_stopped(BT_TRANSPORT_LE);
                    WICED_BT_TRACE("\ndisconnecting..");
                    blehidlink_enterDisconnected();
                }
            }

            //if high duty cycle directed advertising stops
            if ( (curr_adv_mode == BTM_BLE_ADVERT_DIRECTED_HIGH) &&
                     (app_adv_mode == BTM_BLE_ADVERT_DIRECTED_LOW))
            {
                wiced_ble_hidd_link_directed_adv_stop();
            }
//#if defined(ENDLESS_LE_ADVERTISING_WHILE_DISCONNECTED)
#if 0
            // btstack will switch to low adv mode automatically when high adv mode timeout,
            // for HIDD, we want to stop adv instead
            else if ((curr_adv_mode == BTM_BLE_ADVERT_UNDIRECTED_HIGH) &&
                         (app_adv_mode == BTM_BLE_ADVERT_UNDIRECTED_LOW))
            {
                wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
            }
#endif
            // if we are reconnecting and adv stops, we enter disconnected state
            if (ble_hidd_link.subState == BLEHIDLINK_RECONNECTING && !app_adv_mode)
            {
                blehidlink_setState(BLEHIDLINK_DISCONNECTED);
#ifdef AUTO_RECONNECT
                if(ble_hidd_link.auto_reconnect && (wiced_hidd_host_transport() == BT_TRANSPORT_BR_EDR) && !wiced_hal_batmon_is_low_battery_shutdown())
                    wiced_ble_hidd_link_connect();
#endif
            }
            break;

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
            WICED_BT_TRACE("\nScan State Change: %d", p_event_data->ble_scan_state_changed );
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            WICED_BT_TRACE("\nBTM_BLE_CONNECTION_PARAM_UPDATE status:%d", p_event_data->ble_connection_param_update.status);
            if (!p_event_data->ble_connection_param_update.status)
            {
                wiced_ble_hidd_link_conn_update_complete();
            }
            break;
#endif
        default:
            WICED_BT_TRACE("\nUnhandled management_cback event: %d!!!", event );
            break;
    }
    return result;
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
const wiced_bt_cfg_settings_t * wiced_hidd_cfg()
{
    return hidd.wiced_bt_hid_cfg_settings_ptr;
}

/////////////////////////////////////////////////////////////////////////////////
/// wiced_ble_hidd_start
///
/// \param p_bt_management_cback  - application bt_management callback function
///        p_bt_cfg_settings      - bt configuration setting
///        wiced_bt_cfg_buf_pools - buffer pool configuration
/////////////////////////////////////////////////////////////////////////////////
#if is_newFamily
#define lhl_ctl_adr 0x00338130
#else
#define lhl_ctl_adr 0x00336130
#endif
void wiced_hidd_start(wiced_result_t (*p_bt_app_init)(),
                      wiced_bt_management_cback_t   * p_bt_management_cback,
                      const wiced_bt_cfg_settings_t * p_bt_cfg_settings,
                      const wiced_bt_cfg_buf_pool_t * p_bt_cfg_buf_pools)
{
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
#if LED_SUPPORT
    wiced_init_timer( &led.blinking_timer, led_blink_handler, 0, WICED_MILLI_SECONDS_TIMER );
#endif

    // Bit 1: Enables KeyScan so it can detect the keys pressed before power up.
    REG32(lhl_ctl_adr)  |= HW_CTRL_SCAN_CTRL_MASK << 1;

#if is_208xxFamily
    // For 208xx, the chip id is identified by Radio id register; however, the register may get disabled after entering ePDS;
    // therefore, we read it once at power up and save the id.
    wiced_hidd_chip();
#endif

    app_management_cback_ptr = p_bt_management_cback;
    app_init_ptr = p_bt_app_init;
    if (!p_bt_cfg_settings || !p_bt_cfg_buf_pools)
    {
        WICED_BT_TRACE("\nbt or buff_pool configration is undefined!!"); while(1);
    }
    else
    {
        hidd.wiced_bt_hid_cfg_settings_ptr = p_bt_cfg_settings;
        wiced_bt_stack_init (hidd_management_cback, p_bt_cfg_settings, p_bt_cfg_buf_pools);
        WICED_BT_TRACE("\n\n<<%s start>>",p_bt_cfg_settings->device_name);
    }
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_allowed_hidoff(wiced_bool_t en)
{
//    WICED_BT_TRACE("\nHIDOFF is %sAllowed",en? "":"not ");
    hidd.allowHIDOFF = en ? 1 : 0;
}

////////////////////////////////////////////////////////////////////////////////
// wiced_hidd_activity_detected
////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_activity_detected()
{
#ifdef BR_EDR_SUPPORT
    extern void bthidlink_activityDetected();
    bthidlink_activityDetected();
#endif
}


#ifdef WICED_BT_TRACE_ENABLE
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void trace_array(void * ptr, uint32_t len)
{
    uint8_t * cPtr = (uint8_t *) ptr;
    int cnt=0;
    while (len--)
    {
        WICED_BT_TRACE("%02X ",*cPtr++);
        if (len && !(++cnt & 0xf))
            WICED_BT_TRACE("\n");
    }
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void strace_array(char * str, void * ptr, uint32_t len)
{
    WICED_BT_TRACE("%s",str);
    trace_array(ptr, len);
}
#endif
