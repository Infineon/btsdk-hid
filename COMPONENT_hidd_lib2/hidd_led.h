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
* File Name: hidd_led.h
*
* Abstract: This module implements LED functions
*
* Functions:
*
*******************************************************************************/
#ifndef __HIDD_LIB_LED_H__
#define __HIDD_LIB_LED_H__
#include "wiced_platform.h"

// Use first defined LED for HIDD code indicator
#define HIDD_LED             0

enum {
    HIDD_LED_CODE_MISSING_BT_AND_MEM_POOL_CFG  = 1,
    HIDD_LED_CODE_MISSING_LE_CFG  = 2,
    HIDD_LED_CODE_MISSING_BREDR_CFG  = 3,
};
#define HIDD_START_UP_BLINK_COUNT 5
#define HIDD_START_UP_BLINK_SPEED 100

enum {
    LED_OFF,
    LED_ON,
};

#if LED_SUPPORT
 #if LED_SUPPORT==2
  #define LED_USE_PWM
 #endif

/*******************************************************************************
 * Function Name: hidd_led_init
 ********************************************************************************
 * Summary:
 *    Initialize LED module. This function must be called once first before using any of
 *    other LED functions. It uses LED configuration from Platform and initialize hardware
 *    to default state (LED off state)
 *
 *    If this is Power On Reset, it will blink first, index 0, LED 5 times to indicate LED is
 *    initialized and ready to use. For apps, this is also an indicator as system start up.
 *
 * Parameters:
 *    uint8_t led_count;                     -- number of LED
 *    wiced_platform_led_config_t * led_cfg  -- pointer to platfrom LED config.
 *
 * Return:
 *    none
 *
 *******************************************************************************/
 void hidd_led_init(uint8_t count, const wiced_platform_led_config_t * cfg);

/*******************************************************************************
 * Function Name: hidd_led_is_blinking
 ********************************************************************************
 * Summary:
 *    Return true if the requested LED is blinking.
 *
 * Parameters:
 *    idx -- LED index
 *
 * Return:
 *    True when LED is blinking.
 *
 *******************************************************************************/
 wiced_bool_t hidd_led_is_blinking(uint8_t idx);

/*******************************************************************************
 * Function Name: hidd_led_on
 ********************************************************************************
 * Summary:
 *    Turn on LED
 *
 * Parameters:
 *    idx -- LED index
 *
 * Return:
 *    None
 *
 *******************************************************************************/
 void hidd_led_on(uint8_t idx);

/*******************************************************************************
 * Function Name: hidd_led_off
 ********************************************************************************
 * Summary:
 *    Turn off LED
 *
 * Parameters:
 *    idx -- LED index
 *
 * Return:
 *    None
 *
 *******************************************************************************/
 void hidd_led_off(uint8_t idx);

/*******************************************************************************
 * Function Name: hidd_led_blink
 ********************************************************************************
 * Summary:
 *    Blink the LED
 *
 * Parameters:
 *    idx -- LED index
 *    count -- how many times to blink, use value 0 to blink forever
 *    how_fast_in_ms -- the LED on/off duration in ms
 *
 * Return:
 *    None
 *
 *******************************************************************************/
 void hidd_led_blink(uint8_t idx, uint32_t count, uint32_t how_fast_in_ms);

/*******************************************************************************
 * Function Name: hidd_led_blink_code
 ********************************************************************************
 * Summary:
 *    Blink an error code
 *
 * Parameters:
 *    idx -- LED index
 *    code -- error code. The code is the fast blink count followed by long off. This repeats forever.
 *
 * Return:
 *    None
 *
 *******************************************************************************/
 void hidd_led_blink_code(uint8_t idx, uint8_t code);

/*******************************************************************************
 * Function Name: hidd_led_blink_stop
 ********************************************************************************
 * Summary:
 *    Stop LED blinking and set LED back to ON or OFF state when it started to blink.
 *
 * Parameters:
 *    idx -- LED index
 *
 * Return:
 *    None
 *
 *******************************************************************************/
 void hidd_led_blink_stop(uint8_t idx);

 // PWM functions are not implemented yet
 #ifdef LED_USE_PWM
 void hidd_pwm_led_init(uint8_t idx, uint8_t off_level);
 void hidd_pwm_led_on(uint8_t idx, uint8_t percent);
 void hidd_pwm_led_off(uint8_t idx);
 #else
 #define hidd_pwm_led_init(idx, off_level)
 #define hidd_pwm_led_on(idx, percent)
 #define hidd_pwm_led_off(idx)
 #endif
#else
 #define hidd_led_is_blinking(idx) FALSE
 #define hidd_led_init(cnt,cfg)
 #define hidd_led_on(idx)
 #define hidd_led_off(idx)
 #define hidd_led_blink(idx, count, how_fast_in_ms)
 #define hidd_led_blink_code(idx, code)
 #define hidd_led_blink_stop(idx)
 #define hidd_pwm_led_init(idx, off_level)
 #define hidd_pwm_led_on(idx, percent)
 #define hidd_pwm_led_off(idx)
#endif // LED_SUPPORT
#endif // __HIDD_LIB_LED_H__
