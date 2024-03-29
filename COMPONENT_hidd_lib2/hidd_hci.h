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

/** @file
 *
 * HCI hidd handling routines
 *
 */
#ifndef __BLE_HID_HCI_H__
#define __BLE_HID_HCI_H__

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;

#define DEVICE_CAPABILITY_LEN 3

#ifdef TESTING_USING_HCI

#include "wiced_bt_dev.h"
#include "wiced_transport.h"
#if BTSTACK_VER < 0x03000001
#include "hidd_hci_v1.h"
#else
#include "hidd_hci_v3.h"
#endif

enum {
    KEY_IR      = 0xf0,
    KEY_AUDIO   = 0xf1,
    KEY_MOTION  = 0xf2,
    KEY_CONNECT = 0xf3,
};

typedef void (*hidd_app_hci_key_callback_t ) (uint8_t key, wiced_bool_t pressed);

void hidd_hci_control_init();
void hidd_hci_control_set_capability(char audio, char mouse, char ir);
void hidd_hci_control_send_pairing_complete_evt( uint8_t result, uint8_t *p_bda, uint8_t type );
void hidd_hci_control_send_disconnect_evt( uint8_t reason, uint16_t con_handle );
void hidd_hci_control_send_connect_evt( uint8_t addr_type, BD_ADDR addr, uint16_t con_handle, uint8_t role );
void hidd_hci_control_send_advertisement_state_evt( uint8_t state );
void hidd_hci_control_send_paired_host_info();
void hidd_hci_control_send_state_change( uint8_t transport, uint8_t state );
void hidd_hci_control_enable_trace();
void hidd_hci_control_register_key_handler(hidd_app_hci_key_callback_t key_handler);
void hidd_hci_control_transport_status( wiced_transport_type_t type );
uint32_t hidd_hci_dev_handle_command( uint8_t * p_data, uint32_t length );

 #define hidd_hci_control_send_data( code, buf, len ) wiced_transport_send_data( code, buf, len )
#else
 #define hidd_hci_control_init()
 #define hidd_hci_control_set_capability(a,m,i)
 #define hidd_hci_control_send_pairing_complete_evt( result, p_bda, type )
 #define hidd_hci_control_send_disconnect_evt( reason, con_handle )
 #define hidd_hci_control_send_connect_evt( addr_type, addr, con_handle, role )
 #define hidd_hci_control_send_advertisement_state_evt( state )
 #define hidd_hci_control_send_paired_host_info()
 #define hidd_hci_control_send_state_change( transport, state )
 #define hidd_hci_control_send_data( code, buf, len )
 #define hidd_hci_control_register_key_handler( handler )
 #ifdef HCI_TRACES_ENABLED
  void hidd_hci_control_enable_trace();
 #else
  #define hidd_hci_control_enable_trace()
 #endif
#endif
#endif
