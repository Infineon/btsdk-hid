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

/** @file
 *
 * HCI hidd handling routines
 *
 */
#ifndef __BLE_HID_HCI_H__
#define __BLE_HID_HCI_H__

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;

#ifdef TESTING_USING_HCI
#include "wiced_bt_dev.h"

typedef struct
{
    uint8_t * rpt_buf;
    uint8_t   rpt_type;
    uint8_t   rpt_id;
    uint16_t  length;
} hci_rpt_db_t;

void hci_control_init_(int cnt, hci_rpt_db_t * rpt_db_ptr);
void hci_control_send_pairing_complete_evt( uint8_t result, uint8_t *p_bda, uint8_t type );
void hci_control_send_disconnect_evt( uint8_t reason, uint16_t con_handle );
void hci_control_send_connect_evt( uint8_t addr_type, BD_ADDR addr, uint16_t con_handle, uint8_t role );
void hci_control_send_advertisement_state_evt( uint8_t state );
void hci_control_send_paired_host_info();
void hci_control_send_state_change( uint8_t transport, uint8_t state );
void hci_control_enable_trace();
 #define hci_control_send_data( code, buf, len ) wiced_transport_send_data( code, buf, len )
 #ifdef BLE_SUPPORT
  #define hci_control_init(a,b) hci_control_init_(a,b)
 #else
  #define hci_control_init() hci_control_init_(0,NULL)
 #endif
#else
 #define hci_control_init(...)
 #define hci_control_send_pairing_complete_evt( result, p_bda, type )
 #define hci_control_send_disconnect_evt( reason, con_handle )
 #define hci_control_send_connect_evt( addr_type, addr, con_handle, role )
 #define hci_control_send_advertisement_state_evt( state )
 #define hci_control_send_paired_host_info()
 #define hci_control_send_state_change( transport, state )
 #define hci_control_send_data( code, buf, len )
 #ifdef HCI_TRACES_ENABLED
  void hci_control_enable_trace();
 #else
  #define hci_control_enable_trace()
 #endif
#endif
#endif
