/*
 * Copyright 2019, Cypress Semiconductor Corporation or a subsidiary of
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

#ifdef TESTING_USING_HCI
#include "wiced_bt_dev.h"

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;

typedef struct
{
    uint8_t * rpt_buf;
    uint8_t   rpt_type;
    uint8_t   rpt_id;
    uint16_t  length;
} hci_rpt_db_t;

void hci_control_le_init(int cnt, hci_rpt_db_t * rpt_db_ptr);
void hci_control_le_handle_event(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
void hci_control_le_send_disconnect_evt( uint8_t reason, uint16_t con_handle );
void hci_control_le_send_connect_event( uint8_t addr_type, BD_ADDR addr, uint16_t con_handle, uint8_t role );
void hci_control_le_enable_trace();
#else
 #define hci_control_le_init( cnt, rpt_db_ptr )
 #define hci_control_le_handle_event(event, p_event_data)
 #define hci_control_le_send_disconnect_evt( reason, con_handle )
 #define hci_control_le_send_connect_event( addr_type, addr, con_handle, role )
 #define hci_control_le_enable_trace()
#endif
#endif
