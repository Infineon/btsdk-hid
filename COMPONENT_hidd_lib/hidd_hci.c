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

#include "sparcommon.h"
#include "gki_target.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_l2c.h"
#include "wiced_platform.h"
#include "hci_control_api.h"
#include "wiced_transport.h"
#if defined(CYW43012C0)
 #include "wiced_hal_watchdog.h"
#else
 #include "wiced_hal_wdog.h"
#endif
#include "blehidlink.h"
#include "wiced_memory.h"
#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20719B0) || defined(CYW20721B1) || defined(CYW20719B2) || defined(CYW20735B0) || defined(CYW43012C0) )
#include "wiced_bt_app_common.h"
#endif
#include "hidd_lib.h"

#if defined(TESTING_USING_HCI)

static int hci_rpt_cnt = 0;
static hci_rpt_db_t * rpt_db = NULL;

#ifndef HCI_UART_BAUD_RATE
 #define HCI_UART_BAUD_RATE  HCI_UART_DEFAULT_BAUD // 3M
#endif

#define READ_LITTLE_ENDIAN_TO_UINT16(into, m,dl) (into) = ((m)[0] | ((m)[1]<<8)); (m) +=2; (dl)-=2;

#ifdef BLE_SUPPORT
/*
 *  Send advertise state change event to uart
 */
void hci_control_send_advertisement_state_evt( uint8_t state )
{
    hci_control_send_data( HCI_CONTROL_LE_EVENT_ADVERTISEMENT_STATE, &state, 1 );
}

/*
 * Handle host command to send HID report.
 */
void hci_ble_hid_dev_handle_send_report( uint8_t type, uint8_t *p_data, uint16_t length )
{
    uint8_t  report_id = *p_data++;
    uint16_t attr_handle;
    wiced_bool_t OK_to_send = WICED_FALSE;
    int i;

    //send report only when ble link is connected
    if(wiced_ble_hidd_link_is_connected())
    {
        if(wiced_hidd_cfg()->security_requirement_mask)
        {
            if (wiced_blehidd_is_link_encrypted())
            {
                OK_to_send = WICED_TRUE;
            }
        }
        else
        {
            OK_to_send = WICED_TRUE;
        }
    }
    else //start LE advertising
    {
        wiced_ble_hidd_link_connect();
    }

    if ( (rpt_db!=NULL) && OK_to_send && (wiced_bt_buffer_poolutilization (HCI_ACL_POOL_ID) < 80) && (--length > 0 ))
    {
        // find out if it is valid report to send
        for (i=0;i<hci_rpt_cnt;i++)
        {
            if ((type == rpt_db[i].rpt_type) && (report_id == rpt_db[i].rpt_id) && (length <= rpt_db[i].length))
                break;
        }

        // valid report info?
        if (i<hci_rpt_cnt)
        {
             //set gatt attribute value here before sending the report
            memcpy(rpt_db[i].rpt_buf, p_data, rpt_db[i].length);

            // Send the report
            wiced_ble_hidd_link_send_report(rpt_db[i].rpt_id, rpt_db[i].rpt_type, p_data, rpt_db[i].length);
        }
        else
        {
            WICED_BT_TRACE("\nInvalid rpt data: type:%d rpt_id:%d rpt_len:%d", type, report_id, length);
        }
    }
}

#endif

#ifdef BR_EDR_SUPPORT

/*
 * Handle host command to send HID report.
 */
void hci_hid_dev_handle_send_report( uint8_t channel, uint8_t type, uint8_t *p_data, uint16_t length )
{
    uint8_t  report_id = *p_data;
    uint16_t attr_handle;
    wiced_bool_t OK_to_send = WICED_FALSE;

    //send report only when link is connected
    if(wiced_bt_hidd_link_is_connected())
    {
        if(wiced_hidd_cfg()->security_requirement_mask)
        {
            if (bt_hidd_link.encrypt_status.encrypted)
            {
                OK_to_send = WICED_TRUE;
            }
        }
        else
        {
            OK_to_send = WICED_TRUE;
        }
    }
    else //enter discoverable/reconnecting
    {
        wiced_bt_hidd_link_connect();
    }

    if ( OK_to_send && (wiced_bt_buffer_poolutilization (HCI_ACL_POOL_ID) < 80) && (length > 0 ))
    {
        // Send the scroll rpt.
        wiced_bt_hidd_send_data(channel == HCI_CONTROL_HID_REPORT_CHANNEL_CONTROL, type, p_data, length);
    }
}

#endif

/*
 *  Pass protocol traces up through the UART
 */
void hci_hidd_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t * p_data )
{
    //send the trace
    wiced_transport_send_hci_trace( NULL, type, length, p_data  );
}

/*
 * handle reset command from UART
 */
void hci_hidd_dev_handle_reset_cmd( void )
{
    // trip watch dog now.
    wiced_hal_wdog_reset_system( );
}

/*
 * handle command from UART to configure traces
 */
void hci_control_handle_trace_enable( uint8_t *p_data )
{
    uint8_t hci_trace_enable = *p_data++;
//    wiced_debug_uart_types_t route_debug = (wiced_debug_uart_types_t)*p_data;
    wiced_debug_uart_types_t route_debug = WICED_ROUTE_DEBUG_TO_PUART; // force to use PUART for now until we figure out why using WICED_HCI will cause BR/EDR pairing to fail.

    WICED_BT_TRACE("\nHCI Traces:%s DebugRoute:%d", hci_trace_enable?"Enable":"Disable", route_debug);

    if ( hci_trace_enable )
    {
        /* Register callback for receiving hci traces */
        // Disable while streaming audio over the uart.
        wiced_bt_dev_register_hci_trace( hci_hidd_hci_trace_cback );
    }
    else
    {
        wiced_bt_dev_register_hci_trace( NULL);
    }
    wiced_set_debug_uart( route_debug );
}

/*
 * handle command to send paired host device address
 */
void hci_control_send_paired_host_info()
{
    uint8_t   tx_buf[(BD_ADDR_LEN+1)*HIDD_HOST_LIST_MAX+1];
    uint8_t   len = hidd_host_getInfo(tx_buf);
//    WICED_BT_TRACE("\nSend host info, count = %d", tx_buf[0]);
    hci_control_send_data( HCI_CONTROL_HIDD_EVENT_HOST_ADDR, tx_buf, len );
}

/*
 * send device state change
 */
void hci_control_send_state_change(uint8_t transport, uint8_t state)
{
    uint8_t   tx_buf[3];

    tx_buf[0] = 0;  // reserved for host id
    tx_buf[1] = transport;
    tx_buf[2] = state;
//    WICED_BT_TRACE("\nSend state change %d",state);
    hci_control_send_data( HCI_CONTROL_HIDD_EVENT_STATE_CHANGE, tx_buf, 3 );
}

/*
 * handle command to send local Bluetooth device address
 */
void hci_hidd_dev_handle_set_local_bda( uint8_t *bda )
{
    uint8_t   tx_buf[15];
    uint8_t   cmd = 0;
    int result;

    BD_ADDR bd_addr;
    STREAM_TO_BDADDR (bd_addr, bda);
    wiced_bt_set_local_bdaddr( bd_addr, BLE_ADDR_PUBLIC );
    WICED_BT_TRACE("\nSet address: %B", bd_addr);
#ifdef BR_EDR_SUPPORT
    wiced_bt_l2cap_set_desire_role(HCI_ROLE_SLAVE);
#endif
}

/* Handle get version command */
void hci_control_misc_handle_get_version( void )
{
    uint8_t   tx_buf[15];
    uint8_t   cmd = 0;
    uint32_t  chip = wiced_hidd_chip();

    tx_buf[cmd++] = WICED_SDK_MAJOR_VER;
    tx_buf[cmd++] = WICED_SDK_MINOR_VER;
    tx_buf[cmd++] = WICED_SDK_REV_NUMBER;
    tx_buf[cmd++] = WICED_SDK_BUILD_NUMBER & 0xFF;
    tx_buf[cmd++] = (WICED_SDK_BUILD_NUMBER>>8) & 0xFF;
    tx_buf[cmd++] = chip & 0xFF;
    tx_buf[cmd++] = (chip>>8) & 0xFF;
    tx_buf[cmd++] = (chip>>24) & 0xFF;
    tx_buf[cmd++] = 0;  // not used

    /* Send MCU app the supported features */
    tx_buf[cmd++] = HCI_CONTROL_GROUP_HIDD;

    hci_control_send_data( HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd );
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// send_report from HCI
/////////////////////////////////////////////////////////////////////////////////////////////
void hci_control_send_report(uint8_t * p_data, uint32_t data_len)
{
#ifdef BLE_SUPPORT
    if (wiced_hidd_host_transport()==BT_TRANSPORT_LE)
    {
        // for BLE there is no channel, skip p_data[0]
        hci_ble_hid_dev_handle_send_report( p_data[1], &p_data[2], ( uint16_t )( data_len - 2 ) );
    }
#endif
#ifdef BR_EDR_SUPPORT
    if (wiced_hidd_host_transport() == BT_TRANSPORT_BR_EDR)
    {
        hci_hid_dev_handle_send_report( p_data[0], p_data[1], &p_data[2], ( uint16_t )( data_len - 2 ) );
    }
#endif
}

void hci_hidd_handle_command( uint16_t cmd_opcode, uint8_t * p_data, uint32_t data_len )
{
    uint8_t bytes_written;

    WICED_BT_TRACE("\nCmd:%04X ", cmd_opcode);

    switch( cmd_opcode )
    {
    case HCI_CONTROL_COMMAND_RESET:
        WICED_BT_TRACE("Reset");
        hci_hidd_dev_handle_reset_cmd( );
        break;

    case HCI_CONTROL_COMMAND_TRACE_ENABLE:
        WICED_BT_TRACE("Trace %d %d", *p_data, *(p_data+1));
        hci_control_handle_trace_enable( p_data );
        break;

    case HCI_CONTROL_COMMAND_SET_LOCAL_BDA:
        WICED_BT_TRACE("Set address %B", p_data);
        hci_hidd_dev_handle_set_local_bda( p_data );
        break;

    case HCI_CONTROL_HIDD_COMMAND_ACCEPT_PAIRING:
        WICED_BT_TRACE("pairing");
        wiced_hidd_enter_pairing();
        break;

    case HCI_CONTROL_HIDD_COMMAND_HID_HOST_ADDR:  // reqesting for host paired host address
        WICED_BT_TRACE("Host info req.");
        hci_control_send_paired_host_info();
#ifdef BLE_SUPPORT
        hci_control_send_state_change(BT_TRANSPORT_LE, ble_hidd_link.subState);
#endif
#ifdef BR_EDR_SUPPORT
        hci_control_send_state_change(BT_TRANSPORT_BR_EDR, bt_hidd_link.subState);
#endif
        break;

    case HCI_CONTROL_HIDD_COMMAND_CONNECT:
        WICED_BT_TRACE("Connect");
        if (wiced_hidd_link_is_connected())
        {
            WICED_BT_TRACE("\nAlready connected, disconnecting instead...");
            wiced_hidd_disconnect();
        }
        else
        {
            wiced_hidd_link_connect();
        }
        break;

    case HCI_CONTROL_HIDD_COMMAND_DISCONNECT:
        WICED_BT_TRACE("Disconnect");
        if (wiced_hidd_link_is_connected())
        {
            wiced_hidd_disconnect();
        }
        else
        {
            WICED_BT_TRACE("\n Already disconnected, connecting instead...");
            wiced_hidd_link_connect();
        }
        break;

    case HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA:
    case HCI_CONTROL_HIDD_COMMAND_VIRTUAL_UNPLUG:
        WICED_BT_TRACE("Remove pairing");
        wiced_hidd_host_remove();
        break;

    case HCI_CONTROL_HIDD_COMMAND_SEND_REPORT:
        STRACE_ARRAY("Send Report:", p_data, data_len);
        hci_control_send_report(p_data, data_len);
        break;

    case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
        WICED_BT_TRACE("Get Version");
        hci_control_misc_handle_get_version();
        break;

    default:
        WICED_BT_TRACE("Ignored");
        break;
    }
}

uint32_t hci_hidd_dev_handle_command( uint8_t * p_data, uint32_t length )
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t status = HCI_CONTROL_STATUS_SUCCESS;
    uint8_t * p_rx_buf = p_data;

    if ( !p_rx_buf )
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }
    //Expected minimum 4 byte as the wiced header
    if( length < 4 )
    {
        WICED_BT_TRACE("\ninvalid params");
        wiced_transport_free_buffer( p_rx_buf );
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    READ_LITTLE_ENDIAN_TO_UINT16(opcode, p_data, length);     // Get opcode
    READ_LITTLE_ENDIAN_TO_UINT16(payload_len, p_data, length); // Get len

    hci_hidd_handle_command( opcode, p_data, payload_len );

    //Freeing the buffer in which data is received
    wiced_transport_free_buffer( p_rx_buf );
    return ( 0 );
}

/*
 *  transfer connection event to uart
 */
void hci_control_send_connect_evt( uint8_t addr_type, BD_ADDR addr, uint16_t con_handle, uint8_t role )
{
    int i;
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;

    *p++ = addr_type;
    for ( i = 0; i < 6; i++ )
        *p++ = addr[5 - i];
    *p++ = con_handle & 0xff;
    *p++ = ( con_handle >> 8 ) & 0xff;
    *p++ = role;

    hci_control_send_data( HCI_CONTROL_LE_EVENT_CONNECTED, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  transfer disconnection event to UART
 */
void hci_control_send_disconnect_evt( uint8_t reason, uint16_t con_handle )
{
    uint8_t   tx_buf [3];
    uint8_t   *p = tx_buf;

    *p++ = con_handle & 0xff;
    *p++ = ( con_handle >> 8 ) & 0xff;
    *p++ = reason;

    hci_control_send_data( HCI_CONTROL_LE_EVENT_DISCONNECTED, tx_buf, ( int )( p - tx_buf ) );
}

/*
 * Send notification to the host that pairing has been completed
 */
void hci_control_send_pairing_complete_evt( uint8_t result, uint8_t *p_bda, uint8_t type )
{
    uint8_t tx_buf[12];
    uint8_t *p = tx_buf;
    int i;

    *p++ = result;

    for ( i = 0; i < 6; i++ )
        *p++ = p_bda[5 - i];

    *p++ = type;

    hci_control_send_data( HCI_CONTROL_EVENT_PAIRING_COMPLETE, tx_buf, ( int ) ( p - tx_buf ) );
}

/*
 * hci_control_transport_status
 * This callback function is called when the MCU opens the Wiced UART
 */
void hci_hid_control_transport_status( wiced_transport_type_t type )
{
    WICED_BT_TRACE("\nhci_control_transport_status %x", type );

    // Tell Host that App is started
    hci_control_send_data( HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0 );
}

const wiced_transport_cfg_t  transport_cfg =
{
    WICED_TRANSPORT_UART,
    {{ WICED_TRANSPORT_UART_HCI_MODE, HCI_UART_BAUD_RATE }},
    { 0, 0},
    hci_hid_control_transport_status,
    hci_hidd_dev_handle_command,
    NULL
};

void hci_control_enable_trace()
{
    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace( hci_hidd_hci_trace_cback );
}

void hci_control_init_(int cnt, hci_rpt_db_t * rpt_db_ptr)
{
    // For BLE only, ignored in BR_EDR
    hci_rpt_cnt = cnt;
    rpt_db = rpt_db_ptr;

    WICED_BT_TRACE("\nTESTING_USING_HCI");
    wiced_transport_init( &transport_cfg );
    hci_control_enable_trace();
}

#else
 #ifdef HCI_TRACES_ENABLED
//
// The HCI commands are dumped PUART in byte array.
// User the tool TraceToSpy.exe to read the dump file to btspy to decode the HCI messages
//
void hci_trace_to_puart_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    WICED_BT_TRACE("\nHCI event type %d len %d\n", type,length );
    wiced_trace_array(  p_data, length );
}

void hci_control_enable_trace()
{
    WICED_BT_TRACE("\nHCI TRACE ENABLE");
    wiced_bt_dev_register_hci_trace( hci_trace_to_puart_cback );
}

 #endif
#endif