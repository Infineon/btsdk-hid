/**
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

/**
 * file hidd_lib.c
 *
 * This file consists of the implementation of functions that are
 * necessary for developing hid device use cases.y
 */

#include "wiced_bt_trace.h"
#include "gatt_utils_lib.h"
#include "hidd_lib.h"

#if HIDD_TRACE
# define HIDD_LIB_TRACE     WICED_BT_TRACE
# if LIB_TRACE>1
#  define HIDD_LIB_TRACE2   WICED_BT_TRACE
# else
#  define HIDD_LIB_TRACE2(...)
# endif
#else
# define HIDD_LIB_TRACE(...)
# define HIDD_LIB_TRACE2(...)
#endif

/******************************************************************************************
 * typedefs
 ******************************************************************************************/

/**
 * HIDD private variables
 */
static struct
{
    hidd_cfg_t * cfg;
    uint16_t     conn_id;

} hidd = {};

/******************************************************************************************
 * public functions
 ******************************************************************************************/

/**
 * Send HIDD input report by passing the report pointer and report length.
 */
wiced_bt_gatt_status_t hidd_send_report(void * ptr, uint16_t len)
{
    uint8_t * rpt = ptr;
    uint8_t rpt_id = *rpt++;
    len--;
    return hidd_send_data(rpt_id, HID_PAR_REP_TYPE_INPUT, rpt, len);
}

/**
 * Send HIDD report by providing report id, report type, report data pointer, and data length.
 */
wiced_bt_gatt_status_t hidd_send_data(uint8_t rpt_id, uint8_t rpt_type, uint8_t * p_data, uint16_t len)
{
    hidd_rpt_t * rpt = hidd.cfg->rpt_table;

    if (!hidd.conn_id)
    {
        HIDD_LIB_TRACE("Error!link is not up");
    }

    // based on report ID, find the handle in report table
    for( int i = 0; i < hidd.cfg->rpt_table_size; i++ )
    {
        // report id and report type must match
        if((rpt->rpt_type == rpt_type) && (rpt->rpt_id == rpt_id))
        {
            uint16_t handle = rpt->handle_val;

            // we found the handle value attribute
            const gatt_db_lookup_table_t * p_attribute = wiced_bt_util_get_attribute(hidd.cfg->gatt_lookup_table, handle);

            if (p_attribute != NULL)
            {
                // if not pointing to the same place, we copy the data
                if (p_attribute->p_data != p_data)
                {
                    // The length should be enough, but to prevent memory corruption, we check to make sure.
                    if (p_attribute->max_len < len)
                        len = p_attribute->max_len;
                    memcpy(p_attribute->p_data, p_data, len);
                }

                HIDD_LIB_TRACE("send notification, conn:0x%04x handle:0x%04x len:%d", link_conn_id(), handle, len);
                return wiced_bt_gatt_server_send_notification( hidd.conn_id, handle, len, p_attribute->p_data, NULL );
            }
            return WICED_BT_GATT_ATTRIBUTE_NOT_FOUND;
        }
        rpt++;
    }
    HIDD_LIB_TRACE("Error! Cannot find rpt_id:%d rpt_type:%d in rpt_table", rpt_id, rpt_type);
    return WICED_BT_GATT_INVALID_CFG;
}

/**
 * This function is called when GATT write handle is HID handle
 */
wiced_bt_gatt_status_t hidd_gatt_write_handler( uint16_t conn_id, wiced_bt_gatt_write_req_t * p_wr_data )
{
    uint16_t handle = p_wr_data->handle;

    // Check if the handle is for CCCD
    for (int i = 0; i < hidd.cfg->rpt_table_size; i++)
    {
        if (handle == hidd.cfg->rpt_table[i].handle_cccd)
        {
            if (hidd.cfg->cccd_cb)
            {
                return hidd.cfg->cccd_cb( conn_id, p_wr_data );
            }
        }
        // Check if this is writing to HID report
        else if (p_wr_data->handle == hidd.cfg->rpt_table[i].handle_val)
        {
            if (hidd.cfg->rpt_cb)
            {
                return hidd.cfg->rpt_cb( hidd.cfg->rpt_table[i].rpt_id, hidd.cfg->rpt_table[i].rpt_type, conn_id, p_wr_data );
            }
        }
    }

    return WICED_BT_GATT_ATTRIBUTE_NOT_FOUND;
}

/**
 * This function gathers all report's CCCD flags saved in given data pointers
 */
void hidd_get_cccd_flags(uint16_t * nflags, uint16_t * iflags)
{
    *nflags = *iflags = 0;
    uint16_t bit = 1;
    const gatt_db_lookup_table_t * p_attribute;

    for (int i = 0; i < hidd.cfg->rpt_table_size; i++)
    {
        p_attribute = wiced_bt_util_get_attribute(hidd.cfg->gatt_lookup_table, hidd.cfg->rpt_table[i].handle_cccd);

        if( p_attribute )
        {
            if (*(uint16_t *) p_attribute->p_data & GATT_CLIENT_CONFIG_NOTIFICATION)
            {
                *nflags |= bit;
            }
        }
        bit <<= 1;
    }
}

/**
 * This function go through report table to set notification and indications flags to each handle
 */
void hidd_set_cccd_flags(uint16_t notif_flags, uint16_t indicate_flags)
{
    const gatt_db_lookup_table_t * p_attribute;
    uint16_t bit = 1;

    for (int i = 0; i < hidd.cfg->rpt_table_size; i++)
    {
        p_attribute = wiced_bt_util_get_attribute(hidd.cfg->gatt_lookup_table, hidd.cfg->rpt_table[i].handle_cccd);

        if( p_attribute )
        {
            *(uint16_t *) p_attribute->p_data = ((notif_flags & bit) ? GATT_CLIENT_CONFIG_NOTIFICATION : 0) |
                                                ((indicate_flags & bit) ? GATT_CLIENT_CONFIG_INDICATION : 0);
            HIDD_LIB_TRACE("handle %x CCCD is set to %04x",rpt_map[i].handle_cccd, *(uint16_t *) p_attribute->p_data);
        }
        bit <<= 1;
    }
}

/**
 * This function clears all report's CCCD flags in each handle
 */
void hidd_clear_cccd_flags()
{
    const gatt_db_lookup_table_t * p_attribute;

    HIDD_LIB_TRACE("Clear all CCCD flags");
    for (int i = 0; i < hidd.cfg->rpt_table_size; i++)
    {
        p_attribute = wiced_bt_util_get_attribute(hidd.cfg->gatt_lookup_table, hidd.cfg->rpt_table[i].handle_cccd);

        if( p_attribute )
        {
            *(uint16_t *) p_attribute->p_data = 0;
        }
    }
}

/**
 * When link state changes, this function should be called to set the connection id.
 * A value 0 for connection id indicates the link is down.
 */
void hidd_set_conn_id( uint16_t conn_id )
{
    hidd.conn_id = conn_id;
}

/**
 * This is the first function to initialize this lib
 */
void hidd_init(hidd_cfg_t * cfg)
{
    hidd.cfg = cfg;
}

/* [] END OF FILE */
