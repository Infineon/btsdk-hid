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
#include "hidd_lib.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

#if HIDD_TRACE
# define HIDD_LIB_TRACE     WICED_BT_TRACE
# if HIDD_TRACE>1
#  define HIDD_LIB_TRACE2   HIDD_LIB_TRACE
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

} hidd = {0};

/******************************************************************************************
 * public functions
 ******************************************************************************************/

/**
 * Get attribute data from the look up table
 */
const gatt_db_lookup_table_t * hidd_get_attribute(uint16_t handle)
{
    gatt_db_lookup_table_t * p_attribute = hidd.cfg->gatt_lookup_table;
    uint16_t limit = hidd.cfg->gatt_lookup_table_size ? hidd.cfg->gatt_lookup_table_size : app_gatt_db_ext_attr_tbl_size; // for backward comptiability, if gatt_lookup_table_size is not assigned, use BT configurator generated code value.

    if (handle)
    {
        while(limit--)
        {
            if(p_attribute->handle == handle)
            {
                return p_attribute;
            }

            p_attribute++;
        }
        WICED_BT_TRACE("Requested attribute 0x%04x not found!!!", handle);
    }
    return NULL;
}

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
            const gatt_db_lookup_table_t * p_attribute = hidd_get_attribute(handle);

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

                HIDD_LIB_TRACE("send notification, conn:0x%04x handle:0x%04x len:%d", hidd.conn_id, handle, len);
                return wiced_bt_gatt_send_notification( hidd.conn_id, handle, len, p_attribute->p_data );
            }
            return WICED_BT_GATT_ATTRIBUTE_NOT_FOUND;
        }
        rpt++;
    }
    WICED_BT_TRACE("Error! Cannot find rpt_id:%d rpt_type:%d in rpt_table", rpt_id, rpt_type);
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
        // Check if this is writing to HID report
        if (p_wr_data->handle == hidd.cfg->rpt_table[i].handle_val)
        {
            HIDD_LIB_TRACE("writing HID report handle 0x%04x", handle);
            if (hidd.cfg->rpt_cb)
            {
                return hidd.cfg->rpt_cb( hidd.cfg->rpt_table[i].rpt_id, hidd.cfg->rpt_table[i].rpt_type, conn_id, p_wr_data );
            }
        }
    }

    UNUSED_VARIABLE(handle);
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
        p_attribute = hidd_get_attribute(hidd.cfg->rpt_table[i].handle_cccd);

        if( p_attribute )
        {
            if (*(uint16_t *) p_attribute->p_data & GATT_CLIENT_CONFIG_NOTIFICATION)
            {
                *nflags |= bit;
            }
        }
        bit <<= 1;
    }
    HIDD_LIB_TRACE("hidd_get_cccd_flags got 0x%04x, 0x%04x", *nflags, *iflags);
}

/**
 * This function go through report table to set notification and indications flags to each handle
 */
void hidd_set_cccd_flags(uint16_t notif_flags, uint16_t indicate_flags)
{
    const gatt_db_lookup_table_t * p_attribute;
    uint16_t bit = 1;

    HIDD_LIB_TRACE("hidd_set_cccd_flags to 0x%04x, 0x%04x", notif_flags, indicate_flags);
    for (int i = 0; i < hidd.cfg->rpt_table_size; i++)
    {
        p_attribute = hidd_get_attribute(hidd.cfg->rpt_table[i].handle_cccd);

        if( p_attribute )
        {
            *(uint16_t *) p_attribute->p_data = ((notif_flags & bit) ? GATT_CLIENT_CONFIG_NOTIFICATION : 0) |
                                                ((indicate_flags & bit) ? GATT_CLIENT_CONFIG_INDICATION : 0);
            HIDD_LIB_TRACE2("handle %x CCCD is set to 0x%04x",hidd.cfg->rpt_table[i].handle_cccd, *(uint16_t *) p_attribute->p_data);
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

    HIDD_LIB_TRACE("Clear all CCCD flags, rpt table size = %d", hidd.cfg->rpt_table_size);
    for (int i = 0; i < hidd.cfg->rpt_table_size; i++)
    {
        HIDD_LIB_TRACE2("Index %d CCCD handle 0x%04x", i, hidd.cfg->rpt_table[i].handle_cccd);

        if (hidd.cfg->rpt_table[i].handle_cccd)
        {
            p_attribute = hidd_get_attribute(hidd.cfg->rpt_table[i].handle_cccd);

            if( p_attribute )
            {
                HIDD_LIB_TRACE2("Index %d CCCD handle 0x%04x cleared", i, hidd.cfg->rpt_table[i].handle_cccd);
                *(uint16_t *) p_attribute->p_data = 0;
            }
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
