/*
 *  Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
 *  an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * 
 *  This software, including source code, documentation and related
 *  materials ("Software") is owned by Cypress Semiconductor Corporation
 *  or one of its affiliates ("Cypress") and is protected by and subject to
 *  worldwide patent protection (United States and foreign),
 *  United States copyright laws and international treaty provisions.
 *  Therefore, you may use this Software only as provided in the license
 *  agreement accompanying the software package from which you
 *  obtained this Software ("EULA").
 *  If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 *  non-transferable license to copy, modify, and compile the Software
 *  source code solely for use in connection with Cypress's
 *  integrated circuit products.  Any reproduction, modification, translation,
 *  compilation, or representation of this Software except as specified
 *  above is prohibited without the express written permission of Cypress.
 * 
 *  Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 *  EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 *  reserves the right to make changes to the Software without notice. Cypress
 *  does not assume any liability arising out of the application or use of the
 *  Software or any product or circuit described in the Software. Cypress does
 *  not authorize its products for use in any products where a malfunction or
 *  failure of the Cypress product may reasonably be expected to result in
 *  significant property damage, injury or death ("High Risk Product"). By
 *  including Cypress's product in a High Risk Product, the manufacturer
 *  of such system or application assumes all risk of such use and in doing
 *  so agrees to indemnify Cypress against all liability.
 */

/**
 * @file   hidh_lib_core.h
 *
 * @brief  hidh_lib core function header file
 */

#ifndef __HIDH_LIB_CORE_H_
#define __HIDH_LIB_CORE_H_

#include "wiced_bt_dev.h"
#include "wiced_bt_gatt.h"
#include "hidh_lib_int.h"

/******************************************************
 *                 Global Variables
 ******************************************************/
extern hidh_le_cb_t hidh_le_cb;

/*
 * @brief     Allocate a structure to save information about a LE HID device
 *            This function is called for LE HID Host Initialization.
 *            This function must be called, once, before any other LE HIDH functions.
 *
 * @param[in] bdaddr   : BD Addr
 *
 * @return    None
 */
hidh_le_dev_t *hidh_le_core_dev_alloc(wiced_bt_device_address_t bdaddr);

/*
 * @brief     Free a structure to save information about a LE HID device
 *
 * @param[in] p_dev   : device pointer
 *
 * @return    None
 */
void hidh_le_core_dev_free(hidh_le_dev_t *p_dev);

/*
 * @brief     Retrieve a structure containing information about a LE HID device from it's BdAddr
 *
 * @param[in] bdaddr   : BD Addr
 *
 * @return    device pointer
 */
hidh_le_dev_t *hidh_le_core_dev_from_bdaddr(wiced_bt_device_address_t bdaddr);

/*
 * @brief     Retrieve a structure containing information about a LE HID device from it's Connection Handle
 *
 * @param[in] conn_id   : device handle
 *
 * @return    device pointer
 */
hidh_le_dev_t *hidh_le_core_dev_from_conn_id(uint16_t conn_id);

/*
 * @brief     This function is called when the LE HID GATT operation is complete
 *
 * @param[in] p_dev   : device pointer
 * @param[in] status  : gatt complete status
 *
 * @return    none
 */
void hidh_le_core_gatt_complete(hidh_le_dev_t *p_dev,
        hidh_le_status_t status);

/*
 * @brief     This function will check (one by one) all the event filter registered.
 *            If none 'catch' it, the event will be sent to the application
 *
 * @param[in] event         : event
 * @param[in] p_event_data  : event data
 *
 * @return    none
 */
void hidh_le_core_callback(hidh_le_event_t event,
        hidh_le_event_data_t *p_event_data);

#endif // __HIDH_LIB_CORE_H_

/* end of file */
