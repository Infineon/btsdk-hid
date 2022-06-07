/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
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
* File Name: hidd_host.h
*
* Abstract: This file defines an interface for managing LE host lists, i.e.
* device address, link key, client configuration characteristic descriptor value
*******************************************************************************/

#ifndef _HIDD_HOST_LIST_
#define _HIDD_HOST_LIST_

#include "wiced_bt_dev.h"

#define HIDD_HOST_LIST_MAX 1

///////////////////////////////////////////////////////////////////////////////
/// Return current host count
///        returns WICED_TRUE if a host is paired
///////////////////////////////////////////////////////////////////////////////
uint8_t hidd_host_count();
#define hidd_is_paired() hidd_host_count()

///////////////////////////////////////////////////////////////////////////////
/// Return true if host already in the list
///////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_host_exist(const BD_ADDR host_bd_addr);

///////////////////////////////////////////////////////////////////////////////
/// Return current transport type
///
///        returns BT_TRANSPORT_LE
///                BT_TRANSPORT_BR_EDR
///                or 0 if not paired
///
///////////////////////////////////////////////////////////////////////////////
wiced_bt_transport_t hidd_host_transport();

///////////////////////////////////////////////////////////////////////////////
/// Return hidd_host_addr
///
///        returns host address, NULL if not paired
///
///////////////////////////////////////////////////////////////////////////////
uint8_t * hidd_host_addr();

///////////////////////////////////////////////////////////////////////////////
/// hidd_host_remove()
///////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_host_remove(void);

///////////////////////////////////////////////////////////////////////////////
/// hidd_host_remove()
///////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_host_remove_addr(wiced_bt_device_address_t host_bd_addr);

///////////////////////////////////////////////////////////////////////////////
/// hidd_host_set_flags set host flags
///
/// \param flgas
///
///////////////////////////////////////////////////////////////////////////////
uint16_t hidd_host_set_flags(const BD_ADDR bdAddr, uint16_t enable, uint16_t flags);

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
uint16_t hidd_host_get_flags();

///////////////////////////////////////////////////////////////////////////////
/// hidd_host_remove()
///////////////////////////////////////////////////////////////////////////////
void hidd_host_remove_all(void);

///////////////////////////////////////////////////////////////////////////////
/// Returns host addr type
///
///    returns If paired host is LE, return address type, otherwise return 0
///
///////////////////////////////////////////////////////////////////////////////
uint8_t hidd_host_addr_type();

///////////////////////////////////////////////////////////////////////////////////////////////////
/// hidd_host_setAddrType: activate host and set the address type
///   if host does not exist in database, it creates and add to a new host.
///   if host already exist, replace current host and move to active host.
///
/// \param bdAddr host address to activate
/// \param transport host address type
///
///////////////////////////////////////////////////////////////////////////////////////////////////
void hidd_host_setAddrType(const BD_ADDR bdAddr, uint8_t addrType);

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_host_isBonded();

/////////////////////////////////////////////////////////////////////////////////
/// hidd_host_getInfo
///   get paired host info.
///
/// \return data length
/////////////////////////////////////////////////////////////////////////////////
uint8_t hidd_host_getInfo(uint8_t * buf);

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void hidd_host_setBonded(wiced_bool_t bonded);

///////////////////////////////////////////////////////////////////////////////////////////////////
/// Read HID host information from NVRAM VS section and initialize hidhostlist_List
///
/// \param none
///
/// \return none
//////////////////////////////////////////////////////////////////////////////////////////////////
void hidd_host_init(void);

///////////////////////////////////////////////////////////////////////////////////////////////////
/// hidd_host_activate: set host with address as active host
//    if host is new, add a new host at the top
//    if host already exist and not on the top, move to the top
//    otherwise, it is already on the top, return FALSE for doing nothing.
///
/// \param bdAddr host address to activate
///
///////////////////////////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_host_activate(const BD_ADDR bdAddr);

///////////////////////////////////////////////////////////////////////////////////////////////////
/// hidd_host_setTransport: set host with address as active host
//    if host is new, add a new host at the top
//    if host already exist and not on the top, move to the top
//    otherwise, it is already on the top, return FALSE for doing nothing.
///
/// \param bdAddr host address to activate
///        transport, host transport
///
///////////////////////////////////////////////////////////////////////////////////////////////////
void hidd_host_setTransport(const BD_ADDR bdAddr, wiced_bt_transport_t transport);

///////////////////////////////////////////////////////////////////////////////////////////////////
/// hidd_host_setLinkKey: activate host with link info
///   if host does not exist in database, it creates and add to new host.
///   if host already exist, replace current host and move to active host.
///
/// \param bdAddr host address to activate
/// \param link_key host link_key
/// \param transport host tranport type
///
///////////////////////////////////////////////////////////////////////////////////////////////////
void hidd_host_setLinkKey(const BD_ADDR bdAddr, wiced_bt_device_link_keys_t * link_key);

///////////////////////////////////////////////////////////////////////////////////////////////////
/// hidd_host_getLinkKey: get a copy of active link key data
///  return FALSE if bdaddr does not match
///////////////////////////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_host_getLinkKey(const BD_ADDR bdAddr, wiced_bt_device_link_keys_t * link_key);
const wiced_bt_device_link_keys_t * hidd_host_getLinkKeyPtr();

#endif // _HIDD_HOST_LIST_
