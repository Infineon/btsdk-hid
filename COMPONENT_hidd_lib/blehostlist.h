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

/*******************************************************************************
*
* File Name: blehostlist.h
*
* Abstract: This file defines an interface for managing LE host lists, i.e.
* device address, link key, client configuration characteristic descriptor value
*******************************************************************************/

#ifndef _BLE_HOST_LIST_
#define _BLE_HOST_LIST_

//Host info VS ID
#define   VS_BLE_HOST_LIST  (WICED_NVRAM_VSID_START+1)


#define BLEHOSTLIST_MAX 1
#define BLEHOSTLIST_EFFECTIVE_FLAGS_MASK  (0x7FFF)

#pragma pack(1)
typedef PACKED struct
{
    uint8_t bdAddress[BD_ADDR_LEN];
    wiced_bt_device_link_keys_t  link_keys;

    uint16_t flags : 15;
    uint16_t entryValid : 1;

    uint8_t  addrType;
    uint8_t  is_bonded;
} BleHostList;
#pragma pack()

///////////////////////////////////////////////////////////////////////////////////////////////////
/// Read HID host information from NVRAM VS section and initialize blehostlist_List
///
/// \param none
///
/// \return none
//////////////////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_host_info_init(void);

///////////////////////////////////////////////////////////////////////////////////////////////////
/// get the number of HID host
///
/// \param none
///
/// \return the number of HID host
//////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t wiced_ble_hidd_host_info_get_number(void);

///////////////////////////////////////////////////////////////////////////////////////////////////
/// Check if the first HID host is bonded
///
/// \param none
///
/// \return WICED_TRUE/WICED_FALSE
//////////////////////////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_ble_hidd_host_info_is_bonded(void);

///////////////////////////////////////////////////////////////////////////////////////////////////
/// get first HID host BD ADDR
///
/// \param none
///
/// \return BD ADDR of the HID host
//////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t *wiced_ble_hidd_host_info_get_bdaddr (void);

///////////////////////////////////////////////////////////////////////////////////////////////////
/// get first HID host Link Keys
///
/// \param none
///
/// \return Link Keys of the HID host
//////////////////////////////////////////////////////////////////////////////////////////////////
wiced_bt_device_link_keys_t *wiced_ble_hidd_host_info_get_link_keys(void);

///////////////////////////////////////////////////////////////////////////////////////////////////
/// Add new HID host to the first position of blehostlist_List
///
/// \param bdAddr - BD ADDR of the HID Host
/// \param bdAddrType - Address Type of the HID Host
/// \param link_keys - pointer to the Link keys of the HID Host
/// \param flags  - the bitmap corresponding to client characteristic configuration values for notification
///
/// \return none
//////////////////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_host_info_add_first(uint8_t* bdAddr, uint8_t bdAddrType, wiced_bt_device_link_keys_t *link_keys, uint16_t flags);

///////////////////////////////////////////////////////////////////////////////////////////////////
/// get client configuration values for the HID host
///
/// \param bdAddr - BD ADDR of the HID Host
/// \param bdAddrType - Address Type of the HID Host
///
/// \return the bitmap corresponding to client characteristic configuration values for notification
//////////////////////////////////////////////////////////////////////////////////////////////////
int32_t wiced_ble_hidd_host_info_get_flags(uint8_t* bdAddr, uint8_t bdAddrType);

///////////////////////////////////////////////////////////////////////////////////////////////////
/// set the HID host to invalid and update NVRAM VS section
///
/// \param bdAddr - BD ADDR of the HID Host
/// \param bdAddrType - Address Type of the HID Host
///
/// \return none
//////////////////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_host_info_delete(uint8_t* bdAddr, uint8_t addrType);

///////////////////////////////////////////////////////////////////////////////////////////////////
/// Delete all HID hosts from NVRAM VS section and reset blehostlist_List to 0
///
/// \param none
///
/// \return none
//////////////////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_host_info_delete_all(void);

///////////////////////////////////////////////////////////////////////////////////////////////////
/// update client configuration value for the HID host
///
/// \param bdAddr - BD ADDR of the HID Host
/// \param bdAddrType - Address Type of the HID Host
/// \param enable - allow/disallow notification
/// \param featureBit - bit/bits in the bitmap of client configuration characteristics
///
/// \return the final flags
//////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t wiced_ble_hidd_host_info_update_flags(uint8_t* bdAddr, uint8_t bdAddrType, uint16_t enable,uint16_t featureBit);

///////////////////////////////////////////////////////////////////////////////////////////////////
/// retrieve the first HID host (BD ADDR and address type) in the list of blehostlist_List
///
/// \param bdAddr - pointer point to the BD ADDR
/// \param bdAddrType - pointer point to the address type
///
/// \return WICED_TRUE/WICED_FALSE
//////////////////////////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_ble_hidd_host_info_get_first_host(uint8_t** bdAddr, uint8_t *bdAddrType);

///////////////////////////////////////////////////////////////////////////////////////////////////
/// update the first HID host (BD ADDR and address type) in the list of blehostlist_List
///
/// \param bdAddr - BD ADDR
/// \param bdAddrType - address type
///
/// \return WICED_TRUE/WICED_FALSE
//////////////////////////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_ble_hidd_host_info_update_first_host(uint8_t *bdAddr, uint8_t bdAddrType);

///////////////////////////////////////////////////////////////////////////////////////////////////
/// Add host to the Resolving List (RL) if its address type is not public address
///
/// \param none
///
/// \return none
//////////////////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_host_info_add_to_resolving_list(void);
#endif
