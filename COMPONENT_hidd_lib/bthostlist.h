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
* File Name: bthostlist.h
*
* Abstract: BT Host List definitions and API functions
*
* Functions:
*
*******************************************************************************/
#ifndef _BT_PAIRING_LIST_H
#define _BT_PAIRING_LIST_H

#include "wiced_bt_dev.h"

//Host info VS ID
#define VS_BT_HOST_LIST   (WICED_NVRAM_VSID_START+2)

#define BT_PAIRING_HOST_MAX 1


#pragma pack(1)
typedef PACKED struct
{
    /// BD address of the bonded host
    BD_ADDR  bdAddr;

    /// Link key of the bonded host
    wiced_bt_device_link_keys_t  link_keys;

    union
    {
        uint16_t flags;

        struct  {

        /// Flag to indicate that BRR is to be used with this host
        uint16_t    brrEnabled : 1;

        /// Flag to indicate that UCD has been enabled and to be used with this host
        uint16_t    ucdEnabled : 1;

        /// Flag reserved
        uint16_t    reserved_1 : 1;

        /// Flag indicating whether link key is present
        uint16_t    linkKeyPresent : 1;

        /// Reserved bits
        uint16_t  reserved : 12;
        } fields;
    } un;

} tBtHostInfo;
#pragma pack()

enum
{
    /// Flag whether we need to use BRR when connecting to this host
    HOST_BRR_ENABLED = 0x01,

    /// Flag for whether we need to used UCD with this host
    HOST_UCD_ENABLED = 0x02,

    /// Mask for the position of the kink key present flag
    HOST_LINK_KEY_PRESENT_MASK = 0x08,

};

#define HOST_INFO_NOT_FOUND    0xff
#define HOST_INFO_INDEX_TOP    0

///////////////////////////////////////////////////////////////////////////////////////////////////
/// Read HID host information from NVRAM VS section and initialize btpairingHostInfoList
//////////////////////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_host_info_init(void);

////////////////////////////////////////////////////////////////////////////////
/// Get the number of HID hosts
///
/// \return number of HID hosts in this list
////////////////////////////////////////////////////////////////////////////////
uint8_t wiced_bt_hidd_host_info_get_number(void);

////////////////////////////////////////////////////////////////////////////////
/// Returns the index of the given device in the device list
///
/// \param bdAddr BD address of the device to find
///
/// \return index of host if it exists in the list, 0xFF if the host is not in the list
////////////////////////////////////////////////////////////////////////////////
uint8_t wiced_bt_hidd_host_info_find_index(const BD_ADDR bdAddr);

////////////////////////////////////////////////////////////////////////////////
/// Get the link key associated with the index of the devices
///
/// \param bdAddr device whose link key is needed
///
/// \return linkKey where the link key should be returned if found; NULL otherwise
////////////////////////////////////////////////////////////////////////////////
wiced_bt_device_link_keys_t *wiced_bt_hidd_host_info_get_linkkey_by_index(uint8_t index);

////////////////////////////////////////////////////////////////////////////////
/// Get the link key associated with the given device.
///
/// \param bdAddr device whose link key is needed
///
/// \return linkKey where the link key should be returned if found; NULL otherwise
////////////////////////////////////////////////////////////////////////////////
wiced_bt_device_link_keys_t *wiced_bt_hidd_host_info_get_linkkey_by_bdaddr(const BD_ADDR bdAddr);

////////////////////////////////////////////////////////////////////////////////
/// Get the BD addr associated with the index of the devices
///
/// \param index in the host list whose BD addr is needed
///
/// \return BD addr should be returned if found; NULL otherwise
////////////////////////////////////////////////////////////////////////////////
uint8_t *wiced_bt_hidd_host_info_get_bdaddr_by_index(uint8_t index);

////////////////////////////////////////////////////////////////////////////////
/// Removes the given device from the list. If the device is found and removed,
/// commits the host list to the VS.
///
/// \param bdAddr BD address of the device to remove
///
/// \return TRUE if the device was found and removed, FALSE if the device was not in the list
////////////////////////////////////////////////////////////////////////////////
uint8_t wiced_bt_hidd_host_info_remove_host(const BD_ADDR bdAddr);

////////////////////////////////////////////////////////////////////////////////
/// Moves the given device to the top (index 0) of the list and commits to the
/// volatile section if the device is present. If the device is not present,
/// it does nothing
///
/// \param bdAddr BD address of the device to move to the top
///
/// \return Old index of the device, 0xFF if the device is not in the list
////////////////////////////////////////////////////////////////////////////////
uint8_t wiced_bt_hidd_host_info_move_host_to_top(const BD_ADDR bdAddr);

////////////////////////////////////////////////////////////////////////////////
/// Adds the given device at index 0 and commits to VS. If the link key pointer
/// is NULL, it sets the link key to zero. The table is shifted down to make
/// room for the new device. If the table is already full the last element is
/// lost. This function checks for the existence of the device at top in the
/// list and if it does (and link key and flags also match), returns without
/// commiting. It Increases list size by one unless list is already at max size
///
/// \param bdAddr bluetooth address of device to add. Must not be null
/// \param linkKey link key to use with this address. If null, a 0 link key is
///        inserted
/// \param flags The extra flags that need to be stored for this host
////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_host_info_add_host_at_top(const BD_ADDR  bdAddr, wiced_bt_device_link_keys_t *linkKey, uint16_t flags);
////////////////////////////////////////////////////////////////////////////////
/// Update the link key at the given index if the provided key is different
/// than the stored key. If the link key is updated, commit the VS
///
/// \param index into the list
/// \param linkKey the new link key. Must not be NULL
///
/// \return TRUE if the link key was changed, FALSE if the link key stored is left unchanged
////////////////////////////////////////////////////////////////////////////////
uint8_t wiced_bt_hidd_host_info_update_linkkey_by_index(uint8_t index, wiced_bt_device_link_keys_t *linkKey);

////////////////////////////////////////////////////////////////////////////////
/// Update the link key associated with the given BD address and commit the VS.
/// If the BD address is not in the list, does nothing.
///
/// \param bdAddr host address whose link key should be updated
/// \param linkKey the new link key. Must not be NULL
///
/// \return TRUE if the link key was changed, FALSE if the link key stored is left unchanged
///           or if the host is not in the list
////////////////////////////////////////////////////////////////////////////////
uint8_t wiced_bt_hidd_host_info_update_linkkey_by_bdaddr(const BD_ADDR bdAddr, wiced_bt_device_link_keys_t *linkKey);


#endif
