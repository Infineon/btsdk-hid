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
* File Name: bthostlist.c
*
* Abstract: This file implements the BT Host List storing/retrieving to/from NVRAM
*
* Functions:
*
*******************************************************************************/
#ifndef LE_HIDD_ONLY

#include "bthostlist.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"

tBtHostInfo btpairingHostInfoList[BT_PAIRING_HOST_MAX];
uint8_t btpairingHostInfoListNum = 0;


void bthostlist_clear(void);
void bthostlist_commit(void);
uint16_t bthostlist_getFlagsbyIndex(uint8_t index);


///////////////////////////////////////////////////////////////////////////////////////////////////
/// Read HID host information from NVRAM VS section and initialize btpairingHostInfoList
///
/// \param none
///
/// \return none
//////////////////////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_host_info_init(void)
{
    uint16_t dataSize;
    wiced_result_t result;


    bthostlist_clear();

    dataSize = wiced_hal_read_nvram(VS_BT_HOST_LIST, BT_PAIRING_HOST_MAX * sizeof(tBtHostInfo), (uint8_t *)btpairingHostInfoList, &result);

    if( dataSize == 0 )
    {
        WICED_BT_TRACE("btpairingHostInfoList is empty\n");
    }
    else
    {
        btpairingHostInfoListNum = dataSize / sizeof(tBtHostInfo);
#if 0
        wiced_bt_dev_add_device_to_address_resolution_db ( &btpairingHostInfoList[0].link_keys );
        WICED_BT_TRACE("\n load_keys_for_address_resolution %B\n", (uint8_t*)&btpairingHostInfoList[0].link_keys );
        wiced_trace_array((uint8_t*)&btpairingHostInfoList[0].link_keys.key_data, BTM_SECURITY_KEY_DATA_LEN);
#endif
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// initialize btpairingHostInfoList to all 0
///
/// \param none
///
/// \return none
//////////////////////////////////////////////////////////////////////////////////////////////////
void bthostlist_clear(void)
{
    btpairingHostInfoListNum = 0;

    memset(btpairingHostInfoList, 0x00, BT_PAIRING_HOST_MAX * sizeof(tBtHostInfo));
}

////////////////////////////////////////////////////////////////////////////////
/// Shifts down the list at index.
/// If the index in invalid, it return false and does nothing.
/// Otherwise, btpairingHostInfoListNum increased by 1 after shifting.
///
/// For example, if the index 1 and the btpairingHostInfoListNum is 4 (we have 4 host elements),
/// element 1-3 are moved to elements 2-4 and element 1 is cleared.
/// Element 0 is untouched.
///
/// \param index of element to freed for new host to be inserted
/// \return TRUE if shifted
////////////////////////////////////////////////////////////////////////////////
uint8_t bthostlist_shiftListDown(uint8_t index)
{
    // make sure the index is valid
    if (index <= btpairingHostInfoListNum)
    {
        if (index < btpairingHostInfoListNum)
        {
            // Use memmove to ensure that overalpping areas are moved correctly
            memmove(&btpairingHostInfoList[index+1],
                    &btpairingHostInfoList[index],
                    sizeof(tBtHostInfo)*(btpairingHostInfoListNum - index));
        }
        // Clear the new element data at index
        memset(&btpairingHostInfoList[index], 0, sizeof(tBtHostInfo));

        // Now we have one more host element
        btpairingHostInfoListNum++;

        return TRUE;
    }
    return FALSE;
}

////////////////////////////////////////////////////////////////////////////////
/// Shifts the host list up to the index. After a valid shifting, btpairingHostInfoListNum will be declemented.
///
/// For example if this is called with index = 1 and we btpairingHostInfoListNum = 4 (4 host elements),
/// element 2-3 are moved to elements 1-2 and element 3 is cleared and btpairingHostInfoListNum will become 3.
/// Element 0 is untouched.
///
/// \param index, the element overwritten by the shift
////////////////////////////////////////////////////////////////////////////////
void bthostlist_shiftListUp(uint8_t index)
{
    // make sure index is valid
    if (index < btpairingHostInfoListNum)
    {
        // We are removing one host
        btpairingHostInfoListNum--;

        // Use memmove to ensure that overalpping areas are moved correctly
        memmove(&btpairingHostInfoList[index],
                &btpairingHostInfoList[index+1],
                sizeof(tBtHostInfo)*(btpairingHostInfoListNum - index));

        // Clear the freed element
        memset(&btpairingHostInfoList[btpairingHostInfoListNum], 0, sizeof(tBtHostInfo));
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Returns the index of the given device in the device list
/// \param bdAddr BD address of the device to find
/// \return index of host if it exists in the list, HOST_INFO_NOT_FOUND if the host is not in the list
////////////////////////////////////////////////////////////////////////////////
uint8_t wiced_bt_hidd_host_info_find_index(const BD_ADDR bdAddr)
{
    uint8_t index;

    // Go through all the valid entries in the table
    for (index=0; index < btpairingHostInfoListNum; index++)
    {
        if (memcmp(&btpairingHostInfoList[index].bdAddr, bdAddr, sizeof(btpairingHostInfoList[index].bdAddr)) == 0)
        {
            // Got it! Return the index
            return index;
        }
    }

    // If we get here, the address doesn't exist.
    return HOST_INFO_NOT_FOUND;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the link key associated with the given device.
/// \param bdAddr device whose link key is needed
/// \return linkKey where the link key should be returned if found; NULL otherwise
////////////////////////////////////////////////////////////////////////////////
wiced_bt_device_link_keys_t *wiced_bt_hidd_host_info_get_linkkey_by_bdaddr(const BD_ADDR bdAddr)
{
    return wiced_bt_hidd_host_info_get_linkkey_by_index( wiced_bt_hidd_host_info_find_index(bdAddr) );
}

////////////////////////////////////////////////////////////////////////////////
/// Get the link key associated with the index of the devices
/// \param index in the host list whose link key is needed
/// \return linkKey where the link key should be returned if found; NULL otherwise
////////////////////////////////////////////////////////////////////////////////
wiced_bt_device_link_keys_t *wiced_bt_hidd_host_info_get_linkkey_by_index(uint8_t index)
{
    // Do we have it
    if (index < btpairingHostInfoListNum)
    {
        // Yes. Do we have a link key?
        if (btpairingHostInfoList[index].un.fields.linkKeyPresent)
        {
            return (&btpairingHostInfoList[index].link_keys);
        }
    }

    // Nope. Inform caller we don't have a link key for this device
    return NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the BD addr associated with the index of the devices
/// \param index in the host list whose BD addr is needed
/// \return BD addr should be returned if found; NULL otherwise
////////////////////////////////////////////////////////////////////////////////
uint8_t *wiced_bt_hidd_host_info_get_bdaddr_by_index(uint8_t index)
{
    // Do we have it
    if (index < btpairingHostInfoListNum)
    {
        return btpairingHostInfoList[index].bdAddr;
    }

    // Nope. Inform caller we don't have a link key for this device
    return NULL;
}
////////////////////////////////////////////////////////////////////////////////
/// Get all BD addresses in the hostList
/// \param pointer to buffer to fill with BD addresses
/// \return number of hosts in the hostList
////////////////////////////////////////////////////////////////////////////////
uint8_t bthostlist_getAllHostAddresses(uint8_t* buffer)
{
    uint8_t i;

    // copy the BD_ADDR of each hostList entry into user provided buffer
    for (i = 0; i < btpairingHostInfoListNum; i++)
    {
        memcpy( buffer + i * sizeof(BD_ADDR), &btpairingHostInfoList[i], sizeof(BD_ADDR) );
    }

    return btpairingHostInfoListNum;
}

////////////////////////////////////////////////////////////////////////////////
/// Removes the given device from the list. If the device is found and removed,
/// commits the host list to the VS.
/// \param bdAddr BD address of the device to remove
/// \return TRUE if the device was found and removed, FALSE if the device was not in the list
////////////////////////////////////////////////////////////////////////////////
uint8_t wiced_bt_hidd_host_info_remove_host(const BD_ADDR bdAddr)
{
    uint8_t index = wiced_bt_hidd_host_info_find_index(bdAddr);

    // Check if the host is in the list
    if ( index == HOST_INFO_NOT_FOUND)
    {
        // Host is not in the list. Return FALSE
        return FALSE;
    }

    // The device is in the list. Compress the list, moving everything up
    // overwriting the device entry.
    bthostlist_shiftListUp(index);

    // Now commit the change
    bthostlist_commit();

    // Done!
    return TRUE;
}

////////////////////////////////////////////////////////////////////////////////
/// Removes extra hosts from the host list and commits the host list to the VS.
/// \return TRUE if any extra hosts were found and removed, otherwise FALSE
////////////////////////////////////////////////////////////////////////////////
uint8_t bthostlist_removeExtraHosts(void)
{
    if (btpairingHostInfoListNum <= 1)
    {
        // there are no extra Hosts
        return FALSE;
    }

    // Clear all entries except the first one
    memset(&btpairingHostInfoList[1], 0, (BT_PAIRING_HOST_MAX - 1) * sizeof(tBtHostInfo));

    // there is now one host
    btpairingHostInfoListNum = 1;

    // commit the change
    bthostlist_commit();

    // done
    return TRUE;
}

////////////////////////////////////////////////////////////////////////////////
/// Moves the given device to the top (index 0) of the list and commits to the
/// volatile section if the device is present. If the device is not present,
/// it does nothing
/// \param bdAddr BD address of the device to move to the top
/// \return Old index of the device, HOST_INFO_NOT_FOUND if the device is not in the list
////////////////////////////////////////////////////////////////////////////////
uint8_t wiced_bt_hidd_host_info_move_host_to_top(const BD_ADDR bdAddr)
{
    uint8_t index = wiced_bt_hidd_host_info_find_index(bdAddr);
    tBtHostInfo tmpEntry;

    // Check if the host is in the list
    if ( index != HOST_INFO_NOT_FOUND)
    {
        // Save copy of the device entry
        memcpy(&tmpEntry, &btpairingHostInfoList[index], sizeof(tBtHostInfo));

        // delete current one
        bthostlist_shiftListUp(index);

        // now we want to add to the top
        bthostlist_shiftListDown(HOST_INFO_INDEX_TOP);

        // Copy the saved host to the top
        memcpy(&btpairingHostInfoList[HOST_INFO_INDEX_TOP], &tmpEntry, sizeof(tBtHostInfo));

        // Now commit to the VS
        bthostlist_commit();
    }

    // Send the old index back
    return index;
}

////////////////////////////////////////////////////////////////////////////////
/// Adds the given host at the given index and commits the VS.
/// If the link key pointer is NULL, it sets the link key to zero.
/// Note that this overrides the value
/// at the given index. This function does not update the number of elements
/// in the list.
/// \param bdAddr bluetooth address of device to add. Must not be null
/// \param linkKey link key to use with this address. If null, the entry is
///          created with no link key present
/// \param index in the table where to put this device. Any value here is overwritten
/// \param flags The extra flags for BRR, UCD, Multicast etc that needs to be stored
////////////////////////////////////////////////////////////////////////////////
void bthostlist_addHostAtIndex(const BD_ADDR bdAddr,
                                   wiced_bt_device_link_keys_t *linkKey,
                                   uint8_t index, uint16_t flags)
{
    // Copy the BD address
    memcpy(&btpairingHostInfoList[index].bdAddr, bdAddr, sizeof(BD_ADDR));

    // Set the host feature flags - the linkKeyPresent flag will be set up correctly below
    btpairingHostInfoList[index].un.flags = flags;

    // Check if the link key pointer is NULL
    if (!linkKey)
    {
        // Zero out the link key
        memset(&btpairingHostInfoList[index].link_keys, 0, sizeof(wiced_bt_device_link_keys_t));

        // Flag that link key is not present
        btpairingHostInfoList[index].un.fields.linkKeyPresent = FALSE;
    }
    else
    {
        // Copy the link key
        memcpy(&btpairingHostInfoList[index].link_keys, linkKey, sizeof(wiced_bt_device_link_keys_t));

        // Flag that the link key is present
        btpairingHostInfoList[index].un.fields.linkKeyPresent = TRUE;
    }

    // Commit to VS
    bthostlist_commit();

}

////////////////////////////////////////////////////////////////////////////////
/// Adds the given device at index 0 and commits to VS. If the link key pointer
/// is NULL, it sets the link key to zero. The table is shifted down to make
/// room for the new device. If the table is already full the last element is
/// lost. This function checks for the existence of the device at top in the
/// list and if it does (and link key and flags also match), returns without
/// commiting. It Increases list size by one unless list is already at max size
/// \param bdAddr bluetooth address of device to add. Must not be null
/// \param linkKey link key to use with this address. If null, a 0 link key is
///        inserted
/// \param flags The extra flags that need to be stored for this host
////////////////////////////////////////////////////////////////////////////////
void wiced_bt_hidd_host_info_add_host_at_top(const BD_ADDR  bdAddr,
                                 wiced_bt_device_link_keys_t *linkKey, uint16_t flags)
{
    uint8_t idx = wiced_bt_hidd_host_info_find_index(bdAddr);

    // if idx is not already at top, we need to do something
    if (idx != HOST_INFO_INDEX_TOP)
    {
        // if host is already in the list and it is not top, we need to shift to the top
        if (idx != HOST_INFO_NOT_FOUND)
        {
            // delete current host element
            bthostlist_shiftListUp(idx);
        }
        idx = HOST_INFO_INDEX_TOP;
        // now we make room for the new host
        bthostlist_shiftListDown(idx);
    }
    else
    {
        // If this host is already at the top, check if we need to update information
        if (bthostlist_getFlagsbyIndex(idx) == flags)
        {
            // Check if link key is present and the same as what we know or not present
            // and not provided
            if((linkKey &&
                btpairingHostInfoList[idx].un.fields.linkKeyPresent &&
                (memcmp(&btpairingHostInfoList[idx].link_keys, linkKey, sizeof(wiced_bt_device_link_keys_t)) == 0)) ||
               (!linkKey && !btpairingHostInfoList[idx].un.fields.linkKeyPresent))
            {
                return;
            }
        }
    }
    // Add the host at the top and commits to the VS
    bthostlist_addHostAtIndex(bdAddr, linkKey, idx, flags);
}

////////////////////////////////////////////////////////////////////////////////
/// Update the link key at the given index if the provided key is different
/// than the stored key. If the link key is updated, commit the VS
///
/// \param index into the list
/// \param linkKey the new link key. Must not be NULL
///
/// \return TRUE if the link key was changed, FALSE if the link key stored is left unchanged
////////////////////////////////////////////////////////////////////////////////
uint8_t wiced_bt_hidd_host_info_update_linkkey_by_index(uint8_t index,
                                     wiced_bt_device_link_keys_t *linkKey)
{
    // Check if link key is not present or different
    if ((!btpairingHostInfoList[index].un.fields.linkKeyPresent) ||
        (memcmp(linkKey, &btpairingHostInfoList[index].link_keys, sizeof(wiced_bt_device_link_keys_t)) != 0))
    {
        // Save it
        memcpy(&btpairingHostInfoList[index].link_keys, linkKey, sizeof(wiced_bt_device_link_keys_t));

        // Flag that the link key is present
        btpairingHostInfoList[index].un.fields.linkKeyPresent = 1;

        // Commit the VS
        bthostlist_commit();

        // Inform the caller
        return TRUE;
    }

    // Key is the same. Leave it alone
    return FALSE;
}

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
uint8_t wiced_bt_hidd_host_info_update_linkkey_by_bdaddr(const BD_ADDR bdAddr,
                                     wiced_bt_device_link_keys_t *linkKey)
{
    // Find the index associated with this host
    uint8_t index = wiced_bt_hidd_host_info_find_index(bdAddr);

    // Did we find the host
    if (index != HOST_INFO_NOT_FOUND)
    {
        // Yes. Update associated link key and return result
        return wiced_bt_hidd_host_info_update_linkkey_by_index(index, linkKey);
    }

    // Didn't find the host. Do nothing and let caller know we changed nothing
    return FALSE;
}

////////////////////////////////////////////////////////////////////////////////
/// Update the host flags associated with the given BD addr
/// If the BD address is not in the list, does nothing.
///
/// \param bdAddr host address whose feature flags should be updated
/// \param flags the new flags
///
/// \return TRUE if successful; else false
////////////////////////////////////////////////////////////////////////////////
uint8_t bthostlist_updateFlags(const BD_ADDR bdAddr, UINT16 flags)
{
    // Find the index associated with this host
    uint8_t index = wiced_bt_hidd_host_info_find_index(bdAddr);

    // Did we find the host
    if (index != HOST_INFO_NOT_FOUND)
    {
        // Clear out all bits except link key present
        btpairingHostInfoList[index].un.flags &= HOST_LINK_KEY_PRESENT_MASK;

        // Update the feature flags
        btpairingHostInfoList[index].un.flags |= (flags & ~HOST_LINK_KEY_PRESENT_MASK);

        // Commit the VS
        bthostlist_commit();

        return TRUE;
    }

    // Didn't find the host. Do nothing and let caller know we changed nothing
    return FALSE;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the flags for the nth element. Bounds not checked!
/// \param index index index of the entry to retrieve
/// \return flags
////////////////////////////////////////////////////////////////////////////////
uint16_t bthostlist_getFlagsbyIndex(uint8_t index)
{
    return (btpairingHostInfoList[index].un.flags & ~HOST_LINK_KEY_PRESENT_MASK);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the flags for the nth element. Bounds not checked!
/// \param bdAddr BD_ADDR of the entry to retrieve
/// \return flags or 0 if not found
////////////////////////////////////////////////////////////////////////////////
uint16_t bthostlist_getFlagsbyBdAddr(const BD_ADDR bdAddr)
{
    uint8_t index = wiced_bt_hidd_host_info_find_index(bdAddr);

    return (index != HOST_INFO_NOT_FOUND) ? bthostlist_getFlagsbyIndex(index) : 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the number of HID hosts
///
/// \return number of HID hosts in this list
////////////////////////////////////////////////////////////////////////////////
uint8_t wiced_bt_hidd_host_info_get_number(void)
{
    return btpairingHostInfoListNum;
}


////////////////////////////////////////////////////////////////////////////////
/// Commit the current contents of this host list to the volatile section
/// of non-volatile storage. If the write fails, panic.
////////////////////////////////////////////////////////////////////////////////
void bthostlist_commit(void)
{
    wiced_result_t result;

    // Commit number of devices to NV
    if(btpairingHostInfoListNum)
    {
        // There are devices to commit
        if(wiced_hal_write_nvram( VS_BT_HOST_LIST,
                           btpairingHostInfoListNum * sizeof(tBtHostInfo),
                          (uint8_t *)btpairingHostInfoList,
                          &result) != btpairingHostInfoListNum * sizeof(tBtHostInfo))
        {
            WICED_BT_TRACE("bthostlist_commit failed\n");
        }
    }
    else
    {
        // No devices to commit - so wipe out all.
        wiced_hal_delete_nvram(VS_BT_HOST_LIST, &result);
    }


}

#endif //#ifndef LE_HIDD_ONLY
