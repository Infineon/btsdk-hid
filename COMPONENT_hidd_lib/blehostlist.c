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
* File Name: blehostlist.c
*
* Abstract: This file implements the BLE Host List storing/retrieving to/from NVRAM
*
* Functions:
*
*******************************************************************************/
#ifndef BT_HIDD_ONLY

#include "blehidlink.h"
#include "blehostlist.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"

BleHostList blehostlist_List[BLEHOSTLIST_MAX];
uint8_t blehostlist_ListNum = 0;
uint8_t blehostlist_ListNumMax = BLEHOSTLIST_MAX;
//uint8_t blehostlist_commit_enabled=0;

static uint8_t hostAddedToRL = 0; //indicate if we've already call wiced_bt_dev_add_device_to_address_resolution_db API.

extern wiced_bt_device_link_keys_t  blehostlist_link_keys;

void blehostlist_clear(void);
void blehostlist_commit(void);
///////////////////////////////////////////////////////////////////////////////////////////////////
/// Read HID host information from NVRAM VS section and initialize blehostlist_List
///
/// \param none
///
/// \return none
//////////////////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_host_info_init(void)
{
    uint8_t tmp;
    wiced_result_t result;

    blehostlist_clear();

    tmp = wiced_hal_read_nvram(VS_BLE_HOST_LIST, sizeof(BleHostList), (uint8_t *)blehostlist_List, &result);

    if( tmp == 0 )
    {
        WICED_BT_TRACE("clientConfigInfoList is empty\n");
    }
    else
    {
        blehostlist_ListNum = tmp / sizeof(BleHostList);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// initialize blehostlist_List to all 0
///
/// \param none
///
/// \return none
//////////////////////////////////////////////////////////////////////////////////////////////////
void blehostlist_clear(void)
{
    blehostlist_ListNum = 0;

    memset(&blehostlist_List[0], 0x00, blehostlist_ListNumMax * sizeof(BleHostList));
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// Add new HID host to the first position of blehostlist_List
///
/// \param bdAddr - BD ADDR of the HID Host
/// \param bdAddrType - Address Type of the HID Host
/// \param link_keys - pointer to the Link keys of the HID Host
/// \param local_keys - pointer to the Local Identity Keys of the HID Host
/// \param flags  - the bitmap corresponding to client characteristic configuration values for notification
///
/// \return none
//////////////////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_host_info_add_first(uint8_t* bdAddr, uint8_t bdAddrType, wiced_bt_device_link_keys_t *link_keys, uint16_t flags)
{
    WICED_BT_TRACE("wiced_ble_hidd_host_info_add_first. is_bonded: %d\n", wiced_blehidd_is_device_bonded());

    memcpy((void*)&blehostlist_List[0].bdAddress, bdAddr, BD_ADDR_LEN);
    memcpy(&blehostlist_List[0].link_keys, link_keys, sizeof(wiced_bt_device_link_keys_t));

    blehostlist_List[0].addrType = bdAddrType;

    blehostlist_List[0].flags = flags & (BLEHOSTLIST_EFFECTIVE_FLAGS_MASK);

    blehostlist_List[0].entryValid = 1;

    blehostlist_List[0].is_bonded = wiced_blehidd_is_device_bonded();

    if (!blehostlist_ListNum)
        blehostlist_ListNum++;

    //if (blehostlist_commit_enabled)
    {
        blehostlist_commit();
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// get client configuration values for the HID host
///
/// \param bdAddr - BD ADDR of the HID Host
/// \param bdAddrType - Address Type of the HID Host
///
/// \return the bitmap corresponding to client characteristic configuration values for notification
//////////////////////////////////////////////////////////////////////////////////////////////////
int32_t wiced_ble_hidd_host_info_get_flags(uint8_t* bdAddr, uint8_t bdAddrType)
{
    uint8_t* host_bdAddr;
    uint8_t host_addrType;
    uint8_t i;

    if (wiced_blehidd_is_device_bonded())
    {
        if (wiced_ble_hidd_host_info_get_first_host(&host_bdAddr,&host_addrType))
        {
            if (!((bdAddrType == host_addrType) &&
                (memcmp(bdAddr, host_bdAddr, BD_ADDR_LEN) == 0)))
            {
                wiced_ble_hidd_host_info_update_first_host(bdAddr,bdAddrType);
            }
        }
        else
        {
            WICED_BT_TRACE("BUG..You should not see this !!!\n");
        }
    }

    for(i = 0; i < blehostlist_ListNum; i++)
    {
        if((blehostlist_List[i].addrType == bdAddrType) &&
           (blehostlist_List[i].entryValid) &&
           (memcmp(blehostlist_List[i].bdAddress, bdAddr, BD_ADDR_LEN) == 0))
        {
            return blehostlist_List[i].flags & BLEHOSTLIST_EFFECTIVE_FLAGS_MASK;
        }
    }

    return -1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// set client configuration values for specified HID host
///
/// \param bdAddr - BD ADDR of the HID Host
/// \param bdAddrType - Address Type of the HID Host
/// \param flags  - the bitmap corresponding to client characteristic configuration values for notification
///
/// \return none
//////////////////////////////////////////////////////////////////////////////////////////////////
void blehostlist_setFlags (uint8_t* bdAddr, uint8_t addrType, uint16_t flags)
{
    uint8_t i;

    for(i = 0; i < blehostlist_ListNum; i++)
    {
        if((blehostlist_List[i].addrType == addrType) &&
           (blehostlist_List[i].entryValid) &&
           (memcmp(blehostlist_List[i].bdAddress, bdAddr, BD_ADDR_LEN) == 0))
        {
            blehostlist_List[i].flags = flags & BLEHOSTLIST_EFFECTIVE_FLAGS_MASK;

            //if (blehostlist_commit_enabled)
            {
                blehostlist_commit();
            }
            return;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// save BleHostList to NVRAM VS section
///
/// \param none
///
/// \return none
//////////////////////////////////////////////////////////////////////////////////////////////////
void blehostlist_commit(void)
{
    wiced_result_t result;

    if(wiced_hal_write_nvram( VS_BLE_HOST_LIST,
                           sizeof(blehostlist_List[0]) * blehostlist_ListNum,
                          (uint8_t *)&blehostlist_List[0],
                          &result) != sizeof(BleHostList) * blehostlist_ListNum)
    {
        WICED_BT_TRACE("blehostlist_commit failed\n");
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// get the number of HID host
///
/// \param none
///
/// \return the number of HID host
//////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t wiced_ble_hidd_host_info_get_number(void)
{
    return blehostlist_ListNum;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// Check if the first HID host is bonded
///
/// \param none
///
/// \return WICED_TRUE/WICED_FALSE
//////////////////////////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_ble_hidd_host_info_is_bonded(void)
{
    if (blehostlist_ListNum)
        return (blehostlist_List[0].is_bonded);

    return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// get first HID host BD ADDR
///
/// \param none
///
/// \return BD ADDR of the HID host
//////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t *wiced_ble_hidd_host_info_get_bdaddr (void)
{
    if (blehostlist_ListNum)
        return (blehostlist_List[0].bdAddress);

    return NULL;

}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// get first HID host Link Keys
///
/// \param none
///
/// \return Link Keys of the HID host
//////////////////////////////////////////////////////////////////////////////////////////////////
wiced_bt_device_link_keys_t *wiced_ble_hidd_host_info_get_link_keys(void)
{
    if (blehostlist_ListNum)
        return (&blehostlist_List[0].link_keys);

    return NULL;

}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// set the HID host to invalid and update NVRAM VS section
///
/// \param bdAddr - BD ADDR of the HID Host
/// \param bdAddrType - Address Type of the HID Host
///
/// \return none
//////////////////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_host_info_delete(uint8_t* bdAddr, uint8_t addrType)
{
    uint8_t i;

    for(i = 0; i < blehostlist_ListNum; i++)
    {
        if((blehostlist_List[i].addrType == addrType) &&
           (blehostlist_List[i].entryValid) &&
           (memcmp(blehostlist_List[i].bdAddress, bdAddr, BD_ADDR_LEN) == 0))
        {
            blehostlist_List[i].entryValid = 0;
            blehostlist_commit();
            return;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// Delete all HID hosts from NVRAM VS section and reset BleHostList to 0
///
/// \param none
///
/// \return none
//////////////////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_host_info_delete_all(void)
{
    wiced_result_t result;

    WICED_BT_TRACE("Deleting all CCIL\n");
    wiced_hal_delete_nvram(VS_BLE_HOST_LIST, &result);
    blehostlist_clear();
    hostAddedToRL = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// update client configuration value for the HID host
///
/// \param bdAddr - BD ADDR of the HID Host
/// \param bdAddrType - Address Type of the HID Host
/// \param enable - allow/disabllow notification
/// \param featureBit - bit/bits in the bitmap of client configuration characteristics
///
/// \return the final flags
//////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t wiced_ble_hidd_host_info_update_flags(uint8_t* bdAddr, uint8_t bdAddrType, uint16_t enable, uint16_t featureBit)
{
    int32_t currentFlags = -1;
    uint16_t desiredFlags = 0;

    if((currentFlags = wiced_ble_hidd_host_info_get_flags(bdAddr, bdAddrType)) != -1)
    {
        desiredFlags = (uint16_t)currentFlags;

        desiredFlags = enable ? desiredFlags | featureBit : desiredFlags & ~(featureBit);

        if(desiredFlags != (uint16_t)currentFlags)
        {
            blehostlist_setFlags(bdAddr,bdAddrType, desiredFlags);
        }
    }
    else
    {
        desiredFlags = enable ? featureBit : 0;
        wiced_ble_hidd_host_info_add_first(bdAddr, bdAddrType, &blehostlist_link_keys, desiredFlags);
    }

    WICED_BT_TRACE("Committing notificaion Flag:0x%04x\n", desiredFlags);
    return desiredFlags;
}

#if 0
void blehostlist_enableCommitCCIL(uint8_t enable)
{
    blehostlist_commit_enabled = enable;
}

uint8_t blehostlist_IsEnableCommitCCIL(void)
{
    return blehostlist_commit_enabled;
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
/// retrieve the first HID host (BD ADDR and address type) in the list of blehostlist_List
///
/// \param bdAddr - pointer point to the BD ADDR
/// \param bdAddrType - pointer point to the address type
///
/// \return WICED_TRUE/WICED_FALSE
//////////////////////////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_ble_hidd_host_info_get_first_host(uint8_t** bdAddr, uint8_t *bdAddrType)
{
    if (blehostlist_List[0].entryValid)
    {
        *bdAddr = (uint8_t *)&blehostlist_List[0].bdAddress;
        *bdAddrType = blehostlist_List[0].addrType;
        return WICED_TRUE;
    }
    else
    {
        return WICED_FALSE;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// update the first HID host (BD ADDR and address type) in the list of blehostlist_List
///
/// \param bdAddr - BD ADDR
/// \param bdAddrType - address type
///
/// \return WICED_TRUE/WICED_FALSE
//////////////////////////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_ble_hidd_host_info_update_first_host(uint8_t *bdAddr, uint8_t bdAddrType)
{
    if (blehostlist_List[0].entryValid)
    {
        memcpy((void*)&blehostlist_List[0].bdAddress, bdAddr, BD_ADDR_LEN);
        blehostlist_List[0].addrType = bdAddrType;
        return 1;
    }
    else
    {
        return 0;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// Add host to the Resolving List (RL) if its address type is not public address
///
/// \param none
///
/// \return none
//////////////////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_host_info_add_to_resolving_list(void)
{
    if (hostAddedToRL)
        return;

    hostAddedToRL = 1;

    //if host's addr type is not BLE_ADDR_PUBLIC, add host to resolution list
    if( blehostlist_ListNum != 0 && blehostlist_List[0].addrType != 0)
    {
        wiced_bt_dev_add_device_to_address_resolution_db ( &blehostlist_List[0].link_keys );
        WICED_BT_TRACE("load_keys_for_address_resolution\n");
    }
}

#endif //#ifndef BT_HIDD_ONLY
