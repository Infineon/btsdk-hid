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

/*******************************************************************************
*
* File Name: blehostlist.c
*
* Abstract: This file implements the BLE Host List storing/retrieving to/from NVRAM
*
* Functions:
*
*******************************************************************************/
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_nvram.h"
#include "hidd_lib.h"

#define COMMIT_DELAY 1000     // 1 sec to commit

//Host info VS ID
#define   VS_HIDD_HOST_LIST  (WICED_NVRAM_VSID_START+1)

#define HIDD_HOST_LIST_ELEMENT_SIZE sizeof(tHidd_HostInfo)
#define HIDD_HOST_LIST_SIZE (HIDD_HOST_LIST_MAX * HIDD_HOST_LIST_ELEMENT_SIZE)

#define BLEHOSTLIST_EFFECTIVE_FLAGS_MASK  (0x7FFF)
#define HOST_INFO_NOT_FOUND    0xff
#define HOST_INFO_INDEX_TOP    0

#pragma pack(1)

typedef PACKED union
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
    uint16_t    reserved : 12;
    } fields;

} tBR_EDR_HostInfo;

typedef PACKED struct
{
    uint16_t flags : 15;
    uint16_t is_bonded : 1;

    uint8_t  addrType;
} tBle_HostInfo;

typedef PACKED union
{
    tBR_EDR_HostInfo br_edr;
    tBle_HostInfo        le;
} tBt_HostInfo;

typedef PACKED struct
{
    /// BD address of the bonded host
    BD_ADDR                         bdAddr;

    /// Link key of the bonded host
    wiced_bt_device_link_keys_t     link_keys;

    // transport
    wiced_bt_transport_t            transport;

    tBt_HostInfo                    bt;
} tHidd_HostInfo;

#pragma pack()

typedef struct
{
    // host list NVRAM cache
    tHidd_HostInfo list[HIDD_HOST_LIST_MAX];

    // paired host count
    uint8_t count;

    // timer to commit NVRAM host list
    wiced_timer_t commitTimer;

    // paired host count
    uint8_t reconnectHost;

} tHost;

tHost host={};

//////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// Private functions //////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
/// This function is the timeout handler for commit_timer
////////////////////////////////////////////////////////////////////////////////
static void host_commitTimerCb( uint32_t arg )
{
    wiced_result_t result;

    wiced_hal_write_nvram( VS_HIDD_HOST_LIST, HIDD_HOST_LIST_SIZE, (uint8_t *) host.list, &result);
    // save host info to NVRAM
    if(result)
    {
        WICED_BT_TRACE("\nhost_Update failed to commit to NVRAM");
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// save BleHostList to NVRAM VS section
///
/// \param none
///
/// \return none
//////////////////////////////////////////////////////////////////////////////////////////////////
static void host_Update(uint32_t delay)
{
    if (wiced_is_timer_in_use(&host.commitTimer))
    {
        wiced_stop_timer(&host.commitTimer);
    }

    if (delay)
    {
        wiced_start_timer(&host.commitTimer, delay);
    }
    else
    {
        host_commitTimerCb(0);
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// initialize hidhostlist_List to all 0
///
/// \param none
///
/// \return none
//////////////////////////////////////////////////////////////////////////////////////////////////
static void host_Cear(void)
{
    host.count = 0;

    memset(host.list, 0x00, HIDD_HOST_LIST_SIZE);
}

////////////////////////////////////////////////////////////////////////////////
/// Shifts down the list at index.
/// If the index in invalid, it return false and does nothing.
/// Otherwise, host.count increased by 1 after shifting.
///
/// For example, if the index 1 and the host.count is 4 (we have 4 host elements),
/// element 1-3 are moved to elements 2-4 and element 1 is cleared.
/// Element 0 is untouched.
///
/// \param index of element to freed for new host to be inserted
/// \return TRUE if shifted
////////////////////////////////////////////////////////////////////////////////
static uint8_t host_ShiftDown(uint8_t index)
{
    // make sure we have room to shift and the index is valid
    if (index <= host.count && host.count < HIDD_HOST_LIST_MAX)
    {
#if HIDD_HOST_LIST_MAX > 1
        // check if need to shift
        if (index < host.count)
        {
            // Use memmove to ensure that overalpping areas are moved correctly
            memmove(&host.list[index+1],
                    &host.list[index],
                    HIDD_HOST_LIST_ELEMENT_SIZE*(host.count - index));
        }
#endif
        if (index<HIDD_HOST_LIST_MAX)
        {
            // Clear the new element data at index
            memset((void *) &host.list[(int) index], 0, HIDD_HOST_LIST_ELEMENT_SIZE);
        }

        if (host.count < HIDD_HOST_LIST_MAX)
        {
            // Now we have one more host element
            host.count++;
        }

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
static void host_ShiftUp(uint8_t index)
{
    // make sure index is valid
    if (index < host.count)
    {
        // We are removing one host
        if (host.count)
        {
            host.count--;
        }

#if HIDD_HOST_LIST_MAX > 1
        if ((index+1) < HIDD_HOST_LIST_MAX)
        {
            // Use memmove to ensure that overalpping areas are moved correctly
            memmove(&host.list[index],
                    &host.list[index+1],
                    HIDD_HOST_LIST_ELEMENT_SIZE*(host.count - index));
        }
#endif

        if (host.count<HIDD_HOST_LIST_MAX)
        {
            // Clear the freed element
            memset((void *) &host.list[(int) host.count], 0, HIDD_HOST_LIST_ELEMENT_SIZE);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// Returns the pointer of the host element by giveing address
/// \param bdAddr BD address of the device to find
/// \return index of host if it exists in the list, HOST_INFO_NOT_FOUND if the host is not in the list
////////////////////////////////////////////////////////////////////////////////
static uint8_t host_findAddr(const BD_ADDR bdAddr)
{
    uint8_t index;

    // Go through all the valid entries in the table
    for (index=HOST_INFO_INDEX_TOP; index < host.count; index++)
    {
        if (memcmp(&host.list[index].bdAddr, bdAddr, BD_ADDR_LEN) == 0)
        {
            // Got it! Return the index
            return index;
        }
    }

    // If we get here, the address doesn't exist.
    return HOST_INFO_NOT_FOUND;
}

/////////////////////////////////////////////////////////////////////////////////
// host_del: delete host
//           if host is LE and private address, remove it from resolving list also.
//
// return WICED_TRUE if host already exists
/////////////////////////////////////////////////////////////////////////////////
static void host_del(uint8_t i)
{
    // make sure index is valid
    if (i < host.count)
    {
#ifdef BLE_SUPPORT
        if ((host.list[i].transport == BT_TRANSPORT_LE) &&  // LE transport
             host.list[i].bt.le.addrType)                   // not public address
        {
            wiced_bt_dev_remove_device_from_address_resolution_db(&host.list[i].link_keys);
        }
#endif
        wiced_bt_dev_delete_bonded_device(host.list[i].bdAddr);

        // delete current host element
        host_ShiftUp(i);

#ifdef BR_EDR_SUPPORT
        // if we are removing active host (i=0)
        if (i==HOST_INFO_INDEX_TOP)
        {
            // if the new active host is BRDER, we set to new host
            if (!host.count && host.list[HOST_INFO_INDEX_TOP].transport == BT_TRANSPORT_BR_EDR)
            {
                bthidlink_setHostAddr(host.list[HOST_INFO_INDEX_TOP].bdAddr);
            }
        }
#endif
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
/////////////////// Protected functions (Opened to hidd_lib) /////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
// hidd_host_activate
//    if host is new, add a new host at the top
//    if host already exist and not on the top, move to the top
//    otherwise, it is already on the top, return FALSE for doing nothing.
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_host_activate(const BD_ADDR bdAddr)
{
    uint8_t index = host_findAddr(bdAddr);

    // if host is already on the top, it is the active host already, we simply return false.
    // When it is not on top, we do activate from bottem.
    if (index != HOST_INFO_INDEX_TOP)
    {
        wiced_bool_t found = index != HOST_INFO_NOT_FOUND;

#if HIDD_HOST_LIST_MAX > 1
        tHidd_HostInfo tempHost;

        // if host is already in the list, save it
        if (found && index < HIDD_HOST_LIST_MAX)
        {
            // save current host info
            tempHost = host.list[index];
            host_del(index);
        }
#endif
        // now we make room at the top for the new host
        host_ShiftDown(HOST_INFO_INDEX_TOP);

        WICED_BT_TRACE("\n%s host %B", found ? "Updating" : "Adding", bdAddr);
#if HIDD_HOST_LIST_MAX > 1
        if (found && index < HIDD_HOST_LIST_MAX)
        {
            // restore original host info
            host.list[HOST_INFO_INDEX_TOP] = tempHost;
        }
        else
#endif
        {
            memcpy(host.list[HOST_INFO_INDEX_TOP].bdAddr, bdAddr, BD_ADDR_LEN);
            // default transport to LE
            host.list[HOST_INFO_INDEX_TOP].transport = BT_TRANSPORT_LE;
        }
#ifdef BR_EDR_SUPPORT
        bthidlink_setHostAddr(bdAddr);
#endif
        host_Update(COMMIT_DELAY);
    }
    return index != HOST_INFO_INDEX_TOP;
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void hidd_host_setTransport(const BD_ADDR bdAddr, wiced_bt_transport_t transport)
{
    hidd_host_activate(bdAddr);
    if (host.list[HOST_INFO_INDEX_TOP].transport != transport)
    {
        host.list[HOST_INFO_INDEX_TOP].transport = transport;
        host_Update(COMMIT_DELAY);
    }
}

#ifdef BLE_SUPPORT
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_host_isBonded()
{
    return host.count ? host.list[HOST_INFO_INDEX_TOP].bt.le.is_bonded : FALSE;
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
void hidd_host_setBonded(wiced_bool_t bonded)
{
    WICED_BT_TRACE("\nhidd_host_setBonded %d", bonded);
    if (host.count)
    {
        if (host.list[HOST_INFO_INDEX_TOP].bt.le.is_bonded != bonded)
        {
            host.list[HOST_INFO_INDEX_TOP].bt.le.is_bonded = bonded;
            host_Update(COMMIT_DELAY);
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// hidd_host_setAddrType: activate host and set the address type
///   if host does not exist in database, it creates and add to a new host.
///   if host already exist, replace current host and move to active host.
///
/// \param bdAddr host address to activate
/// \param transport host address type
///
///////////////////////////////////////////////////////////////////////////////////////////////////
void hidd_host_setAddrType(const BD_ADDR bdAddr, uint8_t addrType)
{
    WICED_BT_TRACE("\nhidd_host_setAddrType %d", addrType);
    hidd_host_activate(bdAddr);
    if (host.list[HOST_INFO_INDEX_TOP].bt.le.addrType != addrType)
    {
        host.list[HOST_INFO_INDEX_TOP].bt.le.addrType = addrType;
        host_Update(COMMIT_DELAY);
    }
}

#endif

#if defined(TESTING_USING_HCI)
/////////////////////////////////////////////////////////////////////////////////
/// hidd_host_getInfo
///   get paired host info.
///
/// \return data length
/////////////////////////////////////////////////////////////////////////////////
uint8_t hidd_host_getInfo(uint8_t * buf)
{
    uint8_t ofst = 0, idx;

    buf[ofst++] = host.count | (wiced_hidd_link_is_connected() ? 0x80 : 0);
    for (idx=0; idx<host.count; idx++)
    {
        buf[ofst++] = (host.list[idx].transport==BT_TRANSPORT_BR_EDR) ? 0 : 0x80 | host.list[idx].bt.le.addrType;
#if 0
        memcpy(&buf[ofst], host.list[idx].bdAddr, BD_ADDR_LEN);
        ofst += BD_ADDR_LEN;
#else
        for (int i=BD_ADDR_LEN;i;) buf[ofst++] = host.list[idx].bdAddr[--i];
#endif
    }
    return ofst;
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
/// hidd_host_getLinkKey: get a copy of active link key data
///  return FALSE if bdaddr does not match
///////////////////////////////////////////////////////////////////////////////////////////////////
wiced_bool_t hidd_host_getLinkKey(const BD_ADDR bdAddr, wiced_bt_device_link_keys_t * link_key)
{
    uint8_t index = host_findAddr(bdAddr);
    if (index != HOST_INFO_NOT_FOUND)
    {
        memcpy(link_key, &host.list[index].link_keys, sizeof(wiced_bt_device_link_keys_t));
        return TRUE;
    }
    return FALSE;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// hidd_host_getLinkKeyPtr: returns active host link key pointer
///////////////////////////////////////////////////////////////////////////////////////////////////
const wiced_bt_device_link_keys_t * hidd_host_getLinkKeyPtr()
{
    return &host.list[HOST_INFO_INDEX_TOP].link_keys;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// activate host
///   if host does not exist in database, it creates and add to a new host.
///   if host already exist, replace current host and move to active host.
///
/// \param bdAddr host address to activate
/// \param link_key host link_key
/// \param transport host tranport type
///
///////////////////////////////////////////////////////////////////////////////////////////////////
void hidd_host_setLinkKey(const BD_ADDR bdAddr, wiced_bt_device_link_keys_t * link_keys)
{
    tHidd_HostInfo * ptr = &host.list[HOST_INFO_INDEX_TOP];

    hidd_host_activate(bdAddr);
    WICED_BT_TRACE("\n%s link key", link_keys ? "Update":"Clear");

    if (link_keys)
    {
        // update link key
        memcpy(&ptr->link_keys, link_keys, sizeof(wiced_bt_device_link_keys_t));

#ifdef BLE_SUPPORT
        // add for LE type
        if (ptr->transport == BT_TRANSPORT_LE)
        {
            if (wiced_hidd_host_addr_type())
            {
                wiced_bt_dev_add_device_to_address_resolution_db ( link_keys );
            }
        }
#endif
#ifdef BR_EDR_SUPPORT
        if (ptr->transport == BT_TRANSPORT_BR_EDR)
        {
            ptr->bt.br_edr.fields.linkKeyPresent = TRUE;
        }
#endif
    }
    else
    {
#ifdef BR_EDR_SUPPORT
        if (ptr->transport == BT_TRANSPORT_BR_EDR)
        {
            // clear link key
            memset(&ptr->link_keys, 0, sizeof(wiced_bt_device_link_keys_t));
            ptr->bt.br_edr.fields.linkKeyPresent = FALSE;
        }
#endif
    }
    host_Update(COMMIT_DELAY);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
/// Read HID host information from NVRAM VS section and initialize hidhostlist_List
///
/// \param none
///
/// \return none
//////////////////////////////////////////////////////////////////////////////////////////////////
void hidd_host_init(void)
{
    uint8_t dataSize, index = 0;
    wiced_result_t result;
    BD_ADDR nullAddr = {0};

    host_Cear();

    //timer to allow shut down sleep (SDS)
    wiced_init_timer( &host.commitTimer, host_commitTimerCb, 0, WICED_MILLI_SECONDS_TIMER );

    if (wiced_hal_read_nvram(VS_HIDD_HOST_LIST, HIDD_HOST_LIST_SIZE, (uint8_t *)host.list, &result) == HIDD_HOST_LIST_SIZE)
    {
        while (index < HIDD_HOST_LIST_MAX)
        {
            if (!memcmp(nullAddr, host.list[index].bdAddr, BD_ADDR_LEN))
            {
                break;
            }
            WICED_BT_TRACE("%s\n %d. %B (%s host)",index?"":"\nPaired host list", index, host.list[index].bdAddr, host.list[index].transport == BT_TRANSPORT_LE ? "LE" : "BR/EDR");
#ifdef BLE_SUPPORT
            if (host.list[index].transport == BT_TRANSPORT_LE && host.list[index].bt.le.addrType)
            {
                wiced_bt_dev_add_device_to_address_resolution_db ( &host.list[index].link_keys );
            }
#endif
            index++;
        }
        host.count = index;
    }

    if( !host.count)
    {
        WICED_BT_TRACE("\nHost list empty");
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// Public functions //////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t wiced_hidd_host_set_flags(const BD_ADDR bdAddr, uint16_t enable, uint16_t flags)
{
    hidd_host_activate(bdAddr);

    uint16_t desiredFlags=0;

#ifdef BLE_SUPPORT
    if (host.list[HOST_INFO_INDEX_TOP].transport==BT_TRANSPORT_LE)
    {
        desiredFlags = host.list[HOST_INFO_INDEX_TOP].bt.le.flags;
        host.list[HOST_INFO_INDEX_TOP].bt.le.flags = desiredFlags = (enable ? desiredFlags | flags : desiredFlags & ~(flags));
    }
#endif
#ifdef BR_EDR_SUPPORT
    if (host.list[HOST_INFO_INDEX_TOP].transport==BT_TRANSPORT_BR_EDR)
    {
        desiredFlags = host.list[HOST_INFO_INDEX_TOP].bt.br_edr.flags;
        host.list[HOST_INFO_INDEX_TOP].bt.br_edr.flags = desiredFlags = enable ? desiredFlags | flags : desiredFlags & ~(flags);
    }
#endif
    WICED_BT_TRACE("\n%s %04x, cccd flags:%04x",enable?"Set":"Clear", flags, desiredFlags);
    host_Update(COMMIT_DELAY);
    return desiredFlags;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// Delete active host from NVRAM
///
/// \param none
///
///////////////////////////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_hidd_host_remove()
{
    if (host.count)
    {
        host_del(HOST_INFO_INDEX_TOP);
        host_Update(COMMIT_DELAY);
        return TRUE;
    }
    return FALSE;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// Delete host with given addr from NVRAM
///
/// \param none
///
///////////////////////////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_hidd_host_remove_addr(wiced_bt_device_address_t bdAddr)
{
    if (wiced_hidd_host_exist(bdAddr))
    {
        host_del(HOST_INFO_INDEX_TOP);
        host_Update(COMMIT_DELAY);
        return TRUE;
    }
    return FALSE;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// Delete all HID hosts from NVRAM VS section and reset BleHostList to 0
///
/// \param none
///
/// \return none
//////////////////////////////////////////////////////////////////////////////////////////////////
void wiced_hidd_host_remove_all(void)
{
    while (wiced_hidd_host_remove());
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// Return current host info
///        returns host address, NULL if not paired
/////////////////////////////////////////////////////////////////////////////////////////////
uint8_t * wiced_hidd_host_addr()
{
    return wiced_hidd_is_paired() ? host.list[0].bdAddr : NULL;
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
wiced_bool_t wiced_hidd_host_exist(const BD_ADDR host_bd_addr)
{
    return host_findAddr(host_bd_addr) != HOST_INFO_NOT_FOUND;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// returns bonded host count
///
/// \param none
///
/// \return bonded host count
//////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t wiced_hidd_host_count(void)
{
    return host.count;
}

#ifdef BLE_SUPPORT
// BR_EDR should not need this function
/////////////////////////////////////////////////////////////////////////////////////////////
/// Return current host info
///        returns host address type, 0 if not paired or it is not LE
/////////////////////////////////////////////////////////////////////////////////////////////
uint8_t wiced_hidd_host_addr_type()
{
    return wiced_hidd_host_transport()==BT_TRANSPORT_LE ? host.list[0].bt.le.addrType : 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
uint16_t wiced_hidd_host_get_flags()
{
    if (host.count && wiced_hidd_host_transport()==BT_TRANSPORT_LE)
    {
        return host.list[HOST_INFO_INDEX_TOP].bt.le.flags;
    }
    return 0;
}

#endif

/////////////////////////////////////////////////////////////////////////////////////////////
/// Return current transport type
///
///        returns BT_TRANSPORT_LE
///                BT_TRANSPORT_BR_EDR
///                or 0 if not paired
/////////////////////////////////////////////////////////////////////////////////////////////
wiced_bt_transport_t wiced_hidd_host_transport()
{
    return wiced_hidd_is_paired() ? host.list[0].transport : 0;
}

#if 0
///////////////////////////////////////////////////////////////////////////////////////////////////
/// report current reconnecting host is connected
///
/// \param first: when WICED_TRUE, reset reconnect host first one
///               otherwise advanced to next one
///
/// \return reconnecting host info, NULL if no more host to reconnect
//////////////////////////////////////////////////////////////////////////////////////////////////
tHidd_HostInfo * wiced_hidd_reconnect(wiced_bool_t first)
{
    if (first)
    {
        // reset host index to top
        host.reconnectHost = HOST_INFO_INDEX_TOP;
    }
    else
    {
        // next host
        host.reconnectHost++;
    }

    return wiced_hidd_reconnect_info();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// report current reconnecting host info
///
/// \param none
///
/// \return reconnecting host info, NULL if no more host to reconnect
//////////////////////////////////////////////////////////////////////////////////////////////////
tHidd_HostInfo * wiced_hidd_reconnect_info(void)
{
    // make sure it is a valid host
    if (host.reconnect < count)
    {
        return host.list[reconnect];
    }
    return NULL;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
/// report current reconnecting host is connected
///
/// \param none
///
/// \return none
//////////////////////////////////////////////////////////////////////////////////////////////////
void wiced_ble_hidd_host_reconnected(void)
{
    // make sure it is a valid host
    if (host.reconnect < count)
    {
        // if current connected host is not at the top, move it to top
        if (host.reconnect != HOST_INFO_INDEX_TOP)
        {
            tHidd_HostInfo tempHost;

            // save current host info
            memcpy(tempHost, host.list[host.reconnect], HIDD_HOST_LIST_ELEMENT_SIZE);

            // delete current host element
            host_ShiftUp(host.reconnect);

            // make room at the top
            host_ShiftDown(HOST_INFO_INDEX_TOP);

            // restore host info to the top
            memcpy(host.list[HOST_INFO_INDEX_TOP], tempHost, HIDD_HOST_LIST_ELEMENT_SIZE);

            host.reconnect = HOST_INFO_INDEX_TOP;

            host_Update(0);
        }
    }
}
#endif
