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
 * @file hidh_lib.h
 *
 * @brief Bluetooth Low Energy (LE) HID Host (HIDH) Library Functions
 *
 */

#ifndef __HIDH_LIB_H_
#define __HIDH_LIB_H_

#include "wiced_bt_dev.h"
#include "wiced_bt_gatt.h"
#include "cyhal_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup  hidh_le_api_functions   HID Host Role (HIDH) over LE
 * @ingroup     wiced_bt_hid
 *
 * The Human Interface Device Host Role (HIDH) LE library of the SDK provide a simple method
 * for an application to integrate HIDH functionality.
 * This library is typically used to connect to LE HID Devices such as Bluetooth (classic) Mice,
 * Keyboards or Remote Control.
 *
 * @{
*/

/**
 * @brief LE HIDH Maximum HID Devices.
 */
#ifndef HIDH_DEV_MAX
#define HIDH_DEV_MAX                   3
#endif
/**
 * @brief LE HIDH Maximum Characteristics for an HID Devices.
 */
#define HIDH_DEV_CHAR_MAX              14

/**
 * @brief LE HIDH Maximum Reports of an HID Devices.
 */
#define HIDH_REPORT_DESC_MAX           10

/**
 * @brief LE HIDH Handle Offset.
 *  We use an offset to convert ConnectionId to LE-HIDH_Handle (to do distinguish BR-HID handles)
 */
#define HIDH_HANDLE_OFFSET             20

/**
 * @brief LE HIDH WakeUp Pattern Maximum Length.
 */
#define HIDH_WAKEUP_PATTERN_LEN_MAX    10

/**
 * @brief LE HIDH WakeUp Pattern Maximum Number.
 */
#define HIDH_WAKEUP_PATTERN_NB_MAX     1

/**
 * @brief LE HIDH Event Filter Maximum Number.
 */
#define HIDH_EVENT_FILTER_NB_MAX       1

/**
 * @brief HIDH Events.
 *
 * LE HID Host event received by the LE HID Host callback (see hidh_le_cback_t)
 */
typedef enum
{
    HIDH_OPEN_EVT,             /**< Connection Open. */
    HIDH_CLOSE_EVT,            /**< Connection Closed. */
    HIDH_DESCRIPTOR_EVT,       /**< HID Descriptor received. */
    HIDH_GATT_CACHE_EVT,       /**< LE HID GATT Cache. */
    HIDH_REPORT_EVT,           /**< HID Report received from peer HID Device. */
    HIDH_SET_REPORT_EVT,       /**< Set HID Report confirmation. */
    HIDH_GET_REPORT_EVT,       /**< Get HID Report confirmation. */
    HIDH_VIRTUAL_UNPLUG_EVT,   /**< Virtual Unplug. */
    HIDH_SET_PROTOCOL_EVT,     /**< HID Set Protocol confirmation. */
} hidh_le_event_t;

/**
 * @brief LE HID Error codes.
 *
 * This enumeration contains the list of HID Error codes.
 */

typedef enum
{
    HIDH_STATUS_SUCCESS = 0,        /**< Operation success */
    HIDH_STATUS_ERROR,              /**< Generic internal error */
    HIDH_STATUS_GATT_ERROR,         /**< Generic GATT error */
    HIDH_STATUS_INVALID_PARAM,      /**< Invalid Parameter */
    HIDH_STATUS_INVALID_CONN_ID,    /**< Invalid Parameter - conn_id */
    HIDH_STATUS_INVALID_DEV,        /**< Invalid Parameter - Dev */
    HIDH_STATUS_INVALID_BDADDR,     /**< Invalid Parameter - Addr */
    HIDH_STATUS_INVALID_LENGTH,     /**< Invalid Parameter - len */
    HIDH_STATUS_MEM_FULL,           /**< Memory full */
    HIDH_STATUS_CONNECTION_FAILED,  /**< Not able to establish Baseband connection */
    HIDH_STATUS_UNSUPPORTED,        /**< Feature unsupported by peer device */
    HIDH_STATUS_NOT_YET_IMPLEMENTED,/**< Not Yet Implemented */
    HIDH_STATUS_NOT_INITIALIZED,    /**< Not Initialized */
    HIDH_STATUS_NOT_CONNECTED,      /**< Not connected */
    HIDH_STATUS_INVALID_HANDLE,     /**< Invalid Handle */
} hidh_le_status_t;

/**
 * @brief LE HID Report Type.
 *
 * This enumeration contains the list of HID Report Types.
 */
typedef enum
{
    HIDH_REPORT_TYPE_RESERVED = 0,   /**< reserved         */
    HIDH_REPORT_TYPE_INPUT,          /**< input report     */
    HIDH_REPORT_TYPE_OUTPUT,         /**< output report    */
    HIDH_REPORT_TYPE_FEATURE         /**< feature report   */
} hidh_le_report_type_t;

/**
 * @brief HID Protocol definition (Regular Report or Boot Report mode).
 *
 */
typedef enum
{
    HIDH_PROTOCOL_REPORT = 0,       /**< Protocol Mode Report. */
    HIDH_PROTOCOL_BOOT,             /**< Protocol Mode Boot. */
} hidh_le_protocol_t;

/**
 * @brief HID Device WakeUp Commands
 */
typedef enum
{
    HIDH_WAKEUP_PATTERN_CMD_ADD = 1,/**< Command to add */
    HIDH_WAKEUP_PATTERN_CMD_DEL,    /**< Command to delete */
    HIDH_WAKEUP_PATTERN_CMD_LIST,   /**< Command to list added pattern */
} hidh_le_wakeup_pattern_cmd_t;

/**
 * @brief Data associated with HIDH_OPEN_EVT.
 *
 * This event is received:
 *  - After the hidh_le_open function is called or
 *  - When a peer device reconnects (reconnection allowed with hidh_le_add)
 *
 */
typedef struct
{
    wiced_bt_device_address_t bdaddr;/**< Device Address */
    uint16_t handle;                 /**< Connect handle */
    hidh_le_status_t status;         /**< Connect status */
} hidh_le_connected_t;

/**
 * @brief Data associated with HIDH_CLOSE_EVT.
 *
 * This event is received:
 *  - After the hidh_le_disconnect function is called or
 *  - When a peer device disconnects
 *
 */
typedef struct
{
    uint16_t handle;                        /**< Disconnected connect handle */
    wiced_bt_gatt_disconn_reason_t reason;  /**< Disconnect reason */
} hidh_le_disconnected_t;

/**
 * @brief Data associated with HIDH_DESCRIPTOR_EVT.
 *
 * This event is received after call to the hidh_le_get_descriptor function.
 */
typedef struct
{
    uint16_t                    handle;         /**< HIDH Connection Handle. */
    hidh_le_status_t            status;         /**< HIDH Operation Status. */
    uint8_t                     *p_descriptor;  /**< HID Descriptor of the peer HID device. */
    uint16_t                    length;         /**< Length of the HID Descriptor */
} hidh_le_descriptor_t;

/**
 * @brief Data associated with HIDH_REPORT_EVT.
 *
 * This event is received when the peer HID Device sends a report (e.g. Button pressed/released).
 */
typedef struct
{
    uint16_t                    handle;     /**< HIDH Connection Handle. */
    uint8_t                     report_id;  /**< HID Report Id. */
    uint8_t                     *p_data;    /**< HID Report data. */
    uint16_t                    length;     /**< HID Report length. */
} hidh_le_report_t;

/**
 * @brief LE HID GATT Characteristic.
 *
 * This structure contains the informations of a LE HID GATT Characteristic.
 */
typedef struct
{
    uint16_t uuid16;                            /**< 16-bit UUID */
    uint16_t handle;                            /**< Attribute Handle. */
    uint16_t val_handle;                        /**< The Value Handle. */
    wiced_bt_gatt_char_properties_t properties; /**< GATT characteristic properties mask (see #wiced_bt_gatt_char_properties_e) */
} hidh_le_gatt_char_t;

/**
 * @brief LE HID GATT Report.
 *
 * This structure contains the informations of a LE HID Report.
 */
typedef struct
{
    uint16_t handle;                            /**< Attribute Handle.  */
    uint16_t val_handle;                        /**< The Value Handle.  */
    uint8_t rpt_id;                             /**< Report Id          */
    hidh_le_report_type_t rpt_type;             /**< Report Type        */
} hidh_le_gatt_report_t;

/**
 * @brief Data associated with HIDH_GATT_CACHE_EVT.
 *
 * This event is received to save the LE HID GATT Cache information in NVRAM.
 */
typedef struct
{
    wiced_bt_device_address_t bdaddr;                           /**< LE HID Device Address. */

    uint8_t characteristics_nb;                                 /**< Number of Characteristics in table. */
    hidh_le_gatt_char_t characteristics[HIDH_DEV_CHAR_MAX];     /**< Characteristics table. */

    uint8_t report_descs_nb;                                    /**< Number of Report Descriptors in table. */
    hidh_le_gatt_report_t report_descs[HIDH_REPORT_DESC_MAX];   /**< Report Descriptors in table. */
} hidh_le_gatt_cache_t;

/**
 * @brief Data associated with HIDH_SET_REPORT_EVT.
 *
 * This event is received after call to the hidh_le_set_report function.
 */
typedef struct
{
    uint16_t          handle;     /**< HIDH Connection Handle. */
    hidh_le_status_t  status;     /**< HIDH Operation Status. */
} hidh_le_set_report_t;

/**
 * @brief Data associated with HIDH_GET_REPORT_EVT.
 *
 * This event is received after call to the hidh_le_get_report function.
 */
typedef struct
{
    uint16_t                    handle;     /**< HIDH Connection Handle. */
    hidh_le_status_t            status;     /**< HIDH Operation Status. */
    uint8_t                     *p_data;    /**< HID Report data. */
    uint16_t                    length;     /**< HID Report length. */
} hidh_le_get_report_t;

/**
 * @brief Data associated with HIDH_SET_PROTOCOL_EVT.
 *
 * This event is received after call to the hidh_le_set_protocol function.
 */
typedef struct
{
    uint16_t                    handle;     /**< HIDH Connection Handle. */
    hidh_le_status_t            status;     /**< HIDH Operation Status. */
} hidh_le_set_protocol_t;

/**
 * @brief Data associated with HIDH_VIRTUAL_UNPLUG_EVT.
 *
 * This event is received when the peer HID Device sends a Virtual UnPlug event.
 * Upon reception of this event, the device will be disconnected and removed from the HID host
 * Database, but the application (embedded and/or MCU) must erase all Pairing information about
 * this device.
 */
typedef struct
{
    uint16_t                    handle;     /**< HIDH Connection Handle. */
} hidh_le_virtual_unplug_t;

typedef union
{
    hidh_le_connected_t    connected;       /**< HIDH_OPEN_EVT */
    hidh_le_disconnected_t disconnected;    /**< HIDH_CLOSE_EVT */
    hidh_le_descriptor_t   descriptor;      /**< HIDH_DESCRIPTOR_EVT */
    hidh_le_set_report_t   set_report;      /**< HIDH_SET_REPORT_EVT */
    hidh_le_get_report_t   get_report;      /**< HIDH_GET_REPORT_EVT */
    hidh_le_report_t       report;          /**< HIDH_REPORT_EVT */
    hidh_le_gatt_cache_t   gatt_cache;      /**< HIDH_GATT_CACHE_EVT */
    hidh_le_set_protocol_t set_protocol;    /**< HIDH_SET_PROTOCOL_EVT */
    hidh_le_virtual_unplug_t virtual_unplug;/**< HIDH_VIRTUAL_UNPLUG_EVT */
} hidh_le_event_data_t;

/**
 * @brief       HIDH Callback function type hidh_le_cback_t
 *              LE HID Host Event callback (registered with hidh_le_init)
 *
 * @param[in]   event: LE HIDH event received
 * @param[in]   p_data : Data (pointer on union of structure) associated with the event
 *
 * @return      None
 */
typedef void hidh_le_cback_t(hidh_le_event_t event,
        hidh_le_event_data_t *p_event_data);

/**
 * @brief           HIDH Callback function type hidh_le_filter_cback_t
 *                  LE HID Host Event Filter callback (registered with hidh_le_filter_register)
 *
 * @param[in]       event: LE HIDH event received
 * @param[in]       p_data : Data (pointer on union of structure) associated with the event
 *
 * @return          WICED_TRUE if event filtered (will not be sent to Application).
 *                  WICED_FALSE if event not filtered (will be sent to Application).
 */
typedef wiced_bool_t hidh_le_filter_cback_t(hidh_le_event_t event,
        hidh_le_event_data_t *p_event_data);

/**
 * @brief     Before any other HIDH library functions can be called, application must call
 *            this function first to initialize/start LE HID library
 *
 * @param[in] p_cback : Callback for LE HIDH event notification
 *
 *            This function is called for LE HID Host Initialization.
 *            This function must be called, once, before any other LE HIDH functions.
 *
 * @return    None
 */
hidh_le_status_t hidh_le_init(hidh_le_cback_t *p_callback);

/**
 * @brief     Application calls this function to initiate connection to a LE HID Device
 *
 * @detail    Open HID Host connection to an HID Device
 *            The first HID connection to an HID Device must always be initiated by the LE
 *            HID Host device.
 *            If this function returns a successfull status, the HIDH_OPEN_EVT
 *            event will be sent once the connection will be established.
 *
 * @param[in] bdaddr     : BD ADDR
 * @param[in] addr_type  : Address type
 *
 * @return    Result code (see hidh_le_status_t)
 *            HIDH_STATUS_OK if opening in progress, otherwise error.
 */
hidh_le_status_t hidh_le_connect(wiced_bt_device_address_t bdaddr,
        wiced_bt_ble_address_type_t addr_type);

/**
 * @brief     Application calls this function to disconnect a LE HID Device
 *
 * @detail    Disconnect LE HID Host connection to a LE HID Device
 *            If this function returns a successfull status, the HIDH_CLOSE_EVT
 *            event will be sent once the connection will be established.
 *
 * @param[in] handle : device handle
 *
 * @return    Result code (see hidh_le_status_t)
 *            HIDH_STATUS_OK if opening in progress, otherwise error.
 */
hidh_le_status_t hidh_le_disconnect(uint16_t bhidh_le_conn_handle);

/**
 * @brief     When the device is bonded, application calls this function
 *            to add a LE HID Device (to allow it to reconnect)
 *
 * @detail    Add an LE HID Device (to the known HID Device list).
 *            This function is, typically, called during application startup to allow
 *            a peer LE HID Device to reconnect.
 *            When a peer LE HID Device will reconnect, the HIDH_OPEN_EVT
 *            event will be sent to the application.
 *            Note, that a peer device is not automatically added during the initial HID
 *            connection. The application must explicitly Add it with this API.
 *            During the initial connection, the library will retrieve the GATT database
 *            containing the Attributes and Report descriptions. The Application must
 *            save it (in NVRAM) and pass it in this API.
 *
 * @param[in] bdaddr          : BD ADDR
 * @param[in] addr_type       : Address type
 * @param[in] p_gatt_cache    : Discovered device gatt info to be added
 *
 * @return    Result code (see hidh_le_status_t)
 *            HIDH_STATUS_OK if opening in progress, otherwise error.
 */
hidh_le_status_t hidh_le_add(wiced_bt_device_address_t bdaddr,
        wiced_bt_ble_address_type_t addr_type, hidh_le_gatt_cache_t *p_gatt_cache);

/**
 * @brief     To remove a LE HID Device (to do not allow it to reconnect).
 *
 * @param[in] bdaddr : BD ADDR
 *
 * @return    Result code (see hidh_le_status_t)
 *            HIDH_STATUS_OK if opening in progress, otherwise error.
 */
hidh_le_status_t hidh_le_remove(wiced_bt_device_address_t bdaddr);

/**
 * @brief     Set (send) a Report to a peer LE HID Device
 *
 * @detail    This function is called to set (send) a LE HID Report to an HID Device.
 *            This function can be used, for example, to control the 'Caps Lock" led of
 *            a Bluetooth Keyboard.
 *            Upon completion, a HIDH_SET_REPORT_EVT event will be sent to the
 *            application.
 *
 * @param[in] handle       : device handle
 * @param[in] report_type  : report type
 * @param[in] report_id    : report id
 * @param[in] p_data       : report data pointer
 * @param[in] length       : report length
 *
 * @return    Result code (see hidh_le_status_t)
 *            HIDH_STATUS_OK if operation is progress, otherwise error.
 */
hidh_le_status_t hidh_le_set_report(uint16_t handle,
        hidh_le_report_type_t report_type, uint8_t report_id, uint8_t *p_data,
        uint16_t length);

/**
 * @brief     This function is called to Get (receive) an LE HID Report from a connected
 *            LE HID Device.
 *            This function can be used, for example, to read the last HID Report received.
 *
 * @param[in] uint16_t handle                    : device handle
 * @param[in] hidh_le_report_type_t report_type  : report type
 * @param[in] uint8_t report_id                  : report id
 * @param[in] uint16_t length                    : length to get
 *
 * @return    Result code (see hidh_le_status_t)
 *            HIDH_STATUS_OK if operation is progress, otherwise error.
 */
hidh_le_status_t hidh_le_get_report(uint16_t handle,
        hidh_le_report_type_t type, uint8_t report_id, uint16_t length);

/**
 * @brief     Sends Set HID Protocol to a peer HID Device.
 *            This function is called to change the HID Protocol of a connected HID Device.
 *
 * @param[in] handle    : device handle
 * @param[in] protocol  : new protocol to set
 *
 * @return    Result code (see hidh_le_status_t)
 *            HIDH_STATUS_OK if successful, otherwise error.
 */
hidh_le_status_t hidh_le_set_protocol(uint16_t handle,
        hidh_le_protocol_t protocol);

/**
 * @brief     When a LE connection is established, application must call this function to bring up
 *            a hid device.
 *
 * @param[in] p_conn_status : GATT connection status
 *
 * @return    None.
 */
void hidh_le_up(wiced_bt_gatt_connection_status_t *p_conn_status);

/**
 * @brief     When a LE connection is lost, application must call this function to disconnect the device.
 *
 * @param[in] wiced_bt_gatt_connection_status_t *p_conn_status : GATT connection status
 *
 * @return    none.
 */
void hidh_le_down(wiced_bt_gatt_connection_status_t *p_conn_status);

/**
 * @brief     Application must call this function when encryption status changes
 *
 * @param[in] p_encryption_changed : encryption change event data
 *
 * @return    None.
 */
void hidh_le_encryption_changed(wiced_bt_dev_encryption_status_t *p_encryption_changed);

/**
 * @brief     Application must call this function when GATT_DISCOVERY_RESULT_EVT event is received
 *
 * @param[in] p_discovery_result : discovery result data
 *
 * @return    Result code (see wiced_bt_gatt_status_t)
 */
wiced_bt_gatt_status_t hidh_le_gatt_discovery_result(
        wiced_bt_gatt_discovery_result_t *p_discovery_result);

/**
 * @brief     Application must call this function when GATT_DISCOVERY_CPLT_EVT event is received
 *
 * @param[in] p_discovery_complete : discovery complete data
 *
 * @return    Result code (see wiced_bt_gatt_status_t)
 */
wiced_bt_gatt_status_t hidh_le_gatt_discovery_complete(
        wiced_bt_gatt_discovery_complete_t *p_discovery_complete);

/**
 * @brief     Application must call this function when GATT_OPERATION_CPLT_EVT event is received
 *
 * @param[in] p_operation_complete : operation complete data
 *
 * @return    Result code (see wiced_bt_gatt_status_t)
 */
wiced_bt_gatt_status_t hidh_le_gatt_operation_complete(
        wiced_bt_gatt_operation_complete_t *p_operation_complete);

/**
 * @brief     LE HIDH libraries (e.g. LE HIDH Audio) can use this function to register a 'Filter Callback'
 *
 * @param[in] p_callback      : LE HIDH Filter callback
 *
 * @return    @hidh_le_status_t HIDH_STATUS_SUCCESS or error code @endlink
 */
hidh_le_status_t hidh_le_filter_register(
        hidh_le_filter_cback_t *p_callback);

/**
 * @brief     Application call this function to set the WakeUp pattern
 *            This function must be called after the hidh_le_add function is called
 *
 * @param[in] bdaddr      : BD Addr
 * @param[in] command     : wakeup pattern command
 * @param[in] report_id   : report id
 * @param[in] p_pattern   : Wakup pattern data
 * @param[in] pattern_len : Wakup pattern data length
 *
 * @return    @hidh_le_status_t HIDH_STATUS_SUCCESS or error code @endlink
 */
hidh_le_status_t hidh_le_wakeup_pattern_set(wiced_bt_device_address_t bdaddr,
        hidh_le_wakeup_pattern_cmd_t command, uint16_t report_id,
        uint8_t *p_pattern, uint16_t pattern_len);

/**
 * @brief     Application call this function to enable/disable the WakeUp
 *
 * @detail    This function is typically called after hidh_le_wakeup_pattern_set
 *            The p_data and data_len, could be used (later) to add additional GPIO Control (duration,
 *            pattern, etc.).
 *
 * @param[in] enable    : to enable or disable
 * @param[in] gpio_num  : GPIO pin
 * @param[in] polarity  : Wake up (interrupt) priority
 * @param[in] p_data    : WakeUp ReportId's pattern data to filter
 * @param[in] data_len  : WakeUp ReportId's pattern data length to filter
 *
 * @return    @hidh_le_status_t HIDH_STATUS_SUCCESS or error code @endlink
 */
hidh_le_status_t hidh_le_wakeup_pattern_control(wiced_bool_t enable,
        cyhal_gpio_t gpio_num, uint8_t polarity, uint8_t *p_data, uint8_t data_len);

/**
 * @brief     Application must call this function to retrieve the HID descriptor of a connected LE HID Device
 *
 * @param[in] handle  : device handle
 *
 * @return    Result code (see wiced_bt_gatt_status_t)
 */
wiced_bt_gatt_status_t hidh_le_get_descriptor(uint16_t handle);


/** @} hidh_le_api_functions */

#ifdef __cplusplus
}
#endif

#endif // __HIDH_LIB_H_

/* end of file */
