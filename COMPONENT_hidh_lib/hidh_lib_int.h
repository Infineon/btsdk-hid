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
 * @file   hidh_lib_int.h
 */
#ifndef __HIDH_LIB_INT_H_
#define __HIDH_LIB_INT_H_

#include "wiced_bt_dev.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#include "hidh_lib.h"
#include "cyhal_gpio.h"

/******************************************************
 *                     Constants
 ******************************************************/

/* LE HID Trace macro(s) */
#if HIDH_TRACE
#define LE_HIDH_TRACE(format, ...)        WICED_BT_TRACE("%s " format, __FUNCTION__, ##__VA_ARGS__)
#else
#define LE_HIDH_TRACE(...)
#endif

/* LE_HIDH_TRACE_ERR is always enabled */
#define LE_HIDH_TRACE_ERR(format, ...)    WICED_BT_TRACE("Err: %s " format, __FUNCTION__, ##__VA_ARGS__)

/*
 * @brief   enums for gatt related state code
 */
typedef enum
{
    HIDH_GATTC_STATE_EMPTY = 0,
    HIDH_GATTC_STATE_SEARCHING_SRV,
    HIDH_GATTC_STATE_SEARCHING_CHAR,
    HIDH_GATTC_STATE_SEARCHING_CHAR_DESC,
    HIDH_GATTC_STATE_SEARCHING_REPORT_REF,
    HIDH_GATTC_STATE_CONFIGURE_NOTIFICATION,
    HIDH_GATTC_STATE_DONE
} hidh_le_gattc_state_t;

/******************************************************
 *                     Structures
 ******************************************************/

/*
 * @brief   Structure containing LE HID GATT Client information
 */
typedef struct
{
    hidh_le_gattc_state_t db_state;                     /**< GATT State machine */
    uint16_t start_handle;                              /**< First HID GATT Handle */
    uint16_t end_handle;                                /**< Last HID GATT Handle */

    uint16_t search_char_desc_index;                    /**< Current Searched Descriptor Index */

    uint8_t characteristics_nb;                         /**< Number of Characteristics in table */
    hidh_le_gatt_char_t characteristics[HIDH_DEV_CHAR_MAX];

    uint8_t report_descs_nb;                            /**< Number of Report Descriptors in table */
    hidh_le_gatt_report_t report_descs[HIDH_REPORT_DESC_MAX];

    uint8_t report_val_hdl_nb;                          /**< Number of Report Report Val Handle in table */
    uint16_t report_val_hdl[HIDH_REPORT_DESC_MAX];

    uint8_t client_char_configs_nb;                     /**< Number of Report Client Char Config in table */
    uint16_t client_char_configs[HIDH_REPORT_DESC_MAX];
} hidh_le_gattc_dev_t;

/* Device state (bitfield) */
#define HIDH_DEV_STATE_ALLOCATED        0x01
#define HIDH_DEV_STATE_CONNECTED        0x02
#define HIDH_DEV_STATE_ADDED            0x04
#define HIDH_DEV_STATE_ENCRYPTED        0x08
#define HIDH_DEV_STATE_OP_READ_DESC     0x10
#define HIDH_DEV_STATE_OP_SET_REPORT    0x20
#define HIDH_DEV_STATE_OP_GET_REPORT    0x40
#define HIDH_DEV_STATE_OP_SET_PROTOCOL  0x80
#define HIDH_DEV_STATE_OP_PENDING_MSK   (HIDH_DEV_STATE_OP_READ_DESC |  \
                                         HIDH_DEV_STATE_OP_SET_REPORT | \
                                         HIDH_DEV_STATE_OP_GET_REPORT | \
                                         HIDH_DEV_STATE_OP_SET_PROTOCOL)
typedef uint8_t hidh_le_dev_state_t;

/*
 * @brief   WakeUp pattern data structure define
 */
typedef struct
{
    uint8_t report_id;                              /**< Report ID. ReportId 0 is invalid. Use 0 to indicate this element is not in use */
    uint16_t length;                                /**< Data Pattern length */
    uint8_t pattern[HIDH_WAKEUP_PATTERN_LEN_MAX];   /**< Data Pattern */
} hidh_le_wakeup_pattern_t;

/*
 * @brief   Structure containing the in information of a peer HID LE Device
 */
typedef struct
{
    wiced_bt_device_address_t bdaddr;                                       /**< BD Addr */
    wiced_bt_ble_address_type_t addr_type;                                  /**< Addr type */
    uint16_t conn_id;                                                       /**< Conn Id */
    hidh_le_dev_state_t dev_state_msk;                                      /**< State mask */
    hidh_le_gattc_dev_t database;                                           /**< GATT data for this device */
    hidh_le_wakeup_pattern_t wakeup_patterns[HIDH_WAKEUP_PATTERN_NB_MAX];   /**< The wakeup pattern database */
} hidh_le_dev_t;

/*
 * @brief   Structure containing the in information of a peer HID LE Device
 */
typedef struct
{
    uint8_t enable;             /**< To enable or disable */
    cyhal_gpio_t gpio_num;      /**< GPIO to wakeup system */
    uint8_t polarity;           /**< The polarity to for the GPIO to wake up system */
} hidh_le_wakeup_control_t;

/*
 * @brief   Structure for callbacks
 */
typedef struct
{
    hidh_le_dev_t devices[HIDH_DEV_MAX];                                    /**< Device Array */
    hidh_le_cback_t *p_callback;                                            /**< Application callback function */
    hidh_le_wakeup_control_t wakeup_control;                                /**< Wakeup control */
    hidh_le_filter_cback_t *p_filter_callbacks[HIDH_EVENT_FILTER_NB_MAX];   /**< filter check callback function array */
} hidh_le_cb_t;

#endif // __HIDH_LIB_INT_H_

/* end of file */
