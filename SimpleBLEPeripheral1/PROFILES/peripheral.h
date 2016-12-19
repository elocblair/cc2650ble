/**
  @headerfile:    peripheral.h
  $Date: 2016-01-11 13:54:08 -0800 (Mon, 11 Jan 2016) $
  $Revision: 44602 $

  @mainpage TI BLE GAP Peripheral Role for for RTOS Applications

  This GAP profile advertises and allows connections.

  Copyright 2009 - 2015 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
*/

#ifndef PERIPHERAL_H
#define PERIPHERAL_H

#ifdef __cplusplus
extern "C"
{
#endif

/*-------------------------------------------------------------------
 * INCLUDES
 */

/*-------------------------------------------------------------------
 * CONSTANTS
 */

/** @defgroup GAPROLE_PROFILE_PARAMETERS GAP Role Parameters
 * @{
 */
#define GAPROLE_PROFILEROLE         0x300  //!< Reading this parameter will return GAP Role type. Read Only. Size is uint8_t.
#define GAPROLE_IRK                 0x301  //!< Identity Resolving Key. Read/Write. Size is uint8_t[KEYLEN]. Default is all 0, which means that the IRK will be randomly generated.
#define GAPROLE_SRK                 0x302  //!< Signature Resolving Key. Read/Write. Size is uint8_t[KEYLEN]. Default is all 0, which means that the SRK will be randomly generated.
#define GAPROLE_SIGNCOUNTER         0x303  //!< Sign Counter. Read/Write. Size is uint32_t. Default is 0.
#define GAPROLE_BD_ADDR             0x304  //!< Device's Address. Read Only. Size is uint8_t[B_ADDR_LEN]. This item is read from the controller.
#define GAPROLE_ADVERT_ENABLED      0x305  //!< Enable/Disable Advertising. Read/Write. Size is uint8_t. Default is TRUE=Enabled.
#define GAPROLE_ADVERT_OFF_TIME     0x306  //!< Advertising Off Time for Limited advertisements (in milliseconds). Read/Write. Size is uint16_t. Default is 30 seconds.
#define GAPROLE_ADVERT_DATA         0x307  //!< Advertisement Data. Read/Write. Max size is uint8_t[B_MAX_ADV_LEN].  Default is "02:01:01", which means that it is a Limited Discoverable Advertisement.
#define GAPROLE_SCAN_RSP_DATA       0x308  //!< Scan Response Data. Read/Write. Max size is uint8_t[B_MAX_ADV_LEN]. Defaults to all 0.
#define GAPROLE_ADV_EVENT_TYPE      0x309  //!< Advertisement Type. Read/Write. Size is uint8_t.  Default is GAP_ADTYPE_ADV_IND (defined in GAP.h).
#define GAPROLE_ADV_DIRECT_TYPE     0x30A  //!< Direct Advertisement Address Type. Read/Write. Size is uint8_t. Default is ADDRTYPE_PUBLIC (defined in GAP.h).
#define GAPROLE_ADV_DIRECT_ADDR     0x30B  //!< Direct Advertisement Address. Read/Write. Size is uint8_t[B_ADDR_LEN]. Default is NULL.
#define GAPROLE_ADV_CHANNEL_MAP     0x30C  //!< Which channels to advertise on. Read/Write Size is uint8_t. Default is GAP_ADVCHAN_ALL (defined in GAP.h)
#define GAPROLE_ADV_FILTER_POLICY   0x30D  //!< Filter Policy. Ignored when directed advertising is used. Read/Write. Size is uint8_t. Default is GAP_FILTER_POLICY_ALL (defined in GAP.h).
#define GAPROLE_CONNHANDLE          0x30E  //!< Connection Handle. Read Only. Size is uint16_t.
#define GAPROLE_RSSI_READ_RATE      0x30F  //!< How often to read the RSSI during a connection. Read/Write. Size is uint16_t. The value is in milliseconds. Default is 0 = OFF. Obsolete - Do not use.
#define GAPROLE_PARAM_UPDATE_ENABLE 0x310  //!< Slave Connection Parameter Update Enable. Read/Write. Size is uint8_t. If TRUE then automatic connection parameter update request is sent. Default is FALSE.
#define GAPROLE_MIN_CONN_INTERVAL   0x311  //!< Minimum Connection Interval to allow (n * 1.25ms).  Range: 7.5 msec to 4 seconds (0x0006 to 0x0C80). Read/Write. Size is uint16_t. Default is 7.5 milliseconds (0x0006).
#define GAPROLE_MAX_CONN_INTERVAL   0x312  //!< Maximum Connection Interval to allow (n * 1.25ms).  Range: 7.5 msec to 4 seconds (0x0006 to 0x0C80). Read/Write. Size is uint16_t. Default is 4 seconds (0x0C80).
#define GAPROLE_SLAVE_LATENCY       0x313  //!< Update Parameter Slave Latency. Range: 0 - 499. Read/Write. Size is uint16_t. Default is 0.
#define GAPROLE_TIMEOUT_MULTIPLIER  0x314  //!< Update Parameter Timeout Multiplier (n * 10ms). Range: 100ms to 32 seconds (0x000a - 0x0c80). Read/Write. Size is uint16_t. Default is 1000.
#define GAPROLE_CONN_BD_ADDR        0x315  //!< Address of connected device. Read only. Size is uint8_t[B_MAX_ADV_LEN]. Set to all zeros when not connected.
#define GAPROLE_CONN_INTERVAL       0x316  //!< Current connection interval.  Read only. Size is uint16_t.  Range is 7.5ms to 4 seconds (0x0006 to 0x0C80).  Default is 0 (no connection).
#define GAPROLE_CONN_LATENCY        0x317  //!< Current slave latency.  Read only.  Size is uint16_t.  Range is 0 to 499. Default is 0 (no slave latency or no connection).
#define GAPROLE_CONN_TIMEOUT        0x318  //!< Current timeout value.  Read only.  size is uint16_t.  Range is 100ms to 32 seconds.  Default is 0 (no connection).
#define GAPROLE_PARAM_UPDATE_REQ    0x319  //!< Slave Connection Parameter Update Request. Write. Size is uint8_t. If TRUE then connection parameter update request is sent.
#define GAPROLE_STATE               0x31A  //!< Reading this parameter will return GAP Peripheral Role State. Read Only. Size is uint8_t.
#define GAPROLE_ADV_NONCONN_ENABLED 0x31B  //!< Enable/Disable Non-Connectable Advertising.  Read/Write.  Size is uint8_t.  Default is FALSE=Disabled.
#define GAPROLE_BD_ADDR_TYPE        0x31C  //!< Address type of connected device. Read only. Size is uint8_t.
#define GAPROLE_CONN_TERM_REASON    0x31D  //!< Reason of the last connection terminated event. Size is uint8_t.
   
/** @} End GAPROLE_PROFILE_PARAMETERS */

/*-------------------------------------------------------------------
 * TYPEDEFS
 */

/**
 * GAP Peripheral Role States.
 */
//! [GAP Peripheral Role State]
typedef enum
{
  GAPROLE_INIT = 0,                       //!< Waiting to be started
  GAPROLE_STARTED,                        //!< Started but not advertising
  GAPROLE_ADVERTISING,                    //!< Currently Advertising
  GAPROLE_ADVERTISING_NONCONN,            //!< Currently using non-connectable Advertising
  GAPROLE_WAITING,                        //!< Device is started but not advertising, is in waiting period before advertising again
  GAPROLE_WAITING_AFTER_TIMEOUT,          //!< Device just timed out from a connection but is not yet advertising, is in waiting period before advertising again
  GAPROLE_CONNECTED,                      //!< In a connection
  GAPROLE_CONNECTED_ADV,                  //!< In a connection + advertising
  GAPROLE_ERROR                           //!< Error occurred - invalid state
} gaprole_States_t;
//! [GAP Peripheral Role State]

/**
 *  Possible actions the peripheral device may take if an unsuccessful parameter
 *  update is received.
 *
 *  Parameters for GAPRole_SendUpdateParam() only
 */

#define GAPROLE_NO_ACTION                    0 // Take no action upon unsuccessful parameter updates
#define GAPROLE_RESEND_PARAM_UPDATE          1 // Continue to resend request until successful update
#define GAPROLE_TERMINATE_LINK               2 // Terminate link upon unsuccessful parameter updates

/*-------------------------------------------------------------------
 * MACROS
 */

/*-------------------------------------------------------------------
 * Profile Callbacks
 */

/**
 * Callback when the connection parameters are updated.
 */
typedef void (*gapRolesParamUpdateCB_t)(uint16_t connInterval,
                                        uint16_t connSlaveLatency,
                                        uint16_t connTimeout);

/**
 * Callback when the device has been started.  Callback event to
 * the Notify of a state change.
 */
typedef void (*gapRolesStateNotify_t)(gaprole_States_t newState);

/**
 * Callback structure - must be setup by the application and used when 
 *                      GAPRole_StartDevice() is called.
 */
typedef struct
{
  gapRolesStateNotify_t    pfnStateChange;  //!< Whenever the device changes state
} gapRolesCBs_t;

/*-------------------------------------------------------------------
 * API FUNCTIONS
 */

/**
 * @defgroup GAPROLES_PERIPHERAL_API GAP Peripheral Role API Functions
 *
 * @{
 */

/**
 * @brief       Set a GAP Role parameter.
 *
 *  NOTE: You can call this function with a GAP Parameter ID and it will set the
 *        GAP Parameter.  GAP Parameters are defined in (gap.h).  Also,
 *        the "len" field must be set to the size of a "uint16_t" and the
 *        "pValue" field must point to a "uint16_t".
 *
 * @param       param - Profile parameter ID: @ref GAPROLE_PROFILE_PARAMETERS
 * @param       len - length of data to write
 * @param       pValue - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return      SUCCESS or INVALIDPARAMETER (invalid paramID)
 */
extern bStatus_t GAPRole_SetParameter(uint16_t param, uint8_t len, void *pValue);

/**
 * @brief       Get a GAP Role parameter.
 *
 *  NOTE: You can call this function with a GAP Parameter ID and it will get a
 *        GAP Parameter.  GAP Parameters are defined in (gap.h).  Also, the
 *        "pValue" field must point to a "uint16_t".
 *
 * @param       param - Profile parameter ID: @ref GAPROLE_PROFILE_PARAMETERS
 * @param       pValue - pointer to location to get the value.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return      SUCCESS or INVALIDPARAMETER (invalid paramID)
 */
extern bStatus_t GAPRole_GetParameter(uint16_t param, void *pValue);

/**
 * @brief       Does the device initialization.  Only call this function once.
 *
 * @param       pAppCallbacks - pointer to application callbacks.
 *
 * @return      SUCCESS or bleAlreadyInRequestedMode
 */
extern bStatus_t GAPRole_StartDevice(gapRolesCBs_t *pAppCallbacks);

/**
 * @brief       Terminates the existing connection.
 *
 * @return      SUCCESS or bleIncorrectMode
 */
extern bStatus_t GAPRole_TerminateConnection(void);

/**
 * @brief       Update the parameters of an existing connection
 *
 * @param       connInterval - the new connection interval
 * @param       latency - the new slave latency
 * @param       connTimeout - the new timeout value
 * @param       handleFailure - what to do if the update does not occur.
 *              Method may choose to terminate connection, try again, or take no action
 *
 * @return      SUCCESS, bleNotConnected or bleInvalidRange
 */
extern bStatus_t GAPRole_SendUpdateParam(uint16_t minConnInterval, 
                                         uint16_t maxConnInterval,
                                         uint16_t latency, uint16_t connTimeout,
                                         uint8_t handleFailure);

/**
 * @brief       Register application's callbacks.
 *
 * @param       pParamUpdateCB - pointer to param update callback.
 *
 * @return      none
 */
extern void GAPRole_RegisterAppCBs(gapRolesParamUpdateCB_t *pParamUpdateCB);

/**
 * @} End GAPROLES_PERIPHERAL_API
 */


/*-------------------------------------------------------------------
 * TASK FUNCTIONS - Don't call these. These are system functions.
 */
   
/**
 * @brief       Task creation function for the GAP Peripheral Role.
 *
 * @param       none
 *
 * @return      none
 */
extern void GAPRole_createTask(void);

/*-------------------------------------------------------------------
-------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* PERIPHERAL_H */
