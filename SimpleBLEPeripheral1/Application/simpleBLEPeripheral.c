/*******************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2016-01-07 16:59:59 -0800 (Thu, 07 Jan 2016) $
  Revision:       $Revision: 44594 $

  Description:    This file contains the Simple BLE Peripheral sample 
                  application for use with the CC2650 Bluetooth Low Energy 
                  Protocol Stack.

  Copyright 2013 - 2015 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

  saves 6 seconds of data to fram then dumps it all after connecting, enabling notifications and writing a 3 to char 1
*******************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <xdc/runtime/System.h>

#include "hci_tl.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#if defined(SENSORTAG_HW)
#include "bsp_spi.h"
#endif // SENSORTAG_HW

#if defined(FEATURE_OAD) || defined(IMAGE_INVALIDATE)
#include "oad_target.h"
#include "oad.h"
#endif //FEATURE_OAD || IMAGE_INVALIDATE

#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "ICallBleAPIMSG.h"

#include "util.h"
#include "board_lcd.h"
#include "board_key.h"
#include "Board.h"

#include "simpleBLEPeripheral.h"
#include <ti/drivers/prcm.h>
#include "math.h"
#include <ti/drivers/ssi.h>

#include <ti/drivers/lcd/LCDDogm1286.h>

#include <driverlib/i2c.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>
#include <ti/sysbios/family/arm/cc26xx/Power.h>

/*********************************************************************
 * CONSTANTS
 */
// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

#ifndef FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     6

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     6
#else
// Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     6

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     6
#endif // FEATURE_OAD

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         5

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          1000

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// How often to perform periodic event (in msec)
#define SBP_PERIODIC_EVT_PERIOD               10 //cole

#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE                       ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

// Task configuration
#define SBP_TASK_PRIORITY                     1

#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE                   644
#endif

// Internal Events for RTOS application
#define SBP_STATE_CHANGE_EVT                  0x0001
#define SBP_CHAR_CHANGE_EVT                   0x0002
#define SBP_PERIODIC_EVT                      0x0004
#define SBP_CONN_EVT_END_EVT                  0x0008

//COLE spi defines

#define SCLK IOID_22
#define MOSI IOID_20
#define MISO IOID_21
#define ADXL_CS IOID_7
#define ADXL GPIO_PIN_7
#define FRAM_CS IOID_10
#define FRAM GPIO_PIN_10

//#define I2C_ADDRESS IOID_24
//#define ADDRESS GPIO_PIN_24

#define SYSCLK_FREQ 48000000
#define SPICLK_FREQ 2000000
#define FRAME_LENGTH 16
#define SAMP_DUR 150 // in ms

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
  appEvtHdr_t hdr;  // event header.
} sbpEvt_t;

/*********************************************************************
 * LOCAL VARIABLES
 */

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

#if defined(FEATURE_OAD)
// Event data from OAD profile.
static Queue_Struct oadQ;
static Queue_Handle hOadQ;
#endif //FEATURE_OAD

// events flag for internal application events.
static uint16_t events;

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[SBP_TASK_STACK_SIZE];

// Profile state and parameters
//static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
  // complete name
  0x10,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x53,   // 'S'
  0x48,   // 'H'
  0x4F,   // 'O'
  0x43,   // 'C'
  0x4B,   // 'K'
  0x43,   // 'C'
  0x4C,   // 'L'
  0x4F,   // 'O'
  0x43,   // 'C'
  0x4B,   // 'K'
  0x65,   // 'e'
  0x6C,   // 'l'
  0x6F,   // 'o'
  0x63,   // 'c'


  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),   // 100ms
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),   // 1s
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
#ifdef FEATURE_OAD
  LO_UINT16(OAD_SERVICE_UUID),
  HI_UINT16(OAD_SERVICE_UUID)
#else
  LO_UINT16(SIMPLEPROFILE_SERV_UUID),
  HI_UINT16(SIMPLEPROFILE_SERV_UUID),
#endif //!FEATURE_OAD
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "wir31ess";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;
uint16_t ADXLbuffer[105];

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimpleBLEPeripheral_init( void );
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg);
static void SimpleBLEPeripheral_processStateChangeEvt(
                                                     gaprole_States_t newState);
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID);
static void SimpleBLEPeripheral_performPeriodicTask(void);

static void SimpleBLEPeripheral_sendAttRsp(void);
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status);

static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState);
#ifndef FEATURE_OAD
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID);
#endif //!FEATURE_OAD
static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state);

#ifdef FEATURE_OAD
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData);
#endif //FEATURE_OAD

static void SimpleBLEPeripheral_clockHandler(UArg arg);

//cole heap variables

void ADXL_SPI_init();
void SPI_read(uint32_t sensor, uint32_t cmd, uint32_t *value);
void SPI_write(uint32_t sensor, uint32_t cmd, uint32_t *get);
//static double convertSensorReading(uint32_t sensorHigh, uint32_t sensorLow, double scalar);
void ReadBosch(uint8_t BoschReg, uint8_t NumReg, uint8_t *data, uint8_t deviceAddress);
void WriteBosch(uint8_t BoschReg, uint8_t data, uint8_t deviceAddress);

uint8_t count = 1;
uint8_t counter = 0;
uint32_t high, low;
double val;
uint8_t charValue1[192];
uint8_t charValue2[18];
//uint16_t ADXLbuffer[105]; // 96 for ADXL and 9 for bosch gyro
void ADXL_read();
void doUrgentWork();
void pinInterruptHandler(PIN_Handle handle, PIN_Id pinId);
Void urgentWorkTaskFunc(UArg arg0, UArg arg1);
void moveToFRAM();
void readFRAMid();
void enableWriteToFRAM();
void readFromFRAM();
void writeToFRAM();
void gyroRead1();
void gyroRead2();
void gyroRead3();
void readAllGyro();
void readAllGyro2();
void readAllGyro3();
int firstFlag = 1;
PIN_Config pinTable[] = {
	  Board_UART_CTS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH   | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	  IOID_1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH   | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	  IOID_23 | PIN_INPUT_EN | PIN_PULLUP,

    PIN_TERMINATE
};

Semaphore_Handle cole;
Task_Struct urgentWorkTask;
//static uint8_t urgentWorkTaskStack[256];
static PIN_Handle pinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State pinState;

uint16_t FRAMregisterRead = 0xD00B;
uint16_t FRAMregisterAcc = 0;
uint16_t FRAMregisterGyro = 0;
uint16_t registerCount = 0;
int FRAMfull = 0;
int connectedFlag = 0;
int boschCount = 0;
uint8_t boschData[3000];
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t SimpleBLEPeripheral_gapRoleCBs =
{
  SimpleBLEPeripheral_stateChangeCB     // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL, // Passcode callback (not used by application)
  NULL  // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
#ifndef FEATURE_OAD
static simpleProfileCBs_t SimpleBLEPeripheral_simpleProfileCBs =
{
  SimpleBLEPeripheral_charValueChangeCB // Characteristic value change callback
};
#endif //!FEATURE_OAD

#ifdef FEATURE_OAD
static oadTargetCBs_t simpleBLEPeripheral_oadCBs =
{
  SimpleBLEPeripheral_processOadWriteCB // Write Callback.
};
#endif //FEATURE_OAD



/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = sbpTaskStack;
  taskParams.stackSize = SBP_TASK_STACK_SIZE;
  taskParams.priority = SBP_TASK_PRIORITY;

  Task_construct(&sbpTask, SimpleBLEPeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_init(void)
{
  // ******************************************************************
  // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
  // ******************************************************************
  // Register the current thread as an ICall dispatcher application
  // so that the application can send and receive messages.
  ICall_registerApp(&selfEntity, &sem);

  // Hard code the BD Address till CC2650 board gets its own IEEE address
  //uint8 bdAddress[B_ADDR_LEN] = { 0xAD, 0xD0, 0x0A, 0xAD, 0xD0, 0x0A };
  //HCI_EXT_SetBDADDRCmd(bdAddress);

  // Set device's Sleep Clock Accuracy
  //HCI_EXT_SetSCACmd(500);
  
  // Create an RTOS queue for message from profile to be sent tmaio app.
  appMsgQueue = Util_constructQueue(&appMsg);

  // Create one-shot clocks for internal periodic events.
  Util_constructClock(&periodicClock, SimpleBLEPeripheral_clockHandler,
                      3.3333, 0, false, SBP_PERIODIC_EVT);
  
#ifndef SENSORTAG_HW
  Board_openLCD();
#endif //SENSORTAG_HW
  
#if SENSORTAG_HW
  // Setup SPI bus for serial flash and Devpack interface
  bspSpiOpen();
#endif //SENSORTAG_HW
  
  // Setup the GAP
  GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    // For all hardware platforms, device starts advertising upon initialization
    uint8_t initialAdvertEnable = TRUE;
    //uint8_t initialAdvertEnable = FALSE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16_t advertOffTime = 0;

    uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    //cole try advert not enabled and see if we can wait on an interrupt
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                         &initialAdvertEnable);
    		//FALSE);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                         &advertOffTime);

    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                         scanRspData);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                         &enableUpdateRequest);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMinInterval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                         &desiredMaxInterval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                         &desiredSlaveLatency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                         &desiredConnTimeout);
  }

  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

  // Set advertising interval
  {
    uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }

  // Setup the GAP Bond Manager
  {
    uint32_t passkey = 0; // passkey "000000"
    uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8_t mitm = TRUE;
    uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8_t bonding = TRUE;

    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                            &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);
  }

   // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  DevInfo_AddService();                        // Device Information Service

#ifndef FEATURE_OAD
  SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile
#endif //!FEATURE_OAD

#ifdef FEATURE_OAD
  VOID OAD_addService();                 // OAD Profile
  OAD_register((oadTargetCBs_t *)&simpleBLEPeripheral_oadCBs);
  hOadQ = Util_constructQueue(&oadQ);
#endif

#ifdef IMAGE_INVALIDATE
  Reset_addService();
#endif //IMAGE_INVALIDATE
  
  
#ifndef FEATURE_OAD
  // Setup the SimpleProfile Characteristic Values
  {
    uint8_t charValue1[SIMPLEPROFILE_CHAR5_LEN] = {0};
    uint8_t boschData = 2;
    uint8_t charValue3 = 3;
    uint8_t charValue4[SIMPLEPROFILE_CHAR5_LEN] = {4};
    uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };

    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t),
                               &charValue1);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint8_t),
                               &boschData);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t),
                               &charValue3);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR5_LEN,
                               &charValue4);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
                               charValue5);
  }

  // Register callback with SimpleGATTprofile
  SimpleProfile_RegisterAppCBs(&SimpleBLEPeripheral_simpleProfileCBs);
#endif //!FEATURE_OAD
  
  // Start the Device
  VOID GAPRole_StartDevice(&SimpleBLEPeripheral_gapRoleCBs);
  
  // Start Bond Manager
  VOID GAPBondMgr_Register(&simpleBLEPeripheral_BondMgrCBs);

  // Register with GAP for HCI/Host messages
  GAP_RegisterForMsgs(selfEntity);
  
  // Register for GATT local events and ATT Responses pending for transmission
  GATT_RegisterForMsgs(selfEntity);
  
#if defined FEATURE_OAD
#if defined (HAL_IMAGE_A)
  //LCD_WRITE_STRING("BLE Peripheral A", LCD_PAGE0);
#else
  //LCD_WRITE_STRING("BLE Peripheral B", LCD_PAGE0);
#endif // HAL_IMAGE_A
#else
  //LCD_WRITE_STRING("BLE Peripheral", LCD_PAGE0);
#endif // FEATURE_OAD
}       

/*********************************************************************
 * @fn      SimpleBLEPeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1)
{
  // initialize interrupt
  pinHandle = PIN_open(&pinState, pinTable);
  PIN_registerIntCb(pinHandle, pinInterruptHandler);
  PIN_setInterrupt(pinHandle, IOID_23 | PIN_IRQ_NEGEDGE);

  // Initialize application
  SimpleBLEPeripheral_init();

  Power_setDependency(PERIPH_SSI0);
  Power_setDependency(PERIPH_I2C0);
  Power_setConstraint(Power_SB_DISALLOW);
  Power_setConstraint(Power_IDLE_PD_DISALLOW);
  Power_setConstraint(Power_SD_DISALLOW);
  uint8_t powerState = Power_getConstraintInfo();
  charValue2[0] = powerState;
  uint32_t get;
  //uint8_t data[1];
  ADXL_SPI_init();

  /*
   * 0x00 deivce id = 0xE5
   * 0x1D Shock Int Threshold 780mg/LSB cannot be zero if single or double shock interrupt is enabled
   * 0x1E - 0x20 Offset
   * 0x21 Shock Duration of MAX time that device can be over shock threshold and still count as shock int
   * 		zero disables single and double shock functions
   * 0x22 Shock Latency time where another interrupt cannot occur
   * 		zero disables double shock int
   * 0x23 Shock Window time period after first shock and Latency where second shock can occur
   * 		zero diables double shock int
   * 0x24 Activity interrupt threshold 780mg/LSB
   * 		cannot be zero if activity int is enabled
   * 0x25 Threshold inactivity 780mg/LSB
   * 		cannot be zero if inactivity threshold is enabled
   * 0x26 Time INactivity 1 second per lsb, amount of time it must be below threshold
   * 0x27 Activity/ inactivity control, enable axes for act or inact also chooses ac or dc coupled
   * 0x2A Enable Shock Axes
   * 0x2B readonly shows the source axis of act or shock event
   * 0x2C BW and rate, configured for 100 Hz default
   * 0x2D POwer control
   * 0x2E Interrupt Enable setting of 1 enables interrupts from that function
   * 		configure interrupt maps (0x2F) first
   * 0x30 READ only interrupt source, if 1 that source has triggered an int
   * 0x31 Data Format
   * 0x32 - 0x37 DATA
   * 0x38 FIFO Control
   * */
 /* enableWriteToFRAM();
  writeToFRAM();
  readFromFRAM();*/
  SPI_write(ADXL,0x24FF,&get);   // threshold for activity int
  SPI_write(ADXL,0x1D8F,&get);   // set threshold for shock int 780mg/LSB
  //SPI_write(ADXL,0x21FF,&get);   // duration for shock int
  SPI_write(ADXL,0x2D08,&get);   // measurement mode
  SPI_write(ADXL,0x2F00,&get);   // interrupt map all set to int1
  SPI_write(ADXL,0x2710,&get);   // Activity int axes enabled 7 = all 1 = z
  SPI_write(ADXL,0x2E10,&get);   // interrupt enable
  SPI_write(ADXL,0x3880,&get);   // fifo stream mode


	IOCPinTypeI2c	( I2C0_BASE, IOID_27, IOID_28); //sets pin 27 as data and 28 as clock (11,8 for anterior board)
	I2CMasterInitExpClk ( I2C0_BASE, 48000000, false ); //initialize i2c clock speed, true = 100kbps
	I2CMasterEnable(I2C0_BASE);
	WriteBosch(0x3E, 0x00, 0x29); //Normal Power mode
	WriteBosch(0x07, 0x01, 0x29); // PAGE ID all config ones are on page 1
	WriteBosch(0x0A, 0x00, 0x29); // fastest odr
	WriteBosch(0x07, 0x00, 0x29); //Change back to page 0
	WriteBosch(0x3D, 0x03, 0x29); // change from config mode to gyro only mode
	Util_startClock(&periodicClock);

  // Application main loop
  for (;;)
  {
    // Waits for a signal to the semaphore associated with the calling thread.
    // Note that the semaphore associated with a thread is signaled when a
    // message is queued to the message receive queue of the thread or when
    // ICall_signal() function is called onto the semaphore.
    ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

    if (errno == ICALL_ERRNO_SUCCESS)
    {
      ICall_EntityID dest;
      ICall_ServiceEnum src;
      ICall_HciExtEvt *pMsg = NULL;

      if (ICall_fetchServiceMsg(&src, &dest,
                                (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
      {
        uint8 safeToDealloc = TRUE;
        
        if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
        {
          ICall_Event *pEvt = (ICall_Event *)pMsg;
          
          // Check for BLE stack events first
          if (pEvt->signature == 0xffff)
          {
            if (pEvt->event_flag & SBP_CONN_EVT_END_EVT)
            {
              // Try to retransmit pending ATT Response (if any)
              SimpleBLEPeripheral_sendAttRsp();
            }
          }
          else
          {
            // Process inter-task message
            safeToDealloc = SimpleBLEPeripheral_processStackMsg(
                                                             (ICall_Hdr *)pMsg);
          }
        }

        if (pMsg && safeToDealloc)
        {
          ICall_freeMsg(pMsg);
        }
      }

      // If RTOS queue is not empty, process app message.
      while (!Queue_empty(appMsgQueue))
      {
        sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
        if (pMsg)
        {
          // Process message.
          SimpleBLEPeripheral_processAppMsg(pMsg);

          // Free the space from the message.
          ICall_free(pMsg);
        }
      }
    }

    if (events & SBP_PERIODIC_EVT)
    {

      /*
       * saving previous some previous data may be necessary
       */
      events &= ~SBP_PERIODIC_EVT;
      /*
       * this starts the clock immediately to make sure there is no delay caused by the time spent doing SPI communication
       * we should have no problem getting everything done in 10ms
       */
      Util_startClock(&periodicClock);


      /*
       * Periodic task is the function used for gyroscope testing at 400Hz
       *
       */
      //SimpleBLEPeripheral_performPeriodicTask();

      /*
       * This function moves arbitrary data to the FRAM for now
       */
      if (connectedFlag){
    	    //if (FRAMfull){
    		  uint8_t newValue;
    		  //SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newValue);
    		  //if(newValue == 3){

    		     // uint8_t powerState = Power_getConstraintInfo();
    		      //SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2,1, &powerState);
        		  if (boschCount == 0){
            		  readAllGyro();
        			  //gyroRead1();
            		  //moveToFRAM();
            	  }
            	  if (boschCount == 1){
            		  //gyroRead2();
            		  readAllGyro2();
            	  }
            	  if (boschCount == 2){
              		  //PIN_setOutputValue(pinHandle, Board_UART_CTS, 0);
                      readAllGyro3();
            		  //readFromFRAM();
                      ADXL_read();
            		  boschCount = -1;
            		  SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR5_LEN,
            		  							  charValue2);

            	  }
            	  boschCount++;
    		 // }


         // }
      }

      if (firstFlag){
    		/*IOCPinTypeI2c	( I2C0_BASE, IOID_27, IOID_28); //sets pin 27 as data and 28 as clock (11,8 for anterior board)
    		I2CMasterInitExpClk ( I2C0_BASE, 48000000, false ); //initialize i2c clock speed, true = 100kbps
    		WriteBosch(0x3E, 0x00, 0x29); //Normal Power mode
    		WriteBosch(0x07, 0x01, 0x29); // PAGE ID all config ones are on page 1
    		WriteBosch(0x0A, 0x00, 0x29); // fastest odr
    		WriteBosch(0x07, 0x00, 0x29); //Change back to page 0
    		WriteBosch(0x3D, 0x03, 0x29); // change from config mode to gyro only mode*/
    		//gyroRead1();
    		//readAllGyro();
    		firstFlag = 0;
      }
	  /*
      if(!FRAMfull){
    	  if (boschCount == 0){


    		  readAllGyro();

    	  }
    	  if (boschCount == 1){

    		  readAllGyro2();
    	  }
    	  if (boschCount == 2){

    		  readAllGyro3();
    		  moveToFRAM();
    		  boschCount = -1;
    	  }
    	  boschCount++;
      }
	  if(FRAMfull && !connectedFlag){
		  uint8_t initialAdvertEnable = TRUE;
		  GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
		                         &initialAdvertEnable);
	  }*/
    }
    
#ifdef FEATURE_OAD
    while (!Queue_empty(hOadQ))
    {
      oadTargetWrite_t *oadWriteEvt = Queue_dequeue(hOadQ);

      // Identify new image.
      if (oadWriteEvt->event == OAD_WRITE_IDENTIFY_REQ)
      {
        OAD_imgIdentifyWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }
      // Write a next block request.
      else if (oadWriteEvt->event == OAD_WRITE_BLOCK_REQ)
      {
        OAD_imgBlockWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
      }

      // Free buffer.
      ICall_free(oadWriteEvt);
    }
#endif //FEATURE_OAD
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg)
{
  uint8_t safeToDealloc = TRUE;
    
  switch (pMsg->event)
  {
    case GATT_MSG_EVENT:
      // Process GATT message
      safeToDealloc = SimpleBLEPeripheral_processGATTMsg(
                                                        (gattMsgEvent_t *)pMsg);
      break;

    case HCI_GAP_EVENT_EVENT:
      {
        // Process HCI message
        switch(pMsg->status)
        {
          case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            break;
            
          default:
            break;
        }
      }
      break;
      
    default:
      // do nothing
      break;
  }
  
  return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
  // See if GATT server was unable to transmit an ATT response
  if (pMsg->hdr.status == blePending)
  {
    // No HCI buffer was available. Let's try to retransmit the response
    // on the next connection event.
    if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                   SBP_CONN_EVT_END_EVT) == SUCCESS)
    {
      // First free any pending response
      SimpleBLEPeripheral_freeAttRsp(FAILURE);
      
      // Hold on to the response message for retransmission
      pAttRsp = pMsg;
      
      // Don't free the response message yet
      return (FALSE);
    }
  }
  else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
  {
    // ATT request-response or indication-confirmation flow control is
    // violated. All subsequent ATT requests or indications will be dropped.
    // The app is informed in case it wants to drop the connection.
    
    // Display the opcode of the message that caused the violation.
    //LCD_WRITE_STRING_VALUE("FC Violated:", pMsg->msg.flowCtrlEvt.opcode,
                          // 10, LCD_PAGE5);
  }    
  else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
  {
    // MTU size updated
    //LCD_WRITE_STRING_VALUE("MTU Size:", pMsg->msg.mtuEvt.MTU, 10, LCD_PAGE5);
  }
  
  // Free message payload. Needed only for ATT Protocol messages
  GATT_bm_free(&pMsg->msg, pMsg->method);
  
  // It's safe to free the incoming message
  return (TRUE);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEPeripheral_sendAttRsp(void)
{
  // See if there's a pending ATT Response to be transmitted
  if (pAttRsp != NULL)
  {
    uint8_t status;
    
    // Increment retransmission count
    rspTxRetry++;
    
    // Try to retransmit ATT response till either we're successful or
    // the ATT Client times out (after 30s) and drops the connection.
    status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, 
                          &(pAttRsp->msg));
    if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
    {
      // Disable connection event end notice
      HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);
      
      // We're done with the response message
      SimpleBLEPeripheral_freeAttRsp(status);
    }
    else
    {
      // Continue retrying
      //LCD_WRITE_STRING_VALUE("Rsp send retry:", rspTxRetry, 10, LCD_PAGE5);
    }
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status)
{
  // See if there's a pending ATT response message
  if (pAttRsp != NULL)
  {
    // See if the response was sent out successfully
    if (status == SUCCESS)
    {
      //LCD_WRITE_STRING_VALUE("Rsp sent, retry:", rspTxRetry, 10, LCD_PAGE5);
    }
    else
    {
      // Free response payload
      GATT_bm_free(&pAttRsp->msg, pAttRsp->method);
      
      //LCD_WRITE_STRING_VALUE("Rsp retry failed:", rspTxRetry, 10, LCD_PAGE5);
    }
    
    // Free response message
    ICall_freeMsg(pAttRsp);
    
    // Reset our globals
    pAttRsp = NULL;
    rspTxRetry = 0;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg)
{
  switch (pMsg->hdr.event)
  {
    case SBP_STATE_CHANGE_EVT:
      SimpleBLEPeripheral_processStateChangeEvt((gaprole_States_t)pMsg->
                                                hdr.state);
      break;

    case SBP_CHAR_CHANGE_EVT:
      SimpleBLEPeripheral_processCharValueChangeEvt(pMsg->hdr.state);
      break;

    default:
      // Do nothing.
      break;
  }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_STATE_CHANGE_EVT, newState);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState)
{
#ifdef PLUS_BROADCASTER
  static bool firstConnFlag = false;
#endif // PLUS_BROADCASTER

  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, 
                             systemId);

        //Util_startClock(&periodicClock);
        // Display device address
        //LCD_WRITE_STRING(Util_convertBdAddr2Str(ownAddress), LCD_PAGE1);
        //LCD_WRITE_STRING("Initialized", LCD_PAGE2);
      }
      break;

    case GAPROLE_ADVERTISING:
    	//Util_startClock(&periodicClock);
    	//LCD_WRITE_STRING("Advertising", LCD_PAGE2);
      break;

#ifdef PLUS_BROADCASTER   
    /* After a connection is dropped a device in PLUS_BROADCASTER will continue
     * sending non-connectable advertisements and shall sending this change of 
     * state to the application.  These are then disabled here so that sending 
     * connectable advertisements can resume.
     */
    case GAPROLE_ADVERTISING_NONCONN:
      {
        uint8_t advertEnabled = FALSE;
      
        // Disable non-connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                           &advertEnabled);
      
        advertEnabled = TRUE;
      
        // Enabled connectable advertising.
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &advertEnabled);
        
        // Reset flag for next connection.
        firstConnFlag = false;
        
        SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
      }
      break;
#endif //PLUS_BROADCASTER   

    case GAPROLE_CONNECTED:
      {
        uint8_t peerAddress[B_ADDR_LEN];

        GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

        //Util_startClock(&periodicClock);
        connectedFlag = 1;
        //LCD_WRITE_STRING("Connected", LCD_PAGE2);
        //LCD_WRITE_STRING(Util_convertBdAddr2Str(peerAddress), LCD_PAGE3);

        #ifdef PLUS_BROADCASTER
          // Only turn advertising on for this state when we first connect
          // otherwise, when we go from connected_advertising back to this state
          // we will be turning advertising back on.
          if (firstConnFlag == false)
          {
            uint8_t advertEnabled = FALSE; // Turn on Advertising

            // Disable connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            
            // Set to true for non-connectabel advertising.
            advertEnabled = TRUE;

            // Enable non-connectable advertising.
            GAPRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8_t),
                                 &advertEnabled);
            firstConnFlag = true;
          }
        #endif // PLUS_BROADCASTER
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      //LCD_WRITE_STRING("Connected Advertising", LCD_PAGE2);
      break;

    case GAPROLE_WAITING:
      Util_stopClock(&periodicClock);
      
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);

      //LCD_WRITE_STRING("Disconnected", LCD_PAGE2);

      // Clear remaining lines
      //LCD_WRITE_STRING("", LCD_PAGE3);
      //LCD_WRITE_STRING("", LCD_PAGE4);
      //LCD_WRITE_STRING("", LCD_PAGE5);
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      SimpleBLEPeripheral_freeAttRsp(bleNotConnected);
      
      //LCD_WRITE_STRING("Timed Out", LCD_PAGE2);
      
      // Clear remaining lines
      //LCD_WRITE_STRING("", LCD_PAGE3);
      //LCD_WRITE_STRING("", LCD_PAGE4);
      //LCD_WRITE_STRING("", LCD_PAGE5);

      #ifdef PLUS_BROADCASTER
        // Reset flag for next connection.
        firstConnFlag = false;
      #endif //#ifdef (PLUS_BROADCASTER)
      break;

    case GAPROLE_ERROR:
      //LCD_WRITE_STRING("Error", LCD_PAGE2);
      break;

    default:
      //LCD_WRITE_STRING("", LCD_PAGE2);
      break;
  }

  // Update the state
  //gapProfileState = newState;
}

#ifndef FEATURE_OAD
/*********************************************************************
 * @fn      SimpleBLEPeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID)
{
  SimpleBLEPeripheral_enqueueMsg(SBP_CHAR_CHANGE_EVT, paramID);
}
#endif //!FEATURE_OAD

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID)
{
#ifndef FEATURE_OAD
  uint8_t newValue;

  switch(paramID)
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newValue);

      //LCD_WRITE_STRING_VALUE("Char 1:", (uint16_t)newValue, 10, LCD_PAGE4);
      break;

    case SIMPLEPROFILE_CHAR2:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR2, &newValue);

      //LCD_WRITE_STRING_VALUE("Char 2:", (uint16_t)newValue, 10, LCD_PAGE4);
      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR3, &newValue);

      //LCD_WRITE_STRING_VALUE("Char 3:", (uint16_t)newValue, 10, LCD_PAGE4);
      break;

    case SIMPLEPROFILE_CHAR5:
      SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR5, &newValue);

      //LCD_WRITE_STRING_VALUE("Char 5:", (uint16_t)newValue, 10, LCD_PAGE4);
      break;
    default:
      // should not reach here!
      break;
  }
#endif //!FEATURE_OAD
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (SBP_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_performPeriodicTask(void)
{
#ifndef FEATURE_OAD
	uint8_t data[1];
	ADXL_SPI_init();
	uint32_t temp;
	 /* SPI_write(ADXL,0x24FF,&temp);   // threshold for activity int
	  //SPI_write(ADXL,0x1D8F,&get);   // set threshold for shock int 780mg/LSB
	  //SPI_write(ADXL,0x21FF,&get);   // duration for shock int
	  SPI_write(ADXL,0x2D08,&temp);   // measurement mode
	  SPI_write(ADXL,0x2F00,&temp);   // interrupt map all set to int1
	  SPI_write(ADXL,0x2710,&temp);   // Activity int axes enabled 7 = all 1 = z
	  SPI_write(ADXL,0x2E10,&temp);   // interrupt enable*/
	//SPI_write(ADXL,0x1D8F,&temp);   // set threshold for shock int 780mg/LSB
	//SPI_write(ADXL,0x2101,&temp);
	/*SPI_write(ADXL,0x244F,&temp);
	SPI_write(ADXL,0x2770,&temp);
	SPI_write(ADXL,0x2D08,&temp);   // measurement mode
	SPI_write(ADXL,0x2F00,&temp);
	SPI_write(ADXL,0x2E10,&temp);   // interrupt enable
	SPI_write(ADXL,0x2A07,&temp);
	SPI_write(ADXL,0x2F00,&temp);*/
	IOCPinTypeI2c	( I2C0_BASE, IOID_27, IOID_28); //sets pin 27 as data and 28 as clock
	I2CMasterInitExpClk ( I2C0_BASE, 48000000, false ); //initialize i2c clock speed, true = 100kbps
	//WriteBosch(0x3E, 0x00, 0x28);
	//WriteBosch(0x3D, 0x03, 0x28);
	//WriteBosch(0x0A, 0x00, 0x28);
	WriteBosch(0x3E, 0x00, 0x29); //Normal Power mode
	WriteBosch(0x07, 0x01, 0x29); // PAGE ID all config ones are on page 1
	WriteBosch(0x0A, 0x00, 0x29); // fastest odr
	//WriteBosch(0x17, 0x88, 0x29); // interrupt stuff
	//WriteBosch(0x18, 0x02, 0x29); // interrupt
	WriteBosch(0x07, 0x00, 0x29); //Change back to page 0
	WriteBosch(0x3D, 0x03, 0x29); // change from config mode to gyro only mode
	if (count == 3 ){
		//ReadBosch(0x14,1,data, 0x28);
		//charValue1[13] = data[0];
		//ReadBosch(0x15,1,data, 0x28);
		//charValue1[12] = data[08];
		//ReadBosch(0x14,1,data, 0x29);
		//charValue1[15] = data[0];
		//ReadBosch(0x15,1,data, 0x29);
		//charValue1[14] = data[0];
		/*ReadBosch(0x16,1,data);
		charValue1[15] = data[0];
		ReadBosch(0x17,1,data);
		charValue1[14] = data[0];
		ReadBosch(0x18,1,data);
		charValue1[17] = data[0];
		ReadBosch(0x19,1,data);
		charValue1[16] = data[0];*/
		//charValue1[14] = counter++;

		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR5_LEN,
								  charValue1);
		count = 1;
	}
		if (count == 2){
		//ReadBosch(0x14,1,data, 0x28);
		//charValue1[7] = data[0];
		//ReadBosch(0x15,1,data, 0x28);
		//charValue1[6] = data[0];
		ReadBosch(0x14,1,data, 0x29);
		charValue1[9] = data[0];
		ReadBosch(0x15,1,data, 0x29);
		charValue1[8] = data[0];
		/*ReadBosch(0x16,1,data);
		charValue1[9] = data[0];
		ReadBosch(0x17,1,data);
		charValue1[8] = data[0];
		ReadBosch(0x18,1,data);
		charValue1[11] = data[0];
		ReadBosch(0x19,1,data);
		charValue1[10] = data[0];*/
		SPI_read(ADXL, 0xB300, &temp);
		charValue1[12] = (uint8_t) temp;
		SPI_read(ADXL, 0xB200, &temp);
		charValue1[13] = (uint8_t) temp;
		SPI_read(ADXL, 0xB500, &temp);
		charValue1[14] = (uint8_t) temp;
		SPI_read(ADXL, 0xB400, &temp);
		charValue1[15] = (uint8_t) temp;
		SPI_read(ADXL, 0xB700, &temp);
		charValue1[16] = (uint8_t) temp;
		SPI_read(ADXL, 0xB600, &temp);
		charValue1[17] = (uint8_t) temp;
		count = 3;
	}


	if(count == 1){
		//ADXL_SPI_init();
		//uint32_t temp;
		/*SPI_read(ADXL,0xA400, &temp);
		charValue1[0] = (uint8_t) temp;
		SPI_read(ADXL,0xB000, &temp);
		charValue1[1] = (uint8_t) temp;*/
		//ReadBosch(0x00,1,data, 0x28);
		//charValue1[1] = data[0];
		//ReadBosch(0x01,1,data, 0x28);
		//charValue1[0] = data[0];
		/*ReadBosch(0x14,1,data, 0x29);
		charValue1[3] = data[0];
		ReadBosch(0x15,1,data, 0x29);
		charValue1[2] = data[0];*/
		/*ReadBosch(0x16,1,data);
		charValue1[3] = data[0];
		ReadBosch(0x17,1,data);
		charValue1[2] = data[0];
		ReadBosch(0x18,1,data);
		charValue1[5] = data[0];
		ReadBosch(0x19,1,data);
		charValue1[4] = data[0];*/
		//ADXL_read();

		//count = 2;
	}

#endif //!FEATURE_OAD
}


#if defined(FEATURE_OAD)
/*********************************************************************
 * @fn      SimpleBLEPeripheral_processOadWriteCB
 *
 * @brief   Process a write request to the OAD profile.
 *
 * @param   event      - event type:
 *                       OAD_WRITE_IDENTIFY_REQ
 *                       OAD_WRITE_BLOCK_REQ
 * @param   connHandle - the connection Handle this request is from.
 * @param   pData      - pointer to data for processing and/or storing.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData)
{
  oadTargetWrite_t *oadWriteEvt = ICall_malloc( sizeof(oadTargetWrite_t) + \
                                             sizeof(uint8_t) * OAD_PACKET_SIZE);
  
  if ( oadWriteEvt != NULL )
  {
    oadWriteEvt->event = event;
    oadWriteEvt->connHandle = connHandle;
    
    oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
    memcpy(oadWriteEvt->pData, pData, OAD_PACKET_SIZE);

    Queue_enqueue(hOadQ, (Queue_Elem *)oadWriteEvt);
    
    // Post the application's semaphore.
    Semaphore_post(sem);
  }
  else
  {
    // Fail silently.
  }
}
#endif //FEATURE_OAD

/*********************************************************************
 * @fn      SimpleBLEPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_clockHandler(UArg arg)
{
  // Store the event.
  events |= arg;

  // Wake up the application.
  Semaphore_post(sem);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_enqueueMsg(uint8_t event, uint8_t state)
{
  sbpEvt_t *pMsg;

  // Create dynamic pointer to message.
  if ((pMsg = ICall_malloc(sizeof(sbpEvt_t))))
  {
    pMsg->hdr.event = event;
    pMsg->hdr.state = state;

    // Enqueue the message.
    Util_enqueueMsg(appMsgQueue, sem, (uint8*)pMsg);
  }
}


/*********************************************************************
*********************************************************************/
//cole function definitions

void SPI_read(uint32_t sensor, uint32_t cmd, uint32_t *value)
{
	uint32_t temp;
	GPIOPinWrite(sensor, 0);
	SSIDataPut(SSI0_BASE, cmd);
	while(SSIBusy(SSI0_BASE));
	GPIOPinWrite(sensor,1);
	SSIDataGet(SSI0_BASE, &temp);
	*value = temp&0x00ff;
	while(SSIBusy(SSI0_BASE));
}

void SPI_write(uint32_t sensor, uint32_t cmd, uint32_t *get)
{
	GPIOPinWrite(sensor, 0);
	SSIDataPut(SSI0_BASE, cmd);
	SSIDataGet(SSI0_BASE, get);
	while(SSIBusy(SSI0_BASE));
	GPIOPinWrite(sensor, 1);
}

void ADXL_SPI_init()
{
	IOCPinTypeSsiMaster(SSI0_BASE,MISO,MOSI,IOID_UNUSED,SCLK);
	IOCPinTypeGpioOutput(ADXL_CS);
	IOCPinTypeGpioOutput(FRAM_CS);
	GPIOPinWrite(ADXL,1);
	GPIOPinWrite(FRAM,1);
	SSIDisable(SSI0_BASE);
	SSIConfigSetExpClk(SSI0_BASE,SYSCLK_FREQ,SSI_FRF_MOTO_MODE_3,SSI_MODE_MASTER,SPICLK_FREQ,FRAME_LENGTH);
	SSIEnable(SSI0_BASE);
}


void WriteBosch( uint8_t BoschReg, uint8_t DataToWrite, uint8_t deviceAddress){
    I2CMasterSlaveAddrSet(I2C0_BASE, deviceAddress, false);
    I2CMasterDataPut(I2C0_BASE, BoschReg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C0_BASE));
    I2CMasterDataPut(I2C0_BASE, DataToWrite);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C0_BASE));
}
void ReadBosch( uint8_t BoschReg, uint8_t NumReg, uint8_t *data, uint8_t deviceAddress ){
    I2CMasterSlaveAddrSet(I2C0_BASE, deviceAddress, false);
    I2CMasterDataPut(I2C0_BASE, BoschReg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C0_BASE));

    I2CMasterSlaveAddrSet(I2C0_BASE, deviceAddress, true);
    if (NumReg ==1){
      I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
      while(I2CMasterBusy(I2C0_BASE));
    }
    else {
      I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
      while(I2CMasterBusy(I2C0_BASE));
      *data = I2CMasterDataGet(I2C0_BASE);
      data++;
    while(NumReg>2){
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while(I2CMasterBusy(I2C0_BASE));
        NumReg = NumReg-1;
      }
      I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    }
    while(I2CMasterBusy(I2C0_BASE));
    *data = I2CMasterDataGet(I2C0_BASE);
    data++;
}

void pinInterruptHandler(PIN_Handle handle, PIN_Id pinId){
	//Semaphore_post(cole);
	doUrgentWork();
}
void doUrgentWork(void)
{
	uint_t pinVal = PIN_getOutputValue(Board_UART_CTS);
	PIN_setOutputValue(pinHandle, Board_UART_CTS, !pinVal);

}
void readFRAMid(){
    // read device id
	uint32_t temp;
	GPIOPinWrite(FRAM, 0);
	SSIDataPut(SSI0_BASE, 0x9F00); // charValue1[4] = 0 first byte
	SSIDataPut(SSI0_BASE,0x0000);  // 6
	SSIDataPut(SSI0_BASE,0x0000);  // 8
	SSIDataPut(SSI0_BASE,0x0000);  // 10
	SSIDataPut(SSI0_BASE,0x0000);  // 12
	SSIDataGet(SSI0_BASE,&temp);
	charValue1[4] = (temp >> 8);
	charValue1[5] = (uint8_t) temp;
	SSIDataGet(SSI0_BASE,&temp);
	charValue1[6] = (temp >> 8);
	charValue1[7] = (uint8_t) temp;
	SSIDataGet(SSI0_BASE,&temp);
	charValue1[8] = (temp >> 8);
	charValue1[9] = (uint8_t) temp;
	SSIDataGet(SSI0_BASE,&temp);
	charValue1[10] = (temp >> 8);
	charValue1[11] = (uint8_t) temp;
	SSIDataGet(SSI0_BASE,&temp);
	charValue1[12] = (temp >> 8);
	charValue1[13] = (uint8_t) temp;
	while(SSIBusy(SSI0_BASE));
	GPIOPinWrite(FRAM, 1);
}


void moveToFRAM(){
	/* FRAM notes FM25V10 1Mbit
	 *
	 * 3 byte address required for any read or write operation
	 * 		address is 17 bytes so the first 7 are don't cares
	 * after the CS goes low the first byte received is treated as the opcode
	 * 	    the chip uses standard opcodes for memory accesses
	 * status register
	 * 		bit 0 dc
	 * 		bit 1 write enable (default 0, disabled)
	 * 		bit 2 and 3 block protection bits
	 * 		bit 456 dc
	 * 		bit 7 write protect enable
	 * writes begin with a WREN opcode (0000 0110b) and the cs chip being asserted and deasserted
	 * 		the next opcode is WRITE (0000 0010b) followed by a 3 byte address
	 * 		subsequent bytes are data bytes which are written sequentially
	 * 		writes continue as long as the bus master continues to issue clocks and cs stays low
	 * 		if 0x1ffff(131,071) is reached the counter rolls over to 0x00000
	 * 		**** keep an int counter going that adds 192 each time moveToFRAM is called
	 *
	 * 	Math time
	 * 		we have 131071 8 bit registers
	 * 		every 10ms we get 32 samples and each sample is 6 bytes
	 * 			so 192 bytes
	 * 			19200 bytes/1 second
	 * 			131071/19200 = 6.83 seconds of data before rolling over
	 *
	 * SPI Read
	 * 		Read opcode (0x02) followed by the register to start at followed by bits that
	 * 		will be ignored to keep the clock going
	 * 		deassert CS
	 *
	 *
	 *
	 *
	 */

	SSIConfigSetExpClk(SSI0_BASE,SYSCLK_FREQ,SSI_FRF_MOTO_MODE_3,SSI_MODE_MASTER,SPICLK_FREQ,FRAME_LENGTH);
	SSIEnable(SSI0_BASE);

	ADXL_read();
	enableWriteToFRAM();
	writeToFRAM();

}
void ADXL_read(){
	uint32_t temp;
	int i;
	for ( i = 0; i<96; i+=3){
		GPIOPinWrite(ADXL,0);
		SSIDataPut(SSI0_BASE, 0xF200);
		SSIDataGet(SSI0_BASE, &temp);
		uint16_t LSBx = (uint8_t) temp;
		SSIDataPut(SSI0_BASE, 0x0000);
		SSIDataGet(SSI0_BASE, &temp);
		uint16_t MSBx = (temp >> 8);
		ADXLbuffer[i] = (uint16_t) (LSBx  | (MSBx<<8));
		uint16_t LSBy = (uint8_t) temp;
		SSIDataPut(SSI0_BASE, 0x0000);
		SSIDataGet(SSI0_BASE, &temp);
		uint16_t MSBy =  (temp >> 8);
		ADXLbuffer[i+1] = (uint16_t) (LSBy | (MSBy<<8));
		uint16_t LSBz = (uint8_t) temp;
		SSIDataPut(SSI0_BASE, 0x0000);

		while(SSIBusy(SSI0_BASE));
		GPIOPinWrite(ADXL,1);

		SSIDataGet(SSI0_BASE, &temp);
		uint16_t MSBz =  (temp >> 8);
		ADXLbuffer[i+2] = (uint16_t) (LSBz | (MSBz<<8));

	}
}

void enableWriteToFRAM(){
	uint32_t temp;
	SPI_write(FRAM,0x0600,&temp);  //Write enable opcode

	SPI_write(FRAM,0x0142,&temp);  //Write Status register

	SPI_write(FRAM,0x0600,&temp);  //Write must be reenabled after status register write

	SPI_write(FRAM,0x0500,&temp);  //Read status register

}
void writeToFRAM(){
	uint32_t temp;
	uint16_t ADXLbufferRead[105];

	//int i;
	//int j = 1;
	//this works and writes BBCCDDEE
	/*GPIOPinWrite(FRAM, 0);
	SSIDataPut(SSI0_BASE,0x0200); //WRITE OPCODE
	SSIDataPut(SSI0_BASE,0x0000);
	SSIDataPut(SSI0_BASE,ADXLbuffer[0]); // from here on out is your data
	SSIDataPut(SSI0_BASE,ADXLbuffer[1]);
	//SSIDataPut(SSI0_BASE,0xBBBB); // from here on out is your data
	//SSIDataPut(SSI0_BASE,0xCCCC);
	while(SSIBusy(SSI0_BASE));
	GPIOPinWrite(FRAM, 1);
	SSIDataGet(SSI0_BASE,&temp);
	SSIDataGet(SSI0_BASE,&temp);
	SSIDataGet(SSI0_BASE,&temp);
	SSIDataGet(SSI0_BASE,&temp);*/
	if (registerCount < 618 ){
		GPIOPinWrite(FRAM, 0);
		/*if(registerCount < 342){
			SSIDataPut(SSI0_BASE,0x0200);//WRITE OPCODE
		}
		else{
			SSIDataPut(SSI0_BASE,0x0201);
		}
		SSIDataPut(SSI0_BASE,FRAMregisterAcc);
		SSIDataGet(SSI0_BASE,&temp);
		SSIDataGet(SSI0_BASE,&temp);
		for (i=0; i<96; i+=3){
			//SSIDataPut(SSI0_BASE,ADXLbuffer[i]); // from here on out is your data
			//SSIDataPut(SSI0_BASE,ADXLbuffer[i+1]);
			//SSIDataPut(SSI0_BASE,ADXLbuffer[i+2]);
			SSIDataPut(SSI0_BASE,i);
			SSIDataPut(SSI0_BASE,i+1);
			SSIDataPut(SSI0_BASE,i+2);
			SSIDataGet(SSI0_BASE,&temp);
			SSIDataGet(SSI0_BASE,&temp);
			SSIDataGet(SSI0_BASE,&temp);
		}
		GPIOPinWrite(FRAM,1);
		GPIOPinWrite(FRAM,0);*/
		int juan = 0;
		if (juan){
			SSIDataPut(SSI0_BASE,0x0201);
			SSIDataPut(SSI0_BASE,0xD00B+FRAMregisterGyro);
			SSIDataGet(SSI0_BASE,&temp);
			SSIDataGet(SSI0_BASE,&temp);
			ADXLbuffer[96] = (uint16_t) (((uint16_t)charValue2[0]) << 8) | (uint16_t) charValue2[1];
			//SSIDataPut(SSI0_BASE,ADXLbuffer[96]);
			SSIDataPut(SSI0_BASE,1);
			SSIDataGet(SSI0_BASE,&temp);
			ADXLbuffer[97] = (uint16_t) (((uint16_t)charValue2[2]) << 8) | (uint16_t) charValue2[3];
			//SSIDataPut(SSI0_BASE,ADXLbuffer[97]);
			SSIDataPut(SSI0_BASE,2);
			SSIDataGet(SSI0_BASE,&temp);
			ADXLbuffer[98] = (uint16_t) (((uint16_t)charValue2[4]) << 8) | (uint16_t) charValue2[5];
			//SSIDataPut(SSI0_BASE,ADXLbuffer[98]);
			SSIDataPut(SSI0_BASE,3);
			SSIDataGet(SSI0_BASE,&temp);
			ADXLbuffer[99] = (uint16_t) (((uint16_t)charValue2[6]) << 8) | (uint16_t) charValue2[7];
			//SSIDataPut(SSI0_BASE,ADXLbuffer[99]);
			SSIDataPut(SSI0_BASE,4);
			SSIDataGet(SSI0_BASE,&temp);
			ADXLbuffer[100] = (uint16_t) (((uint16_t)charValue2[8]) << 8) | (uint16_t) charValue2[9];
			//SSIDataPut(SSI0_BASE,ADXLbuffer[100]);
			SSIDataPut(SSI0_BASE,5);
			SSIDataGet(SSI0_BASE,&temp);
			ADXLbuffer[101] = (uint16_t) (((uint16_t)charValue2[10]) << 8) | (uint16_t) charValue2[11];
			//SSIDataPut(SSI0_BASE,ADXLbuffer[101]);
			SSIDataPut(SSI0_BASE,6);
			SSIDataGet(SSI0_BASE,&temp);
			ADXLbuffer[102] = (uint16_t) (((uint16_t)charValue2[12]) << 8) | (uint16_t) charValue2[13];
			//SSIDataPut(SSI0_BASE,ADXLbuffer[102]);
			SSIDataPut(SSI0_BASE,7);
			SSIDataGet(SSI0_BASE,&temp);
			ADXLbuffer[103] = (uint16_t) (((uint16_t)charValue2[14]) << 8) | (uint16_t) charValue2[15];
			//SSIDataPut(SSI0_BASE,ADXLbuffer[103]);
			SSIDataPut(SSI0_BASE,8);
			SSIDataGet(SSI0_BASE,&temp);
			ADXLbuffer[104] = (uint16_t) (((uint16_t)charValue2[16]) << 8) | (uint16_t) charValue2[17];
			//SSIDataPut(SSI0_BASE,ADXLbuffer[104]);
			SSIDataPut(SSI0_BASE,9);
			SSIDataGet(SSI0_BASE,&temp);
		}
		else{
			//uint16_t vtemp;
			SSIDataPut(SSI0_BASE,0x0201);
			SSIDataPut(SSI0_BASE,0xD00B+FRAMregisterGyro);
			SSIDataGet(SSI0_BASE,&temp);
			SSIDataGet(SSI0_BASE,&temp);
			//uint16_t value = 0;

			//use temporary variable instead of ADXL buffer
			ADXLbuffer[96] = (uint16_t) (((uint16_t)charValue2[0]) << 8) | (uint16_t) charValue2[1];

			//ADXLbuffer[96] = 27;
			//value = ADXLbuffer[96];
			SSIDataPut(SSI0_BASE,ADXLbuffer[96]);
			//SSIDataPut(SSI0_BASE,1);
			SSIDataGet(SSI0_BASE,&temp);
			ADXLbuffer[97] = (uint16_t) (((uint16_t)charValue2[2]) << 8) | (uint16_t) charValue2[3];
			//ADXLbuffer[97] = 18;
			SSIDataPut(SSI0_BASE,ADXLbuffer[97]);

			//SSIDataPut(SSI0_BASE,2);
			SSIDataGet(SSI0_BASE,&temp);
			ADXLbuffer[98] = (uint16_t) (((uint16_t)charValue2[4]) << 8) | (uint16_t) charValue2[5];
			//ADXLbuffer[98] = 112;
			SSIDataPut(SSI0_BASE,ADXLbuffer[98]);
			//SSIDataPut(SSI0_BASE,3);
			SSIDataGet(SSI0_BASE,&temp);
			ADXLbuffer[99] = (uint16_t) (((uint16_t)charValue2[6]) << 8) | (uint16_t) charValue2[7];
			SSIDataPut(SSI0_BASE,ADXLbuffer[99]);
			//SSIDataPut(SSI0_BASE,4);
			SSIDataGet(SSI0_BASE,&temp);
			ADXLbuffer[100] = (uint16_t) (((uint16_t)charValue2[8]) << 8) | (uint16_t) charValue2[9];
			SSIDataPut(SSI0_BASE,ADXLbuffer[100]);
			//SSIDataPut(SSI0_BASE,5);
			SSIDataGet(SSI0_BASE,&temp);
			ADXLbuffer[101] = (uint16_t) (((uint16_t)charValue2[10]) << 8) | (uint16_t) charValue2[11];
			SSIDataPut(SSI0_BASE,ADXLbuffer[101]);
			//SSIDataPut(SSI0_BASE,6);
			SSIDataGet(SSI0_BASE,&temp);
			ADXLbuffer[102] = (uint16_t) (((uint16_t)charValue2[12]) << 8) | (uint16_t) charValue2[13];
			SSIDataPut(SSI0_BASE,ADXLbuffer[102]);
			//SSIDataPut(SSI0_BASE,7);
			SSIDataGet(SSI0_BASE,&temp);
			ADXLbuffer[103] = (uint16_t) (((uint16_t)charValue2[14]) << 8) | (uint16_t) charValue2[15];
			SSIDataPut(SSI0_BASE,ADXLbuffer[103]);
			//SSIDataPut(SSI0_BASE,8);
			SSIDataGet(SSI0_BASE,&temp);
			ADXLbuffer[104] = (uint16_t) (((uint16_t)charValue2[16]) << 8) | (uint16_t) charValue2[17];
			SSIDataPut(SSI0_BASE,ADXLbuffer[104]);
			//SSIDataPut(SSI0_BASE,9);
			SSIDataGet(SSI0_BASE,&temp);
		}
		while(SSIBusy(SSI0_BASE));
		GPIOPinWrite(FRAM, 1);
		/*GPIOPinWrite(FRAM, 0);
		//if(registerCount < 10922){
		//	SSIDataPut(SSI0_BASE,0x0300);
	//	}
	//	else{
			SSIDataPut(SSI0_BASE,0x0301);
			SSIDataGet(SSI0_BASE,&temp);
		//}
		SSIDataPut(SSI0_BASE,FRAMregisterRead);
		SSIDataGet(SSI0_BASE,&temp);
		SSIDataPut(SSI0_BASE,0x0000);
		SSIDataPut(SSI0_BASE,0x0000);
		SSIDataPut(SSI0_BASE,0x0000);

		while(SSIBusy(SSI0_BASE));
		GPIOPinWrite(FRAM,1);
		SSIDataGet(SSI0_BASE,&temp);
		ADXLbufferRead[0] =  temp;
		//charValue1[12] = (uint8_t) temp;
		//charValue1[13] = (temp >> 8);
		SSIDataGet(SSI0_BASE,&temp);
		ADXLbufferRead[1] =  temp;
		//charValue1[14] = (uint8_t) temp;
		//charValue1[15] = (temp >> 8);
		SSIDataGet(SSI0_BASE,&temp);
		ADXLbufferRead[2] =  temp;
		//charValue1[16] = (uint8_t) temp;
		//charValue1[17] = (temp >> 8);
		FRAMregisterAcc += 192;
		FRAMregisterGyro += 18;
		FRAMregisterRead += 18;
		if (ADXLbufferRead[0] == ADXLbuffer[96] ){
			if(ADXLbufferRead[1] == ADXLbuffer[97]){
				PIN_setOutputValue(pinHandle, Board_UART_CTS, 0);
			}

		}
		else {
			PIN_setOutputValue(pinHandle,IOID_1,0);
		}*/
		registerCount++;
	}
	else{
		FRAMfull = 1;
		registerCount = 0;
		FRAMregisterAcc = 0;
		FRAMregisterGyro = 0;
		FRAMregisterRead = 0xD00B;
		PIN_setOutputValue(pinHandle, Board_UART_CTS, 0);
		PIN_setOutputValue(pinHandle,IOID_1,1);
	}

}
void readFromFRAM(){
	SSIConfigSetExpClk(SSI0_BASE,SYSCLK_FREQ,SSI_FRF_MOTO_MODE_3,SSI_MODE_MASTER,SPICLK_FREQ,FRAME_LENGTH);
	SSIEnable(SSI0_BASE);
	uint32_t temp;
	//if (registerCount < 21845){
	if(registerCount <12276){
		GPIOPinWrite(FRAM, 0);
		//if(registerCount < 10922){
		//	SSIDataPut(SSI0_BASE,0x0300);
	//	}
	//	else{
			SSIDataPut(SSI0_BASE,0x0301);
			SSIDataGet(SSI0_BASE,&temp);
		//}
		SSIDataPut(SSI0_BASE,FRAMregisterRead);
		SSIDataGet(SSI0_BASE,&temp);
		SSIDataPut(SSI0_BASE,0x0000);
		SSIDataPut(SSI0_BASE,0x0000);
		SSIDataPut(SSI0_BASE,0x0000);

		while(SSIBusy(SSI0_BASE));
		GPIOPinWrite(FRAM,1);
		SSIDataGet(SSI0_BASE,&temp);
		charValue1[12] = (uint8_t) temp;
		charValue1[13] = (temp >> 8);
		SSIDataGet(SSI0_BASE,&temp);
		charValue1[14] = (uint8_t) temp;
		charValue1[15] = (temp >> 8);
		SSIDataGet(SSI0_BASE,&temp);
		charValue1[16] = (uint8_t) temp;
		charValue1[17] = (temp >> 8);
		charValue1[11] = counter;
		counter++;
		if (counter == 32){
			counter = 0;
		}
		FRAMregisterRead += 6;
		registerCount+=6;
		SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR5_LEN,
								  charValue1);
	}
	else {
		//Power_releaseDependency(PERIPH_SSI0);
		//Power_releaseDependency(PERIPH_I2C0);
		//Power_releaseConstraint(Power_SB_DISALLOW);
		//Power_releaseConstraint(Power_IDLE_PD_DISALLOW);
	}
}

void readAllGyro(){

	//IOCPinTypeI2c	( I2C0_BASE, IOID_27, IOID_28); //sets pin 27 as data and 28 as clock (11,8 for anterior board)
	//I2CMasterInitExpClk ( I2C0_BASE, 48000000, false ); //initialize i2c clock speed, true = 100kbps
	I2CMasterEnable(I2C0_BASE);
	//I2CMasterInitExpClk ( I2C0_BASE, 48000000, false );
	//IOCPinTypeI2c	( I2C0_BASE, IOID_27, IOID_28);
	//WriteBosch(0x3E, 0x00, 0x29); //Normal Power mode
	//WriteBosch(0x07, 0x01, 0x29); // PAGE ID all config ones are on page 1
	//WriteBosch(0x0A, 0x00, 0x29); // fastest odr
	//WriteBosch(0x07, 0x00, 0x29); //Change back to page 0
	//WriteBosch(0x3D, 0x03, 0x29); // change from config mode to gyro only mode


	I2CMasterSlaveAddrSet(I2C0_BASE, 0x29, false);
	I2CMasterDataPut(I2C0_BASE, 0x14);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C0_BASE));

    I2CMasterSlaveAddrSet(I2C0_BASE, 0x29, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C0_BASE));
    charValue2[1] = I2CMasterDataGet(I2C0_BASE);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(I2CMasterBusy(I2C0_BASE));
    charValue2[0] = I2CMasterDataGet(I2C0_BASE);
    //ADXLbuffer[96] = (uint16_t) (((uint16_t)charValue2[0]) << 8) | (uint16_t) charValue2[1];
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(I2CMasterBusy(I2C0_BASE));
    charValue2[3] = I2CMasterDataGet(I2C0_BASE);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(I2CMasterBusy(I2C0_BASE));
    charValue2[2] = I2CMasterDataGet(I2C0_BASE);
    //ADXLbuffer[97] = (uint16_t) (((uint16_t)charValue2[2]) << 8) | (uint16_t) charValue2[3];
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(I2CMasterBusy(I2C0_BASE));
    charValue2[5] = I2CMasterDataGet(I2C0_BASE);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(I2CMasterBusy(I2C0_BASE));
    charValue2[4] = I2CMasterDataGet(I2C0_BASE);
    //ADXLbuffer[98] = (uint16_t) (((uint16_t)charValue2[4]) << 8) | (uint16_t) charValue2[5];

}
void readAllGyro2(){
	//IOCPinTypeI2c	( I2C0_BASE, IOID_27, IOID_28); //sets pin 27 as data and 28 as clock (11,8 for anterior board)
	//I2CMasterInitExpClk ( I2C0_BASE, 48000000, false ); //initialize i2c clock speed, true = 100kbps
	//I2CMasterEnable(I2C0_BASE);
	//WriteBosch(0x3E, 0x00, 0x29); //Normal Power mode
	//WriteBosch(0x07, 0x01, 0x29); // PAGE ID all config ones are on page 1
	//WriteBosch(0x0A, 0x00, 0x29); // fastest odr
	//WriteBosch(0x07, 0x00, 0x29); //Change back to page 0
	//WriteBosch(0x3D, 0x03, 0x29); // change from config mode to gyro only mode
	I2CMasterEnable(I2C0_BASE);
	//IOCPinTypeI2c	( I2C0_BASE, IOID_27, IOID_28);
	//I2CMasterInitExpClk ( I2C0_BASE, 48000000, false );
	I2CMasterSlaveAddrSet(I2C0_BASE, 0x29, false);
	I2CMasterDataPut(I2C0_BASE, 0x14);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C0_BASE));

    I2CMasterSlaveAddrSet(I2C0_BASE, 0x29, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C0_BASE));
    charValue2[7] = I2CMasterDataGet(I2C0_BASE);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(I2CMasterBusy(I2C0_BASE));
    charValue2[6] = I2CMasterDataGet(I2C0_BASE);
    //ADXLbuffer[99] = (uint16_t) (((uint16_t)charValue2[6]) << 8) | (uint16_t) charValue2[7];
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(I2CMasterBusy(I2C0_BASE));
    charValue2[9] = I2CMasterDataGet(I2C0_BASE);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(I2CMasterBusy(I2C0_BASE));
    charValue2[8] = I2CMasterDataGet(I2C0_BASE);
    //ADXLbuffer[100] = (uint16_t) (((uint16_t)charValue2[8]) << 8) | (uint16_t) charValue2[9];
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(I2CMasterBusy(I2C0_BASE));
    charValue2[11] = I2CMasterDataGet(I2C0_BASE);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(I2CMasterBusy(I2C0_BASE));
    charValue2[10] = I2CMasterDataGet(I2C0_BASE);
    //ADXLbuffer[101] = (uint16_t) (((uint16_t)charValue2[10]) << 8) | (uint16_t) charValue2[11];
}
void readAllGyro3(){
	/*IOCPinTypeI2c	( I2C0_BASE, IOID_27, IOID_28); //sets pin 27 as data and 28 as clock (11,8 for anterior board)
	I2CMasterInitExpClk ( I2C0_BASE, 48000000, false ); //initialize i2c clock speed, true = 100kbps
	I2CMasterEnable(I2C0_BASE);
	WriteBosch(0x3E, 0x00, 0x29); //Normal Power mode
	WriteBosch(0x07, 0x01, 0x29); // PAGE ID all config ones are on page 1
	WriteBosch(0x0A, 0x00, 0x29); // fastest odr
	WriteBosch(0x07, 0x00, 0x29); //Change back to page 0
	WriteBosch(0x3D, 0x03, 0x29); // change from config mode to gyro only mode
*/
	I2CMasterEnable(I2C0_BASE);
	//I2CMasterInitExpClk ( I2C0_BASE, 48000000, false );
	//IOCPinTypeI2c	( I2C0_BASE, IOID_27, IOID_28);
	I2CMasterSlaveAddrSet(I2C0_BASE, 0x29, false);
	I2CMasterDataPut(I2C0_BASE, 0x14);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C0_BASE));

    I2CMasterSlaveAddrSet(I2C0_BASE, 0x29, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C0_BASE));
    charValue2[13] = I2CMasterDataGet(I2C0_BASE);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(I2CMasterBusy(I2C0_BASE));
    charValue2[12] = I2CMasterDataGet(I2C0_BASE);
    //DXLbuffer[102] = (uint16_t) (((uint16_t)charValue2[12]) << 8) | (uint16_t) charValue2[13];
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(I2CMasterBusy(I2C0_BASE));
    charValue2[15] = I2CMasterDataGet(I2C0_BASE);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(I2CMasterBusy(I2C0_BASE));
    charValue2[14] = I2CMasterDataGet(I2C0_BASE);
    //ADXLbuffer[103] = (uint16_t) (((uint16_t)charValue2[14]) << 8) | (uint16_t) charValue2[15];
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
    while(I2CMasterBusy(I2C0_BASE));
    charValue2[17] = I2CMasterDataGet(I2C0_BASE);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(I2CMasterBusy(I2C0_BASE));
    charValue2[16] = I2CMasterDataGet(I2C0_BASE);
    uint8_t powerState = Power_getConstraintInfo();
    uint8_t counter;

    //SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2,1, &counter);
    //counter++;
    //ADXLbuffer[104] = (uint16_t) (((uint16_t)charValue2[16]) << 8) | (uint16_t) charValue2[17];
	//SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, SIMPLEPROFILE_CHAR5_LEN,
//							  charValue2);

}










