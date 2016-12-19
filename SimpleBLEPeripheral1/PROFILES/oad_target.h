/**************************************************************************************************
  Filename:       oad_target.h
  Revised:        $Date: 2015-07-20 11:31:07 -0700 (Mon, 20 Jul 2015) $
  Revision:       $Revision: 44370 $

  Description:    This file contains OAD Target header file.  These are common
                  API prototypes, constants and macros used by the OAD profile 
                  and underlying implementations.

  Copyright 2012 - 2015 Texas Instruments Incorporated. All rights reserved.

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
**************************************************************************************************/
#ifndef OAD_TARGET_H
#define OAD_TARGET_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#ifndef BOOT_LOADER
#include <ti/sysbios/knl/Queue.h>
#endif //BOOT_LOADER
  
/*********************************************************************
 * CONSTANTS
 */
  
#define OAD_SERVICE_UUID       0xFFC0
#define OAD_IMG_IDENTIFY_UUID  0xFFC1
#define OAD_IMG_BLOCK_UUID     0xFFC2
#define OAD_IMG_COUNT_UUID     0xFFC3

#define OAD_RESET_SERVICE_UUID 0xFFD0
#define OAD_RESET_CHAR_UUID    0xFFD1
   
// Offsets in bytes.
#define OAD_IMG_CRC_OSET       0x0000

// Image header is placed 4 bytes above OAD_IMG_CRC_OSET.  the CRC and CRC
// shadow are NOT part of the image header.
#define OAD_IMG_HDR_OSET       0x0004

#define OAD_CHAR_CNT           3

// OAD Characteristic Indices
#define OAD_CHAR_IMG_IDENTIFY  0
#define OAD_CHAR_IMG_BLOCK     1
#define OAD_CHAR_IMG_COUNT     2
   
// Image Identification size
#define OAD_IMG_ID_SIZE        4

// Image header size (version + length + image id size)
#define OAD_IMG_HDR_SIZE       ( 2 + 2 + OAD_IMG_ID_SIZE )

// The Image is transported in 16-byte blocks in order to avoid using blob operations.
#define OAD_BLOCK_SIZE         16
#define OAD_BLOCKS_PER_PAGE    (HAL_FLASH_PAGE_SIZE / OAD_BLOCK_SIZE)
#define OAD_BLOCK_MAX          (OAD_BLOCKS_PER_PAGE * OAD_IMG_D_AREA)

//Callback Events
#define OAD_WRITE_IDENTIFY_REQ 0x01
#define OAD_WRITE_BLOCK_REQ    0x02
#define OAD_IMAGE_COMPLETE     0x03

// Default Image A Page
#if !defined OAD_IMG_A_PAGE
// By default Image A starts on page 0 although it does not used all of it.
#define OAD_IMG_A_PAGE         0
#endif // OAD_IMG_A_PAGE
 
// Starting offset of Image A into its first page.
#if !defined OAD_IMG_A_OSET
#define OAD_IMG_A_OSET         0x0600
#endif // OAD_IMG_A_OSET
  
// Default image A size
#if !defined OAD_IMG_A_AREA
// By default this mean 6 contiguous pages are used, excluding page 31.
#define OAD_IMG_A_AREA         6
#endif // OAD_IMG_A_AREA

#if !defined OAD_IMG_B_PAGE
// Image-A/B can be very differently sized areas when implementing BIM vice OAD boot loader.
#define OAD_IMG_B_PAGE         6
#endif // OAD_IMG_B_PAGE

// Default image B size  
#if !defined OAD_IMG_B_AREA
#define OAD_IMG_B_AREA         9
#endif // OAD_IMG_B_PAGE

// Starting offset of Image B into its first page.
#if !defined OAD_IMG_B_OSET
#define OAD_IMG_B_OSET         0x0000
#endif // OAD_IMG_B_OSET
  
// Default image E size
#if !defined OAD_IMG_E_AREA
#define OAD_IMG_E_AREA         0x1C
#endif // OAD_IMG_E_AREA
  
// Starting offset of Image E
#if !defined OAD_IMG_E_PAGE
#define OAD_IMG_E_PAGE         1
#endif // OAD_IMG_E_PAGE
  
#if !defined OAD_IMG_E_OSET
#define OAD_IMG_E_OSET         0x0000
#endif // OAD_IMG_E_OSET
  
#if defined HAL_IMAGE_B
#define OAD_IMG_D_PAGE         OAD_IMG_A_PAGE
#define OAD_IMG_D_AREA         OAD_IMG_A_AREA
#define OAD_IMG_R_PAGE         OAD_IMG_B_PAGE
#define OAD_IMG_R_AREA         OAD_IMG_B_AREA
#define OAD_IMG_R_OSET         OAD_IMG_B_OSET
#elif defined HAL_IMAGE_E   // using external flash
#define OAD_IMG_D_PAGE         OAD_IMG_E_PAGE
#define OAD_IMG_D_AREA         OAD_IMG_E_AREA
#define OAD_IMG_R_PAGE         OAD_IMG_E_PAGE
#define OAD_IMG_R_AREA         OAD_IMG_E_AREA
#define OAD_IMG_R_OSET         OAD_IMG_E_OSET
#else // HAL_IMAGE_A, a non-upgradeable Image-A
#define OAD_IMG_D_PAGE         OAD_IMG_B_PAGE
#define OAD_IMG_D_AREA         OAD_IMG_B_AREA
#define OAD_IMG_R_PAGE         OAD_IMG_A_PAGE
#define OAD_IMG_R_AREA         OAD_IMG_A_AREA
#define OAD_IMG_R_OSET         OAD_IMG_A_OSET  
#endif // HAL_IMAGE_B  
  
#define OAD_IMG_NP_FLAG    0x01
#define OAD_IMG_APP_FLAG   0x02
#define OAD_IMG_STACK_FLAG 0x04

/*********************************************************************
 * MACROS
 */

/*
 * OAD_IMG_VER() sets the least significant bit of the build version to signify
 * which image region the image is built to run in.  If the LSB is set to 1, 
 * then the Image B region is intended.  Otherwise, it is built for the image A
 * region.  When that builds Image Header is sent OTA to the currently running 
 * Image, OAD_IMG_ID() is performed as a simple check to make sure that the 
 * current Image running and  the Image to OTD are not both and Image A or 
 * Image B builds.  OAD_VER_NUM() unmodifies the OAD image version number for
 * uses such as writing to the LCD.
 */
      
// Macros to get Image ID (LSB) and Version Number
#define OAD_IMG_ID( ver )    ( (ver) & 0x01 )
#define OAD_VER_NUM( ver )   ( (ver) >> 0x01 )

// Macro to set Image Version
#if defined (HAL_IMAGE_A)
  // Clear LSB for Image A
  #define OAD_IMG_VER( ver ) ( (uint16)( (ver) << 0x01 ) )
#elif defined (HAL_IMAGE_B)
  // Set LSB for Image B
  #define OAD_IMG_VER( ver ) ( (uint16)( ( (ver) << 0x01 ) | 0x01 ) )
#else 
  // Do nothing.
  #define OAD_IMG_VER( ver ) (ver)
#endif // HAL_IMAGE_A

// Number of HAL Flash Words per Flash page
#define OAD_FLASH_PAGE_MULT  ((uint16)(HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE))  
  
#define FLASH_ADDRESS(page, offset) (((page) << 12) + (offset))
  
/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * TYPEDEFS
 */

/* CRC is not included in the header; instead, IAR's 2 byte CRC is placed 4 
 * bytes before the header at the very start of the image. In the 2 bytes 
 * between IAR's CRC and the image header resides the shadow CRC.
 */ 
typedef struct {
  uint16 ver;
  uint16 len;    // Image length in 4-byte blocks (i.e. HAL_FLASH_WORD_SIZE blocks).
  uint8  uid[4]; // User-defined Image Identification bytes.
  uint8  res[4]; // Reserved space for future use.
} img_hdr_t; 

#ifndef BOOT_LOADER
typedef struct
{
  Queue_Elem _elem;
  uint8_t  event;
  uint16_t connHandle;
  uint8_t  *pData;
} oadTargetWrite_t;
#endif //BOOT_LOADER

 /*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      OADTarget_open
 *
 * @brief   Open an OAD target for download. 
 *
 * @param   None.
 *
 * @return  TRUE if OAD target successfully opened
 */
extern uint8_t OADTarget_open(void);


/*********************************************************************
 * @fn      OADTarget_close
 *
 * @brief   Close an OAD target after a download has finished
 *
 * @param   None.
 *
 * @return  None.
 */
extern void OADTarget_close(void);

/*********************************************************************
 * @fn      OADTarget_hasExternalFlash
 *
 * @brief   Check if the target has external flash
 *
 * @param   None.
 *
 * @return  None.
 */
extern bool OADTarget_hasExternalFlash(void);

/*********************************************************************
 * @fn      OADTarget_storeImageHeader
 *
 * @brief   Store the image header of the new image
 *
 * @param   pValue - pointer to the new image header
 *
 * @return  None.
 */
extern void OADTarget_storeImageHeader(uint8_t *pValue);

/*********************************************************************
 * @fn      OADTarget_getCurrentImageHeader
 *
 * @brief   Get the current image's header.
 *
 * @param   pValue - pointer to the new image header
 *
 * @return  None.
 */
extern void OADTarget_getCurrentImageHeader(img_hdr_t *pHdr);

/*********************************************************************
 * @fn      OADTarget_getCrc
 *
 * @brief   Get the CRC array from the image that is being downloaded
 *
 * @param   pCrc - pointer to the new image header
 *
 * @return  None.
 */
extern void OADTarget_getCrc(uint16_t *pCrc);

/*********************************************************************
 * @fn      OADTarget_setCrc
 *
 * @brief   Set the CRC shadow of the downloaded image.
 *
 * @param   pCrc - pointer to the new image header
 *
 * @return  None.
 */
extern void OADTarget_setCrc(uint16_t *pCrc);

/*********************************************************************
 * @fn      OADTarget_imageAddress
 *
 * @brief   Get the address of the current image
 *
 * @param   pValue - pointer to the new image header
 *
 * @param   pHdr - pointer to the current image header
 *
 * @return  address
 */
extern uint32_t OADTarget_imageAddress(uint8_t *pValue);


/*********************************************************************
 * @fn      OADTarget_validateNewImage
 *
 * @brief   Determine if a new image should be downloaded or not based on
 *          target specific criteria.
 *
 * @param   pValue - pointer to new Image header information
 * @param   ImgHdr - pointer to contents of current image header
 * @param   blkTot - total number of blocks comprising new image.
 *
 * @return  TRUE to begin OAD otherwise FALSE to reject the image.
 */
extern uint8_t OADTarget_validateNewImage(uint8_t *pValue, img_hdr_t *ImgHdr, 
                                          uint16_t blkTot);

/*********************************************************************
 * @fn      OADTarget_readFlash
 *
 * @brief   Read data from flash.
 *
 * @param   page   - page to read from in flash
 * @param   offset - offset into flash page to begin reading
 * @param   pBuf   - pointer to buffer into which data is read.
 * @param   len    - length of data to read in bytes.
 *
 * @return  None.
 */
extern void OADTarget_readFlash(uint8_t page, uint32_t offset, uint8_t *pBuf, uint16_t len);

/*********************************************************************
 * @fn      OADTarget_writeFlash
 *
 * @brief   Write data to flash.
 *
 * @param   page   - page to write to in flash
 * @param   offset - offset into flash page to begin writing
 * @param   pBuf   - pointer to buffer of data to write
 * @param   len    - length of data to write in bytes
 *
 * @return  None.
 */
extern void OADTarget_writeFlash(uint8_t page, uint32_t offset, uint8_t *pBuf, uint16_t len);

/*********************************************************************
 * @fn      OADTarget_eraseFlash
 *
 * @brief   Erase selected flash page.
 *
 * @param   page - the page to erase.
 *
 * @return  None.
 */
extern void OADTarget_eraseFlash(uint8_t page);

/*********************************************************************
 * @fn      OADTarget_systemReset
 *
 * @brief   Prepare system for a reset and trigger a reset to the boot
 *          image manager.
 *
 * @param   None.
 *
 * @return  None.
 */
extern void OADTarget_systemReset(void);

/*******************************************************************************
 * @fn      saveImageInfo
 *
 * @brief   Save image information in the meta-data area
 *
 * @return  None.
 */
extern void saveImageInfo(void);

/*******************************************************************************
 * @fn      getImageFlag
 *
 * @brief   Get the image type flag.
 *
 * @param   None.
 *
 * @return  Image type or 0 if unknown.
 */
extern uint8_t getImageFlag(void);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OAD_TARGET_H */
