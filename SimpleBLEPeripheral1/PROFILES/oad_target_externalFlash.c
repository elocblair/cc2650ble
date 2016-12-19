/**************************************************************************************************
  Filename:       oad_target_externalFlash.c
  Revised:        $Date: 2015-06-25 09:06:27 -0700 (Thu, 25 Jun 2015) $
  Revision:       $Revision: 44240 $

  Description:    This file contains the external flash target implementation
                  of the OAD profile.

  Copyright 2014 - 2015 Texas Instruments Incorporated. All rights reserved.

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

/*********************************************************************
 * INCLUDES
 */
#ifdef FEATURE_OAD
#include <string.h>
#include "hal_board.h"
#include "oad_target.h"
#include "ext_flash.h"
#include "ext_flash_layout.h"
#include <driverlib/vims.h>

#ifdef POWER_SAVING
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>
#endif  // POWER_SAVING

/*********************************************************************
 * Constants and macros
 */
#define PROG_BUF_SIZE             16
#define PAGE_0                    0
#define PAGE_1                    1
#define PAGE_31                   31

#define MAX_BLOCKS                (EFL_SIZE_IMAGE_APP / OAD_BLOCK_SIZE)

#ifndef APP_IMAGE_START
#define APP_IMAGE_START           1
#endif // APP_IMAGE_START
   
/*********************************************************************
 * PRIVATE VARIABLES
 */
static bool isOpen = false;
static ExtImageInfo_t imgInfo;

/*********************************************************************
 * PRIVATE FUNCTIONS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*********************************************************************
 * @fn      OADTarget_open
 *
 * @brief   Open an OAD target for download. 
 *
 * @param   none
 *
 * @return  TRUE if OAD target successfully opened
 */
uint8_t OADTarget_open(void)
{
  isOpen = extFlashOpen();
  
  return isOpen;
}


/*********************************************************************
 * @fn      OADTarget_close
 *
 * @brief   Close an OAD target after a download has finished
 *
 * @param   none
 *
 * @return  none
 */
void OADTarget_close(void)
{
  if (isOpen)
  {
    isOpen = false;
    extFlashClose();
  }
}

/*********************************************************************
 * @fn      OADTarget_hasExternalFlash
 *
 * @brief   Check if the target has external flash
 *
 * @param   none
 *
 * @return  none
 */
bool OADTarget_hasExternalFlash(void)
{
  return true;
}

/*********************************************************************
 * @fn      OADTarget_getCurrentImageHeader
 *
 * @brief   Get the header of the running image.
 *
 * @param   pValue - pointer to store running image header.
 *
 * @return  none
 */
void OADTarget_getCurrentImageHeader(img_hdr_t *pHdr)
{ 
  if (extFlashOpen())
  {
    //N.B: it is assumed that OADTarget_storeImageHeader() has been called prior
    // to this function.
    uint32_t metaDataAddr;
    ExtImageInfo_t tempHdr;
  
    if (imgInfo.imgType == EFL_OAD_IMG_TYPE_STACK || imgInfo.imgType == EFL_OAD_IMG_TYPE_NP)
    {
      metaDataAddr = EFL_IMAGE_INFO_ADDR_BLE;
    }
    else // Assume imgInfo.imgType == EFL_OAD_IMG_TYPE_APP
    {
      metaDataAddr = EFL_IMAGE_INFO_ADDR_APP;
    }
    
    extFlashRead(metaDataAddr, sizeof(ExtImageInfo_t), (uint8_t*)&tempHdr);
    extFlashClose();

    pHdr->len = tempHdr.len;
    pHdr->ver = tempHdr.ver == 0xFF ? 0x00 : tempHdr.ver; // In case metadata does not exist.
    memcpy(pHdr->uid,tempHdr.uid,sizeof(tempHdr.uid));
    pHdr->res[0] = HI_UINT16(tempHdr.addr);
    pHdr->res[1] = LO_UINT16(tempHdr.addr);
    pHdr->res[2] = tempHdr.imgType;
    pHdr->res[3] = 0xFF;   
  }
}

/*********************************************************************
 * @fn      OADTarget_validateNewImage
 *
 * @brief   Determine if a new image should be downloaded or not based on
 *          target specific criteria.
 *
 * @param   pValue - pointer to new Image header information
 * @param   pCur - pointer to contents of current image header
 * @param   blkTot - total number of blocks comprising new image.
 *
 * @return  TRUE to begin OAD otherwise FALSE to reject the image.
 */
uint8_t OADTarget_validateNewImage(uint8_t *pValue, img_hdr_t *pCur, 
                                   uint16_t blkTot)
{
  img_hdr_t *pNew;
  bool valid;
  
  pNew = (img_hdr_t *)pValue;
  valid = false;
 
  // Check if number of blocks make sense
  if ((blkTot > MAX_BLOCKS) || (blkTot == 0))
  {
    return false;
  }
  
  // Check if current header is invalid
  if (pCur->ver == 0xFFFF || pCur->ver == 0x0000)
  {
    // Accept the image.
	return true;
  }
 
  // By default, accept an image if version is 0.
  if ((pNew->ver == 0) )
  {
    valid = true;
  }
  
  // If not already validated, check if new image is a later version than the
  // current image.
  if (!valid)
  {
    valid =  pNew->ver > pCur->ver;
  }
  
  return valid;
}

/*********************************************************************
 * @fn      OADTarget_storeImageHeader
 *
 * @brief   Store the image header of the new image
 *
 * @param   pValue - pointer to the new image header
 *
 * @return  none
 */
void OADTarget_storeImageHeader(uint8_t *pValue)
{
  img_hdr_t* pNew;
  
  pNew = (img_hdr_t*)(pValue +4);
  
  // Storage image header (written to external flash before reboot)
  imgInfo.crc[0] = BUILD_UINT16(pValue[0],pValue[1]);
  imgInfo.crc[1] = BUILD_UINT16(pValue[2],pValue[3]);
  imgInfo.addr = BUILD_UINT16(pNew->res[0],pNew->res[1]);
  imgInfo.ver = pNew->ver;
  imgInfo.len = pNew->len;
  memcpy(imgInfo.uid,pNew->uid,sizeof(imgInfo.uid));
  imgInfo.imgType = pNew->res[2];
}

/*********************************************************************
 * @fn      OADTarget_imageAddress
 *
 * @brief   Get the address to store the new image
 *
 * @param   pValue - pointer to the new image header
 *
 * @param   pHdr - pointer to the current image header
 *
 * @return  address
 */
uint32_t OADTarget_imageAddress(uint8_t *pValue)
{
  img_hdr_t *pNew;
  uint32_t extAddr;
  
  pNew = (img_hdr_t *)pValue;
 
  switch (pNew->res[2])
  {
    // Application start at the beginning of external flash.
    case EFL_OAD_IMG_TYPE_APP:
      extAddr = EFL_ADDR_IMAGE_APP;
      break;
      
    // All other images are placed into the next available image slot.
    default: 
      extAddr = EFL_ADDR_IMAGE_BLE;
      break;
  }
  
  return extAddr;
}

/*********************************************************************
 * @fn      OADTarget_getCrc
 *
 * @brief   Get the CRC array from the image that is being downloaded
 *
 * @param   pCrc - pointer to the new image header
 *
 * @return  None
 */
void OADTarget_getCrc(uint16_t *pCrc)
{
  // Copy CRC information from header information.
  pCrc[0] = imgInfo.crc[0];
  pCrc[1] = imgInfo.crc[1];
}

/*********************************************************************
 * @fn      OADTarget_setCrc
 *
 * @brief   Set the CRC shadow of the downloaded image.
 *
 * @param   pCrc - pointer to the new image header
 *
 * @return  Non
 */
void OADTarget_setCrc(uint16_t *pCrc)
{
  // Update shadow CRC to verify 
  imgInfo.crc[1] = pCrc[1];
}

/*********************************************************************
 * @fn      OADTarget_enableCache
 *
 * @brief   Prepares system for a write to flash, if necessary.
 *
 * @param   None.
 *
 * @return  None.
 */
void OADTarget_enableCache(void)
{
  // Do nothing.
}

/*********************************************************************
 * @fn      OADTarget_disableCache
 *
 * @brief   Resumes system after a write to flash, if necessary.
 *
 * @param   None.
 *
 * @return  None.
 */
void OADTarget_disableCache(void)
{
  // Do nothing.
}

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
void OADTarget_readFlash(uint8_t page, uint32_t offset, uint8_t *pBuf, 
                         uint16_t len)
{
  extFlashRead(FLASH_ADDRESS(page,offset), len, pBuf);
}

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
void OADTarget_writeFlash(uint8_t page, uint32_t offset, uint8_t *pBuf, 
                          uint16_t len)
{
  extFlashWrite(FLASH_ADDRESS(page,offset), len, pBuf);
}

/*********************************************************************
 * @fn      OADTarget_eraseFlash
 *
 * @brief   Erase selected flash page.
 *
 * @param   page - the page to erase.
 *
 * @return  None.
 */
void OADTarget_eraseFlash(uint8_t page)
{
  extFlashErase(FLASH_ADDRESS(page,0), HAL_FLASH_PAGE_SIZE);
}

/*********************************************************************
 * @fn      OADTarget_systemReset
 *
 * @brief   Prepare system for a reset and trigger a reset to the boot
 *          image manager. If the image contains a new BIM, copy it to 
 *          page '0' before resetting. This is a critical operation.
 *
 * @param   None.
 *
 * @return  None.
 */
void OADTarget_systemReset(void)
{ 
  // Reset to the bootloader.
  HAL_SYSTEM_RESET();
}

/*******************************************************************************
 * @fn      saveImageInfo
 *
 * @brief   Save image information in the meta-data area
 *
 * @return  none
 */
void saveImageInfo(void)
{
  uint32_t addr;
 
  if (imgInfo.imgType == EFL_OAD_IMG_TYPE_APP)
  {
    addr = EFL_IMAGE_INFO_ADDR_APP;
  }
  else
  {
    addr = EFL_IMAGE_INFO_ADDR_BLE;
  }
  
  // Erase old meta data.
  extFlashErase(addr, HAL_FLASH_PAGE_SIZE);
  
  // Set status so that bootloader pull in the new image.
  imgInfo.status = 0xFF;
  
  // Write new meta data.
  extFlashWrite(addr, sizeof(ExtImageInfo_t),
                (uint8_t*)&imgInfo);
}

/*******************************************************************************
 * @fn      getImageFlag
 *
 * @brief   Get the image type flag.
 *
 * @return  Image type or 0 if unknown.
 */
uint8_t getImageFlag(void)
{

  uint8_t flag = 0;
  
  if (imgInfo.imgType == EFL_OAD_IMG_TYPE_APP)
  {
    flag = OAD_IMG_APP_FLAG;
  }
  else if (imgInfo.imgType == EFL_OAD_IMG_TYPE_STACK)
  {
    flag = OAD_IMG_STACK_FLAG;
  }
  else if (imgInfo.imgType == EFL_OAD_IMG_TYPE_NP)
  {
    flag = OAD_IMG_NP_FLAG;
  }
  
  return flag;
}

#endif //FEATURE_OAD

