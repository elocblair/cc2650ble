/*******************************************************************************
*  Filename:       bsp_spi.h
*  Revised:        $Date$
*  Revision:       $Revision$
*
*  Description:    Layer added on top of RTOS driver for backward
*                  compatibility with non RTOS SPI driver.
*
*  Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

  /**
  * Open SPI interface
  *
  * @return none
  */
  extern void bspSpiOpen(void);

  /**
  * Close SPI interface
  *
  * @return True when successful.
  */
  extern void bspSpiClose(void);

  /**
  * Clear data from SPI interface
  *
  * @return none
  */
  extern void bspSpiFlush(void);

  /**
  * Read from an SPI device
  *
  * @return 0 when successful.
  */
  extern int bspSpiRead(uint8_t *buf, size_t length);

  /**
  * Write to an SPI device
  *
  * @return 0 when successful.
  */
  extern int bspSpiWrite(const uint8_t *buf, size_t length);

  /**
  * Write and read to/from an SPI device in the same transaction
  *
  * @return 0 when successful.
  */
  extern  int bspSpiWriteRead(uint8_t *buf, uint8_t wlen, uint8_t rlen);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SPI_H */
