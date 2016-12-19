/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Types.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>
#include <ti/drivers/uart/UARTCC26XX.h>
#include <ti/drivers/pin/PINCC26XX.h>

/* driverlib header files */
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <inc/hw_types.h>
#include <driverlib/uart.h>
#include <driverlib/sys_ctrl.h>
#include <driverlib/ioc.h>
#include <driverlib/aon_ioc.h>

/* UARTCC26XX functions */
void         UARTCC26XX_close(UART_Handle handle);
int          UARTCC26XX_control(UART_Handle handle, unsigned int cmd, void *arg);
void         UARTCC26XX_init(UART_Handle handle);
UART_Handle  UARTCC26XX_open(UART_Handle handle, UART_Params *params);
int          UARTCC26XX_read(UART_Handle handle, void *buffer, size_t size);
int          UARTCC26XX_readPolling(UART_Handle handle, void *buf, size_t size);
void         UARTCC26XX_readCancel(UART_Handle handle);
int          UARTCC26XX_write(UART_Handle handle, const void *buffer,
                            size_t size);
int          UARTCC26XX_writePolling(UART_Handle handle, const void *buf,
                                   size_t size);
void         UARTCC26XX_writeCancel(UART_Handle handle);

/* UARTCC26XX internal functions */
static void  UARTCC26XX_initHw(UART_Handle handle);
static bool  UARTCC26XX_initIO(UART_Handle handle);
static Power_NotifyResponse uartPostNotify(Power_Event eventType, uint32_t clientArg);

/* UART function table for UARTCC26XX implementation */
const UART_FxnTable UARTCC26XX_fxnTable = {
    UARTCC26XX_close,
    UARTCC26XX_control,
    UARTCC26XX_init,
    UARTCC26XX_open,
    UARTCC26XX_read,
    UARTCC26XX_readPolling,
    UARTCC26XX_readCancel,
    UARTCC26XX_write,
    UARTCC26XX_writePolling,
    UARTCC26XX_writeCancel
};

static const uint32_t dataLength[] = {
    UART_CONFIG_WLEN_5, /* UART_LEN_5 */
    UART_CONFIG_WLEN_6, /* UART_LEN_6 */
    UART_CONFIG_WLEN_7, /* UART_LEN_7 */
    UART_CONFIG_WLEN_8  /* UART_LEN_8 */
};

static const uint32_t stopBits[] = {
    UART_CONFIG_STOP_ONE,   /* UART_STOP_ONE */
    UART_CONFIG_STOP_TWO    /* UART_STOP_TWO */
};

static const uint32_t parityType[] = {
    UART_CONFIG_PAR_NONE,   /* UART_PAR_NONE */
    UART_CONFIG_PAR_EVEN,   /* UART_PAR_EVEN */
    UART_CONFIG_PAR_ODD,    /* UART_PAR_ODD */
    UART_CONFIG_PAR_ZERO,   /* UART_PAR_ZERO */
    UART_CONFIG_PAR_ONE     /* UART_PAR_ONE */
};

/* Guard to avoid power constraints getting out of sync */
static volatile bool uartPowerConstraint;

/*
 *  ======================== PIN driver objects ================================
 *  Simple callback to post a semaphore for the blocking mode.
 */
/* PIN driver state object */
static PIN_State pinState;
/* PIN driver handle */
static PIN_Handle hPin;

/*
 *  ================================= Macro ====================================
 *  TODO: Move me
 */
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

/*
 * Function for checking whether flow control is enabled.
 */
static inline bool isFlowControlEnabled(UARTCC26XX_HWAttrs const  *hwAttrs) {
    return ((hwAttrs->ctsPin != PIN_UNASSIGNED) && (hwAttrs->rtsPin != PIN_UNASSIGNED));
}

/*
 * Ensure safe setting of the standby disallow constraint.
 */
static inline void threadSafeStdbyDisSet() {
    unsigned int  key;

    /* Disable interrupts */
    key = Hwi_disable();

    if (!uartPowerConstraint) {
      /* Set constraints to guarantee operation */
      Power_setConstraint(Power_SB_DISALLOW);
      uartPowerConstraint = true;
    }

    /* Re-enable interrupts */
    Hwi_restore(key);
}

/*
 * Ensure safe releasing of the standby disallow constraint.
 */
static inline void threadSafeStdbyDisRelease() {
    unsigned int  key;

    /* Disable interrupts */
    key = Hwi_disable();

    if (uartPowerConstraint) {
        /* release constraint since operation is done */
        Power_releaseConstraint(Power_SB_DISALLOW);
        uartPowerConstraint = false;
    }

    /* Re-enable interrupts */
    Hwi_restore(key);
}

/*
 *  ======== writeSemCallback ========
 *  Simple callback to post a semaphore for the blocking mode.
 */
static void writeSemCallback(UART_Handle handle, void *buffer, size_t count)
{
    UARTCC26XX_Object *object = handle->object;

    Log_print1(Diags_USER1, "UART:(%p) posting write semaphore",
                ((UARTCC26XX_HWAttrs const *)(handle->hwAttrs))->baseAddr);

    Semaphore_post(Semaphore_handle(&(object->writeSem)));
}

/*
 *  ======== readSemCallback ========
 *  Simple callback to post a semaphore for the blocking mode.
 */
static void readSemCallback(UART_Handle handle, void *buffer, size_t count)
{
    UARTCC26XX_Object *object = handle->object;

    Log_print1(Diags_USER1, "UART:(%p) posting read semaphore",
                ((UARTCC26XX_HWAttrs const *)(handle->hwAttrs))->baseAddr);

    Semaphore_post(Semaphore_handle(&(object->readSem)));
}

/*
 *  ======== writeData ========
 *  Write and process data to the UART.
 */
static int32_t writeData(UART_Handle handle, int32_t size)
{
    UARTCC26XX_Object           *object;
    UARTCC26XX_HWAttrs const    *hwAttrs;

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Send characters until FIFO is full or done. */
    while (size) {
        /* Send the next character and increment counts. */
        if (!UARTCharPutNonBlocking(hwAttrs->baseAddr, *(unsigned char *)(object->writeBuf))) {
            /* Character was not sent */
            break;
        }
        Log_print2(Diags_USER2, "UART:(%p) Wrote character 0x%x",
                   hwAttrs->baseAddr, *(unsigned char *)object->writeBuf);
        object->writeBuf = (unsigned char *)object->writeBuf + 1;
        size--;
        object->writeCount++;
    }
    return (size);
}

/*
 *  ======== readData ========
 *  Read and process data from the UART.
 */
static int32_t readData(UART_Handle handle, int32_t size)
{
    int32_t                     readIn;
    UARTCC26XX_Object           *object;
    UARTCC26XX_HWAttrs const    *hwAttrs;

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Receive chars until empty or done. */
    while (size && (readIn = (int32_t)UARTCharGetNonBlocking(hwAttrs->baseAddr)) != -1) {

        Log_print2(Diags_USER2, "UART:(%p) Read character 0x%x",
                                 hwAttrs->baseAddr, (uint8_t)readIn);

        /* Update status. */
        *((unsigned char *)object->readBuf) = (uint8_t)readIn;
        object->readBuf = (unsigned char *)object->readBuf + 1;
        object->readCount++;
        size--;
    }
    return (size);
}

/*
 *  ======== readFinishedDoCallback ========
 *  Read finished - make callback
 *
 *  When all expected bytes are received or receive timeout has occured, read
 *  has succeeded. The RX interrupts are disabled and standby is allowed again,
 *  but RX is not disabled. The application should either issue a new read to
 *  ensure we don't get an overflow or call readCancel(...) so the RX is turned
 *  off.
 *
 *  @param(handle)         The UART_Handle for ongoing read.
 */
static void readFinishedDoCallback(UART_Handle handle)
{
    UARTCC26XX_Object           *object;
    UARTCC26XX_HWAttrs const    *hwAttrs;

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /*       Release power constraint. */
    threadSafeStdbyDisRelease();

    /*       Disable RX interrupts */
    UARTIntDisable(hwAttrs->baseAddr, UART_INT_OE | UART_INT_BE
                   | UART_INT_PE | UART_INT_FE | UART_INT_RT |
                   UART_INT_RX);

    /* Reset the read buffer so we can pass it back */
    object->readBuf = (unsigned char *)object->readBuf - object->readCount;

    /* Do Callback */
    object->readCallback(handle, object->readBuf,
                         object->readCount);

    Log_print2(Diags_USER1, "UART:(%p) Read finished, %d bytes read",
               hwAttrs->baseAddr, object->readCount);
}

/*
 *  ======== startTxFifoEmptyClk ========
 *  Last write to TX FIFO is done, but not shifted out yet. Start a clock
 *  which will trigger when the TX FIFO should be empty.
 *
 *  @param  handle           The UART_Handle for ongoing write.
 *  @param  numOfDataInFifo  The number of data present in FIFO after last write
 */
static void startTxFifoEmptyClk(UART_Handle handle, unsigned int numOfDataInFifo)
{
    UARTCC26XX_Object           *object;

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;

    /* Ensure that the clock is stopped so we can set a new timeout */
    Clock_stop((Clock_Handle) &(object->txFifoEmptyClk));

    /* No more to write, but data is not shiftet out properly yet.
     *   1. Compute appropriate wait time for FIFO to empty out
     *       - 1 bit for start bit
     *       - 5+(object->dataLength) for total data length
     *       - +1 to "map" from stopBits to actual number of bits
     *       - 1000000 so we get 1 us resolution
     *       - 100 (100us) for margin
     */
    unsigned int writeTimeoutUs = (numOfDataInFifo*(1+5+(object->dataLength)+(stopBits[object->stopBits]+1))*1000000)/object->baudRate + 100;
    /*   2. Configure clock object to trigger when FIFO is empty */
    Clock_setTimeout((Clock_Handle) &(object->txFifoEmptyClk), (writeTimeoutUs/Clock_tickPeriod));
    Clock_start((Clock_Handle) &(object->txFifoEmptyClk));
}

/*
 *  ======== writeFinishedDoCallback ========
 *  Write finished - make callback
 *
 *  This function is called when the txFifoEmptyClk times out. The TX FIFO
 *  should now be empty and all bytes have been transmitted. The TX will be
 *  turned off, TX interrupt is disabled and standby is allowed again.
 *
 *  @param(handle)         The UART_Handle for ongoing write.
 */
static void writeFinishedDoCallback(UART_Handle handle)
{
    UARTCC26XX_Object           *object;
    UARTCC26XX_HWAttrs const    *hwAttrs;

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Stop the txFifoEmpty clock */
    Clock_stop((Clock_Handle) &(object->txFifoEmptyClk));

    /*   Function verifies that the FIFO is empty via BUSY flag */
    /*   If not yet ready start the periodic timer and wait another period*/
    /*   Return.. */
    if(UARTBusy(hwAttrs->baseAddr)){
        /* The UART is still busy.
         * Wait 500 us before checking again or 1 tick period if the
         * Clock_tickPeriod is larger than 500 us.
         */
        Clock_setTimeout((Clock_Handle) &(object->txFifoEmptyClk), MAX((500/Clock_tickPeriod),1));
        Clock_start((Clock_Handle) &(object->txFifoEmptyClk));
        return;
    }

    /* Release constraint since transaction is done */
    threadSafeStdbyDisRelease();

    /* Disable TX interrupt */
    UARTIntDisable(hwAttrs->baseAddr, UART_INT_TX);

    /* Disable TX */
    HWREG(UART0_BASE + UART_O_CTL) &= ~(UART_CTL_TXE);

    /* Reset the write buffer so we can pass it back */
    object->writeBuf = (unsigned char *)object->writeBuf - object->writeCount;

    /* Make callback */
    object->writeCallback(handle, (uint8_t*)object->writeBuf,
                          object->writeCount);
    Log_print2(Diags_USER1, "UART:(%p) Write finished, %d bytes written",
               hwAttrs->baseAddr, object->writeCount);
}

/*
 *  ======== writeTxFifoFlush ========
 *  Write cancelled or timed out, the TX FIFO must be flushed out.
 *
 *  This function is called either from writeCancel or when a blocking write
 *  has timed out. The HW does not support a simple API for flushing the TX FIFO
 *  so a workaround is done in SW.
 *
 *  @pre The TX FIFO empty clock must have been started in blocking mode.
 *
 *  @param object         Pointer to UART object
 *  @param hwAttrs        Pointer to UART hwAttrs
 */
static void writeTxFifoFlush(UARTCC26XX_Object  *object, UARTCC26XX_HWAttrs const  *hwAttrs)
{
    /*It is not possible to flush the TX FIFO with simple write to HW, doing workaround:
     * 0. Disable TX interrupt
     */
    UARTIntDisable(hwAttrs->baseAddr, UART_INT_TX);
    /* 1. Ensure TX IO will stay high when connected to GPIO */
    PIN_setOutputEnable(hPin, hwAttrs->txPin, 1);
    PIN_setOutputValue(hPin, hwAttrs->txPin, 1);
    /* 2. Disconntect tx from IO, and set it as "GPIO" */
    PINCC26XX_setMux(hPin, hwAttrs->txPin, IOC_PORT_GPIO);
    /* 3. Disconnect cts */
    PINCC26XX_setMux(hPin, hwAttrs->ctsPin, IOC_PORT_GPIO);
    /*4. Wait for TX FIFO to become empty.
     *    CALLBACK: Idle until the TX FIFO is empty, i.e. no longer busy.
     *    BLOCKING: Periodically check if TX is busy emptying the FIFO.
     *              Must be handled at TX FIFO empty clock timeout:
     *                - the timeout/finish function must check the status
     */
    if(object->writeMode == UART_MODE_CALLBACK) {
        /* Wait until the TX FIFO is empty. CALLBACK mode can be used from
         * hwi/swi context, so we cannot use semaphore..
         */
        while(UARTBusy(hwAttrs->baseAddr));
    } else { /* i.e. UART_MODE_BLOCKING */
        /* Pend on semaphore again..(this time forever since we are flushing
         * TX FIFO and nothing should be able to stop it..
         */
        Semaphore_pend(Semaphore_handle(&(object->writeSem)), BIOS_WAIT_FOREVER);
    }
    /* 5. Revert to active pins before returning */
    PINCC26XX_setMux(hPin, hwAttrs->txPin, IOC_PORT_MCU_UART0_TX);
    if(isFlowControlEnabled(hwAttrs)) {
        PINCC26XX_setMux(hPin, hwAttrs->ctsPin, IOC_PORT_MCU_UART0_CTS);
    }
}

/*
 *  ======== UARTCC26XX_hwiIntFxn ========
 *  Hwi function that processes UART interrupts.
 *
 *  Six UART interrupts are enabled when configured for RX: Receive timeout,
 *  receive FIFO is 4/8 full and all four error interrupts.
 *
 *  One interrupt is enabled when configured for TX: Transmit FIFO is 7/8
 *  empty.
 *
 *  The RX and TX can operate independently of each other.
 *
 *  When the read or write is finished they will
 *  post the semaphore or make the callback and log the transfer.
 *
 *  @param  arg         The UART_Handle for this Hwi.
 */
void UARTCC26XX_hwiIntFxn(UArg arg)
{
    unsigned long            intStatus;
    unsigned long            errStatus;
    UARTCC26XX_Object        *object;
    UARTCC26XX_HWAttrs const *hwAttrs;

    /* Get the pointer to the object and hwAttrs */
    object = ((UART_Handle)arg)->object;
    hwAttrs = ((UART_Handle)arg)->hwAttrs;

    /* Clear interrupts */
    intStatus = UARTIntStatus(hwAttrs->baseAddr, true);
    UARTIntClear(hwAttrs->baseAddr, intStatus);

    Log_print2(Diags_USER2, "UART:(%p) Interrupt with mask 0x%x",
                             hwAttrs->baseAddr, intStatus);

    /* Basic error handling */
    if(intStatus & (UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE)) {

        /* If overrun, the error bit is set immediately */
        if(intStatus & UART_INT_OE) {
            /* check receive status register */
            errStatus = UARTRxErrorGet(hwAttrs->baseAddr);
            UARTRxErrorClear(hwAttrs->baseAddr);
            /* read out min of readsize and fifo size */
            int32_t bytesToRead = MIN(UARTCC26XX_FIFO_SIZE, object->readSize);
            readData((UART_Handle)arg, bytesToRead);
        }
        else {
            /*
             * else, for break, framing and par. error bits are at error index
             * (we would like to keep the "surviving" bits)
             */
            while(object->readSize > 0) {
                /* read one data */
                readData((UART_Handle)arg, 1);
                /* check receive status register if byte is OK */
                errStatus = UARTRxErrorGet(hwAttrs->baseAddr);
                UARTRxErrorClear(hwAttrs->baseAddr);
                if(errStatus != 0) {
                    /*
                     * last read was not including data, reset data vars
                     * (i.e. upd readBuf pointer, decr readCount)
                     */
                    object->readBuf = (unsigned char *)object->readBuf - 1;
                    object->readCount--;
                    /* the FIFO index with error is reached, stop reading more. */
                    break;
                }
                else {
                    /* current read was not the problem update readSize */
                    object->readSize--;
                }
            }
        }

        /* Report current error status */
        object->status = (UART_Status)errStatus;

        /* Break and clean up any ongoing transaction */
        UARTCC26XX_readCancel((UART_Handle)arg);
    }
    else {
        /*   If RT (Receive Timeout) occured it means we are reading */
        /*   (handle in swi?) */
        if(intStatus & UART_INT_RT) {
            /* Read data from FIFO.*/
            object->readSize = readData((UART_Handle)arg, object->readSize);
            /*     If return partial is set */
            if(object->readRetPartial) {
                /* Partial read accepted, read succeeded by def, set readSize
                * to allow callback to trigger new _read()
                */
                object->readSize = 0;
                /*       Read succeeded */
                readFinishedDoCallback((UART_Handle)arg);
            }
            /*     else - return when all bytes have arrived  */
            else {
            /*       If FIFO contained last bytes */
                if(!object->readSize) {
            /*         Read succeeded  */
                    readFinishedDoCallback((UART_Handle)arg);
                }
            }
        }
        else {
            /* RX interrupt, since CTS is handled by HW */
            if (intStatus & UART_INT_RX) {
                /* Read whatever is less of:
                *   - FIFO_THR bytes - 1 (leave one byte to trigger RT timeout)
                *   - object->readSize (never read more than requested from application)
                * since RT timeout will only trigger if FIFO is not empty.
                */
                int32_t bytesToRead = MIN(object->readFifoThreshold-1, object->readSize);
                /* Do read, all bytes we request to read, are present in FIFO */
                readData((UART_Handle)arg, bytesToRead);
                /* Decrement objec->readSize with the actual number of bytes read
                *  There will always be at least bytesToRead number of bytes in FIFO
                */
                object->readSize -= bytesToRead;
                /*      If FIFO contained last bytes */
                if (!object->readSize) {
                /*        Read succeeded. */
                    readFinishedDoCallback((UART_Handle)arg);
                }
            }
        }
    }
    /* Write if there are characters to be written. */
    if ((intStatus & UART_INT_TX) && object->writeSize) {
        /* Using writtenLast=writeSize before last write
         *(since writeSize=0 when the last finishes)
         */
        uint32_t writtenLast = object->writeSize;
        object->writeSize = writeData((UART_Handle)arg, object->writeSize);
        if(!object->writeSize) {
            /* No more to write, but data is not shiftet out properly yet.
             * Start TX FIFO Empty clock, which will trigger the txFifoEmptyClk
             * function.
             * +UART_TH_FIFO_1_8 because it is 4 bytes left in TX FIFO when the TX FIFO
             * threshold interrupt occurs.
             */
             startTxFifoEmptyClk((UART_Handle)arg, (writtenLast+UART_TH_FIFO_1_8));
        }
    }
}

/*!
 *  @brief UART CC26XX initialization
 *
 *  @pre    Calling context: Hwi, Swi, Task, Main
 *
 *  @param handle  A UART_Handle
 *
 */
void UARTCC26XX_init(UART_Handle handle)
{
    UARTCC26XX_Object           *object;

    /* Get the pointer to the object */
    object = handle->object;
    object->opened = false;

    /* Init the power constraint flag. */
    uartPowerConstraint = false;
}

/*!
 *  @brief  Function to initialize the CC26XX UART peripheral specified by the
 *          particular handle. The parameter specifies which mode the UART
 *          will operate.
 *
 *  The function will set a dependency on it power domain, i.e. power up the
 *  module and enable the clock. The IOs are allocated. Neither the RX nor TX
 *  will be enabled, and none of the interrupts are enabled.
 *
 *  @pre    UART controller has been initialized
 *          Calling context: Task
 *
 *  @param  handle        A UART_Handle
 *
 *  @param  params        Pointer to a parameter block, if NULL it will use
 *                        default values
 *
 *  @return A UART_Handle on success or a NULL on an error or if it has been
 *          already opened
 *
 *  @sa     UARTCC26XX_close()
 */
UART_Handle UARTCC26XX_open(UART_Handle handle, UART_Params *params)
{
    unsigned int                    key;
    /* Use union to save on stack allocation */
    union {
        Hwi_Params                  hwiParams;
        Semaphore_Params            semParams;
        Clock_Params                clkParams;
    } paramsUnion;
    UARTCC26XX_Object               *object;
    UARTCC26XX_HWAttrs const        *hwAttrs;
    UART_Params                     uartParams;

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Disable preemption while checking if the UART is open. */
    key = Hwi_disable();

    /* Check if the UART is open already with the base addr. */
    if (object->opened == true) {
        Hwi_restore(key);

        Log_warning1("UART:(%p) already in use.", hwAttrs->baseAddr);

        return (NULL);
    }
    object->opened = true;
    Hwi_restore(key);

    /* If params are NULL use defaults. */
    if (params == NULL) {
        UART_Params_init(&uartParams);
        params = &uartParams;
    }

    /* Check that a callback is set */
    Assert_isTrue((params->readMode != UART_MODE_CALLBACK) ||
                  (params->readCallback != NULL), NULL);
    Assert_isTrue((params->writeMode != UART_MODE_CALLBACK) ||
                  (params->writeCallback != NULL), NULL);

    /* Initialize the UART object */
    object->readMode       = params->readMode;
    object->writeMode      = params->writeMode;
    object->readTimeout    = params->readTimeout;
    object->writeTimeout   = params->writeTimeout;
    object->readCallback   = params->readCallback;
    object->writeCallback  = params->writeCallback;
    object->readReturnMode = params->readReturnMode;
    object->readDataMode   = params->readDataMode;
    object->writeDataMode  = params->writeDataMode;
    object->baudRate       = params->baudRate;
    object->dataLength     = params->dataLength;
    object->stopBits       = params->stopBits;
    object->parityType     = params->parityType;

    /* Set UART transaction variables to defaults. */
    object->writeBuf = NULL;
    object->readBuf = NULL;
    object->writeCount = 0;
    object->readCount = 0;
    object->writeSize = 0;
    object->readSize = 0;
    object->writeCR = false;
    object->readRetPartial = false;
    object->readFifoThreshold = UART_TH_FIFO_4_8;
    object->writeFifoThreshold = UART_FIFO_TX1_8;

    /* Register power dependency - i.e. power up and enable clock for UART. */
    Power_setDependency(hwAttrs->powerMngrId);

    /* Initialize the UART hardware module */
    UARTCC26XX_initHw(handle);

    /* Configure IOs, make sure it was successful */
    if(!UARTCC26XX_initIO(handle)) {
        /* Trying to use UART driver when some other driver or application
        *  has already allocated these pins, error!
        */
        Log_warning0("Could not allocate pins, already in use.");
        /* Disable UART */
        UARTDisable(hwAttrs->baseAddr);
        /* Release power dependency - i.e. potentially power down serial domain. */
        Power_releaseDependency(hwAttrs->powerMngrId);
        /* Mark the module as available */
        key = Hwi_disable();
        object->opened = false;
        Hwi_restore(key);
        /* Signal back to application that UART driver was not succesfully opened */
        return (NULL);
    }

    /* Create Hwi object for this UART peripheral. */
    Hwi_Params_init(&(paramsUnion.hwiParams));
    paramsUnion.hwiParams.arg = (UArg)handle;
    Hwi_construct(&(object->hwi), hwAttrs->intNum, UARTCC26XX_hwiIntFxn,
                  &(paramsUnion.hwiParams), NULL);

    /* Initialize semaphore */
    Semaphore_Params_init(&(paramsUnion.semParams));
    paramsUnion.semParams.mode = Semaphore_Mode_BINARY;

    /* If write mode is blocking create a semaphore and set callback. */
    if (object->writeMode == UART_MODE_BLOCKING) {
        Semaphore_construct(&(object->writeSem), 0, &(paramsUnion.semParams));
        object->writeCallback = &writeSemCallback;
    }

    /* If read mode is blocking create a semaphore and set callback. */
    if (object->readMode == UART_MODE_BLOCKING) {
        Semaphore_construct(&(object->readSem), 0, &(paramsUnion.semParams));
        object->readCallback = &readSemCallback;
    }

    /* Create clock object to be used for write FIFO empty callback */
    Clock_Params_init(&paramsUnion.clkParams);
    paramsUnion.clkParams.period    = 0;
    paramsUnion.clkParams.startFlag = false;
    paramsUnion.clkParams.arg       = (UArg) handle;
    Clock_construct(&(object->txFifoEmptyClk),
                    (Clock_FuncPtr) &writeFinishedDoCallback,
                    10, &(paramsUnion.clkParams));

    /* Register notification function */
    Power_registerNotify(&object->uartPostObj, Power_AWAKE_STANDBY, (Fxn)uartPostNotify, (uint32_t)handle, NULL );

    /* UART opened successfully */
    Log_print1(Diags_USER1, "UART:(%p) opened", hwAttrs->baseAddr);

    /* Return the handle */
    return (handle);
}

/*!
 *  @brief  Function to close a given CC26XX UART peripheral specified by the
 *          UART handle.
 *
 *  Will disable the UART, disable all UART interrupts and release the
 *  dependency on the corresponding power domain.
 *
 *  @pre    UARTCC26XX_open() had to be called first.
 *          Calling context: Task
 *
 *  @param  handle  A UART_Handle returned from UART_open()
 *
 *  @sa     UARTCC26XX_open
 */
void UARTCC26XX_close(UART_Handle handle)
{
    unsigned int                 key;
    UARTCC26XX_Object            *object;
    UARTCC26XX_HWAttrs const     *hwAttrs;

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Deallocate pins */
    PIN_close(hPin);

    /* Disable all UART module interrupts. */
    UARTIntDisable(hwAttrs->baseAddr, UART_INT_OE | UART_INT_BE | UART_INT_PE |
                                      UART_INT_FE | UART_INT_RT | UART_INT_TX |
                                      UART_INT_RX | UART_INT_CTS);
    /* Disable UART */
    UARTDisable(hwAttrs->baseAddr);

    /* Release power dependency - i.e. potentially power down serial domain. */
    Power_releaseDependency(hwAttrs->powerMngrId);

    /* Destruct the SYS/BIOS objects. */
    Hwi_destruct(&(object->hwi));
    if (object->writeMode == UART_MODE_BLOCKING) {
        Semaphore_destruct(&(object->writeSem));
    }
    if (object->readMode == UART_MODE_BLOCKING) {
        Semaphore_destruct(&(object->readSem));
    }
    Clock_destruct(&(object->txFifoEmptyClk));

    /* Mark the module as available */
    key = Hwi_disable();
    object->opened = false;
    Hwi_restore(key);

    /* Unregister power notification objects */
    Power_unregisterNotify(&object->uartPostObj);

    Log_print1(Diags_USER1, "UART:(%p) closed", hwAttrs->baseAddr);
}


/*!
 *  @brief  Function for setting control parameters of the UART
 *          after it has been opened.
 *
 *  @pre    UARTCC26XX_open() has to be called first.
 *          Calling context: Hwi, Swi, Task
 *
 *  @param  handle A UART handle returned from UARTCC26XX_open()
 *
 *  @param  cmd  The command to execute, supported commands are:
 *          | Command                               | Description             |
 *          |-------------------------------------- |-------------------------|
 *          | ::UARTCC26XX_CMD_RETURN_PARTIAL_ENABLE | Enable RETURN_PARTIAL  |
 *          | ::UARTCC26XX_CMD_RETURN_PARTIAL_DISABLE| Disable RETURN_PARTIAL |
 *
 *  @param  *arg  Pointer to command arguments, currently not in use, set to NULL.
 *
 *  @return ::UART_STATUS_SUCCESS if success, or error code if error.
 */
int UARTCC26XX_control(UART_Handle handle, unsigned int cmd, void *arg)
{
    /* Locals */
    UARTCC26XX_Object            *object;

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;

    /* Initialize return value*/
	int ret = UART_STATUS_UNDEFINEDCMD;
    /* Do command*/
	switch(cmd) {

		case UARTCC26XX_CMD_RETURN_PARTIAL_ENABLE:
			/* Enable RETURN_PARTIAL */
			object->readRetPartial = true;
            ret = UART_STATUS_SUCCESS;
			break;

		case UARTCC26XX_CMD_RETURN_PARTIAL_DISABLE:
			/* Disable RETURN_PARTIAL */
			object->readRetPartial = false;
            ret = UART_STATUS_SUCCESS;
			break;

		default:
            /* This command is not defined */
            ret = UART_STATUS_UNDEFINEDCMD;
			break;
	}

    /* Return */
	return (ret);
}

/*!
 *  @brief  Function that writes data to a UART
 *
 *  This function initiates an operation to write data to CC26XX UART
 *  controller.
 *
 *  In ::UART_MODE_BLOCKING, UART_write will block task execution until all
 *  the data in buffer has been written.
 *
 *  In ::UART_MODE_CALLBACK, UART_write does not block task execution, but calls a
 *  callback function specified by writeCallback when the data has been written.
 *
 *  When the write function is called, TX is enabled, TX interrupt is enabled,
 *  and standby is not allowed.
 *
 *  @pre    UARTCC26XX_open() has to be called first.
 *          Calling context: Hwi and Swi (only if using ::UART_MODE_CALLBACK), Task
 *
 *  @param  handle      A UART_Handle returned from UARTCC26XX_open()
 *
 *  @param  buffer      A pointer to buffer containing data to be written
 *
 *  @param  size        The number of bytes in buffer that should be written
 *                      onto the UART.
 *
 *  @return Returns the number of bytes that have been written to the UART,
 *          UART_ERROR on an error.
 *
 */
int UARTCC26XX_write(UART_Handle handle, const void *buffer, size_t size)
{
    unsigned int                     key;
    UARTCC26XX_Object                *object;
    UARTCC26XX_HWAttrs const         *hwAttrs;

    /* Get the pointer to the object */
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Check that there is data to write */
    Assert_isTrue(size != 0, NULL);

    /* Disable preemption while checking if the UART is in use. */
    key = Hwi_disable();
    if (object->writeSize) {
        Hwi_restore(key);
        Log_warning1("UART:(%p) Could not write data, uart in use.",
                    ((UARTCC26XX_HWAttrs const *)(handle->hwAttrs))->baseAddr);

        return (UART_ERROR);
    }

    /* Stop the txFifoEmpty clock in case it was running due to a previous write operation */
    Clock_stop((Clock_Handle) &(object->txFifoEmptyClk));

    /* Update the status of the UART module */
    object->status = UART_OK;

    /* Save the data to be written and restore interrupts. */
    object->writeBuf = buffer;
    object->writeCount = 0;

    Hwi_restore(key);

    /* Set constraints to guarantee transaction */
    threadSafeStdbyDisSet();

    /* Enable TX */
    HWREG(UART0_BASE + UART_O_CTL) |= UART_CTL_TXE;

    uint32_t writtenLast = size;
    /* Fill up TX FIFO */
    if (!(object->writeSize = writeData(handle, size))) {
        /* No more data to transmit - Write is finished but all bytes
        *  may not have been shifted out. */
        startTxFifoEmptyClk((UART_Handle)handle, writtenLast);

        /* If writeMode is blocking, block and get the status. */
        if (object->writeMode == UART_MODE_BLOCKING) {
            /* Pend on semaphore and wait for Hwi to finish. */
            if (!Semaphore_pend(Semaphore_handle(&(object->writeSem)), object->writeTimeout)) {
                /* Reset writeSize */
                object->writeSize = 0;

                /* Set status to TIMED_OUT */
                object->status = UART_TIMED_OUT;

                /* Workaround for flushing the TX FIFO */
                writeTxFifoFlush(object, hwAttrs);

                /* Release constraint */
                threadSafeStdbyDisRelease();

                Log_print2(Diags_USER1, "UART:(%p) Write timed out, %d bytes written",
                           ((UARTCC26XX_HWAttrs const *)(handle->hwAttrs))->baseAddr,
                           object->writeCount);

                /* Return UART_ERROR to indicate something went wrong, object->status set to UART_TIMED_OUT*/
                return UART_ERROR;
            }
            return (object->writeCount);
        }
    } else {

        /* Enable TX interrupts */
        UARTIntEnable(hwAttrs->baseAddr, UART_INT_TX);

        /* If writeMode is blocking, block and get the status. */
        if (object->writeMode == UART_MODE_BLOCKING) {
            /* Pend on semaphore and wait for Hwi to finish. */
            if (!Semaphore_pend(Semaphore_handle(&(object->writeSem)), object->writeTimeout)) {
                /* Semaphore timed out, make the write empty and log the write. */

                /* Starting a timer to enable the posting of semaphore used in writeTxFifoFlush.
                 * writtenLast in this case is equal to full TX FIFO. This is a conservative number as
                 * some of the data might have been sent.
                 */
                startTxFifoEmptyClk((UART_Handle)handle, writtenLast);

                /* Reset writeSize */
                object->writeSize = 0;

                /* Set status to TIMED_OUT */
                object->status = UART_TIMED_OUT;

                /* Workaround for flushing the TX FIFO */
                writeTxFifoFlush(object, hwAttrs);

                /* Release constraint */
                threadSafeStdbyDisRelease();

                Log_print2(Diags_USER1, "UART:(%p) Write timed out, %d bytes written",
                           ((UARTCC26XX_HWAttrs const *)(handle->hwAttrs))->baseAddr,
                           object->writeCount);

                /* Return UART_ERROR to indicate something went wrong (object->status set to UART_TIMED_OUT)*/
                return UART_ERROR;
            }

            /* Return the numbers of samples written */
            return (object->writeCount);
        }
    }
    /* This return will only be active in UART_MODE_CALLBACK mode. */
    return (0);
}

/*!
 *  @brief This function is NOT supported
 *
 *  @pre    UARTCC26XX_open() and has to be called first.
 *          Calling context: Task
 *
 *  @param handle       The UART_Handle for ongoing write.
 *
 *  @param  buffer      A pointer to buffer containing data to be written
 *
 *  @param  size        The number of bytes in buffer that should be written
 *                      onto the UART.
 *
 *  @return Always ::UART_ERROR
 */
int UARTCC26XX_writePolling(UART_Handle handle, const void *buf, size_t size)
{
    /* Not supported */
    return (UART_ERROR);
}

/*!
 *  @brief Function that cancel UART write. Will disable TX interrupt, disable
 *         TX and allow standby.
 *
 *  @pre    UARTCC26XX_open() and has to be called first.
 *          Calling context: Task
 *
 *  @param handle         The UART_Handle for ongoing write.
 */
void UARTCC26XX_writeCancel(UART_Handle handle)
{
    unsigned int                key;
    UARTCC26XX_Object           *object;
    UARTCC26XX_HWAttrs const    *hwAttrs;

    /* Get the pointer to the object */
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Disable interrupts to avoid writing data while changing state. */
    key = Hwi_disable();

    /* Return if there is nothing to write and TX FIFO is empty. */
    if ((!object->writeSize) && (!UARTBusy(hwAttrs->baseAddr))) {
        Hwi_restore(key);
        return;
    }

    /* Set size = 0 to prevent writing and restore interrupts. */
    object->writeSize = 0;
    Hwi_restore(key);

    /* If flow control is enabled, a workaround for flushing the fifo is needed..*/
    if (isFlowControlEnabled(hwAttrs)) {
        writeTxFifoFlush(object, hwAttrs);
    }

    /* Disable TX interrupt */
    UARTIntDisable(hwAttrs->baseAddr, UART_INT_TX);

    /* Disable UART TX */
    HWREG(UART0_BASE + UART_O_CTL) &= ~(UART_CTL_TXE);

    /* Release constraint since transaction is done */
    threadSafeStdbyDisRelease();

    /* Reset the write buffer so we can pass it back */
    object->writeBuf = (unsigned char *)object->writeBuf - object->writeCount;
    object->writeCallback(handle, (uint8_t*)object->writeBuf,
                          object->writeCount);

    Log_print2(Diags_USER1, "UART:(%p) Write canceled, "
                            "%d bytes written",
             ((UARTCC26XX_HWAttrs const *)(handle->hwAttrs))->baseAddr,
               object->writeCount);
}

/*!
 *  @brief  Function for reading from UART interface.
 *
 *  The function will enable the RX, enable all RX interrupts and disallow
 *  chip from going into standby.
 *
 *  @pre    UARTCC26XX_open() has to be called first.
 *          Calling context: Hwi and Swi (only if using ::UART_MODE_CALLBACK), Task
 *
 *  @param  handle A UART handle returned from UARTCC26XX_open()
 *
 *  @param  *buffer  Pointer to read buffer
 *
*  @param  size  Number of bytes to read. If ::UARTCC26XX_CMD_RETURN_PARTIAL_ENABLE
 *                has been set, the read will
 *                return if the reception is inactive for a 32-bit period
 *                (i.e. before all bytes are received).
 *
 *  @return Number of samples read
 *
 *  @sa     UARTCC26XX_open(), UARTCC26XX_readCancel()
 */
int UARTCC26XX_read(UART_Handle handle, void *buffer, size_t size)
{
    unsigned int                     key;
    UARTCC26XX_Object                *object;
    UARTCC26XX_HWAttrs const         *hwAttrs;

    /* Get the pointer to the object */
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Disable preemption while checking if the uart is in use. */
    key = Hwi_disable();
    if (object->readSize) {
        Hwi_restore(key);

        Log_warning1("UART:(%p) Could not read data, uart in use.",
             ((UARTCC26XX_HWAttrs const *)(handle->hwAttrs))->baseAddr);

        return (UART_ERROR);
    }

    /* Set readSize */
    object->readSize = size;

    /* Update the status of the UART module */
    object->status = UART_OK;

    /* Save the data to be read and restore interrupts. */
    object->readBuf = buffer;
    object->readCount = 0;

    Hwi_restore(key);

    /* Set constraint for sleep to guarantee transaction */
    threadSafeStdbyDisSet();

    /* Enable RX */
    HWREG(UART0_BASE + UART_O_CTL) |= UART_CTL_RXE;

    /* Enable RX interrupts */
    UARTIntEnable(hwAttrs->baseAddr, UART_INT_RX | UART_INT_RT |
                  UART_INT_OE | UART_INT_BE | UART_INT_PE | UART_INT_FE);

    /* If readMode is blocking, block and get the status. */
    if (object->readMode == UART_MODE_BLOCKING) {
        /* Pend on semaphore and wait for Hwi to finish. */
        if (!Semaphore_pend(Semaphore_handle(&(object->readSem)),
                            object->readTimeout)) {
            /* Semaphore timed out, make the read empty and log the read. */
            object->readSize = 0;

            /* Disable RX interrupts */
            UARTIntDisable(hwAttrs->baseAddr, UART_INT_OE | UART_INT_BE | UART_INT_PE |
                                              UART_INT_FE | UART_INT_RT | UART_INT_RX);

            /* Release constraint since transaction timed out, allowed to enter standby */
            threadSafeStdbyDisRelease();

            /* Reset the read buffer so we can pass it back */
            object->readBuf = (unsigned char *)object->readBuf - object->readCount;

            /* Set status to TIMED_OUT */
            object->status = UART_TIMED_OUT;

            Log_print2(Diags_USER1, "UART:(%p) Read timed out, %d bytes read",
                     ((UARTCC26XX_HWAttrs const *)(handle->hwAttrs))->baseAddr,
                       object->readCount);

        }
        /* return the numbers of samples read */
        return (object->readCount);
    }

    return (0);
}

/*
 *  ======== UARTCC26XX_readPolling ========
 */
int UARTCC26XX_readPolling(UART_Handle handle, void *buf, size_t size)
{
    /* Not supported */
    return (UART_ERROR);
}

/*!
 *  @brief Function that cancel UART read. Will disable all RX interrupt,
 *         disable
 *         RX and allow standby. Should also be called after a succeeding UART
 *         read if no more bytes are expected and standby is wanted.
 *
 *  @pre    UARTCC26XX_open() has to be called first.
 *          Calling context: Task
 *
 *  @param handle         The UART_Handle for ongoing write.
 */
void UARTCC26XX_readCancel(UART_Handle handle)
{
    unsigned int                 key;
    UARTCC26XX_Object            *object;
    UARTCC26XX_HWAttrs const     *hwAttrs;

    /* Get the pointer to the object */
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Disable interrupts to avoid reading data while changing state. */
    key = Hwi_disable();

    /* Return if there is no read. */
    if (!object->readSize) {
        Hwi_restore(key);
        return;
    }

    /* Set size = 0 to prevent reading and restore interrupts. */
    object->readSize = 0;
    Hwi_restore(key);

    /* Disable RX interrupts */
    UARTIntDisable(hwAttrs->baseAddr, UART_INT_OE | UART_INT_BE | UART_INT_PE |
                                      UART_INT_FE | UART_INT_RT | UART_INT_RX);

    /* Disable RX */
    HWREG(UART0_BASE + UART_O_CTL) &= ~(UART_CTL_RXE);

    /* Release constraint since transaction is done */
    threadSafeStdbyDisRelease();

    /* Reset the read buffer so we can pass it back */
    object->readBuf = (unsigned char *)object->readBuf - object->readCount;
    object->readCallback(handle, object->readBuf, object->readCount);

    Log_print2(Diags_USER1, "UART:(%p) Read canceled, "
                            "%d bytes read",
             ((UARTCC26XX_HWAttrs const *)(handle->hwAttrs))->baseAddr,
               object->readCount);
}

/*
 *  ======== UARTCC26XX_initHW ========
 *  This functions initializes the UART hardware module.
 *
 *  @pre    Function assumes that the UART handle is pointing to a hardware
 *          module which has already been opened.
 */
static void UARTCC26XX_initHw(UART_Handle handle) {
    UARTCC26XX_Object *object;
    UARTCC26XX_HWAttrs const *hwAttrs;
    Types_FreqHz freq;
    uint32_t rxFifoThreshold[] = {
        UART_FIFO_RX1_8,
        UART_FIFO_RX2_8,
        UART_FIFO_RX4_8,
        UART_FIFO_RX6_8,
        UART_FIFO_RX7_8
    };

    /* Get the pointer to the object and hwAttrs */
    object = handle->object;
    hwAttrs = handle->hwAttrs;

    /* Disable UART function. */
    UARTDisable(hwAttrs->baseAddr);

    /* Disable all UART module interrupts. */
    UARTIntDisable(hwAttrs->baseAddr, UART_INT_OE | UART_INT_BE | UART_INT_PE |
                                      UART_INT_FE | UART_INT_RT | UART_INT_TX |
                                      UART_INT_RX | UART_INT_CTS);

    /* Clear all UART interrupts */
    UARTIntClear(hwAttrs->baseAddr, UART_INT_OE | UART_INT_BE | UART_INT_PE |
                                    UART_INT_FE | UART_INT_RT | UART_INT_TX |
                                    UART_INT_RX | UART_INT_CTS);

    /* Set the FIFO level to 7/8 empty and 4/8 full. The setting was initially
     * 7/8 full, but has been changed to 4/8 full. Consider implementing the
     * FIFO levels as parameters in struct.
     */
    UARTFIFOLevelSet(hwAttrs->baseAddr, object->writeFifoThreshold, rxFifoThreshold[(object->readFifoThreshold/8)]);

    /* Configure frame format and baudrate */
    BIOS_getCpuFreq(&freq);
    UARTConfigSetExpClk(hwAttrs->baseAddr,
                        freq.lo,
                        object->baudRate,
                       (dataLength[object->dataLength] |
                        stopBits[object->stopBits] |
                        parityType[object->parityType]));

    Log_print3(Diags_USER1, "UART:(%p) CPU freq: %d; UART baudrate to %d",
                                hwAttrs->baseAddr,
                                freq.lo,
                                object->baudRate);

    /* Enable UART FIFOs */
    HWREG(UART0_BASE + UART_O_LCRH) |= UART_LCRH_FEN;

    /* Enable the UART module */
    HWREG(UART0_BASE + UART_O_CTL) |= UART_CTL_UARTEN;

    /* If Flow Control is enabled, configure hardware controlled flow control */
    if(isFlowControlEnabled(hwAttrs)) {
        HWREG(UART0_BASE + UART_O_CTL) |= (UART_CTL_CTSEN | UART_CTL_RTSEN);
    }
}

/*
 *  ======== UARTCC26XX_initIO ========
 *  This functions initializes the UART IOs.
 *
 *  @pre    Function assumes that the UART handle is pointing to a hardware
 *          module which has already been opened.
 */
static bool UARTCC26XX_initIO(UART_Handle handle) {
    /* Locals */
    UARTCC26XX_HWAttrs const *hwAttrs;
    PIN_Config uartPinTable[5];
    uint32_t i = 0;

    /* Get the pointer to the object and hwAttrs */
    hwAttrs = handle->hwAttrs;

    /* Build local list of pins, allocate through PIN driver and map HW ports */
    uartPinTable[i++] = hwAttrs->rxPin | PIN_INPUT_EN;
    /* Make sure UART_TX pin is driven high after calling PIN_open(...) until
    *  we've set the correct peripheral muxing in PINCC26XX_setMux(...)
    *  This is to avoid falling edge glitches when configuring the UART_TX pin.
    */
    uartPinTable[i++] = hwAttrs->txPin | PIN_INPUT_DIS | PIN_PUSHPULL | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH;
    if(isFlowControlEnabled(hwAttrs)) {
        uartPinTable[i++] = hwAttrs->ctsPin | PIN_INPUT_EN;
        /* Avoiding glitches on the RTS, see comment for TX pin above. */
        uartPinTable[i++] = hwAttrs->rtsPin | PIN_INPUT_DIS | PIN_PUSHPULL | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH;
    }
    /* Terminate pin list */
    uartPinTable[i++] = PIN_TERMINATE;

    /* Open and assign pins through pin driver */
    hPin = PIN_open(&pinState, uartPinTable);

    /* Are pins already allocated */
    if (!hPin) {
        return false;
    }

    /* Set IO muxing for the UART pins */
    PINCC26XX_setMux(hPin, hwAttrs->rxPin, IOC_PORT_MCU_UART0_RX);
    PINCC26XX_setMux(hPin, hwAttrs->txPin, IOC_PORT_MCU_UART0_TX);
    if(isFlowControlEnabled(hwAttrs)) {
        PINCC26XX_setMux(hPin, hwAttrs->ctsPin, IOC_PORT_MCU_UART0_CTS);
        PINCC26XX_setMux(hPin, hwAttrs->rtsPin, IOC_PORT_MCU_UART0_RTS);
    }
    /* Success */
    return true;
}

/*
 *  ======== uartPostNotify ========
 *  This functions is called to notify the UART driver of an ongoing transition
 *  out of sleep mode.
 *
 *  @pre    Function assumes that the UART handle (clientArg) is pointing to a
 *          hardware module which has already been opened.
 */
Power_NotifyResponse uartPostNotify(Power_Event eventType, uint32_t clientArg)
{
    /* Reconfigure the hardware if returning from sleep */
    if(eventType == Power_AWAKE_STANDBY) {
        UARTCC26XX_initHw((UART_Handle) clientArg);
    }
    return Power_NOTIFYDONE;
}
