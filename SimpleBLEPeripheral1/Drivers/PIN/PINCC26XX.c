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


#include <inc/hw_memmap.h>
#include <inc/hw_aon_event.h>
#include <inc/hw_aon_ioc.h>
#include <inc/hw_aon_sysctl.h>
#include <inc/hw_gpio.h>
#include <inc/hw_ioc.h>
#include <inc/hw_ints.h>
#include <inc/hw_ccfg.h>
#include <driverlib/driverlib_release.h>
#include <driverlib/chipinfo.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include <ti/sysbios/family/arm/cc26xx/Power.h>
#include <ti/sysbios/family/arm/cc26xx/PowerCC2650.h>
#include <ti/drivers/pin/PINCC26XX.h>

/*!*****************************************************************************
 *  \file       PINCC26XX.c
 *  \brief      Device-specific PIN & GPIO driver for CC26xx family [impl]
 *
 *  # Overview #
 *  This is the device-specific implementation of the generic PIN driver for the
 *  CC26xx family of devices.
 *
 *******************************************************************************
 */

// Maximum number of pins (# available depends on package configuration)
#define MAX_NUM_PINS 31

/// Number of pins available on device
uint_t PIN_NumPins;

/// Array of handles, one per pin (pin id is index)
PIN_Handle PIN_HandleTable[MAX_NUM_PINS];

/// Pointer to GPIO configuration set by PIN_init(...), save state in order to revert when PIN_close(...)
static const PIN_Config *pPinGpio;

/// Array of indexes into GPIO configuration pPinGpio, one per pin (pin id is index)
static uint8_t PinGpioConfigTable[MAX_NUM_PINS];

/// HW interrupt structure for I/O interrupt handler
static ti_sysbios_family_arm_m3_Hwi_Struct PinHwi;

/// PIN driver semaphore used to implement synchronicity for PIN_open()
static Semaphore_Struct PinSem;



// I/O interrupt service routine
static void PIN_Isr(UArg arg) {
    uint32_t iEvent;

    // Get event flag with lowest index (also pin ID)
    iEvent = PIN_ctz(HWREG(GPIO_BASE+GPIO_O_EVFLAGS31_0));

    // Clear this event flag and clear CM3 interrupt
    HWREG(GPIO_NONBUF_BASE+GPIO_O_EVFLAGS31_0) = (1<<iEvent);
    HWREG(NVIC_UNPEND0) = 1<<(INT_EDGE_DETECT-16);

    PIN_Handle handle;
    PIN_IntCb  pCb;
    if (iEvent<PIN_NumPins) {
        if (handle=PIN_HandleTable[iEvent]) {
            if (pCb=handle->pCbFunc) {
                // Event from existing pin, with an associated handle and a
                // registered callback -> call callback
                pCb(handle, iEvent);
            }
        }
    }
    // Return from ISR after processing single event to yield to other ISRs at
    // same preemption level. Will not unfold stack if interrupt is pending again
}



// Internal utility function for setting IOCFG register for pin
static void PINCC26XX_setIoCfg(PIN_Config bmMask, PIN_Config pinCfg) {
    uint32_t dsCfg;
    PIN_Id pinId = PIN_ID(pinCfg);
    bool bInvChanges;

    if (pinCfg&PIN_GEN) {
        // Translate from device-independent to device-specific PIN_Config values
        pinCfg ^= PIN_GEN|PIN_BM_INPUT_EN|PIN_BM_PULLING;
    }
    // Get existing IOCFG, determine whether inversion changes, mask away what will be updated
    dsCfg = HWREG(IOC_BASE+IOC_O_IOCFG0+4*pinId);
    bInvChanges = (dsCfg^pinCfg)&bmMask&PINCC26XX_INV_INOUT;
    dsCfg &= ~bmMask;

    // Insert what we want to update, possibly revert IRQ edges, write back to IOCFG
    dsCfg |= (pinCfg&bmMask&PINCC26XX_BM_IOCFG);
    if ((bmMask&PINCC26XX_BM_IRQ)==PINCC26XX_BM_IRQ && (dsCfg&PINCC26XX_INV_INOUT)==0) {
        // We're changing IRQ options but inversion will not be enabled -> keep IRQ options
    } else if ((bmMask&PINCC26XX_BM_IRQ)==0 && !bInvChanges) {
        // We're not changing IRQ options and inversion remains unchanged -> keep IRQ options
    } else {
        // We're updating IRQ options and inversion will be enabled, OR
        // we're not updating IRQ options but inversion settings change
        // -> reverse polarity of edge detection when positive-only or negative-only
        switch (dsCfg&PINCC26XX_BM_IRQ) {
        case PINCC26XX_IRQ_POSEDGE:
            dsCfg &= ~PINCC26XX_BM_IRQ;
            dsCfg |= PINCC26XX_IRQ_NEGEDGE;
            break;
        case PINCC26XX_IRQ_NEGEDGE:
            dsCfg &= ~PINCC26XX_BM_IRQ;
            dsCfg |= PINCC26XX_IRQ_POSEDGE;
            break;
        default:
            break;
        }
    }
    HWREG(IOC_BASE+IOC_O_IOCFG0+4*pinId) = dsCfg;

    // Update GPIO output value and enable
    if (bmMask&PINCC26XX_BM_GPIO_OUTPUT_VAL) {
        // Set GPIO output value
        HWREGB(GPIO_BASE+GPIO_O_DOUT3_0+pinId) = (pinCfg&PINCC26XX_BM_GPIO_OUTPUT_VAL) ? 1 : 0;
    }
    if (bmMask&PINCC26XX_BM_GPIO_OUTPUT_EN) {
        // Set GPIO output enable
        uint32_t key = Hwi_disable();
            HWREG(GPIO_BASE+GPIO_O_DOE31_0) =
                (HWREG(GPIO_BASE+GPIO_O_DOE31_0) & ~(1<<pinId)) |
                ((pinCfg&PINCC26XX_BM_GPIO_OUTPUT_EN) ? (1<<pinId) : 0);
        Hwi_restore(key);
    }
}



// Internal utility function for setting mux setting in IOCFG register for pin
static void PINCC26XX_setIoCfgMux(PIN_Id pinId, int32_t nMux) {
    // Read in existing value in IOCFG register and update with supplied mux value
    if (nMux<0) {
        nMux = PINCC26XX_MUX_GPIO;
    }
    uint32_t dsCfg;
    dsCfg = HWREG(IOC_BASE+IOC_O_IOCFG0+4*pinId);
    dsCfg &= ~IOC_IOCFG0_PORT_ID_M;
    dsCfg |= nMux & IOC_IOCFG0_PORT_ID_M;
    HWREG(IOC_BASE+IOC_O_IOCFG0+4*pinId) = dsCfg;
}



PIN_Status PIN_init(const PIN_Config aPinCfg[]) {
    uint_t i;
    uint32_t bmInit;            // Works as long as # pins <=32
    Semaphore_Params semParams;
    Hwi_Params hwiParams;
    uint32_t bmReservedPins = 0;

    // Mark pins that aren't available up front
    if (ChipInfo_GetChipFamily()==FAMILY_CC13xx && ChipInfo_GetPackageType()==PACKAGE_7x7) {
        // CC13xx 7x7 QFN does not have IO #0
        bmReservedPins |= (1<<0);
    }
    if ((HWREG(CCFG_BASE+CCFG_O_MODE_CONF)&CCFG_MODE_CONF_SCLK_LF_OPTION_M)>>CCFG_MODE_CONF_SCLK_LF_OPTION_S == 0x1) {
       // An IO is used for LF clock input
      bmReservedPins |= (1<<((HWREG(CCFG_BASE+CCFG_O_EXT_LF_CLK)&CCFG_EXT_LF_CLK_DIO_M)>>CCFG_EXT_LF_CLK_DIO_S));
   }

    // Make sure we are using correct version of Driverlib
    DRIVERLIB_ASSERT_CURR_RELEASE();

    // Get number of pins available on device (from HW register)
    PIN_NumPins = (HWREG(IOC_BASE+0x0280)&0xFE)>>1;
    // HW register will report 30 DIOs for CC13xx 7x7 (which is correct).
    // However the PIN driver is full of checks to make sure PIN_ID is not out of range.
    // Add one for CC13xx, DIO_0 does not exist and has been reserved, so the rest
    // can safely assume that DIO_1 to DIO_30 (on 7x7 QFN) are available
    if(ChipInfo_GetChipFamily()==FAMILY_CC13xx)
        PIN_NumPins++;

    // Initialize table of pins that have default GPIO configuration
    for (i=0; i<PIN_NumPins; i++) {
        PinGpioConfigTable[i] = PIN_UNASSIGNED;
    }

    // Read in aPinCfg list and create bitmask of which IOs to initialize to
    // default values and which to initialize from aPinConfig
    for (i=bmInit=0; PIN_ID(aPinCfg[i])!=PIN_TERMINATE; i++) {
        // Check that pin exists and is available
        if (PIN_ID(aPinCfg[i])>=PIN_NumPins || bmReservedPins&(1<<PIN_ID(aPinCfg[i]))) {
            return PIN_NO_ACCESS;
        }
        // Mark pin as being in aPinConfig
        bmInit |= (1<<PIN_ID(aPinCfg[i]));
        // For quick reference, store index i in table
        PinGpioConfigTable[PIN_ID(aPinCfg[i])] = i;
    }

    // Set Power dependecies & constraints
    Power_setDependency(PERIPH_GPIO);

    // Save GPIO default setup
    pPinGpio = aPinCfg;

    // Setup semaphore for sequencing accesses to PIN_open()
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    Semaphore_construct(&PinSem, 1, &semParams);

    // Loop thru all pins and configure
    for (i=0; i<PIN_NumPins; i++) {
        if (bmReservedPins&(1<<i)) {
            // Pin is reserved for other purposes -> setup dummy handle
            PIN_HandleTable[i] = (PIN_State*)0x00000004;
        } else {
            if (bmInit&(1<<i)) {
                // Setup all pins in aPinCfg as instructed
                PINCC26XX_setIoCfg(PIN_BM_ALL, aPinCfg[PinGpioConfigTable[i]]);
            } else {
                // Setup all pins not in aPinCfg to default configuration:
                // GPIO, input buffer disable, GPIO output disable, low GPIO output, no pull, no IRQ, no wakeup
                PINCC26XX_setIoCfg(PIN_BM_ALL, PIN_ID(i)|PINCC26XX_NOPULL);
            }
            // Set pin as GPIO and clear pin handle
            PINCC26XX_setIoCfgMux(PIN_ID(i), -1);
            PIN_HandleTable[i] = NULL;
        }
    }

    // Setup HWI handler
    Hwi_Params_init(&hwiParams);
    Hwi_construct(&PinHwi, INT_EDGE_DETECT, PIN_Isr,&hwiParams, NULL);

    // Setup interrupts so that they wake up from standby (use MCU_WU1)
    HWREG(AON_EVENT_BASE+AON_EVENT_O_MCUWUSEL) =
        (HWREG(AON_EVENT_BASE+AON_EVENT_O_MCUWUSEL)&(~AON_EVENT_MCUWUSEL_WU1_EV_M)) |
        AON_EVENT_MCUWUSEL_WU1_EV_PAD;

    // Open latches out to I/Os
    // This might be unnecessary, but isn't when you start from debugger
    HWREG(AON_IOC_BASE+AON_IOC_O_IOCLATCH) = AON_IOC_IOCLATCH_EN;

    // If we boot from shutdown, the IOs are latched, this opens the latches again
    HWREG(AON_SYSCTL_BASE+AON_SYSCTL_O_SLEEPCTL) = AON_SYSCTL_SLEEPCTL_IO_PAD_SLEEP_DIS;

    return PIN_SUCCESS;
}



PIN_Handle PIN_open(PIN_State* pState, const PIN_Config aPinList[]) {
    uint_t i;
    bool bOk;
    uint32_t bmPort;
    PIN_Id pinId;

    // Ensure that only one client at a time can call PIN_open() or PIN_add()
    Semaphore_pend(Semaphore_handle(&PinSem), BIOS_WAIT_FOREVER);

        // Check whether all pins in aPinList are valid and available first
        for (i=0, bOk=true, bmPort=0; aPinList && (pinId=PIN_ID(aPinList[i]))!=PIN_TERMINATE; i++) {
            if (pinId>=PIN_NumPins || PIN_HandleTable[pinId]) {
                bOk = false;
                break;
            } else {
                // Generate bitmask for port operations (always one port on CC26xx)
                bmPort |= (1<<pinId);
            }
        }

        if (!bOk) {
            // Indicate that the pins were not allocatable
            pState = NULL;
        } else {
            // Setup state object
            pState->pCbFunc = NULL;
            pState->bmPort = 0;
            pState->userArg = 0;

            // Configure I/O pins according to aPinList
            for (i=0; aPinList && (pinId=PIN_ID(aPinList[i]))!=PIN_TERMINATE; i++) {
                PIN_HandleTable[pinId] = pState;
                pState->bmPort |= (1<<pinId);
                PIN_setConfig(pState, PIN_BM_ALL, aPinList[i]);
            }
        }

    Semaphore_post(Semaphore_handle(&PinSem));
    return pState;
}



PIN_Status PIN_add(PIN_Handle handle, PIN_Config pinCfg) {
    PIN_Status ret;
    PIN_Id pinId = PIN_ID(pinCfg);

    // Check that handle and pinId is valid
    if (!handle || pinId>=PIN_NumPins) {
        return PIN_NO_ACCESS;
    }

    // Ensure that only one client at a time can call PIN_open() or PIN_add()
    Semaphore_pend(Semaphore_handle(&PinSem), BIOS_WAIT_FOREVER);

        // Check whether pin is available
        if (PIN_HandleTable[pinId]) {
            // Pin already allocated -> do nothing
            ret = PIN_ALREADY_ALLOCATED;
        } else {
            // Allocate pin
            PIN_HandleTable[pinId] = handle;
            handle->bmPort |= (1<<pinId);
            PIN_setConfig(handle, PIN_BM_ALL, pinCfg);
            ret = PIN_SUCCESS;
        }

    Semaphore_post(Semaphore_handle(&PinSem));
    return ret;
}



PIN_Status PIN_remove(PIN_Handle handle, PIN_Id pinId) {
    if (handle && (handle->bmPort&(1<<pinId))) {
        // Deallocate pin
        handle->bmPort &= ~(1<<pinId);
        PIN_HandleTable[pinId] = NULL;
        // Find GPIO default value and revert to it
        if (PinGpioConfigTable[pinId]==PIN_UNASSIGNED) {
            // Revert pin to default configuration:
            // GPIO, input buffer disable, GPIO output disable, low GPIO output, no pull, no IRQ, no wakeup
            PINCC26XX_setIoCfg(PIN_BM_ALL, PIN_ID(pinId)|PIN_INPUT_DIS);
        } else {
            // Revert pin to previous GPIO configuration
            PINCC26XX_setIoCfg(PIN_BM_ALL, pPinGpio[PinGpioConfigTable[pinId]]);
        }
        // Revert to GPIO
        PINCC26XX_setIoCfgMux(PIN_ID(pinId), -1);
        return PIN_SUCCESS;
    } else {
        return PIN_NO_ACCESS;
    }
}



void PIN_close(PIN_Handle handle) {
    uint_t i;

    // No need for sequencing accesses to PIN_close()
    // For each pin in port bitmask
    while (handle->bmPort) {
        // Find lowest index pin
        i = PIN_ctz(handle->bmPort);
        // Deallocate pin
        PIN_remove(handle, i);
    }
}



uint_t PIN_getInputValue(PIN_Id pinId) {
    return PINCC26XX_getInputValue(pinId);
}



PIN_Status PIN_setOutputEnable(PIN_Handle handle, PIN_Id pinId, bool bOutEn) {
    if (PIN_CHKEN && (pinId>=PIN_NumPins || PIN_HandleTable[pinId]!=handle)) {
        // Non-existing pin or pin is not allocated to this client
        return PIN_NO_ACCESS;
    }
    PINCC26XX_setOutputEnable(pinId, bOutEn);
    return PIN_SUCCESS;
}



PIN_Status PIN_setOutputValue(PIN_Handle handle, PIN_Id pinId, uint_t val) {
    if (PIN_CHKEN && (pinId>=PIN_NumPins || PIN_HandleTable[pinId]!=handle)) {
        // Non-existing pin or pin is not allocated to this client
        return PIN_NO_ACCESS;
    }
    PINCC26XX_setOutputValue(pinId, val);
    return PIN_SUCCESS;
}



uint_t PIN_getOutputValue(PIN_Id pinId) {
    return PINCC26XX_getOutputValue(pinId);
}



PIN_Status PIN_setInterrupt(PIN_Handle handle, PIN_Config pinCfg) {
    if (PIN_CHKEN && (PIN_ID(pinCfg)>=PIN_NumPins || PIN_HandleTable[PIN_ID(pinCfg)]!=handle)) {
        // Non-existing pin or pin is not allocated to this client
        return PIN_NO_ACCESS;
    }
    PIN_setConfig(handle, PIN_BM_IRQ, pinCfg);
    return PIN_SUCCESS;
}



PIN_Status PIN_clrPendInterrupt(PIN_Handle handle, PIN_Id pinId) {
    if (PIN_CHKEN && (pinId>=PIN_NumPins || PIN_HandleTable[pinId]!=handle)) {
        // Non-existing pin or pin is not allocated to this client
        return PIN_NO_ACCESS;
    }
    PINCC26XX_clrPendInterrupt(pinId);
    return PIN_SUCCESS;
}



PIN_Status PIN_registerIntCb(PIN_Handle handle, PIN_IntCb pCb) {
    if (handle) {
        handle->pCbFunc = pCb;
        return PIN_SUCCESS;
    } else {
        return PIN_NO_ACCESS;
    }
}



PIN_Config PIN_getConfig(PIN_Id pinId) {
    // Translate from device-specific to device independent PIN_Config values
    return PIN_GEN|PINCC26XX_getConfig(pinId)^(PIN_BM_INPUT_EN|PIN_BM_PULLING);
}



PIN_Status PIN_setConfig(PIN_Handle handle, PIN_Config bmMask, PIN_Config pinCfg) {
    if (PIN_CHKEN && (PIN_ID(pinCfg)>=PIN_NumPins || PIN_HandleTable[PIN_ID(pinCfg)]!=handle)) {
        // Non-existing pin or pin is not allocated to this client
        return PIN_NO_ACCESS;
    }
    PINCC26XX_setIoCfg(bmMask, pinCfg);
    return PIN_SUCCESS;
}



PIN_Config PINCC26XX_getConfig(PIN_Id pinId) {
    // Get IOCFG register value and add in some extras:
    // * pinId
    // * pin GPIO output enable
    // * pin GPIO output value
    uint32_t dsCfg;

    dsCfg = HWREG(IOC_BASE+IOC_O_IOCFG0+4*pinId);
    dsCfg &= PINCC26XX_BM_IOCFG;
    dsCfg |= PIN_ID(pinId);
    dsCfg |= (PINCC26XX_getOutputValue(pinId)) ? PINCC26XX_GPIO_HIGH : PINCC26XX_GPIO_LOW;
    dsCfg |= (HWREG(GPIO_BASE+GPIO_O_DOE31_0)&(1<<pinId)) ? PINCC26XX_GPIO_OUTPUT_EN : 0;
    return dsCfg;
}



int_t PINCC26XX_getMux(PIN_Id pinId) {
    if (PIN_CHKEN && pinId>=PIN_NumPins) {
        // Non-existing pin
        return PIN_NO_ACCESS;
    }
    int32_t dsCfg;
    dsCfg = HWREG(IOC_BASE+IOC_O_IOCFG0+4*pinId);
    dsCfg &= IOC_IOCFG0_PORT_ID_M;
    if (dsCfg==PINCC26XX_MUX_GPIO) {
        dsCfg = -1;
    }
    return dsCfg;
}



PIN_Status PINCC26XX_setMux(PIN_Handle handle, PIN_Id pinId, int32_t nMux) {
    if (PIN_CHKEN && (pinId>=PIN_NumPins || PIN_HandleTable[pinId]!=handle)) {
        // Non-existing pin or pin is not allocated to this client
        return PIN_NO_ACCESS;
    }
    PINCC26XX_setIoCfgMux(pinId, nMux);
    return PIN_SUCCESS;
}



PIN_Status PINCC26XX_setWakeup(const PIN_Config aPinCfg[]) {
    uint_t i;
    // TODO: is this enough?

    for (i=0; PIN_ID(aPinCfg[i])!=PIN_TERMINATE; i++) {
        PINCC26XX_setIoCfg(PINCC26XX_BM_ALL, aPinCfg[i]);
    }
    return PIN_SUCCESS;
}



uint_t PIN_getPortMask(PIN_Handle handle) {
	// On CC26xx there is only one port encompassing all pins
    if (handle) {
        return handle->bmPort;
    } else {
        return 0;
    }
}



uint_t PIN_getPortInputValue(PIN_Handle handle) {
    return PINCC26XX_getPortInputValue(handle);
}



uint_t PIN_getPortOutputValue(PIN_Handle handle) {
    return PINCC26XX_getPortOutputValue(handle);
}



PIN_Status PIN_setPortOutputValue(PIN_Handle handle, uint_t bmOutVal) {
    if (PIN_CHKEN && (handle==NULL || PIN_HandleTable[PIN_ctz(handle->bmPort)]!=handle)) {
        return PIN_NO_ACCESS;
    }
    PINCC26XX_setPortOutputValue(handle, bmOutVal);
    return PIN_SUCCESS;
}



PIN_Status PIN_setPortOutputEnable(PIN_Handle handle, uint_t bmOutEn) {
    if (PIN_CHKEN && (handle==NULL || PIN_HandleTable[PIN_ctz(handle->bmPort)]!=handle)) {
        return PIN_NO_ACCESS;
    }
    PINCC26XX_setPortOutputEnable(handle, bmOutEn);
    return PIN_SUCCESS;
}
