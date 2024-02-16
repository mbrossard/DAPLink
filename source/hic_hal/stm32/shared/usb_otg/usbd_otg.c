/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2020 Arm Limited (or its affiliates). All rights reserved.
 * Copyright (c) 2022 Mathias Brossard <mathias@brossard.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdint.h>
#include <stdbool.h>
#include <rl_usb.h>
#include "cmsis_os2.h" // osDelay()
#include "util.h" // util_assert()

#include "usbd_otg.h"
#include "usb_otg.h"
#include "usb_init.h"

#define __NO_USB_LIB_C
#include "usb_config.c"

#if (USBD_EP_NUM > (USB_OTG_ENDPOINT_NUM + 1))
#error "USB Device does not support enough endpoints"
#endif

#if ((USBD_RX_FIFO_SIZE + USBD_FIFO_SIZE_SUM) > USB_OTG_BUFFER_SIZE)
#error "FIFO sizes too large for buffer"
#endif

// Endpoints runtime information
const uint32_t FifoTxSizes[USB_OTG_ENDPOINT_NUM + 1] = { USBD_FIFO_SIZE_LIST };
static uint32_t OutMaxPacketSize[USB_OTG_ENDPOINT_NUM + 1];
static uint8_t  OutPacketCnt[USB_OTG_ENDPOINT_NUM + 1];
static uint8_t  InPacketCnt[USB_OTG_ENDPOINT_NUM + 1];

// Auxiliary functions

/*
 *  USB Device Flush IN Endpoint Transmit Fifo
 *    Parameters:      adr:   USB Device Address
 *    Return Value:    None
 */

static void USBD_FlushInEpFifo(uint32_t EPNum)
{
    // Wait, with timeout, for flush to end
    uint32_t tout_cnt = SystemCoreClock / 64U;
    while ((OTG->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) != 0U) {
        if (tout_cnt-- == 0U) { return; }
    }

    OTG->GRSTCTL  = (OTG->GRSTCTL & ~USB_OTG_GRSTCTL_TXFNUM_Msk) |
        (((uint32_t) EPNum <<  USB_OTG_GRSTCTL_TXFNUM_Pos) & USB_OTG_GRSTCTL_TXFNUM_Msk);
    OTG->GRSTCTL |= USB_OTG_GRSTCTL_TXFFLSH;

    // Wait, with timeout, for flush to end
    tout_cnt = SystemCoreClock / 64U;
    while ((OTG->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) != 0U) {
        if (tout_cnt-- == 0U) { return; }
    }
}


/*
 *  Set Endpoint for next read
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

static void USBD_EndpointReadSet(uint8_t ep_addr)
{
    uint8_t ep_num = EP_NUM(ep_addr);

    if (ep_num == 0) {
        OTG_DOEPTSIZ(ep_num) = (OutPacketCnt[ep_num] << USB_OTG_DOEPTSIZ_PKTCNT_Pos )  |
                               (1U << USB_OTG_DOEPTSIZ_STUPCNT_Pos ) |
                               (OutMaxPacketSize[ep_num])                              ;
    } else {
        OTG_DOEPTSIZ(ep_num) = (OutPacketCnt[ep_num] << USB_OTG_DOEPTSIZ_PKTCNT_Pos )  |
                               (OutMaxPacketSize[ep_num])                              ;
    }

    // Clear NAK and enable Endpoint
    OTG_DOEPCTL(ep_num) |= USB_OTG_DOEPCTL_EPENA | USB_OTG_DOEPCTL_CNAK;
}


// USBD Driver functions

/*
 *  USB Device Reset Function
 *   Called automatically on USB Device Reset
 *    Return Value:    None
 */

void USBD_Reset(void)
{
    // Clear Endpoint mask registers
    OTG_DEV->DOEPMSK = 0U;
    OTG_DEV->DIEPMSK = 0U;

    for (uint8_t i = 1U; i <= USB_OTG_ENDPOINT_NUM; i++) {
        // Endpoint set NAK
        uint32_t epctl = USB_OTG_DOEPCTL_SNAK;
        if ((OTG_DOEPCTL(i) & USB_OTG_DOEPCTL_EPENA) != 0U) {
            // Disable enabled Endpoint
            epctl |= USB_OTG_DOEPCTL_EPDIS;
        }
        OTG_DOEPCTL(i) = epctl;

        // Endpoint set NAK
        epctl = USB_OTG_DIEPCTL_SNAK;
        if ((OTG_DIEPCTL(i) & USB_OTG_DIEPCTL_EPENA) != 0U) {
            // Disable enabled Endpoint
            epctl |= USB_OTG_DIEPCTL_EPDIS;
        }
        OTG_DIEPCTL(i) = epctl;

        // Clear IN Endpoint interrupts
        OTG_DIEPINT(i) = USB_OTG_DIEPINT_XFRC    |
                         USB_OTG_DIEPINT_EPDISD  |
                         USB_OTG_DIEPINT_TOC     |
                         USB_OTG_DIEPINT_ITTXFE  |
                         USB_OTG_DIEPINT_INEPNE  |
                         USB_OTG_DIEPINT_TXFE    ;

        // Clear OUT Endpoint interrupts
        OTG_DOEPINT(i) = USB_OTG_DOEPINT_XFRC    |
                         USB_OTG_DOEPINT_EPDISD  |
                         USB_OTG_DOEPINT_STUP    |
                         USB_OTG_DOEPINT_OTEPDIS |
                         USB_OTG_DOEPINT_B2BSTUP ;
    }

    // Flush all IN Endpoint FIFOs
    USBD_FlushInEpFifo (0x10U);

    // Set device address to 0
    OTG_DEV->DCFG     = (OTG_DEV->DCFG & ~USB_OTG_DCFG_DAD_Msk);
    OTG_DEV->DAINTMSK =  OTG_DAINT_IEPINT(0U)    |  // Enable IN Endpoint0 interrupt
                         OTG_DAINT_OEPINT(0U)    ;  // Enable OUT Endpoint0 interrupt

    // Enable Setup phase done, OUT Endpoint disabled and OUT transfer complete interrupt
    OTG_DEV->DOEPMSK  =  USB_OTG_DOEPMSK_STUPM   |
                         USB_OTG_DOEPMSK_EPDM    |
                         USB_OTG_DOEPMSK_XFRCM   ;

    // Enable In Endpoint disable and IN transfer complete interrupt
    OTG_DEV->DIEPMSK  =  USB_OTG_DIEPMSK_EPDM    |
                         USB_OTG_DIEPMSK_XFRCM   ;

    // Configure FIFOs
    uint32_t offset   =  USBD_RX_FIFO_SIZE / 4U;
    OTG->GRXFSIZ      =  offset;
    for (uint32_t i = 0; i <= USB_OTG_ENDPOINT_NUM; i++) {
        uint32_t current = FifoTxSizes[i] / 4U;
        OTG->DIEPTXF[i] = offset | (current << USB_OTG_DIEPTXF_INEPTXFD_Pos);
        offset += current;
    }

    OTG_DIEPTSIZ(0) = USB_OTG_DOEPTSIZ_STUPCNT_0            |  // Setup Count
                      (1 << USB_OTG_DOEPTSIZ_PKTCNT_Pos)    |  // Packet Count
                      USBD_MAX_PACKET0                      ;  // Transfer Size

    OutMaxPacketSize[0] = USBD_MAX_PACKET0;
    OutPacketCnt[0] = 1;
    InPacketCnt[0] = 1;
}

/*
 *  USB Device Initialize Function
 *   Called by the User to initialize USB
 *    Return Value:    None
 */

void USBD_Init(void)
{
    otg_usb_gpio_init();                               // Initialize GPIO pins
    otg_usb_clocks_init();                             // Initialize Clocks
    USBD_Connect(0);                                   // Soft disconnect enabled
    otg_usb_reset();                                   // Force reset USB module
    otg_usb_phy_init();                                // Initialize PHY
    USBD_Reset();                                      // Reset variables and endpoint settings
    otg_usb_configure();                               // Set turnaround time, device mode, vbus, speed
    otg_usb_enable();                                  // Clear/Unmask/Emable interrupts
    NVIC_EnableIRQ(OTG_IRQn);                          // Enable OTG interrupt
    USBD_Connect(1);                                   // Soft disconnect disabled
}

/*
 *  USB Device Connect Function
 *   Called by the User to Connect/Disconnect USB Device
 *    Parameters:      con:   Connect/Disconnect
 *    Return Value:    None
 */

void USBD_Connect(BOOL con)
{
    if (con) {
        OTG_DEV->DCTL &= ~USB_OTG_DCTL_SDIS;      // Soft disconnect disabled
        // OTG->GCCFG    |=  USB_OTG_GCCFG_PWRDWN;   // Power up on-chip PHY
    } else {
        OTG_DEV->DCTL |=  USB_OTG_DCTL_SDIS;      // Soft disconnect enabled
        // OTG->GCCFG    &= ~USB_OTG_GCCFG_PWRDWN;   // Power down on-chip PHY
    }
}


/*
 *  USB Device Suspend Function
 *   Called automatically on USB Device Suspend
 *    Return Value:    None
 */

void USBD_Suspend(void)
{
}


/*
 *  USB Device Resume Function
 *   Called automatically on USB Device Resume
 *    Return Value:    None
 */

void USBD_Resume(void)
{
}


/*
 *  USB Device Remote Wakeup Function
 *   Called automatically on USB Device Remote Wakeup
 *    Return Value:    None
 */

void USBD_WakeUp(void)
{
    OTG_DEV->DCTL |=  USB_OTG_DCTL_RWUSIG; // Remote wakeup signaling
    osDelay(1U);
    OTG_DEV->DCTL &= ~USB_OTG_DCTL_RWUSIG;
}


/*
 *  USB Device Remote Wakeup Configuration Function
 *    Parameters:      cfg:   Device Enable/Disable
 *    Return Value:    None
 */

void USBD_WakeUpCfg(BOOL cfg)
{
    /* Not needed                                                               */
}


/*
 *  USB Device Set Address Function
 *    Parameters:      adr:   USB Device Address
 *                     setup: Called in setup stage (!=0), else after status stage
 *    Return Value:    None
 */

void USBD_SetAddress(U32 adr, U32 setup)
{
    if (setup) {
        OTG_DEV->DCFG = (OTG_DEV->DCFG & ~USB_OTG_DCFG_DAD_Msk) |
            ((adr << USB_OTG_DCFG_DAD_Pos) & USB_OTG_DCFG_DAD_Msk);
    }
}


/*
 *  USB Device Configure Function
 *    Parameters:      cfg:   Device Configure/Deconfigure
 *    Return Value:    None
 */

void USBD_Configure (BOOL cfg) {
}


/*
 *  Configure USB Device Endpoint according to Descriptor
 *    Parameters:      pEPD:  Pointer to Device Endpoint Descriptor
 *    Return Value:    None
 */

void USBD_ConfigEP (USB_ENDPOINT_DESCRIPTOR *pEPD) {
    uint16_t ep_mps = pEPD->wMaxPacketSize;
    uint8_t ep_type = pEPD->bmAttributes & USB_ENDPOINT_TYPE_MASK;
    uint8_t ep_num = EP_NUM(pEPD->bEndpointAddress);
    bool ep_dir = EP_DIR(pEPD->bEndpointAddress);

    if ((ep_num > USB_OTG_ENDPOINT_NUM) ||
        // IN endpoint
        ((ep_dir) && ep_mps > FifoTxSizes[ep_num]) ||
        // OUT endpoint
        ((!ep_dir) && (ep_mps + 60U + (8 * USB_OTG_ENDPOINT_NUM)) > USBD_RX_FIFO_SIZE)) {
        return;
    }

    if (ep_dir != 0U) {  // IN Endpoint
        InPacketCnt[ep_num] = 1;

        // Configure IN Endpoint
        OTG_DIEPCTL(ep_num) = ((uint32_t)ep_num  <<  USB_OTG_DIEPCTL_TXFNUM_Pos) |  // FIFO Number
                              ((uint32_t)ep_type <<  USB_OTG_DIEPCTL_EPTYP_Pos ) |  // Endpoint Type
                               ep_mps                                            ;  // Max Packet Size

        if ((ep_type == USB_ENDPOINT_TYPE_BULK) || (ep_type == USB_ENDPOINT_TYPE_INTERRUPT)) {
            // Set DATA0 PID for Interrupt or Bulk Endpoint
            OTG_DIEPCTL(ep_num) |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
        }

        OTG_DIEPCTL(ep_num) |= USB_OTG_DIEPCTL_USBAEP |      // Activate Endpoint
                               USB_OTG_DIEPCTL_SNAK   ;

        if ((OTG_DIEPCTL(ep_num) & USB_OTG_DIEPCTL_EPENA) != 0U) {
            OTG_DIEPCTL(ep_num) |= USB_OTG_DIEPCTL_EPDIS;    // Disable Endpoint
        }

        OTG_DEV->DAINTMSK |= OTG_DAINT_IEPINT(ep_num);       // Enable Endpoint interrupt
    } else {  // OUT Endpoint
        OutMaxPacketSize[ep_num] = ep_mps;
        OutPacketCnt[ep_num] = 1;

        // Configure OUT Endpoint
        OTG_DOEPCTL(ep_num) = ((uint32_t)ep_type <<  USB_OTG_DOEPCTL_EPTYP_Pos) |   // Endpoint Type
                               ep_mps                                           ;   // Max Packet Size
        OTG_DIEPTSIZ(ep_num) = (OutPacketCnt[ep_num] << USB_OTG_DOEPTSIZ_PKTCNT_Pos) |  // Packet Count
                                ep_mps                                               ;  // Transfer Size
        if ((ep_type == USB_ENDPOINT_TYPE_BULK) || (ep_type == USB_ENDPOINT_TYPE_INTERRUPT)) {
            // Set DATA0 PID for Interrupt or Bulk Endpoint
            OTG_DOEPCTL(ep_num) |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
        }

        OTG_DOEPCTL(ep_num) |= USB_OTG_DOEPCTL_USBAEP |      // Activate Endpoint
                               USB_OTG_DOEPCTL_CNAK   |
                               USB_OTG_DOEPCTL_EPENA  ;

        OTG_DEV->DAINTMSK |= OTG_DAINT_OEPINT(ep_num);      // Enable Endpoint interrupt
    }
}


/*
 *  Set Direction for USB Device Control Endpoint
 *    Parameters:      dir:   Out (dir == 0), In (dir <> 0)
 *    Return Value:    None
 */

void USBD_DirCtrlEP(uint32_t dir) {
    /* Nothing to do */
}


/*
 *  Enable USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_EnableEP(uint32_t EPNum) {
    // Nothing to do
}


/*
 *  Set Stall for USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_SetStallEP(U32 EPNum)
{
    uint8_t ep_num = EP_NUM(EPNum);
    bool ep_dir = EP_DIR(EPNum);

    if (ep_dir) { // IN Endpoint
        if ((OTG_DIEPCTL(ep_num) & USB_OTG_DIEPCTL_EPENA) != 0U) {
            // Set flush flag to Flush IN FIFO in Endpoint disabled interrupt
            OTG_DIEPCTL(ep_num)  |= USB_OTG_DIEPCTL_STALL | USB_OTG_DIEPCTL_EPDIS;
        } else {
            OTG_DIEPCTL(ep_num)  |= USB_OTG_DIEPCTL_STALL;
            USBD_FlushInEpFifo (ep_num);
        }
    } else { // OUT Endpoint
        OTG_DEV->DCTL |= USB_OTG_DCTL_SGONAK;                  // Set Global OUT NAK

        // Wait, with timeout, while Global OUT NAK effective is not set
        uint32_t tout_cnt = SystemCoreClock / 64U;
        while ((OTG->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) == 0U) {
            if (tout_cnt-- == 0U) { break; }
        }

        // Stall OUT Endpoint
        if ((OTG_DOEPCTL(ep_num) & USB_OTG_DOEPCTL_EPENA) != 0U) {
            OTG_DOEPCTL(ep_num)  |= USB_OTG_DOEPCTL_STALL | USB_OTG_DOEPCTL_EPDIS;
        } else {
            OTG_DOEPCTL(ep_num)  |= USB_OTG_DOEPCTL_STALL;
        }

        OTG_DEV->DCTL |= USB_OTG_DCTL_CGONAK;                  // Clear global NAK
    }
}

/*
 *  Clear Stall for USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ClrStallEP(U32 EPNum)
{
    uint8_t ep_num = EP_NUM(EPNum);
    bool ep_dir = EP_DIR(EPNum);

    if (ep_dir) {                                 // IN Endpoint
        if ((OTG_DIEPCTL(ep_num) & USB_OTG_DIEPCTL_EPENA) != 0U) {  // If Endpoint enabled
            OTG_DIEPCTL(ep_num) |= USB_OTG_DIEPCTL_EPDIS; // Disable Endpoint
        }

        // Set DATA0 PID for Interrupt and Bulk Endpoint
        if (((OTG_DIEPCTL(ep_num) & USB_OTG_DIEPCTL_EPTYP_Msk) >> USB_OTG_DIEPCTL_EPTYP_Pos) > 1U) {
            OTG_DIEPCTL(ep_num) |= USB_OTG_DIEPCTL_SD0PID_SEVNFRM;
        }

        OTG_DIEPCTL(ep_num) &= ~USB_OTG_DIEPCTL_STALL;    // Clear Stall
    } else {                                            // Clear OUT Endpoint stall
        // Set DATA0 PID for Interrupt and Bulk Endpoint
        if (((OTG_DOEPCTL(ep_num) & USB_OTG_DOEPCTL_EPTYP_Msk) >> USB_OTG_DOEPCTL_EPTYP_Pos) > 1U) {
            OTG_DOEPCTL(ep_num) |= USB_OTG_DOEPCTL_SD0PID_SEVNFRM;
        }
        OTG_DOEPCTL(ep_num) &= ~USB_OTG_DOEPCTL_STALL;    // Clear Stall
    }
}


/*
 *  Read USB Device Endpoint Data
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *                     pData: Pointer to Data Buffer
 *    Return Value:    Number of bytes read
 */

U32 USBD_ReadEP(U32 EPNum, U8 *pData, U32 bufsz)
{
    uint8_t *ptr_dest = pData;
    uint32_t i, j, residue;
    uint8_t ep_num = EP_NUM(EPNum);

    // Check if Endpoint is activated
    if ((OTG_DOEPCTL(ep_num) & USB_OTG_DOEPCTL_USBAEP) == 0U) { return 0U; }

    // Copy data from FIFO
    j = (OTG->GRXSTSP >> 4) & 0x7FFU;
    j = j < bufsz ? j : bufsz;
    i = j / 4U;
    while (i != 0U) {
        __UNALIGNED_UINT32_WRITE(ptr_dest, OTG_RX_FIFO);
        ptr_dest += 4U;
        i--;
    }

    residue = j % 4U;
    if (residue != 0U) {
        uint8_t tmp_buf[4];
        __UNALIGNED_UINT32_WRITE(tmp_buf, OTG_RX_FIFO);
        for (i = 0U; i < residue; i++) {
            *ptr_dest++ = tmp_buf[i];
        }
    }

    while ((OTG->GINTSTS & USB_OTG_GINTSTS_RXFLVL) == 0);
    OTG->GRXSTSP;
    OTG->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;

    return j;
}


/*
 *  Write USB Device Endpoint Data
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *                     pData: Pointer to Data Buffer
 *                     cnt:   Number of bytes to write
 *    Return Value:    Number of bytes written
 */

U32 USBD_WriteEP(U32 EPNum, U8 *pData, U32 cnt)
{
    uint8_t *ptr_src;
    uint32_t i, j, residue;
    uint8_t ep_num = EP_NUM(EPNum);

    // Check if Endpoint is activated
    if ((OTG_DIEPCTL(ep_num) & USB_OTG_DIEPCTL_USBAEP) == 0U) {
        return 0U;
    }

    // Check if enough space in FIFO
    if ((OTG_DTXFSTS(ep_num) * 4U) < cnt) { return 0; }

    // Set transfer size and packet count
    OTG_DIEPTSIZ(ep_num) = (InPacketCnt[ep_num] << USB_OTG_DIEPTSIZ_PKTCNT_Pos) |
                           (InPacketCnt[ep_num] << USB_OTG_DIEPTSIZ_MULCNT_Pos) |
                           cnt                                ;

    // Enable Endpoint and clear NAK
    OTG_DIEPCTL(ep_num) |= USB_OTG_DIEPCTL_EPENA | USB_OTG_DIEPCTL_CNAK;

    ptr_src  = pData;

    // Copy data to FIFO
    i = (cnt + 3U) >> 2;
    while (i != 0U) {
        OTG_TX_FIFO(ep_num) = __UNALIGNED_UINT32_READ(ptr_src);
        ptr_src += 4U;
        i--;
    }

    return cnt;
}


/*
 *  Get USB Device Last Frame Number
 *    Parameters:      None
 *    Return Value:    Frame Number
 */

uint32_t USBD_GetFrame (void)
{
    return ((OTG_DEV->DSTS & USB_OTG_DSTS_FNSOF_Msk) >> USB_OTG_DSTS_FNSOF_Pos);
}


/*
 *  Reset USB Device Endpoint
 *    Parameters:      EPNum: Device Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_ResetEP (uint32_t EPNum) {
    uint8_t ep_num = EP_NUM(EPNum);
    bool ep_dir = EP_DIR(EPNum);

    if (ep_dir) {                                           // IN Endpoint
        OTG_DIEPCTL(ep_num) |=  USB_OTG_DIEPCTL_CNAK;       // Clear NAK
        OTG_DIEPCTL(ep_num) |=  USB_OTG_DIEPCTL_SNAK;       // Set NAK
        OTG_DEV->DIEPMSK    |=  USB_OTG_DIEPMSK_INEPNEM;    // Enable NAK effective interrupt
    } else {                                                // OUT Endpoint
        USBD_EndpointReadSet(EPNum);
    }
}


/*
 *  Disable USB Endpoint
 *    Parameters:      EPNum: Endpoint Number
 *                       EPNum.0..3: Address
 *                       EPNum.7:    Dir
 *    Return Value:    None
 */

void USBD_DisableEP (uint32_t EPNum) {
    uint8_t ep_num = EP_NUM(EPNum);

    if (EP_DIR(EPNum)) {                                    // IN Endpoint
        if ((OTG_DIEPCTL(ep_num) & USB_OTG_DIEPCTL_EPENA) != 0U) {
            // Disable enabled Endpoint and set NAK
            OTG_DIEPCTL(ep_num) |= USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
        } else {
            // Endpoint is already disabled. Set NAK
            OTG_DIEPCTL(ep_num) |= USB_OTG_DIEPCTL_SNAK;

            // Flush IN EP FIFO
            USBD_FlushInEpFifo (ep_num);
        }
    } else {                                                // OUT Endpoint
        if ((OTG_DOEPCTL(ep_num) & USB_OTG_DOEPCTL_EPENA) != 0U) {
            // Disable enabled Endpoint and set NAK
            OTG_DOEPCTL(ep_num) |= USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK;
        } else {
            // Endpoint is already disabled. Set NAK
            OTG_DOEPCTL(ep_num) |= USB_OTG_DOEPCTL_SNAK;
        }
    }
}


/*
 *  USB Device Interrupt Service Routine
 */

#if defined(USB_OTG_USE_FS)
void OTG_FS_IRQHandler(void)
#elif defined(USB_OTG_USE_HS)
void OTG_HS_IRQHandler(void)
#endif
{
    NVIC_DisableIRQ(OTG_IRQn);
    USBD_SignalHandler();
}

void USBD_Handler(void)
{
    uint32_t             val, msk, ep_int;
    uint16_t             num;
    uint8_t              ep_num, i;

    uint32_t gintsts = OTG->GINTSTS & OTG->GINTMSK;

    if ((gintsts & USB_OTG_GINTSTS_USBRST) != 0U) {        // Reset interrupt
        USBD_Reset();
        usbd_reset_core();
        if (USBD_P_Reset_Event) {
            USBD_P_Reset_Event();
        }
        OTG->GINTSTS  =  USB_OTG_GINTSTS_USBRST;
    }

    if ((gintsts & USB_OTG_GINTSTS_USBSUSP) != 0U) {       // Suspend interrupt
        USBD_Suspend();
        if (USBD_P_Suspend_Event) {
            USBD_P_Suspend_Event();
        }
        OTG->GINTSTS  =  USB_OTG_GINTSTS_USBSUSP;
    }

    if ((gintsts & USB_OTG_GINTSTS_WKUINT) != 0U) {        // Resume interrupt
        USBD_Resume();
        if (USBD_P_Resume_Event) {
            USBD_P_Resume_Event();
        }
        OTG->GINTSTS  =  USB_OTG_GINTSTS_WKUINT;
    }

    if ((gintsts & USB_OTG_GINTSTS_ENUMDNE) != 0U) {       // Speed enumeration completed
        switch ((OTG_DEV->DSTS & USB_OTG_DSTS_ENUMSPD_Msk) >> USB_OTG_DSTS_ENUMSPD_Pos) {
          case 0:
            USBD_HighSpeed = __TRUE;
            break;
          case 1:
          case 3:
            USBD_HighSpeed = __FALSE;
            break;
        default:
             break;
        }
        OTG_DIEPCTL(0) &= ~USB_OTG_DIEPCTL_MPSIZ;
        OTG_DIEPCTL(0) |= OutMaxPacketSize[0];
        OTG_DEV->DCTL |= USB_OTG_DCTL_CGINAK;              // Clear global IN NAK
        OTG_DEV->DCTL |= USB_OTG_DCTL_CGONAK;              // Clear global OUT NAK
        OTG->GINTSTS   |= USB_OTG_GINTSTS_ENUMDNE;
    }

    if ((gintsts & USB_OTG_GINTSTS_SOF) != 0U) {           // First SOF after Reset
        OTG->GINTSTS  =  USB_OTG_GINTSTS_SOF;
        if (USBD_P_SOF_Event) {
            USBD_P_SOF_Event();
        }
    }

    if ((gintsts & USB_OTG_GINTSTS_RXFLVL) != 0U) {        // Receive FIFO interrupt
        val =  OTG->GRXSTSR;
        num =  val & USB_OTG_GRXSTSP_EPNUM;

        switch  ((val & USB_OTG_GRXSTSP_PKTSTS_Msk) >> USB_OTG_GRXSTSP_PKTSTS_Pos) {
        case 6:                                           // Setup packet
            if (USBD_P_EP[num]) {
                USBD_P_EP[num](USBD_EVT_SETUP);
            }
            break;
        case 2:                                           // OUT packet
            OTG->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;
            if (USBD_P_EP[num]) {
                USBD_P_EP[num](USBD_EVT_OUT);
            }
            break;
        case 3:                                           // OUT transfer completed
        case 4:                                           // SETUP transaction completed
            OTG->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
            OTG->GRXSTSP;
            break;
        case 1:                                           // Global OUT NAK
            OTG->GRXSTSP;
            break;

        default:
            OTG->GRXSTSP;
            break;
        }
    }

    // OUT Packet
    if ((gintsts & USB_OTG_GINTSTS_OEPINT) != 0U) {
        msk    = (((OTG_DEV->DAINT & OTG_DEV->DAINTMSK) >> 16) & 0xFFFFU);
        ep_num = 0U;
        do {
            if (((msk >> ep_num) & 1U) != 0U) {
                ep_int = OTG_DOEPINT(ep_num) & OTG_DEV->DOEPMSK;

                if ((ep_int & USB_OTG_DOEPINT_EPDISD) != 0U) {  // If Endpoint disabled
                    OTG_DOEPINT(ep_num) |= USB_OTG_DOEPINT_EPDISD;
                }

                // Transfer complete interrupt
                if (((ep_int & USB_OTG_DOEPINT_XFRC) | (ep_int & USB_OTG_DOEPINT_STUP)) != 0U) {
                    USBD_EndpointReadSet(ep_num);
                    OTG_DOEPINT(ep_num) |= USB_OTG_DOEPINT_XFRC;
                }
            }
            ep_num++;
        } while ((msk >> ep_num) != 0U);
    }

    // IN Packet
    if ((gintsts & USB_OTG_GINTSTS_IEPINT) != 0U) {
        msk    = (OTG_DEV->DAINT & OTG_DEV->DAINTMSK & 0xFFFFU);
        ep_num = 0U;
        do {
            if (((msk >> ep_num) & 1U) != 0U) {
                ep_int = OTG_DIEPINT(ep_num) & OTG_DEV->DIEPMSK;

                if ((ep_int & USB_OTG_DIEPINT_EPDISD) != 0U) {  // If Endpoint disabled
                    OTG_DIEPINT(ep_num) = USB_OTG_DIEPINT_EPDISD;
                }

                // IN Endpoint NAK effective
                if ((ep_int & USB_OTG_DIEPINT_INEPNE) != 0U) {
                    OTG_DOEPCTL(ep_num) |= USB_OTG_DIEPCTL_CNAK;
                    OTG_DIEPINT(ep_num) = USB_OTG_DIEPINT_INEPNE;
                }

                // Transmit completed
                if ((ep_int & USB_OTG_DIEPINT_XFRC) != 0U) {
                    OTG_DIEPINT(ep_num) = USB_OTG_DIEPINT_XFRC;
                    if (USBD_P_EP[ep_num]) {
                        USBD_P_EP[ep_num](USBD_EVT_IN);
                    }
                }
            }
            ep_num++;
        } while ((msk >> ep_num != 0U));
    }

    NVIC_EnableIRQ(OTG_IRQn);
}
