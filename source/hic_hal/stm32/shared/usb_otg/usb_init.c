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

#include "cmsis_os2.h" // osDelay()
#include "util.h" // util_assert()

#include "usb_init.h"
#include "usbd_otg.h"
#include "cmsis_compiler.h"

__WEAK void otg_usb_clocks_init()
{
#if defined(USB_OTG_USE_FS)
    /* Enable USB FS Clock */
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
#elif defined(USB_OTG_USE_HS)
#if defined(USB_OTG_HS_USE_EMBEDDED_HS_PHY)
    __HAL_RCC_OTGPHYC_CLK_ENABLE();
    __HAL_RCC_USB_OTG_HS_ULPI_CLK_ENABLE();
    __HAL_RCC_USB_OTG_HS_CLK_ENABLE();
#elif defined(USB_OTG_HS_USE_EXTERNAL_HS_PHY)         // External ULPI High-speed PHY
    __HAL_RCC_USB_OTG_HS_CLK_ENABLE();
    __HAL_RCC_USB_OTG_HS_ULPI_CLK_ENABLE();           // Enable USB HS ULPI Clocks
#elif defined(USB_OTG_HS_USE_EMBEDDED_FS_PHY)         // Internal Full-speed PHY
    __HAL_RCC_USB_OTG_HS_CLK_ENABLE();
#endif
#endif
}

__WEAK void otg_usb_reset()
{
    OTG->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;             // Disable USB Controller Global Interrupt

#if defined(USB_OTG_USE_FS)
    __HAL_RCC_USB_OTG_FS_FORCE_RESET();                // Reset OTG FS module
    osDelay(1U);
    __HAL_RCC_USB_OTG_FS_RELEASE_RESET();              // Clear reset of OTG FS module
    osDelay(1U);
#elif defined(USB_OTG_USE_HS)
    __HAL_RCC_USB_OTG_HS_FORCE_RESET();                // Reset OTG HS module
    osDelay(1U);
    __HAL_RCC_USB_OTG_HS_RELEASE_RESET();              // Clear reset of OTG HS module
    osDelay(1U);
#endif
}

__WEAK void otg_usb_reset_core()
{
    uint32_t ticks;

    // Wait until AHB Master state machine is in the idle condition, max 100 ms
    ticks = 10U;
    while ((OTG->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U) {
        if (ticks == 0U) {
            break;
        }
        ticks--;
        osDelay(1U);
    }

    OTG->GRSTCTL  |=  USB_OTG_GRSTCTL_CSRST;           // Core soft reset

    // Wait Core reset to finish, max 100 ms
    ticks = 10U;
    while ((OTG->GRSTCTL & USB_OTG_GRSTCTL_CSRST) != 0U) {
        if (ticks == 0U) {
            break;
        }
        ticks--;
        osDelay(1U);
    }
    osDelay(1U);

    // Wait until AHB Master state machine is in the idle condition, max 100 ms
    ticks = 10U;
    while ((OTG->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U) {
        if (ticks == 0U) {
            break;
        }
        ticks--;
        osDelay(1U);
    }
}

__WEAK void otg_usb_phy_init()
{
#if defined(USB_OTG_USE_FS)
    OTG->GUSBCFG  |=  USB_OTG_GUSBCFG_PHYSEL;          // Full-speed transceiver
#elif defined(USB_OTG_USE_HS)
#if defined(USB_OTG_HS_USE_EMBEDDED_HS_PHY)            // Internal High-speed PHY
    OTG->GCCFG &= ~(USB_OTG_GCCFG_PWRDWN);
    /* Init The UTMI Interface */
    OTG->GUSBCFG &= ~(USB_OTG_GUSBCFG_TSDPS | USB_OTG_GUSBCFG_ULPIFSLS | USB_OTG_GUSBCFG_PHYSEL);
    /* Select vbus source */
    OTG->GUSBCFG &= ~(USB_OTG_GUSBCFG_ULPIEVBUSD | USB_OTG_GUSBCFG_ULPIEVBUSI);
    /* Select UTMI Interface */
    OTG->GUSBCFG &= ~USB_OTG_GUSBCFG_ULPI_UTMI_SEL;
    OTG->GCCFG |= USB_OTG_GCCFG_PHYHSEN;

    USB_HS_PHYC->USB_HS_PHYC_LDO |= USB_HS_PHYC_LDO_ENABLE;
    /* wait for LDO Ready */
    uint32_t count = 0U;
    while ((USB_HS_PHYC->USB_HS_PHYC_LDO & USB_HS_PHYC_LDO_STATUS) == 0U) {
        if (++count > 200000U) {
            util_assert(0);
        }
    }
    /* Controls PHY frequency operation selection */
    if (HSE_VALUE == 12000000U) { /* HSE = 12MHz */
        USB_HS_PHYC->USB_HS_PHYC_PLL = (0x0U << 1);
    } else if (HSE_VALUE == 12500000U) { /* HSE = 12.5MHz */
        USB_HS_PHYC->USB_HS_PHYC_PLL = (0x2U << 1);
    } else if (HSE_VALUE == 16000000U) { /* HSE = 16MHz */
        USB_HS_PHYC->USB_HS_PHYC_PLL = (0x3U << 1);
    } else if (HSE_VALUE == 24000000U) { /* HSE = 24MHz */
        USB_HS_PHYC->USB_HS_PHYC_PLL = (0x4U << 1);
    } else if (HSE_VALUE == 25000000U) { /* HSE = 25MHz */
        USB_HS_PHYC->USB_HS_PHYC_PLL = (0x5U << 1);
    } else if (HSE_VALUE == 32000000U) { /* HSE = 32MHz */
        USB_HS_PHYC->USB_HS_PHYC_PLL = (0x7U << 1);
    }

    /* Control the tuning interface of the High Speed PHY */
    USB_HS_PHYC->USB_HS_PHYC_TUNE |= USB_HS_PHYC_TUNE_VALUE;

    /* Enable PLL internal PHY */
    USB_HS_PHYC->USB_HS_PHYC_PLL |= USB_HS_PHYC_PLL_PLLEN;

    /* 2ms Delay required to get internal phy clock stable */
    osDelay(1U);
#elif defined(USB_OTG_HS_USE_EMBEDDED_FS_PHY)          // Internal Full-speed PHY
    OTG->GUSBCFG  |= (USB_OTG_GUSBCFG_PHYSEL);         // Full-speed transceiver
#elif defined(USB_OTG_HS_USE_EXTERNAL_HS_PHY)          // External ULPI High-speed PHY
    OTG->GCCFG    &= ~(USB_OTG_GCCFG_PWRDWN);          // Power down Full-speed transceiver
    OTG->GUSBCFG  &= ~(USB_OTG_GUSBCFG_TSDPS      |    // Data line pulsing using utmi_txvalid (default)
                       USB_OTG_GUSBCFG_ULPIFSLS   |    // ULPI interface
                       USB_OTG_GUSBCFG_PHYSEL);        // High-speed transceiver
#if defined(STM32F722xx) || defined(STM32F723xx) || defined(STM32F730xx) || defined(STM32F732xx) || defined(STM32F733xx)
    OTG->GUSBCFG |= USB_OTG_GUSBCFG_ULPI_UTMI_SEL;     // Select ULPI Interface
#endif
    OTG->GUSBCFG  &= ~(USB_OTG_GUSBCFG_ULPIEVBUSD | USB_OTG_GUSBCFG_ULPIEVBUSI); // Select vbus source
#endif
#endif

    otg_usb_reset_core();

#if defined(USB_OTG_USE_FS) || defined(USB_OTG_HS_USE_EMBEDDED_FS_PHY)
    OTG->GCCFG    |=  USB_OTG_GCCFG_PWRDWN;            // Power up Full-speed PHY
#endif
}

__WEAK void otg_usb_configure()
{
#if defined(USB_OTG_USE_FS) || defined(USB_OTG_SET_FS_SPEED)
    // Turnaround timeout depends on the AHB clock dictated by STM32 Reference Manual
    uint32_t turnaround = 0xF;

    if ( SystemCoreClock >= 32000000u ) {
        turnaround = 0x6u;
    } else if ( SystemCoreClock >= 27500000u ) {
        turnaround = 0x7u;
    } else if ( SystemCoreClock >= 24000000u ) {
        turnaround = 0x8u;
    } else if ( SystemCoreClock >= 21800000u ) {
        turnaround = 0x9u;
    } else if ( SystemCoreClock >= 20000000u ) {
        turnaround = 0xAu;
    } else if ( SystemCoreClock >= 18500000u ) {
        turnaround = 0xBu;
    } else if ( SystemCoreClock >= 17200000u ) {
        turnaround = 0xCu;
    } else if ( SystemCoreClock >= 16000000u ) {
        turnaround = 0xDu;
    } else if ( SystemCoreClock >= 15000000u ) {
        turnaround = 0xEu;
    }

    // Set turnaround time
    OTG->GUSBCFG   = ((OTG->GUSBCFG & ~USB_OTG_GUSBCFG_TRDT_Msk) |
                     (((uint32_t)((turnaround) << USB_OTG_GUSBCFG_TRDT_Pos)) & USB_OTG_GUSBCFG_TRDT_Msk) );
#endif

    // Set Device mode
    if (((OTG->GUSBCFG & USB_OTG_GUSBCFG_FDMOD) == 0U) || ((OTG->GUSBCFG & USB_OTG_GUSBCFG_FHMOD) != 0U)) {
        OTG->GUSBCFG &= ~USB_OTG_GUSBCFG_FHMOD;        // Clear force host mode
        OTG->GUSBCFG |=  USB_OTG_GUSBCFG_FDMOD;        // Force device mode
        osDelay(10U);
    }

#ifdef USB_OTG_GCCFG_NOVBUSSENS // STM32F4
    OTG->GCCFG    |=  USB_OTG_GCCFG_NOVBUSSENS;        // Disable VBUS sensing
    OTG->GCCFG    &= ~USB_OTG_GCCFG_VBUSBSEN;
    OTG->GCCFG    &= ~USB_OTG_GCCFG_VBUSASEN;
#else // STM32F7 and STM32H7
    OTG->GCCFG    &= ~USB_OTG_GCCFG_VBDEN;             // Disable VBUS sensing device "B"
    OTG->GOTGCTL  |= USB_OTG_GOTGCTL_BVALOEN;          // B-peripheral session valid override enable
    OTG->GOTGCTL  |= USB_OTG_GOTGCTL_BVALOVAL;
#endif

    OTG_PCGCCTL = 0U;
    OTG_DEV->DCFG |= 0; // DCFG_FRAME_INTERVAL_80
#if defined(USB_OTG_USE_FS)
    // Full Speed
    OTG_DEV->DCFG = (OTG_DEV->DCFG & ~USB_OTG_DCFG_DSPD_Msk) | USB_OTG_DCFG_DSPD;    // 0x3: Full
#elif defined(USB_OTG_USE_HS)
#if (defined(USB_OTG_HS_USE_EMBEDDED_HS_PHY) || defined(USB_OTG_HS_USE_EXTERNAL_HS_PHY))
    // High speed
    OTG_DEV->DCFG = (OTG_DEV->DCFG & ~USB_OTG_DCFG_DSPD_Msk);                        // 0x0: High
#elif defined(USB_OTG_HS_USE_EMBEDDED_FS_PHY)
    // High speed in Full speed
    OTG_DEV->DCFG =  (OTG_DEV->DCFG & ~USB_OTG_DCFG_DSPD_Msk) | USB_OTG_DCFG_DSPD_0; // 0x1: High in Full
#endif
#endif
}

__WEAK void otg_usb_enable()
{
    OTG_PCGCCTL = 0U;
    OTG->GINTMSK = 0U;
    OTG->GINTSTS = 0xBFFFFFFFU;
    OTG->GINTMSK  |= (USB_OTG_GINTMSK_USBSUSPM |       // Unmask interrupts
                      USB_OTG_GINTMSK_USBRST   |
                      USB_OTG_GINTMSK_ENUMDNEM |
                      USB_OTG_GINTMSK_RXFLVLM  |
                      USB_OTG_GINTMSK_IEPINT   |
                      USB_OTG_GINTMSK_OEPINT   |
                      USB_OTG_GINTMSK_WUIM     |
                      USB_OTG_GINTMSK_OTGINT);

    OTG->GAHBCFG  |= (USB_OTG_GAHBCFG_GINT);           // Enable interrupts
}
