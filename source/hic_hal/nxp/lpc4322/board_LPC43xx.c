/**
 * @file    board_LPC43xx.c
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "sdk.h"
#include "LPC43xx.h"

#define XTAL_OSC_HZ 12000000


static void wait_usec(uint32_t clock_hz, volatile uint32_t us)
{
    us *= (clock_hz / 1000000) / 3;

    while (us--);
}


static void clock_setup_base_m4(uint32_t clock_hz)
{
    uint32_t msel;
    uint32_t nsel;
    uint32_t tmp;

    // 0. Select IRC as BASE_M4_CLK source
    tmp = LPC_CGU->BASE_M4_CLK & ~(0x0F << 24);  // clear clk_sel.
    LPC_CGU->BASE_M4_CLK = tmp
                         | (0x01 << 11)          // AUTOBLOCK En
                         | (0x01 << 24)          // CLK_SEL = IRC
                         ;

    // 1. Enable the crystal oscillator @12 MHz.
    LPC_CGU->XTAL_OSC_CTRL &= ~(1 << 2);         // HF: 0=XTAL running at freq < 15 MHz.
    LPC_CGU->XTAL_OSC_CTRL &= ~1;                // Power up XTAL.

    // 2. Wait 250us
    wait_usec(XTAL_OSC_HZ, 250);

    // 3. Reconfigure PLL1 as follows:
    //    - Select the M and N divider values
    //      to produce the final desired
    //      PLL1 output frequency.
    //      Ex: 120MHz => M=10,N=1 => msel=9,nsel=0.
    msel = clock_hz / XTAL_OSC_HZ - 1;
    nsel = 0;
    tmp = LPC_CGU->PLL1_CTRL & ~((0xFF << 16) | (0x03 << 12));
    LPC_CGU->PLL1_CTRL = tmp | (msel << 16) | (nsel << 12);

    //    - Select the crystal oscillator as
    //      clock source for PLL1.
    tmp = LPC_CGU->PLL1_CTRL & ~(0x0F << 24);    // clear CLK_SEL.
    LPC_CGU->PLL1_CTRL = tmp
                         | (0x01 << 11)          // AUTOBLOCK: 0=Disabled, 1=Enabled.
                         | (0x06 << 24)          // CLK_SEL = XTAL.
                         ;

    // 4. Wait for the PLL1 to lock
    while ((LPC_CGU->PLL1_STAT & 1) == 0x0)
    {}

    // 5. Set PLL1 P-divider to divide by 2 (DIRECT=0 and PSEL=0)
    LPC_CGU->PLL1_CTRL &= ~((0x03 << 8) | (1 << 7));

    // 6. Select PLL1 as BASE_M4_CLK source.
    // BASE_M4_CLK is now operating in the
    // mid frequency range (90 MHz - 110 Mhz).
    tmp = LPC_CGU->BASE_M4_CLK & ~(0x0F << 24);  // clear CLK_SEL.
    LPC_CGU->BASE_M4_CLK = tmp
                         | (0x01 << 11)          // AUTOBLOCK: 0=Disabled, 1=Enabled.
                         | (0x09 << 24)          // CLK_SEL = PLL1.
                         ;
    // 7. Wait 20us
    tmp = (XTAL_OSC_HZ * (msel + 1)) / ((nsel + 1) * 2);
    wait_usec(tmp, 20);

    // 8. Set PLL P-divider to direct output mode (DIRECT=1)
    LPC_CGU->PLL1_CTRL |= 1 << 7;

    // BASE_M4_CLK is now operating in the
    // high frequency range.
    SystemCoreClock = (XTAL_OSC_HZ * (msel + 1)) / (nsel + 1);
}

static void clock_setup_pll0usb()
{
    LPC_CGU->PLL0USB_CTRL |= 1;                  // Power down PLL0USB.

    // The USB core require output clock to be running @480 MHz.
    // Setup PLL0USB to generate 480MHz from 12 MHz XTAL.
    LPC_CGU->PLL0USB_NP_DIV = (514 << 12)        // N
                            | (98 << 0);         // P
    LPC_CGU->PLL0USB_MDIV = (0x7FFA << 0)        // MDEC
                          | (0x0B << 17)         // SELP
                          | (0x10 << 22)         // SELI
                          | (0x00 << 28)         // SELR
                          ;
    LPC_CGU->PLL0USB_CTRL = (0 << 0)             // PD: 0=Power Up.
                          | (0x01 << 2)          // DIRECTI: 1=bypass pre-divider.
                          | (0x01 << 3)          // DIRECTO: 1=bypass post-divider.
                          | (0x01 << 4)          // CLKEN
                          | (0x01 << 11)         // AUTOBLOCK: 0=Disabled, 1=Enabled.
                          | (0x06 << 24)         // CLK_SEL = XTAL_OSC.
                          ;

    LPC_CREG->CREG0 &= ~(1 << 5);                // Enable USB0 PHY power.
}


void sdk_init(void)
{
#if defined(DAPLINK_BL)
    clock_setup_base_m4(120000000);

    clock_setup_pll0usb();
#endif
}
