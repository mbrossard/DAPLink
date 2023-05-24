/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018, hathach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb.h"
#include "daplink.h"
#include "util.h"

#include "fsl_device_registers.h"
#include "fsl_gpio.h"
#include "fsl_power.h"
#include "fsl_iocon.h"
#include "fsl_usart.h"

#include "pin_mux.h"

#define XTAL0_CLK_HZ          (16 * 1000 * 1000U)

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+

extern void USBD_SignalHandler(void);

void USB1_IRQHandler(void)
{
#if !defined(TINYDAP) && defined(TUSB_DAPLINK_DEFERRED)
    NVIC_DisableIRQ(USB1_IRQn);
    USBD_SignalHandler();
#else
    tud_int_handler(1);
#if !defined(TINYDAP) && defined(TUSB_DAPLINK_DEFERRED_PARTIAL)
    USBD_SignalHandler();
#endif
#endif
}

void USBD_IntEnable(void)
{
    NVIC_EnableIRQ(USB1_IRQn);
}

/****************************************************************
name: BOARD_BootClockFROHF96M
outputs:
- {id: SYSTICK_clock.outFreq, value: 96 MHz}
- {id: System_clock.outFreq, value: 96 MHz}
settings:
- {id: SYSCON.MAINCLKSELA.sel, value: SYSCON.fro_hf}
sources:
- {id: SYSCON.fro_hf.outFreq, value: 96 MHz}
******************************************************************/
void BootClockFROHF96M(void)
{
  /*!< Set up the clock sources */
  /*!< Set up FRO */
  POWER_DisablePD(kPDRUNCFG_PD_FRO192M); /*!< Ensure FRO is on  */
  CLOCK_SetupFROClocking(12000000U);     /*!< Set up FRO to the 12 MHz, just for sure */
  CLOCK_AttachClk(kFRO12M_to_MAIN_CLK); /*!< Switch to FRO 12MHz first to ensure we can change voltage without
                                             accidentally being below the voltage for current speed */

  CLOCK_SetupFROClocking(96000000U); /*!< Set up high frequency FRO output to selected frequency */

  POWER_SetVoltageForFreq(96000000U); /*!< Set voltage for the one of the fastest clock outputs: System clock output */
  CLOCK_SetFLASHAccessCyclesForFreq(96000000U); /*!< Set FLASH wait states for core */

  /*!< Set up dividers */
  CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 1U, false);     /*!< Set AHBCLKDIV divider to value 1 */

  /*!< Set up clock selectors - Attach clocks to the peripheries */
  CLOCK_AttachClk(kFRO_HF_to_MAIN_CLK); /*!< Switch MAIN_CLK to FRO_HF */

  /*!< Set SystemCoreClock variable. */
  SystemCoreClock = 96000000U;
}

void sdk_init(void)
{
  // Enable IOCON clock
  CLOCK_EnableClock(kCLOCK_Iocon);

  // Init 96 MHz clock
  BootClockFROHF96M();

  // Init all GPIO ports
  GPIO_PortInit(GPIO, 0);
  GPIO_PortInit(GPIO, 1);

#ifdef UART_DEV
  // UART
  IOCON_PinMuxSet(IOCON, UART_RX_PINMUX);
  IOCON_PinMuxSet(IOCON, UART_TX_PINMUX);

  // Enable UART when debug log is on
  CLOCK_AttachClk(kFRO12M_to_FLEXCOMM0);
  usart_config_t uart_config;
  USART_GetDefaultConfig(&uart_config);
  uart_config.baudRate_Bps = CFG_BOARD_UART_BAUDRATE;
  uart_config.enableTx     = true;
  uart_config.enableRx     = true;
  USART_Init(UART_DEV, &uart_config, 12000000);
#endif

  // USB VBUS
  /* PORT0 PIN22 configured as USB0_VBUS */
  // IOCON_PinMuxSet(IOCON, 0U, 22U, IOCON_PIO_DIG_FUNC7_EN);
  init_vbus_pin();

  // Port1 is High Speed

  /* Turn on USB1 Phy */
  POWER_DisablePD(kPDRUNCFG_PD_USB1_PHY);

  /* reset the IP to make sure it's in reset state. */
  RESET_PeripheralReset(kUSB1H_RST_SHIFT_RSTn);
  RESET_PeripheralReset(kUSB1D_RST_SHIFT_RSTn);
  RESET_PeripheralReset(kUSB1_RST_SHIFT_RSTn);
  RESET_PeripheralReset(kUSB1RAM_RST_SHIFT_RSTn);

  /* According to reference manual, device mode setting has to be set by access usb host register */
  CLOCK_EnableClock(kCLOCK_Usbh1); // enable usb0 host clock

  USBHSH->PORTMODE = USBHSH_PORTMODE_SW_PDCOM_MASK; // Put PHY powerdown under software control
  USBHSH->PORTMODE |= USBHSH_PORTMODE_DEV_ENABLE_MASK;

  CLOCK_DisableClock(kCLOCK_Usbh1); // disable usb0 host clock

  /* enable USB Device clock */
  CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_UsbPhySrcExt, XTAL0_CLK_HZ);
  CLOCK_EnableUsbhs0DeviceClock(kCLOCK_UsbSrcUnused, 0U);
  CLOCK_EnableClock(kCLOCK_UsbRam1);

  // Enable PHY support for Low speed device + LS via FS Hub
  USBPHY->CTRL |= USBPHY_CTRL_SET_ENUTMILEVEL2_MASK | USBPHY_CTRL_SET_ENUTMILEVEL3_MASK;

  // Enable all power for normal operation
  USBPHY->PWD = 0;

  USBPHY->CTRL_SET = USBPHY_CTRL_SET_ENAUTOCLR_CLKGATE_MASK;
  USBPHY->CTRL_SET = USBPHY_CTRL_SET_ENAUTOCLR_PHY_PWD_MASK;
}

#define FLASH_CMD_BLANK_CHECK (0x5)

bool flash_is_readable(uint32_t addr, uint32_t length)
{
    // Make sure the core clock is less than 100 MHz, or flash commands won't work.
    util_assert(SystemCoreClock < 100000000);

    // Return true if the address is within internal flash and the flash sector is not erased.
    if (!(addr >= DAPLINK_ROM_START && addr < (DAPLINK_ROM_START + DAPLINK_ROM_SIZE))) {
        return false;
    }

    // Use the blank check command directly. The address is right-shifted to convert to
    // a flash word (16 bytes) address.
    FLASH->STARTA = addr >> 4;
    FLASH->STOPA = (addr + length - 1) >> 4;
    FLASH->INT_CLR_STATUS = FLASH_INT_CLR_STATUS_FAIL_MASK
                            | FLASH_INT_CLR_STATUS_ERR_MASK
                            | FLASH_INT_CLR_STATUS_DONE_MASK
                            | FLASH_INT_CLR_STATUS_ECC_ERR_MASK;
    FLASH->CMD = FLASH_CMD_BLANK_CHECK;

    // Wait until command is complete.
    while (((FLASH->INT_STATUS) & FLASH_INT_STATUS_DONE_MASK) == 0) {
    }

    // Return true (is readable) if the blank check failed, meaning the page is programmed.
    // Return false (not readable) for erased page or failure.
    return ((FLASH->INT_STATUS & (FLASH_INT_STATUS_FAIL_MASK
                                | FLASH_INT_STATUS_ERR_MASK
                                | FLASH_INT_STATUS_ECC_ERR_MASK)) == FLASH_INT_STATUS_FAIL_MASK);
}