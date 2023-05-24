/*
 * Copyright (c) 2013-2020 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "Driver_USART.h"
#include "RP20xx.h"

#define ARM_USART_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
    ARM_USART_API_VERSION,
    ARM_USART_DRV_VERSION};

/* Driver Capabilities */
static const ARM_USART_CAPABILITIES DriverCapabilities = {
    1, /* supports UART (Asynchronous) mode */
    0, /* supports Synchronous Master mode */
    0, /* supports Synchronous Slave mode */
    0, /* supports UART Single-wire mode */
    1, /* supports UART IrDA mode */
    0, /* supports UART Smart Card mode */
    0, /* Smart Card Clock generator available */
    1, /* RTS Flow Control available */
    1, /* CTS Flow Control available */
    0, /* Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE */
    0, /* Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT */
    1, /* RTS Line: 0=not available, 1=available */
    1, /* CTS Line: 0=not available, 1=available */
    1, /* DTR Line: 0=not available, 1=available */
    1, /* DSR Line: 0=not available, 1=available */
    1, /* DCD Line: 0=not available, 1=available */
    1, /* RI Line: 0=not available, 1=available */
    1, /* Signal CTS change event: \ref ARM_USART_EVENT_CTS */
    1, /* Signal DSR change event: \ref ARM_USART_EVENT_DSR */
    1, /* Signal DCD change event: \ref ARM_USART_EVENT_DCD */
    1, /* Signal RI change event: \ref ARM_USART_EVENT_RI */
    0  /* Reserved (must be zero) */
};

typedef struct {
    uint32_t baudrate;
    uint8_t  parity;
    uint8_t  stop_bits;
    uint8_t  data_len;
    uint8_t  cts_en;
    uint8_t  rts_en;
    uint8_t  rx_en;
    uint8_t  tx_en;
    uint8_t  loopback_en;
} USART_DriverSettings;

USART_DriverSettings USART1_Settings;
USART_DriverSettings USART2_Settings;

uint32_t                tx_count[2] = {0, 0};
uint32_t                rx_count[2] = {0, 0};
ARM_USART_SignalEvent_t callback[2] = {NULL, NULL};

#define UART_CALC_BRD(sys_clk, baud_rate)  ((8 * (sys_clk)) / (baud_rate))
#define UART_CALC_BRDI(sys_clk, baud_rate) (UART_CALC_BRD(sys_clk, baud_rate) >> 7)
#define UART_CALC_BRDF(sys_clk, baud_rate) (((UART_CALC_BRD(sys_clk, baud_rate) & 0x7f) + 1) / 2)

//
//   Functions
//

static uint32_t USART_GetClock(void)
{
    uint8_t clk_src = (CLOCKS->CLK_PERI_CTRL & CLOCKS_CLK_PERI_CTRL_AUXSRC_Msk) >> CLOCKS_CLK_PERI_CTRL_AUXSRC_Pos;
    switch (clk_src) {
    case 0:    // clock sys
        return SystemCoreClock;
    case 1:    // clksrc_pll_sys
    case 2:    // clksrc_pll_usb
    case 3:    // rosc_clksrc_ph
        break;
    case 4:    // xosc_clksrc
        return 12000000;
    case 5:    // clksrc_gpin0
    case 6:    // clksrc_gpin1
        break;
    }

    return 0;
}

static void ARM_USART_SignalEvent(uint32_t event)
{
    // function body
}

static ARM_DRIVER_VERSION ARM_USART_GetVersion(void)
{
    return DriverVersion;
}

static ARM_USART_CAPABILITIES ARM_USART_GetCapabilities(void)
{
    return DriverCapabilities;
}

static int32_t ARM_USART_Initialize(void *instance, ARM_USART_SignalEvent_t cb_event)
{
    UART_Type *uart = (UART_Type *) instance;

    // Init defaults
    USART1_Settings.baudrate    = 115200;
    USART1_Settings.parity      = 0;
    USART1_Settings.stop_bits   = 0;
    USART1_Settings.data_len    = 8;
    USART1_Settings.cts_en      = 0;
    USART1_Settings.rts_en      = 0;
    USART1_Settings.rx_en       = 1;
    USART1_Settings.tx_en       = 1;
    USART1_Settings.loopback_en = 0;

    CLOCKS->CLK_PERI_CTRL = CLOCKS_CLK_PERI_CTRL_ENABLE_Msk;

    // Reset peripheral
    SET_BIT(RESETS->RESET, RESETS_RESET_uart0_Msk);
    CLEAR_BIT(RESETS->RESET, RESETS_RESET_uart0_Msk);
    while (!(RESETS->RESET_DONE & RESETS_RESET_DONE_uart0_Msk))
        ;

    // Calculate divisors
    uint32_t clock_hz = USART_GetClock();
    uint32_t i_part   = UART_CALC_BRDI(clock_hz, USART1_Settings.baudrate);
    uint32_t f_part   = UART_CALC_BRDF(clock_hz, USART1_Settings.baudrate);
    uart->UARTIBRD    = i_part;
    uart->UARTFBRD    = f_part;

    // Update settings
    uart->UARTCR =
        ((USART1_Settings.cts_en) ? UART0_UARTCR_CTSEN_Msk : 0) |
        ((USART1_Settings.rts_en) ? UART0_UARTCR_RTSEN_Msk : 0) |
        ((USART1_Settings.rx_en) ? UART0_UARTCR_RXE_Msk : 0) |
        ((USART1_Settings.tx_en) ? UART0_UARTCR_TXE_Msk : 0) |
        ((USART1_Settings.loopback_en) ? UART0_UARTCR_LBE_Pos : 0);

    MODIFY_REG(uart->UARTLCR_H, UART0_UARTLCR_H_WLEN_Msk, 3 << UART0_UARTLCR_H_WLEN_Pos);

    // Enable UART
    SET_BIT(uart->UARTCR, UART0_UARTCR_UARTEN_Msk);

    // we don't support more than 2 UARTS so this is fine
    int uart_no = (uart == UART0) ? 0 : 1;
    if (cb_event) {
        callback[uart_no] = cb_event;
    } else {
        callback[uart_no] = ARM_USART_SignalEvent;
    }

    SET_BIT(uart->UARTLCR_H, UART0_UARTLCR_H_FEN_Msk);

    return ARM_DRIVER_OK;
}

static int32_t ARM_USART_Uninitialize(void *instance)
{
    UART_Type *uart = (UART_Type *) instance;

    // Disable UART
    CLEAR_BIT(uart->UARTCR, UART0_UARTCR_UARTEN_Msk);
    // Reset UART peripheral
    SET_BIT(RESETS->RESET, RESETS_RESET_uart0_Msk);

    return ARM_DRIVER_OK;
}

static int32_t ARM_USART_PowerControl(void *instance, ARM_POWER_STATE state)
{
    UART_Type *uart = (UART_Type *) instance;

    switch (state) {
    case ARM_POWER_OFF:
        CLEAR_BIT(uart->UARTCR, UART0_UARTCR_UARTEN_Msk);
        break;

    case ARM_POWER_LOW:
        break;

    case ARM_POWER_FULL:
        SET_BIT(uart->UARTCR, UART0_UARTCR_UARTEN_Msk);
        break;
    }
    return ARM_DRIVER_OK;
}

static int32_t ARM_USART_Send(void *instance, const void *data, uint32_t num)
{
    UART_Type *uart = (UART_Type *) instance;

    const uint8_t *buffer = data;
    while (num--) {
        // Wait until there is a space in FIFO
        while (uart->UARTFR & UART0_UARTFR_TXFF_Msk)
            ;
        // Place data in FIFO
        uart->UARTDR = *buffer++;
    }

    return ARM_DRIVER_OK;
}

static int32_t ARM_USART_Receive(void *instance, void *data, uint32_t num)
{
    UART_Type *uart = (UART_Type *) instance;

    uint8_t *buffer = data;
    while (num--) {
        // Wait until there is data
        while (uart->UARTFR & UART0_UARTFR_RXFE_Msk)
            ;
        // Read into buffer
        *buffer++ = uart->UARTDR;
    }

    return ARM_DRIVER_OK;
}

static int32_t ARM_USART_Transfer(void *instance, const void *data_out, void *data_in, uint32_t num)
{
    UART_Type *uart = (UART_Type *) instance;
    (void) uart;
    // only in dma
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static uint32_t ARM_USART_GetTxCount(void *instance)
{
    UART_Type *uart = (UART_Type *) instance;
    if (uart == UART0) {
        return tx_count[0];
    } else if (uart == UART1) {
        return tx_count[1];
    }

    return 0;
}

static uint32_t ARM_USART_GetRxCount(void *instance)
{
    UART_Type *uart = (UART_Type *) instance;
    if (uart == UART0) {
        return rx_count[0];
    } else if (uart == UART1) {
        return rx_count[1];
    }

    return 0;
}

static int32_t ARM_USART_Control(void *instance, uint32_t control, uint32_t arg)
{
    UART_Type *uart = (UART_Type *) instance;

    switch (control & ARM_USART_CONTROL_Msk) {
    case ARM_USART_CONTROL_TX:
        (arg) ? SET_BIT(uart->UARTCR, UART0_UARTCR_TXE_Msk) : CLEAR_BIT(uart->UARTCR, UART0_UARTCR_TXE_Msk);
        break;
    case ARM_USART_CONTROL_RX:
        (arg) ? SET_BIT(uart->UARTCR, UART0_UARTCR_RXE_Msk) : CLEAR_BIT(uart->UARTCR, UART0_UARTCR_RXE_Msk);
        break;
    case ARM_USART_MODE_ASYNCHRONOUS:
        UART0->UARTIBRD = UART_CALC_BRDI(USART_GetClock(), USART1_Settings.baudrate);
        UART0->UARTFBRD = UART_CALC_BRDF(USART_GetClock(), USART1_Settings.baudrate);
        break;
    case ARM_USART_CONTROL_BREAK:
        (arg) ? SET_BIT(uart->UARTLCR_H, UART0_UARTLCR_H_BRK_Msk) : CLEAR_BIT(uart->UARTLCR_H, UART0_UARTLCR_H_BRK_Msk);
        break;
    }

    switch (control & ARM_USART_DATA_BITS_Msk) {
    case ARM_USART_DATA_BITS_5:
        MODIFY_REG(uart->UARTLCR_H, UART0_UARTLCR_H_WLEN_Msk, 0 << UART0_UARTLCR_H_WLEN_Pos);
        break;
    case ARM_USART_DATA_BITS_6:
        MODIFY_REG(uart->UARTLCR_H, UART0_UARTLCR_H_WLEN_Msk, 1 << UART0_UARTLCR_H_WLEN_Pos);
        break;
    case ARM_USART_DATA_BITS_7:
        MODIFY_REG(uart->UARTLCR_H, UART0_UARTLCR_H_WLEN_Msk, 2 << UART0_UARTLCR_H_WLEN_Pos);
        break;
    case ARM_USART_DATA_BITS_8:
        MODIFY_REG(uart->UARTLCR_H, UART0_UARTLCR_H_WLEN_Msk, 3 << UART0_UARTLCR_H_WLEN_Pos);
        break;
    }

    switch (control & ARM_USART_PARITY_Msk) {
    case ARM_USART_PARITY_NONE:
        CLEAR_BIT(uart->UARTLCR_H, UART0_UARTLCR_H_PEN_Msk);
        break;
    case ARM_USART_PARITY_EVEN:
        SET_BIT(uart->UARTLCR_H, UART0_UARTLCR_H_PEN_Msk | UART0_UARTLCR_H_EPS_Msk);
        break;
    case ARM_USART_PARITY_ODD:
        CLEAR_BIT(uart->UARTLCR_H, UART0_UARTLCR_H_EPS_Msk);
        SET_BIT(uart->UARTLCR_H, UART0_UARTLCR_H_PEN_Msk);
        break;
    }

    switch (control & ARM_USART_STOP_BITS_Msk) {
    case ARM_USART_STOP_BITS_1:
        CLEAR_BIT(uart->UARTLCR_H, UART0_UARTLCR_H_STP2_Msk);
        break;
    case ARM_USART_STOP_BITS_2:
        SET_BIT(uart->UARTLCR_H, UART0_UARTLCR_H_STP2_Msk);
        break;
    default:;
    }

    switch (control & ARM_USART_FLOW_CONTROL_Msk) {
    case ARM_USART_FLOW_CONTROL_NONE:
        CLEAR_BIT(uart->UARTCR, UART0_UARTCR_RTSEN_Msk);
        CLEAR_BIT(uart->UARTCR, UART0_UARTCR_CTSEN_Msk);
        break;
    case ARM_USART_FLOW_CONTROL_RTS:
        SET_BIT(uart->UARTCR, UART0_UARTCR_RTSEN_Msk);
        CLEAR_BIT(uart->UARTCR, UART0_UARTCR_CTSEN_Msk);
        break;
    case ARM_USART_FLOW_CONTROL_CTS:
        CLEAR_BIT(uart->UARTCR, UART0_UARTCR_RTSEN_Msk);
        SET_BIT(uart->UARTCR, UART0_UARTCR_CTSEN_Msk);
        break;
    case ARM_USART_FLOW_CONTROL_RTS_CTS:
        SET_BIT(uart->UARTCR, UART0_UARTCR_RTSEN_Msk);
        SET_BIT(uart->UARTCR, UART0_UARTCR_CTSEN_Msk);
        break;
    }

    return ARM_DRIVER_OK;
}

static ARM_USART_STATUS ARM_USART_GetStatus(void *instance)
{
    UART_Type *uart = (UART_Type *) instance;

    ARM_USART_STATUS stat = {0};
    uint32_t         fr   = uart->UARTFR;
    uint32_t         rsr  = uart->UARTRSR;
    if (fr & UART0_UARTFR_BUSY_Msk) {
        stat.tx_busy = 1;
    }
    if (rsr & UART0_UARTRSR_BE_Msk) {
        stat.rx_break = 1;
    }
    if (rsr & UART0_UARTRSR_FE_Msk) {
        stat.rx_framing_error = 1;
    }
    if (rsr & UART0_UARTRSR_PE_Msk) {
        stat.rx_parity_error = 1;
    }
    if (rsr & UART0_UARTRSR_OE_Msk) {
        stat.rx_overflow = 1;
    }

    return stat;
}

static int32_t ARM_USART_SetModemControl(void *instance, ARM_USART_MODEM_CONTROL control)
{
    UART_Type *uart = (UART_Type *) instance;

    uint32_t rts = uart->UARTCR & UART0_UARTCR_RTSEN_Msk;
    if ((control == ARM_USART_RTS_CLEAR || control == ARM_USART_RTS_SET) && !rts) {
        return ARM_DRIVER_ERROR;
    }

    switch (control) {
    case ARM_USART_RTS_CLEAR:
        CLEAR_BIT(uart->UARTCR, UART0_UARTCR_RTS_Msk);
        break;
    case ARM_USART_RTS_SET:
        SET_BIT(uart->UARTCR, UART0_UARTCR_RTS_Msk);
        break;
    case ARM_USART_DTR_CLEAR:
        CLEAR_BIT(uart->UARTCR, UART0_UARTCR_DTR_Msk);
        break;
    case ARM_USART_DTR_SET:
        SET_BIT(uart->UARTCR, UART0_UARTCR_DTR_Msk);
        break;
    }

    return ARM_DRIVER_OK;
}

static ARM_USART_MODEM_STATUS ARM_USART_GetModemStatus(void *instance)
{
    UART_Type *uart = (UART_Type *) instance;

    ARM_USART_MODEM_STATUS stat;
    uint32_t               status = uart->UARTFR;
    stat.cts                      = (status & UART0_UARTFR_CTS_Msk) ? 1 : 0;
    stat.dsr                      = (status & UART0_UARTFR_DSR_Msk) ? 1 : 0;
    stat.dcd                      = (status & UART0_UARTFR_DCD_Msk) ? 1 : 0;
    stat.ri                       = (status & UART0_UARTFR_RI_Msk) ? 1 : 0;

    return stat;
}

// End USART Interface

#define ARM_USART_FUNCTIONS(x)                                                               \
    static int32_t ARM_USART_Initialize_##x(ARM_USART_SignalEvent_t cb_event)                \
    {                                                                                        \
        return ARM_USART_Initialize(UART##x, cb_event);                                      \
    }                                                                                        \
    static int32_t ARM_USART_Uninitialize_##x(void)                                          \
    {                                                                                        \
        return ARM_USART_Uninitialize(UART##x);                                              \
    }                                                                                        \
    static int32_t ARM_USART_PowerControl_##x(ARM_POWER_STATE state)                         \
    {                                                                                        \
        return ARM_USART_PowerControl(UART##x, state);                                       \
    }                                                                                        \
    static int32_t ARM_USART_Send_##x(const void *data, uint32_t num)                        \
    {                                                                                        \
        return ARM_USART_Send(UART##x, data, num);                                           \
    }                                                                                        \
    static int32_t ARM_USART_Receive_##x(void *data, uint32_t num)                           \
    {                                                                                        \
        return ARM_USART_Receive(UART##x, data, num);                                        \
    }                                                                                        \
    static int32_t ARM_USART_Transfer_##x(const void *data_out, void *data_in, uint32_t num) \
    {                                                                                        \
        return ARM_USART_Transfer(UART##x, data_out, data_in, num);                          \
    }                                                                                        \
    static uint32_t ARM_USART_GetTxCount_##x(void)                                           \
    {                                                                                        \
        return ARM_USART_GetTxCount(UART##x);                                                \
    }                                                                                        \
    static uint32_t ARM_USART_GetRxCount_##x(void)                                           \
    {                                                                                        \
        return ARM_USART_GetRxCount(UART##x);                                                \
    }                                                                                        \
    static int32_t ARM_USART_Control_##x(uint32_t control, uint32_t arg)                     \
    {                                                                                        \
        return ARM_USART_Control(UART##x, control, arg);                                     \
    }                                                                                        \
    static ARM_USART_STATUS ARM_USART_GetStatus_##x(void)                                    \
    {                                                                                        \
        return ARM_USART_GetStatus(UART##x);                                                 \
    }                                                                                        \
    static int32_t ARM_USART_SetModemControl_##x(ARM_USART_MODEM_CONTROL control)            \
    {                                                                                        \
        return ARM_USART_SetModemControl(UART##x, control);                                  \
    }                                                                                        \
    static ARM_USART_MODEM_STATUS ARM_USART_GetModemStatus_##x(void)                         \
    {                                                                                        \
        return ARM_USART_GetModemStatus(UART##x);                                            \
    }                                                                                        \
                                                                                             \
    extern ARM_DRIVER_USART Driver_USART##x;                                                 \
    ARM_DRIVER_USART        Driver_USART##x = {                                              \
        ARM_USART_GetVersion,                                                         \
        ARM_USART_GetCapabilities,                                                    \
        ARM_USART_Initialize_##x,                                                     \
        ARM_USART_Uninitialize_##x,                                                   \
        ARM_USART_PowerControl_##x,                                                   \
        ARM_USART_Send_##x,                                                           \
        ARM_USART_Receive_##x,                                                        \
        ARM_USART_Transfer_##x,                                                       \
        ARM_USART_GetTxCount_##x,                                                     \
        ARM_USART_GetRxCount_##x,                                                     \
        ARM_USART_Control_##x,                                                        \
        ARM_USART_GetStatus_##x,                                                      \
        ARM_USART_SetModemControl_##x,                                                \
        ARM_USART_GetModemStatus_##x};

ARM_USART_FUNCTIONS(0)
ARM_USART_FUNCTIONS(1)
