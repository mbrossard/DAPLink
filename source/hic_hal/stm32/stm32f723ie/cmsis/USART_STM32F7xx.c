/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2020 Arm Limited (or its affiliates). All 
 * rights reserved.
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
 *
 *
 * $Date:        16. December 2020
 * $Revision:    V1.16
 *
 * Driver:       Driver_USART1, Driver_USART2, Driver_USART3, Driver_USART4,
 *               Driver_USART5, Driver_USART6, Driver_USART7, Driver_USART8,
 * Configured:   via RTE_Device.h or MX_Device.h configuration file
 * Project:      USART Driver for ST STM32F7xx
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                   Value   UART Interface
 *   ---------------------                   -----   --------------
 *   Connect to hardware via Driver_USART# = 1       use USART1
 *   Connect to hardware via Driver_USART# = 2       use USART2
 *   Connect to hardware via Driver_USART# = 3       use USART3
 *   Connect to hardware via Driver_USART# = 4       use UART4
 *   Connect to hardware via Driver_USART# = 5       use UART5
 *   Connect to hardware via Driver_USART# = 6       use USART6
 *   Connect to hardware via Driver_USART# = 7       use UART7
 *   Connect to hardware via Driver_USART# = 8       use UART8
  * --------------------------------------------------------------------------
 * Define used for driver configuration (at compile time):
 *
 *   USART_DCACHE:            0U   disabled USART DCache maintenance operations
 *                            1U    enabled USART DCache maintenance operations
 *     - default value:       1U
 *
 * Note:
 *  For USART DMA transfers, with DCache enabled, data buffers must be 32 Byte
 *  aligned and data size must be n*32 Bytes.
 * -------------------------------------------------------------------------- */

/* History:
 *  Version 1.16
 *    Corrected Data Cache handling
 *  Version 1.15
 *    Corrected Baud rate configuration
 *  Version 1.14
 *    Corrected DMA MemDataAlignment configuration.
 *  Version 1.13
 *    Made I/O output speed configurable.
 *    Updated USART_TRANSFER_INFO structure - tx_buf type changed from uint8_t * to const uint8_t *.
 *  Version 1.12
 *    Corrected POWER_OFF sequence. DMA is DeInitialized after it is aborted.
 *  Version 1.11
 *    Added checking if pointer to USART_PIN is valid, before it is used
 *  Version 1.10
 *    Corrected condition for ARM_USART_SET_IRDA_PULSE control
 *  Version 1.9
 *    Added support for ARM Compiler 6
 *  Version 1.8
 *    Changed USART_ISR_LBD (legacy define) to USART_ISR_LBDF
 *  Version 1.7
 *    Added port configuration for ports supported by new subfamilies.
 *  Version 1.6
 *    Corrected CTS handling
 *  Version 1.5
 *    Corrected Data Bits format configuration
 *  Version 1.4
 *    Added Clearing of Noise detection flag
 *  Version 1.3
 *    Corrected PowerControl function for:
 *      - Unconditional Power Off
 *      - Conditional Power full (driver must be initialized)
 *  Version 1.2
 *    Removed checking if Asynchronous Single-wire mode has been configured in CubeMX
 *  Version 1.1
 *    Corrected unwanted receive stop (Caused by calling USART_Receive function, while receiver is still busy.)
 *  Version 1.0
 *    Initial release
 */
 
 /*! \page stm32f7_usart CMSIS-Driver USART Setup

The CMSIS-Driver USART requires:
  - Setup of USART/UART mode (Asynchronous, Synchronous, Single wire, IrDA or SmartCard)
  - Optional configuration of Hardware Flow Control (only in Asynchronous mode).
 
The example below uses correct settings for STM32F746G-Discovery:  
  - USART1 Mode:             Asynchronous
  - Hardware Flow Control:   Disable

For different boards, refer to the hardware schematics to reflect correct setup values.

The STM32CubeMX configuration steps for Pinout, Clock, and System Configuration are 
listed below. Enter the values that are marked \b bold.
   
Pinout tab
----------
  1. Configure USART1 mode
     - Peripherals \b USART1: Mode=<b>Asynchronous</b>, Hardware Flow Control=<b>Disable</b>
  2. Configure USART1_TX pin (is not the default pin):
     - Click in chip diagram on pin \b PA9 and select \b USART1_TX.
          
Clock Configuration tab
-----------------------
  1. Configure USART1 Clock.
  
Configuration tab
-----------------
  1. Under Connectivity open \b USART1 Configuration:
     - \e optional <b>DMA Settings</b>: setup DMA transfers for Rx and Tx (DMA is optional)\n
       \b Add - Select \b USART1_RX: Stream=DMA2 Stream 2, DMA Request Settings: not used\n
       \b Add - Select \b USART1_TX: Stream=DMA2 Stream 7, DMA Request Settings: not used

     - <b>GPIO Settings</b>: review settings, no changes required
          Pin Name | Signal on Pin | GPIO mode | GPIO Pull-up/Pull..| Maximum out | User Label
          :--------|:--------------|:----------|:-------------------|:------------|:----------
          PB7      | USART1_RX     | Alternate | Pull-up            | High        |.
          PA9      | USART1_TX     | Alternate | Pull-up            | High        |.          
     - <b>NVIC Settings</b>: enable interrupts
          Interrupt Table                      | Enable | Preemption Priority | Sub Priority
          :------------------------------------|:-------|:--------------------|:--------------
          USART1 global interrupt              |\b ON   | 0                   | 0
          DMA2 stream2 global interrupt        |   ON   | 0                   | 0
          DMA2 stream7 global interrupt        |   ON   | 0                   | 0
     - Parameter Settings: not used
     - User Constants: not used
     - Click \b OK to close the USART1 Configuration dialog
*/

/*! \cond */
 
#include "USART_STM32F7xx.h"

#define ARM_USART_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,16)

#ifndef USART_DCACHE
#define USART_DCACHE               1U
#endif

// Driver Version
static const ARM_DRIVER_VERSION usart_driver_version = { ARM_USART_API_VERSION, ARM_USART_DRV_VERSION };

// Receiver Timeout values
#ifndef USART1_RX_TIMEOUT_VAL
#define USART1_RX_TIMEOUT_VAL     30U
#endif
#ifndef USART2_RX_TIMEOUT_VAL
#define USART2_RX_TIMEOUT_VAL     30U
#endif
#ifndef USART3_RX_TIMEOUT_VAL
#define USART3_RX_TIMEOUT_VAL     30U
#endif
#ifndef UART4_RX_TIMEOUT_VAL
#define UART4_RX_TIMEOUT_VAL      30U
#endif
#ifndef UART5_RX_TIMEOUT_VAL
#define UART5_RX_TIMEOUT_VAL      30U
#endif
#ifndef USART6_RX_TIMEOUT_VAL
#define USART6_RX_TIMEOUT_VAL     30U
#endif
#ifndef UART7_RX_TIMEOUT_VAL
#define UART7_RX_TIMEOUT_VAL      30U
#endif
#ifndef UART8_RX_TIMEOUT_VAL
#define UART8_RX_TIMEOUT_VAL      30U
#endif


// USART1
#ifdef MX_USART1

#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
#if (MX_USART1_VM == VM_ASYNC)
extern UART_HandleTypeDef huart1;
#elif (MX_USART1_VM == VM_SYNC)
extern USART_HandleTypeDef husart1;
#elif (MX_USART1_VM == VM_IRDA)
extern IRDA_HandleTypeDef hirda1;
#elif (MX_USART1_VM == VM_SMARTCARD)
extern SMARTCARD_HandleTypeDef hsmartcard1;
#else
#error "Incorrect virtual mode is selected"
#endif
#endif

// USART1 Run-Time Information
static USART_INFO          USART1_Info = { 0U, { 0U, 0U, 0U, 0U, 0U, 0U, 0U }, 0U, 0U, 0U };
static USART_TRANSFER_INFO USART1_TransferInfo;

#ifdef MX_USART1_TX_Pin
  static USART_PIN USART1_tx =  {MX_USART1_TX_GPIOx,  MX_USART1_TX_GPIO_Pin,  MX_USART1_TX_GPIO_AF,  MX_USART1_TX_GPIO_PuPd,  MX_USART1_TX_GPIO_Speed};
#endif
#ifdef MX_USART1_RX_Pin
  static USART_PIN USART1_rx =  {MX_USART1_RX_GPIOx,  MX_USART1_RX_GPIO_Pin,  MX_USART1_RX_GPIO_AF,  MX_USART1_RX_GPIO_PuPd,  MX_USART1_RX_GPIO_Speed};
#endif
#ifdef MX_USART1_CK_Pin
  static USART_PIN USART1_ck =  {MX_USART1_CK_GPIOx,  MX_USART1_CK_GPIO_Pin,  MX_USART1_CK_GPIO_AF,  MX_USART1_CK_GPIO_PuPd,  MX_USART1_CK_GPIO_Speed};
#endif
#ifdef MX_USART1_RTS_Pin
  static USART_PIN USART1_rts = {MX_USART1_RTS_GPIOx, MX_USART1_RTS_GPIO_Pin, MX_USART1_RTS_GPIO_AF, MX_USART1_RTS_GPIO_PuPd, MX_USART1_RTS_GPIO_Speed};
#endif
#ifdef MX_USART1_CTS_Pin
  static USART_PIN USART1_cts = {MX_USART1_CTS_GPIOx, MX_USART1_CTS_GPIO_Pin, MX_USART1_CTS_GPIO_AF, MX_USART1_CTS_GPIO_PuPd, MX_USART1_CTS_GPIO_Speed};
#endif

#ifdef MX_USART1_TX_DMA_Instance
  static void USART1_TX_DMA_Complete (DMA_HandleTypeDef *hdma);

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  static DMA_HandleTypeDef hdma_usart1_tx;
#else
  extern DMA_HandleTypeDef hdma_usart1_tx;
#endif
  static USART_DMA USART1_DMA_Tx = {
    &hdma_usart1_tx,
    USART1_TX_DMA_Complete,
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
    MX_USART1_TX_DMA_Instance,
    MX_USART1_TX_DMA_Channel,
    MX_USART1_TX_DMA_Priority,
    MX_USART1_TX_DMA_IRQn,
    { 0U, 0U, 0U }
#endif
  };
#endif
#ifdef MX_USART1_RX_DMA_Instance
  static void USART1_RX_DMA_Complete (DMA_HandleTypeDef *hdma);

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  static DMA_HandleTypeDef hdma_usart1_rx;
#else
  extern DMA_HandleTypeDef hdma_usart1_rx;
#endif
  static USART_DMA USART1_DMA_Rx = {
    &hdma_usart1_rx,
    USART1_RX_DMA_Complete,
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
    MX_USART1_RX_DMA_Instance,
    MX_USART1_RX_DMA_Channel,
    MX_USART1_RX_DMA_Priority,
    MX_USART1_RX_DMA_IRQn,
    { 0U, 0U, 0U }
#endif
  };
#endif

// USART1 Resources
static const USART_RESOURCES USART1_Resources = {
#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
#if (MX_USART1_VM == VM_ASYNC)
  &huart1,
  VM_ASYNC,
#elif (MX_USART1_VM == VM_SYNC)
  &husart1,
  VM_SYNC,
#elif (MX_USART1_VM == VM_IRDA)
  &hirda1,
  VM_IRDA,
#elif (MX_USART1_VM == VM_SMARTCARD)
  &hsmartcard1,
  VM_SMARTCARD,
#endif
#endif
  {     // Capabilities
    1,  // supports UART (Asynchronous) mode
#ifdef MX_USART1_CK_Pin
    1,  // supports Synchronous Master mode
#else
    0,  // supports Synchronous Master mode
#endif
    0,  // supports Synchronous Slave mode
    1,  // supports UART Single-wire mode
    1,  // supports UART IrDA mode
    1,  // supports UART Smart Card mode
    0,  // Smart Card Clock generator
#ifdef MX_USART1_RTS_Pin
    1,  // RTS Flow Control available
#else
    0,  // RTS Flow Control available
#endif
#ifdef MX_USART1_CTS_Pin
    1,  // CTS Flow Control available
#else
    0,  // CTS Flow Control available
#endif
    1,  // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    1,  // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
#ifdef MX_USART1_RTS_Pin
    1,  // RTS Line: 0=not available, 1=available
#else
    0,  // RTS Line: 0=not available, 1=available
#endif
#ifdef MX_USART1_CTS_Pin
    1,  // CTS Line: 0=not available, 1=available
#else
    0,  // CTS Line: 0=not available, 1=available
#endif
    0,  // DTR Line: 0=not available, 1=available
    0,  // DSR Line: 0=not available, 1=available
    0,  // DCD Line: 0=not available, 1=available
    0,  // RI Line: 0=not available, 1=available
#ifdef MX_USART1_CTS_Pin
    1,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#else
    0,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#endif
    0,  // Signal DSR change event: \ref ARM_USART_EVENT_DSR
    0,  // Signal DCD change event: \ref ARM_USART_EVENT_DCD
    0   // Signal RI change event: \ref ARM_USART_EVENT_RI
#if (defined(ARM_USART_API_VERSION) && (ARM_USART_API_VERSION >= 0x203U))
  , 0U  // Reserved bits
#endif
  },

    USART1,

  // PINS
  {
#ifdef MX_USART1_TX_Pin
    &USART1_tx,
#else
    NULL,
#endif
#ifdef MX_USART1_RX_Pin
    &USART1_rx,
#else
    NULL,
#endif
#ifdef MX_USART1_CK_Pin
    &USART1_ck,
#else
    NULL,
#endif
#ifdef MX_USART1_RTS_Pin 
    &USART1_rts,
#else
    NULL,
#endif
#ifdef MX_USART1_CTS_Pin 
    &USART1_cts,
#else
    NULL,
#endif
  },

    USART1_IRQn,
  { 0U, 0U, 0U },

#ifdef MX_USART1_TX_DMA_Instance
  &USART1_DMA_Tx,
#else
  NULL,
#endif
#ifdef MX_USART1_RX_DMA_Instance
  &USART1_DMA_Rx,
#else
  NULL,
#endif

  USART1_RX_TIMEOUT_VAL,
  &USART1_Info,
  &USART1_TransferInfo
};
#endif

// USART2
#ifdef MX_USART2

#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
#if (MX_USART2_VM == VM_ASYNC)
extern UART_HandleTypeDef huart2;
#elif (MX_USART2_VM == VM_SYNC)
extern USART_HandleTypeDef husart2;
#elif (MX_USART2_VM == VM_IRDA)
extern IRDA_HandleTypeDef hirda2;
#elif (MX_USART2_VM == VM_SMARTCARD)
extern SMARTCARD_HandleTypeDef hsmartcard2;
#else
#error "Incorrect virtual mode is selected"
#endif
#endif

// USART2 Run-Time Information
static USART_INFO          USART2_Info = { 0U, { 0U, 0U, 0U, 0U, 0U, 0U, 0U }, 0U, 0U, 0U };
static USART_TRANSFER_INFO USART2_TransferInfo;

#ifdef MX_USART2_TX_Pin
  static USART_PIN USART2_tx =  {MX_USART2_TX_GPIOx,  MX_USART2_TX_GPIO_Pin,  MX_USART2_TX_GPIO_AF,  MX_USART2_TX_GPIO_PuPd,  MX_USART2_TX_GPIO_Speed};
#endif
#ifdef MX_USART2_RX_Pin
  static USART_PIN USART2_rx =  {MX_USART2_RX_GPIOx,  MX_USART2_RX_GPIO_Pin,  MX_USART2_RX_GPIO_AF,  MX_USART2_RX_GPIO_PuPd,  MX_USART2_RX_GPIO_Speed};
#endif
#ifdef MX_USART2_CK_Pin
  static USART_PIN USART2_ck =  {MX_USART2_CK_GPIOx,  MX_USART2_CK_GPIO_Pin,  MX_USART2_CK_GPIO_AF,  MX_USART2_CK_GPIO_PuPd,  MX_USART2_CK_GPIO_Speed};
#endif
#ifdef MX_USART2_RTS_Pin
  static USART_PIN USART2_rts = {MX_USART2_RTS_GPIOx, MX_USART2_RTS_GPIO_Pin, MX_USART2_RTS_GPIO_AF, MX_USART2_RTS_GPIO_PuPd, MX_USART2_RTS_GPIO_Speed};
#endif
#ifdef MX_USART2_CTS_Pin
  static USART_PIN USART2_cts = {MX_USART2_CTS_GPIOx, MX_USART2_CTS_GPIO_Pin, MX_USART2_CTS_GPIO_AF, MX_USART2_CTS_GPIO_PuPd, MX_USART2_CTS_GPIO_Speed};
#endif

#ifdef MX_USART2_TX_DMA_Instance
  static void USART2_TX_DMA_Complete (DMA_HandleTypeDef *hdma);

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  static DMA_HandleTypeDef hdma_usart2_tx;
#else
  extern DMA_HandleTypeDef hdma_usart2_tx;
#endif
  static USART_DMA USART2_DMA_Tx = {
    &hdma_usart2_tx,
    USART2_TX_DMA_Complete,
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
    MX_USART2_TX_DMA_Instance,
    MX_USART2_TX_DMA_Channel,
    MX_USART2_TX_DMA_Priority,
    MX_USART2_TX_DMA_IRQn,
    { 0U, 0U, 0U }
#endif
  };
#endif
#ifdef MX_USART2_RX_DMA_Instance
  static void USART2_RX_DMA_Complete (DMA_HandleTypeDef *hdma);

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  static DMA_HandleTypeDef hdma_usart2_rx;
#else
  extern DMA_HandleTypeDef hdma_usart2_rx;
#endif
  static USART_DMA USART2_DMA_Rx = {
    &hdma_usart2_rx,
    USART2_RX_DMA_Complete,
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
    MX_USART2_RX_DMA_Instance,
    MX_USART2_RX_DMA_Channel,
    MX_USART2_RX_DMA_Priority,
    MX_USART2_RX_DMA_IRQn,
    { 0U, 0U, 0U }
#endif
  };
#endif

// USART2 Resources
static const USART_RESOURCES USART2_Resources = {
#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
#if (MX_USART2_VM == VM_ASYNC)
  &huart2,
  VM_ASYNC,
#elif (MX_USART2_VM == VM_SYNC)
  &husart2,
  VM_SYNC,
#elif (MX_USART2_VM == VM_IRDA)
  &hirda2,
  VM_IRDA,
#elif (MX_USART2_VM == VM_SMARTCARD)
  &hsmartcard2,
  VM_SMARTCARD,
#endif
#endif
  {     // Capabilities
    1,  // supports UART (Asynchronous) mode
#ifdef MX_USART2_CK_Pin
    1,  // supports Synchronous Master mode
#else
    0,  // supports Synchronous Master mode
#endif
    0,  // supports Synchronous Slave mode
    1,  // supports UART Single-wire mode
    1,  // supports UART IrDA mode
    1,  // supports UART Smart Card mode
    0,  // Smart Card Clock generator
#ifdef MX_USART2_RTS_Pin
    1,  // RTS Flow Control available
#else
    0,  // RTS Flow Control available
#endif
#ifdef MX_USART2_CTS_Pin
    1,  // CTS Flow Control available
#else
    0,  // CTS Flow Control available
#endif
    1,  // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    1,  // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
#ifdef MX_USART2_RTS_Pin
    1,  // RTS Line: 0=not available, 1=available
#else
    0,  // RTS Line: 0=not available, 1=available
#endif
#ifdef MX_USART2_CTS_Pin
    1,  // CTS Line: 0=not available, 1=available
#else
    0,  // CTS Line: 0=not available, 1=available
#endif
    0,  // DTR Line: 0=not available, 1=available
    0,  // DSR Line: 0=not available, 1=available
    0,  // DCD Line: 0=not available, 1=available
    0,  // RI Line: 0=not available, 1=available
#ifdef MX_USART2_CTS_Pin
    1,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#else
    0,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#endif
    0,  // Signal DSR change event: \ref ARM_USART_EVENT_DSR
    0,  // Signal DCD change event: \ref ARM_USART_EVENT_DCD
    0   // Signal RI change event: \ref ARM_USART_EVENT_RI
#if (defined(ARM_USART_API_VERSION) && (ARM_USART_API_VERSION >= 0x203U))
  , 0U  // Reserved bits
#endif
  },

    USART2,

  // PINS
  {
#ifdef MX_USART2_TX_Pin
    &USART2_tx,
#else
    NULL,
#endif
#ifdef MX_USART2_RX_Pin
    &USART2_rx,
#else
    NULL,
#endif
#ifdef MX_USART2_CK_Pin
    &USART2_ck,
#else
    NULL,
#endif
#ifdef MX_USART2_RTS_Pin 
    &USART2_rts,
#else
    NULL,
#endif
#ifdef MX_USART2_CTS_Pin 
    &USART2_cts,
#else
    NULL,
#endif
  },

    USART2_IRQn,
  { 0U, 0U, 0U },

#ifdef MX_USART2_TX_DMA_Instance
  &USART2_DMA_Tx,
#else
  NULL,
#endif
#ifdef MX_USART2_RX_DMA_Instance
  &USART2_DMA_Rx,
#else
  NULL,
#endif

  USART2_RX_TIMEOUT_VAL,
  &USART2_Info,
  &USART2_TransferInfo
};
#endif

// USART3
#ifdef MX_USART3

#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
#if (MX_USART3_VM == VM_ASYNC)
extern UART_HandleTypeDef huart3;
#elif (MX_USART3_VM == VM_SYNC)
extern USART_HandleTypeDef husart3;
#elif (MX_USART3_VM == VM_IRDA)
extern IRDA_HandleTypeDef hirda3;
#elif (MX_USART3_VM == VM_SMARTCARD)
extern SMARTCARD_HandleTypeDef hsmartcard3;
#else
#error "Incorrect virtual mode is selected"
#endif
#endif

// USART3 Run-Time Information
static USART_INFO          USART3_Info = { 0U, { 0U, 0U, 0U, 0U, 0U, 0U, 0U }, 0U, 0U, 0U };
static USART_TRANSFER_INFO USART3_TransferInfo;

#ifdef MX_USART3_TX_Pin
  static USART_PIN USART3_tx =  {MX_USART3_TX_GPIOx,  MX_USART3_TX_GPIO_Pin,  MX_USART3_TX_GPIO_AF,  MX_USART3_TX_GPIO_PuPd,  MX_USART3_TX_GPIO_Speed};
#endif
#ifdef MX_USART3_RX_Pin
  static USART_PIN USART3_rx =  {MX_USART3_RX_GPIOx,  MX_USART3_RX_GPIO_Pin,  MX_USART3_RX_GPIO_AF,  MX_USART3_RX_GPIO_PuPd,  MX_USART3_RX_GPIO_Speed};
#endif
#ifdef MX_USART3_CK_Pin
  static USART_PIN USART3_ck =  {MX_USART3_CK_GPIOx,  MX_USART3_CK_GPIO_Pin,  MX_USART3_CK_GPIO_AF,  MX_USART3_CK_GPIO_PuPd,  MX_USART3_CK_GPIO_Speed};
#endif
#ifdef MX_USART3_RTS_Pin
  static USART_PIN USART3_rts = {MX_USART3_RTS_GPIOx, MX_USART3_RTS_GPIO_Pin, MX_USART3_RTS_GPIO_AF, MX_USART3_RTS_GPIO_PuPd, MX_USART3_RTS_GPIO_Speed};
#endif
#ifdef MX_USART3_CTS_Pin
  static USART_PIN USART3_cts = {MX_USART3_CTS_GPIOx, MX_USART3_CTS_GPIO_Pin, MX_USART3_CTS_GPIO_AF, MX_USART3_CTS_GPIO_PuPd, MX_USART3_CTS_GPIO_Speed};
#endif

#ifdef MX_USART3_TX_DMA_Instance
  static void USART3_TX_DMA_Complete (DMA_HandleTypeDef *hdma);

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  static DMA_HandleTypeDef hdma_usart3_tx;
#else
  extern DMA_HandleTypeDef hdma_usart3_tx;
#endif
  static USART_DMA USART3_DMA_Tx = {
    &hdma_usart3_tx,
    USART3_TX_DMA_Complete,
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
    MX_USART3_TX_DMA_Instance,
    MX_USART3_TX_DMA_Channel,
    MX_USART3_TX_DMA_Priority,
    MX_USART3_TX_DMA_IRQn,
    { 0U, 0U, 0U }
#endif
  };
#endif
#ifdef MX_USART3_RX_DMA_Instance
  static void USART3_RX_DMA_Complete (DMA_HandleTypeDef *hdma);

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  static DMA_HandleTypeDef hdma_usart3_rx;
#else
  extern DMA_HandleTypeDef hdma_usart3_rx;
#endif
  static USART_DMA USART3_DMA_Rx = {
    &hdma_usart3_rx,
    USART3_RX_DMA_Complete,
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
    MX_USART3_RX_DMA_Instance,
    MX_USART3_RX_DMA_Channel,
    MX_USART3_RX_DMA_Priority,
    MX_USART3_RX_DMA_IRQn,
    { 0U, 0U, 0U }
#endif
  };
#endif

// USART3 Resources
static const USART_RESOURCES USART3_Resources = {
#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
#if (MX_USART3_VM == VM_ASYNC)
  &huart3,
  VM_ASYNC,
#elif (MX_USART3_VM == VM_SYNC)
  &husart3,
  VM_SYNC,
#elif (MX_USART3_VM == VM_IRDA)
  &hirda3,
  VM_IRDA,
#elif (MX_USART3_VM == VM_SMARTCARD)
  &hsmartcard3,
  VM_SMARTCARD,
#endif
#endif
  {     // Capabilities
    1,  // supports UART (Asynchronous) mode
#ifdef MX_USART3_CK_Pin
    1,  // supports Synchronous Master mode
#else
    0,  // supports Synchronous Master mode
#endif
    0,  // supports Synchronous Slave mode
    1,  // supports UART Single-wire mode
    1,  // supports UART IrDA mode
    1,  // supports UART Smart Card mode
    0,  // Smart Card Clock generator
#ifdef MX_USART3_RTS_Pin
    1,  // RTS Flow Control available
#else
    0,  // RTS Flow Control available
#endif
#ifdef MX_USART3_CTS_Pin
    1,  // CTS Flow Control available
#else
    0,  // CTS Flow Control available
#endif
    1,  // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    1,  // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
#ifdef MX_USART3_RTS_Pin
    1,  // RTS Line: 0=not available, 1=available
#else
    0,  // RTS Line: 0=not available, 1=available
#endif
#ifdef MX_USART3_CTS_Pin
    1,  // CTS Line: 0=not available, 1=available
#else
    0,  // CTS Line: 0=not available, 1=available
#endif
    0,  // DTR Line: 0=not available, 1=available
    0,  // DSR Line: 0=not available, 1=available
    0,  // DCD Line: 0=not available, 1=available
    0,  // RI Line: 0=not available, 1=available
#ifdef MX_USART3_CTS_Pin
    1,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#else
    0,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#endif
    0,  // Signal DSR change event: \ref ARM_USART_EVENT_DSR
    0,  // Signal DCD change event: \ref ARM_USART_EVENT_DCD
    0   // Signal RI change event: \ref ARM_USART_EVENT_RI
#if (defined(ARM_USART_API_VERSION) && (ARM_USART_API_VERSION >= 0x203U))
  , 0U  // Reserved bits
#endif
  },

    USART3,

  // PINS
  {
#ifdef MX_USART3_TX_Pin
    &USART3_tx,
#else
    NULL,
#endif
#ifdef MX_USART3_RX_Pin
    &USART3_rx,
#else
    NULL,
#endif
#ifdef MX_USART3_CK_Pin
    &USART3_ck,
#else
    NULL,
#endif
#ifdef MX_USART3_RTS_Pin 
    &USART3_rts,
#else
    NULL,
#endif
#ifdef MX_USART3_CTS_Pin 
    &USART3_cts,
#else
    NULL,
#endif
  },

    USART3_IRQn,
  { 0U, 0U, 0U },

#ifdef MX_USART3_TX_DMA_Instance
  &USART3_DMA_Tx,
#else
  NULL,
#endif
#ifdef MX_USART3_RX_DMA_Instance
  &USART3_DMA_Rx,
#else
  NULL,
#endif

  USART3_RX_TIMEOUT_VAL,
  &USART3_Info,
  &USART3_TransferInfo
};
#endif

// UART4
#ifdef MX_UART4

#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
#if (MX_UART4_VM == Asynchronous)
extern UART_HandleTypeDef huart4;
#elif (MX_UART4_VM == IrDA)
extern IRDA_HandleTypeDef hirda4;
#else
#error "Incorrect virtual mode is selected"
#endif
#endif

// UART4 Run-Time Information
static USART_INFO          UART4_Info = { 0U, { 0U, 0U, 0U, 0U, 0U, 0U, 0U }, 0U, 0U, 0U };
static USART_TRANSFER_INFO UART4_TransferInfo;

#ifdef MX_UART4_TX_Pin
  static USART_PIN UART4_tx  = {MX_UART4_TX_GPIOx,  MX_UART4_TX_GPIO_Pin,  MX_UART4_TX_GPIO_AF,  MX_UART4_TX_GPIO_PuPd,  MX_UART4_TX_GPIO_Speed};
#endif
#ifdef MX_UART4_RX_Pin
  static USART_PIN UART4_rx  = {MX_UART4_RX_GPIOx,  MX_UART4_RX_GPIO_Pin,  MX_UART4_RX_GPIO_AF,  MX_UART4_RX_GPIO_PuPd,  MX_UART4_RX_GPIO_Speed};
#endif
#ifdef MX_UART4_RTS_Pin
  static USART_PIN UART4_rts = {MX_UART4_RTS_GPIOx, MX_UART4_RTS_GPIO_Pin, MX_UART4_RTS_GPIO_AF, MX_UART4_RTS_GPIO_PuPd, MX_UART4_RTS_GPIO_Speed};
#endif
#ifdef MX_UART4_CTS_Pin
  static USART_PIN UART4_cts = {MX_UART4_CTS_GPIOx, MX_UART4_CTS_GPIO_Pin, MX_UART4_CTS_GPIO_AF, MX_UART4_CTS_GPIO_PuPd, MX_UART4_CTS_GPIO_Speed};
#endif

#ifdef MX_UART4_TX_DMA_Instance
  static void UART4_TX_DMA_Complete (DMA_HandleTypeDef *hdma);

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  static DMA_HandleTypeDef hdma_uart4_tx;
#else
  extern DMA_HandleTypeDef hdma_uart4_tx;
#endif
  static USART_DMA UART4_DMA_Tx = {
    &hdma_uart4_tx,
    UART4_TX_DMA_Complete,
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
    MX_UART4_TX_DMA_Instance,
    MX_UART4_TX_DMA_Channel,
    MX_UART4_TX_DMA_Priority,
    MX_UART4_TX_DMA_IRQn,
    { 0U, 0U, 0U }
#endif
  };
#endif
#ifdef MX_UART4_RX_DMA_Instance
  static void UART4_RX_DMA_Complete (DMA_HandleTypeDef *hdma);

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  static DMA_HandleTypeDef hdma_uart4_rx;
#else
  extern DMA_HandleTypeDef hdma_uart4_rx;
#endif
  static USART_DMA UART4_DMA_Rx = {
    &hdma_uart4_rx,
    UART4_RX_DMA_Complete,
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
    MX_UART4_RX_DMA_Instance,
    MX_UART4_RX_DMA_Channel,
    MX_UART4_RX_DMA_Priority,
    MX_UART4_RX_DMA_IRQn,
    { 0U, 0U, 0U }
#endif
  };
#endif

// UART4 Resources
static const USART_RESOURCES USART4_Resources = {
#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
#if (MX_UART4_VM == Asynchronous)
  &huart4,
  VM_ASYNC,
#elif (MX_UART4_VM == IrDA)
  &hirda4,
  VM_IRDA,
#endif
#endif
  {     // Capabilities
    1,  // supports UART (Asynchronous) mode
    0,  // supports Synchronous Master mode
    0,  // supports Synchronous Slave mode
    1,  // supports UART Single-wire mode
    1,  // supports UART IrDA mode
    0,  // supports UART Smart Card mode
    0,  // Smart Card Clock generator
#ifdef MX_UART4_RTS_Pin
    1,  // RTS Flow Control available
#else
    0,
#endif
#ifdef MX_UART4_CTS_Pin
    1,  // CTS Flow Control available
#else
    0,
#endif
    1,  // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    1,  // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
#ifdef MX_UART4_RTS_Pin
    1,  // RTS Line: 0=not available, 1=available
#else
    0,
#endif
#ifdef MX_UART4_CTS_Pin
    1,  // CTS Line: 0=not available, 1=available
#else
    0,
#endif
    0,  // DTR Line: 0=not available, 1=available
    0,  // DSR Line: 0=not available, 1=available
    0,  // DCD Line: 0=not available, 1=available
    0,  // RI Line: 0=not available, 1=available
#ifdef MX_UART4_CTS_Pin
    1,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#else
    0,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#endif
    0,  // Signal DSR change event: \ref ARM_USART_EVENT_DSR
    0,  // Signal DCD change event: \ref ARM_USART_EVENT_DCD
    0   // Signal RI change event: \ref ARM_USART_EVENT_RI
#if (defined(ARM_USART_API_VERSION) && (ARM_USART_API_VERSION >= 0x203U))
  , 0U  // Reserved bits
#endif
  },

    UART4,

  // PINS
  {
#ifdef MX_UART4_TX_Pin
    &UART4_tx,
#else
    NULL,
#endif
#ifdef MX_UART4_RX_Pin
    &UART4_rx,
#else
    NULL,
#endif
    NULL,
#ifdef MX_UART4_RTS_Pin
    &UART4_rts,
#else
    NULL,
#endif
#ifdef MX_UART4_CTS_Pin
    &UART4_cts,
#else
    NULL,
#endif
  },

    UART4_IRQn,
  { 0U, 0U, 0U },

#ifdef MX_UART4_TX_DMA_Instance
  &UART4_DMA_Tx,
#else
  NULL,
#endif
#ifdef MX_UART4_RX_DMA_Instance
  &UART4_DMA_Rx,
#else
  NULL,
#endif

  UART4_RX_TIMEOUT_VAL,
  &UART4_Info,
  &UART4_TransferInfo
};
#endif

// UART5
#ifdef MX_UART5

#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
#if (MX_UART5_VM == Asynchronous)
extern UART_HandleTypeDef huart5;
#elif (MX_UART5_VM == IrDA)
extern IRDA_HandleTypeDef hirda5;
#else
#error "Incorrect virtual mode is selected"
#endif
#endif

// UART5 Run-Time Information
static USART_INFO          UART5_Info = { 0U, { 0U, 0U, 0U, 0U, 0U, 0U, 0U }, 0U, 0U, 0U };
static USART_TRANSFER_INFO UART5_TransferInfo;

#ifdef MX_UART5_TX_Pin
  static USART_PIN UART5_tx  = {MX_UART5_TX_GPIOx,  MX_UART5_TX_GPIO_Pin,  MX_UART5_TX_GPIO_AF,  MX_UART5_TX_GPIO_PuPd, MX_UART5_TX_GPIO_Speed};
#endif
#ifdef MX_UART5_RX_Pin
  static USART_PIN UART5_rx  = {MX_UART5_RX_GPIOx,  MX_UART5_RX_GPIO_Pin,  MX_UART5_RX_GPIO_AF,  MX_UART5_RX_GPIO_PuPd, MX_UART5_RX_GPIO_Speed};
#endif
#ifdef MX_UART5_RTS_Pin
  static USART_PIN UART5_rts = {MX_UART5_RTS_GPIOx, MX_UART5_RTS_GPIO_Pin, MX_UART5_RTS_GPIO_AF, MX_UART5_RTS_GPIO_PuPd, MX_UART5_RTS_GPIO_Speed};
#endif
#ifdef MX_UART5_CTS_Pin
  static USART_PIN UART5_cts = {MX_UART5_CTS_GPIOx, MX_UART5_CTS_GPIO_Pin, MX_UART5_CTS_GPIO_AF, MX_UART5_CTS_GPIO_PuPd, MX_UART5_CTS_GPIO_Speed};
#endif

#ifdef MX_UART5_TX_DMA_Instance
  static void UART5_TX_DMA_Complete (DMA_HandleTypeDef *hdma);

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  static DMA_HandleTypeDef hdma_uart5_tx;
#else
  extern DMA_HandleTypeDef hdma_uart5_tx;
#endif
  static USART_DMA UART5_DMA_Tx = {
    &hdma_uart5_tx,
    UART5_TX_DMA_Complete,
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
    MX_UART5_TX_DMA_Instance,
    MX_UART5_TX_DMA_Channel,
    MX_UART5_TX_DMA_Priority,
    MX_UART5_TX_DMA_IRQn,
    { 0U, 0U, 0U }
#endif
  };
#endif
#ifdef MX_UART5_RX_DMA_Instance
  static void UART5_RX_DMA_Complete (DMA_HandleTypeDef *hdma);

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  static DMA_HandleTypeDef hdma_uart5_rx;
#else
  extern DMA_HandleTypeDef hdma_uart5_rx;
#endif
  static USART_DMA UART5_DMA_Rx = {
    &hdma_uart5_rx,
    UART5_RX_DMA_Complete,
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
    MX_UART5_RX_DMA_Instance,
    MX_UART5_RX_DMA_Channel,
    MX_UART5_RX_DMA_Priority,
    MX_UART5_RX_DMA_IRQn,
    { 0U, 0U, 0U }
#endif
  };
#endif

// UART5 Resources
static const USART_RESOURCES USART5_Resources = {
#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
#if (MX_UART5_VM == Asynchronous)
  &huart5,
  VM_ASYNC,
#elif (MX_UART5_VM == IrDA)
  &hirda5,
  VM_IRDA,
#endif
#endif
  {     // Capabilities
    1,  // supports UART (Asynchronous) mode
    0,  // supports Synchronous Master mode
    0,  // supports Synchronous Slave mode
    1,  // supports UART Single-wire mode
    1,  // supports UART IrDA mode
    0,  // supports UART Smart Card mode
    0,  // Smart Card Clock generator
#ifdef MX_UART5_RTS_Pin    
    1,  // RTS Flow Control available
#else
    0,
#endif
#ifdef MX_UART5_CTS_Pin
    1,  // CTS Flow Control available
#else
    0,
#endif
    1,  // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    1,  // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
#ifdef MX_UART5_RTS_Pin
    1,  // RTS Line: 0=not available, 1=available
#else
    0,
#endif
#ifdef MX_UART5_CTS_Pin
    1,  // CTS Line: 0=not available, 1=available
#else
    0,
#endif
    0,  // DTR Line: 0=not available, 1=available
    0,  // DSR Line: 0=not available, 1=available
    0,  // DCD Line: 0=not available, 1=available
    0,  // RI Line: 0=not available, 1=available
#ifdef MX_UART5_CTS_Pin
    1,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#else
    0,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#endif
    0,  // Signal DSR change event: \ref ARM_USART_EVENT_DSR
    0,  // Signal DCD change event: \ref ARM_USART_EVENT_DCD
    0   // Signal RI change event: \ref ARM_USART_EVENT_RI
#if (defined(ARM_USART_API_VERSION) && (ARM_USART_API_VERSION >= 0x203U))
  , 0U  // Reserved bits
#endif
  },

    UART5,

  // PINS
  {
#ifdef MX_UART5_TX_Pin
    &UART5_tx,
#else
    NULL,
#endif
#ifdef MX_UART5_RX_Pin
    &UART5_rx,
#else
    NULL,
#endif
    NULL,
#ifdef MX_UART5_RTS_Pin
    UART5_rts,
#else
    NULL,
#endif
#ifdef MX_UART5_CTS_Pin
    UART5_cts,
#else
    NULL,
#endif
  },

    UART5_IRQn,
  { 0U, 0U, 0U },

#ifdef MX_UART5_TX_DMA_Instance
  &UART5_DMA_Tx,
#else
  NULL,
#endif
#ifdef MX_UART5_RX_DMA_Instance
  &UART5_DMA_Rx,
#else
  NULL,
#endif

  UART5_RX_TIMEOUT_VAL,
  &UART5_Info,
  &UART5_TransferInfo
};
#endif

// USART6
#ifdef MX_USART6

#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
#if (MX_USART6_VM == VM_ASYNC)
extern UART_HandleTypeDef huart6;
#elif (MX_USART6_VM == VM_SYNC)
extern USART_HandleTypeDef husart6;
#elif (MX_USART6_VM == VM_IRDA)
extern IRDA_HandleTypeDef hirda6;
#elif (MX_USART6_VM == VM_SMARTCARD)
extern SMARTCARD_HandleTypeDef hsmartcard6;
#else
#error "Incorrect virtual mode is selected"
#endif
#endif

// USART6 Run-Time Information
static USART_INFO          USART6_Info = { 0U, { 0U, 0U, 0U, 0U, 0U, 0U, 0U }, 0U, 0U, 0U };
static USART_TRANSFER_INFO USART6_TransferInfo;

#ifdef MX_USART6_TX_Pin
  static USART_PIN USART6_tx  = {MX_USART6_TX_GPIOx,  MX_USART6_TX_GPIO_Pin,  MX_USART6_TX_GPIO_AF,  MX_USART6_TX_GPIO_PuPd,  MX_USART6_TX_GPIO_Speed};
#endif
#ifdef MX_USART6_RX_Pin
  static USART_PIN USART6_rx  = {MX_USART6_RX_GPIOx,  MX_USART6_RX_GPIO_Pin,  MX_USART6_RX_GPIO_AF,  MX_USART6_RX_GPIO_PuPd,  MX_USART6_RX_GPIO_Speed};
#endif
#ifdef MX_USART6_CK_Pin
  static USART_PIN USART6_ck  = {MX_USART6_CK_GPIOx,  MX_USART6_CK_GPIO_Pin,  MX_USART6_CK_GPIO_AF,  MX_USART6_CK_GPIO_PuPd,  MX_USART6_CK_GPIO_Speed};
#endif
#ifdef MX_USART6_RTS_Pin
  static USART_PIN USART6_rts = {MX_USART6_RTS_GPIOx, MX_USART6_RTS_GPIO_Pin, MX_USART6_RTS_GPIO_AF, MX_USART6_RTS_GPIO_PuPd, MX_USART6_RTS_GPIO_Speed};
#endif
#ifdef MX_USART6_CTS_Pin
  static USART_PIN USART6_cts = {MX_USART6_CTS_GPIOx, MX_USART6_CTS_GPIO_Pin, MX_USART6_CTS_GPIO_AF, MX_USART6_CTS_GPIO_PuPd, MX_USART6_CTS_GPIO_Speed};
#endif

#ifdef MX_USART6_TX_DMA_Instance
  static void USART6_TX_DMA_Complete (DMA_HandleTypeDef *hdma);

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  static DMA_HandleTypeDef hdma_usart6_tx;
#else
  extern DMA_HandleTypeDef hdma_usart6_tx;
#endif
  static USART_DMA USART6_DMA_Tx = {
    &hdma_usart6_tx,
    USART6_TX_DMA_Complete,
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
    MX_USART6_TX_DMA_Instance,
    MX_USART6_TX_DMA_Channel,
    MX_USART6_TX_DMA_Priority,
    MX_USART6_TX_DMA_IRQn,
    { 0U, 0U, 0U }
#endif
  };
#endif
#ifdef MX_USART6_RX_DMA_Instance
  static void USART6_RX_DMA_Complete (DMA_HandleTypeDef *hdma);

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  static DMA_HandleTypeDef hdma_usart6_rx;
#else
  extern DMA_HandleTypeDef hdma_usart6_rx;
#endif
  static USART_DMA USART6_DMA_Rx = {
    &hdma_usart6_rx,
    USART6_RX_DMA_Complete,
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
    MX_USART6_RX_DMA_Instance,
    MX_USART6_RX_DMA_Channel,
    MX_USART6_RX_DMA_Priority,
    MX_USART6_RX_DMA_IRQn,
    { 0U, 0U, 0U }
#endif
  };
#endif

// USART6 Resources
static const USART_RESOURCES USART6_Resources = {
#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
#if (MX_USART6_VM == VM_ASYNC)
  &huart6,
  VM_ASYNC,
#elif (MX_USART6_VM == VM_SYNC)
  &husart6,
  VM_SYNC,
#elif (MX_USART6_VM == VM_IRDA)
  &hirda6,
  VM_IRDA,
#elif (MX_USART6_VM == VM_SMARTCARD)
  &hsmartcard6,
  VM_SMARTCARD,
#endif
#endif
  {     // Capabilities
    1,  // supports UART (Asynchronous) mode
#ifdef MX_USART6_CK_Pin
    1,  // supports Synchronous Master mode
#else
    0,  // supports Synchronous Master mode
#endif
    0,  // supports Synchronous Slave mode
    1,  // supports UART Single-wire mode
    1,  // supports UART IrDA mode
    1,  // supports UART Smart Card mode
    0,  // Smart Card Clock generator
#ifdef MX_USART6_RTS_Pin
    1,  // RTS Flow Control available
#else
    0,  // RTS Flow Control available
#endif
#ifdef MX_USART6_CTS_Pin
    1,  // CTS Flow Control available
#else
    0,  // CTS Flow Control available
#endif
    1,  // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    1,  // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
#ifdef MX_USART6_RTS_Pin
    1,  // RTS Line: 0=not available, 1=available
#else
    0,  // RTS Line: 0=not available, 1=available
#endif
#ifdef MX_USART6_CTS_Pin
    1,  // CTS Line: 0=not available, 1=available
#else
    0,  // CTS Line: 0=not available, 1=available
#endif
    0,  // DTR Line: 0=not available, 1=available
    0,  // DSR Line: 0=not available, 1=available
    0,  // DCD Line: 0=not available, 1=available
    0,  // RI Line: 0=not available, 1=available
#ifdef MX_USART6_CTS_Pin
    1,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#else
    0,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#endif
    0,  // Signal DSR change event: \ref ARM_USART_EVENT_DSR
    0,  // Signal DCD change event: \ref ARM_USART_EVENT_DCD
    0   // Signal RI change event: \ref ARM_USART_EVENT_RI
#if (defined(ARM_USART_API_VERSION) && (ARM_USART_API_VERSION >= 0x203U))
  , 0U  // Reserved bits
#endif
  },

    USART6,

  // PINS
  {
#ifdef MX_USART6_TX_Pin
    &USART6_tx,
#else
    NULL,
#endif
#ifdef MX_USART6_RX_Pin
    &USART6_rx,
#else
    NULL,
#endif
#ifdef MX_USART6_CK_Pin
    &USART6_ck,
#else
    NULL,
#endif
#ifdef MX_USART6_RTS_Pin 
    &USART6_rts,
#else
    NULL,
#endif
#ifdef MX_USART6_CTS_Pin 
    &USART6_cts,
#else
    NULL,
#endif
  },

    USART6_IRQn,
  { 0U, 0U, 0U },

#ifdef MX_USART6_TX_DMA_Instance
  &USART6_DMA_Tx,
#else
  NULL,
#endif
#ifdef MX_USART6_RX_DMA_Instance
  &USART6_DMA_Rx,
#else
  NULL,
#endif

  USART6_RX_TIMEOUT_VAL,
  &USART6_Info,
  &USART6_TransferInfo
};
#endif

// UART7
#ifdef MX_UART7

#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
#if (MX_UART7_VM == Asynchronous)
extern UART_HandleTypeDef huart7;
#elif (MX_UART7_VM == IrDA)
extern IRDA_HandleTypeDef hirda7;
#else
#error "Incorrect virtual mode is selected"
#endif
#endif

// UART7 Run-Time Information
static USART_INFO          UART7_Info = { 0U, { 0U, 0U, 0U, 0U, 0U, 0U, 0U }, 0U, 0U, 0U };
static USART_TRANSFER_INFO UART7_TransferInfo;

#ifdef MX_UART7_TX_Pin
  static USART_PIN UART7_tx  = {MX_UART7_TX_GPIOx,  MX_UART7_TX_GPIO_Pin,  MX_UART7_TX_GPIO_AF,  MX_UART7_TX_GPIO_PuPd,  MX_UART7_TX_GPIO_Speed};
#endif
#ifdef MX_UART7_RX_Pin
  static USART_PIN UART7_rx  = {MX_UART7_RX_GPIOx,  MX_UART7_RX_GPIO_Pin,  MX_UART7_RX_GPIO_AF,  MX_UART7_RX_GPIO_PuPd,  MX_UART7_RX_GPIO_Speed};
#endif
#ifdef MX_UART7_RTS_Pin
  static USART_PIN UART7_rts = {MX_UART7_RTS_GPIOx, MX_UART7_RTS_GPIO_Pin, MX_UART7_RTS_GPIO_AF, MX_UART7_RTS_GPIO_PuPd, MX_UART7_RTS_GPIO_Speed};
#endif
#ifdef MX_UART7_CTS_Pin
  static USART_PIN UART7_cts = {MX_UART7_CTS_GPIOx, MX_UART7_CTS_GPIO_Pin, MX_UART7_CTS_GPIO_AF, MX_UART7_CTS_GPIO_PuPd, MX_UART7_CTS_GPIO_Speed};
#endif

#ifdef MX_UART7_TX_DMA_Instance
  static void UART7_TX_DMA_Complete (DMA_HandleTypeDef *hdma);

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  static DMA_HandleTypeDef hdma_uart7_tx;
#else
  extern DMA_HandleTypeDef hdma_uart7_tx;
#endif
  static USART_DMA UART7_DMA_Tx = {
    &hdma_uart7_tx,
    UART7_TX_DMA_Complete,
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
    MX_UART7_TX_DMA_Instance,
    MX_UART7_TX_DMA_Channel,
    MX_UART7_TX_DMA_Priority,
    MX_UART7_TX_DMA_IRQn,
    { 0U, 0U, 0U }
#endif
  };
#endif
#ifdef MX_UART7_RX_DMA_Instance
  static void UART7_RX_DMA_Complete (DMA_HandleTypeDef *hdma);

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  static DMA_HandleTypeDef hdma_uart7_rx;
#else
  extern DMA_HandleTypeDef hdma_uart7_rx;
#endif
  static USART_DMA UART7_DMA_Rx = {
    &hdma_uart7_rx,
    UART7_RX_DMA_Complete,
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
    MX_UART7_RX_DMA_Instance,
    MX_UART7_RX_DMA_Channel,
    MX_UART7_RX_DMA_Priority,
    MX_UART7_RX_DMA_IRQn,
    { 0U, 0U, 0U }
#endif
  };
#endif

// UART7 Resources
static const USART_RESOURCES USART7_Resources = {
#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
#if (MX_UART7_VM == Asynchronous)
  &huart7,
  VM_ASYNC,
#elif (MX_UART7_VM == IrDA)
  &hirda7,
  VM_IRDA,
#endif
#endif
  {     // Capabilities
    1,  // supports UART (Asynchronous) mode
    0,  // supports Synchronous Master mode
    0,  // supports Synchronous Slave mode
    1,  // supports UART Single-wire mode
    1,  // supports UART IrDA mode
    0,  // supports UART Smart Card mode
    0,  // Smart Card Clock generator
#ifdef MX_UART7_RTS_Pin
    1,  // RTS Flow Control available
#else
    0,
#endif
#ifdef MX_UART7_CTS_Pin
    1,  // CTS Flow Control available
#else
    0,
#endif
    1,  // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    1,  // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
#ifdef MX_UART7_RTS_Pin
    1,  // RTS Line: 0=not available, 1=available
#else
    0,
#endif
#ifdef MX_UART7_CTS_Pin
    1,  // CTS Line: 0=not available, 1=available
#else
    0,
#endif
    0,  // DTR Line: 0=not available, 1=available
    0,  // DSR Line: 0=not available, 1=available
    0,  // DCD Line: 0=not available, 1=available
    0,  // RI Line: 0=not available, 1=available
#ifdef MX_UART7_CTS_Pin
    1,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#else
    0,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#endif
    0,  // Signal DSR change event: \ref ARM_USART_EVENT_DSR
    0,  // Signal DCD change event: \ref ARM_USART_EVENT_DCD
    0   // Signal RI change event: \ref ARM_USART_EVENT_RI
#if (defined(ARM_USART_API_VERSION) && (ARM_USART_API_VERSION >= 0x203U))
  , 0U  // Reserved bits
#endif
  },

    UART7,

  // PINS
  {
#ifdef MX_UART7_TX_Pin
    &UART7_tx,
#else
    NULL,
#endif
#ifdef MX_UART7_RX_Pin
    &UART7_rx,
#else
    NULL,
#endif
    NULL,
#ifdef MX_UART7_RTS_Pin
    &UART7_rts,
#else
    NULL,
#endif
#ifdef MX_UART7_CTS_Pin
    &UART7_cts,
#else
    NULL,
#endif
  },

    UART7_IRQn,
  { 0U, 0U, 0U },

#ifdef MX_UART7_TX_DMA_Instance
  &UART7_DMA_Tx,
#else
  NULL,
#endif
#ifdef MX_UART7_RX_DMA_Instance
  &UART7_DMA_Rx,
#else
  NULL,
#endif

  UART7_RX_TIMEOUT_VAL,
  &UART7_Info,
  &UART7_TransferInfo
};
#endif

// UART8
#ifdef MX_UART8

#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
#if (MX_UART8_VM == Asynchronous)
extern UART_HandleTypeDef huart8;
#elif (MX_UART8_VM == IrDA)
extern IRDA_HandleTypeDef hirda8;
#else
#error "Incorrect virtual mode is selected"
#endif
#endif

// UART8 Run-Time Information
static USART_INFO          UART8_Info = { 0U, { 0U, 0U, 0U, 0U, 0U, 0U, 0U }, 0U, 0U, 0U };
static USART_TRANSFER_INFO UART8_TransferInfo;

#ifdef MX_UART8_TX_Pin
  static USART_PIN UART8_tx  = {MX_UART8_TX_GPIOx,  MX_UART8_TX_GPIO_Pin,  MX_UART8_TX_GPIO_AF,  MX_UART8_TX_GPIO_PuPd,  MX_UART8_TX_GPIO_Speed};
#endif
#ifdef MX_UART8_RX_Pin
  static USART_PIN UART8_rx  = {MX_UART8_RX_GPIOx,  MX_UART8_RX_GPIO_Pin,  MX_UART8_RX_GPIO_AF,  MX_UART8_RX_GPIO_PuPd,  MX_UART8_RX_GPIO_Speed};
#endif
#ifdef MX_UART8_RTS_Pin
  static USART_PIN UART8_rts = {MX_UART8_RTS_GPIOx, MX_UART8_RTS_GPIO_Pin, MX_UART8_RTS_GPIO_AF, MX_UART8_RTS_GPIO_PuPd, MX_UART8_RTS_GPIO_Speed};
#endif
#ifdef MX_UART8_CTS_Pin
  static USART_PIN UART8_cts = {MX_UART8_CTS_GPIOx, MX_UART8_CTS_GPIO_Pin, MX_UART8_CTS_GPIO_AF, MX_UART8_CTS_GPIO_PuPd, MX_UART8_CTS_GPIO_Speed};
#endif

#ifdef MX_UART8_TX_DMA_Instance
  static void UART8_TX_DMA_Complete (DMA_HandleTypeDef *hdma);

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  static DMA_HandleTypeDef hdma_uart8_tx;
#else
  extern DMA_HandleTypeDef hdma_uart8_tx;
#endif
  static USART_DMA UART8_DMA_Tx = {
    &hdma_uart8_tx,
    UART8_TX_DMA_Complete,
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
    MX_UART8_TX_DMA_Instance,
    MX_UART8_TX_DMA_Channel,
    MX_UART8_TX_DMA_Priority,
    MX_UART8_TX_DMA_IRQn,
    { 0U, 0U, 0U }
#endif
  };
#endif
#ifdef MX_UART8_RX_DMA_Instance
  static void UART8_RX_DMA_Complete (DMA_HandleTypeDef *hdma);

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  static DMA_HandleTypeDef hdma_uart8_rx;
#else
  extern DMA_HandleTypeDef hdma_uart8_rx;
#endif
  static USART_DMA UART8_DMA_Rx = {
    &hdma_uart8_rx,
    UART8_RX_DMA_Complete,
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
    MX_UART8_RX_DMA_Instance,
    MX_UART8_RX_DMA_Channel,
    MX_UART8_RX_DMA_Priority,
    MX_UART8_RX_DMA_IRQn,
    { 0U, 0U, 0U }
#endif
  };
#endif

// UART8 Resources
static const USART_RESOURCES USART8_Resources = {
#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
#if (MX_UART8_VM == Asynchronous)
  &huart8,
  VM_ASYNC,
#elif (MX_UART8_VM == IrDA)
  &hirda8,
  VM_IRDA,
#endif
#endif
  {     // Capabilities
    1,  // supports UART (Asynchronous) mode
    0,  // supports Synchronous Master mode
    0,  // supports Synchronous Slave mode
    1,  // supports UART Single-wire mode
    1,  // supports UART IrDA mode
    0,  // supports UART Smart Card mode
    0,  // Smart Card Clock generator
#ifdef MX_UART8_RTS_Pin
    1,  // RTS Flow Control available
#else
    0,
#endif
#ifdef MX_UART8_CTS_Pin
    1,  // CTS Flow Control available
#else
    0,
#endif
    1,  // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    1,  // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
#ifdef MX_UART8_RTS_Pin
    1,  // RTS Line: 0=not available, 1=available
#else
    0,
#endif
#ifdef MX_UART8_CTS_Pin
    1,  // CTS Line: 0=not available, 1=available
#else
    0,
#endif
    0,  // DTR Line: 0=not available, 1=available
    0,  // DSR Line: 0=not available, 1=available
    0,  // DCD Line: 0=not available, 1=available
    0,  // RI Line: 0=not available, 1=available
#ifdef MX_UART8_CTS_Pin
    1,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#else
    0,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#endif
    0,  // Signal DSR change event: \ref ARM_USART_EVENT_DSR
    0,  // Signal DCD change event: \ref ARM_USART_EVENT_DCD
    0   // Signal RI change event: \ref ARM_USART_EVENT_RI
#if (defined(ARM_USART_API_VERSION) && (ARM_USART_API_VERSION >= 0x203U))
  , 0U  // Reserved bits
#endif
  },

    UART8,

  // PINS
  {
#ifdef MX_UART8_TX_Pin
    &UART8_tx,
#else
    NULL,
#endif
#ifdef MX_UART8_RX_Pin
    &UART8_rx,
#else
    NULL,
#endif
    NULL,
#ifdef MX_UART8_RTS_Pin
    &UART8_rts,
#else
    NULL,
#endif
#ifdef MX_UART8_CTS_Pin
    &UART8_cts,
#else
    NULL,
#endif
  },

    UART8_IRQn,
  { 0U, 0U, 0U },

#ifdef MX_UART8_TX_DMA_Instance
  &UART8_DMA_Tx,
#else
  NULL,
#endif
#ifdef MX_UART8_RX_DMA_Instance
  &UART8_DMA_Rx,
#else
  NULL,
#endif

  UART8_RX_TIMEOUT_VAL,
  &UART8_Info,
  &UART8_TransferInfo
};
#endif

// Function prototypes
void USART_IRQHandler (const USART_RESOURCES *usart);
#ifdef __USART_DMA_TX
static void USART_TX_DMA_Complete(const USART_RESOURCES *usart);
#endif
#ifdef __USART_DMA_RX
static void USART_RX_DMA_Complete(const USART_RESOURCES *usart);
#endif
static int32_t USART_Receive (      void            *data,
                                    uint32_t         num,
                              const USART_RESOURCES *usart);


#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
/**
  \fn          void Enable_GPIO_Clock (GPIO_TypeDef *port)
  \brief       Enable GPIO clock
*/
static void Enable_GPIO_Clock (GPIO_TypeDef *GPIOx) {
#ifdef GPIOA
  if (GPIOx == GPIOA) __GPIOA_CLK_ENABLE();
#endif 
#ifdef GPIOB
  if (GPIOx == GPIOB) __GPIOB_CLK_ENABLE();
#endif
#ifdef GPIOC
  if (GPIOx == GPIOC) __GPIOC_CLK_ENABLE();
#endif
#ifdef GPIOD
  if (GPIOx == GPIOD) __GPIOD_CLK_ENABLE();
#endif
#ifdef GPIOE
  if (GPIOx == GPIOE) __GPIOE_CLK_ENABLE();
#endif
#ifdef GPIOF
  if (GPIOx == GPIOF) __GPIOF_CLK_ENABLE();
#endif
#ifdef GPIOG
  if (GPIOx == GPIOG) __GPIOG_CLK_ENABLE();
#endif
#ifdef GPIOH
  if (GPIOx == GPIOH) __GPIOH_CLK_ENABLE();
#endif
#ifdef GPIOI
  if (GPIOx == GPIOI) __GPIOI_CLK_ENABLE();
#endif
}
#endif

/**
  \fn          void USART_PeripheralReset (USART_TypeDef *usart)
  \brief       USART Reset
*/
static void USART_PeripheralReset (USART_TypeDef *usart) {
  
  if      (usart == USART1) { __HAL_RCC_USART1_FORCE_RESET(); }
  else if (usart == USART2) { __HAL_RCC_USART2_FORCE_RESET(); }
  else if (usart == USART3) { __HAL_RCC_USART3_FORCE_RESET(); }
  else if (usart == UART4)  { __HAL_RCC_UART4_FORCE_RESET();  }
  else if (usart == UART5)  { __HAL_RCC_UART5_FORCE_RESET();  }
  else if (usart == USART6) { __HAL_RCC_USART6_FORCE_RESET(); }
  else if (usart == UART7)  { __HAL_RCC_UART7_FORCE_RESET();  }
  else if (usart == UART8)  { __HAL_RCC_UART8_FORCE_RESET();  }

  __NOP(); __NOP(); __NOP(); __NOP();

  if      (usart == USART1) { __HAL_RCC_USART1_RELEASE_RESET(); }
  else if (usart == USART2) { __HAL_RCC_USART2_RELEASE_RESET(); }
  else if (usart == USART3) { __HAL_RCC_USART3_RELEASE_RESET(); }
  else if (usart == UART4)  { __HAL_RCC_UART4_RELEASE_RESET();  }
  else if (usart == UART5)  { __HAL_RCC_UART5_RELEASE_RESET();  }
  else if (usart == USART6) { __HAL_RCC_USART6_RELEASE_RESET(); }
  else if (usart == UART7)  { __HAL_RCC_UART7_RELEASE_RESET();  }
  else if (usart == UART8)  { __HAL_RCC_UART8_RELEASE_RESET();  }
}

/**
  \fn          uint32_t USART_GetClockFreq (USART_TypeDef *usart)
  \brief       Ger USART Clock Frequency
*/
static uint32_t USART_GetClockFreq (USART_TypeDef *usart) {
  uint32_t freq = 0;
  if (usart == USART1) {
    freq = LL_RCC_GetUSARTClockFreq (LL_RCC_USART1_CLKSOURCE);
  }
  else if (usart == USART2) {
    freq = LL_RCC_GetUSARTClockFreq (LL_RCC_USART2_CLKSOURCE);
  }
  else if (usart == USART3) {
    freq = LL_RCC_GetUSARTClockFreq (LL_RCC_USART3_CLKSOURCE);
  }
  else if (usart == UART4) {
    freq = LL_RCC_GetUARTClockFreq (LL_RCC_UART4_CLKSOURCE);
  }
  else if (usart == UART5) {
    freq = LL_RCC_GetUARTClockFreq (LL_RCC_UART5_CLKSOURCE);
  }
  else if (usart == USART6)  {
    freq = LL_RCC_GetUSARTClockFreq (LL_RCC_USART6_CLKSOURCE);
  }
  else if (usart == UART7) {
    freq = LL_RCC_GetUARTClockFreq (LL_RCC_UART7_CLKSOURCE);
  }
  else if (usart == UART8) {
    freq = LL_RCC_GetUARTClockFreq (LL_RCC_UART8_CLKSOURCE);
  }

  return freq;
}

// USART Driver functions

/**
  \fn          ARM_DRIVER_VERSION USARTx_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION USARTx_GetVersion (void) {
  return usart_driver_version;
}

/**
  \fn          ARM_USART_CAPABILITIES USART_GetCapabilities (const USART_RESOURCES *usart)
  \brief       Get driver capabilities
  \param[in]   usart     Pointer to USART resources
  \return      \ref ARM_USART_CAPABILITIES
*/
static ARM_USART_CAPABILITIES USART_GetCapabilities (const USART_RESOURCES *usart) {
  return usart->capabilities;
}

/**
  \fn          int32_t USART_Initialize (      ARM_USART_SignalEvent_t  cb_event
                                         const USART_RESOURCES         *usart)
  \brief       Initialize USART Interface.
  \param[in]   cb_event  Pointer to \ref ARM_USART_SignalEvent
  \param[in]   usart     Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_Initialize (      ARM_USART_SignalEvent_t  cb_event,
                                 const USART_RESOURCES         *usart) {

  if (usart->info->flags & USART_FLAG_INITIALIZED) {
    // Driver is already initialized
    return ARM_DRIVER_OK;
  }

  // Initialize USART Run-time Resources
  usart->info->cb_event = cb_event;

  usart->info->status.tx_busy          = 0U;
  usart->info->status.rx_busy          = 0U;
  usart->info->status.tx_underflow     = 0U;
  usart->info->status.rx_overflow      = 0U;
  usart->info->status.rx_break         = 0U;
  usart->info->status.rx_framing_error = 0U;
  usart->info->status.rx_parity_error  = 0U;

  usart->info->mode        = 0U;

  // Clear transfer information
  memset((void *)usart->xfer, 0, sizeof(USART_TRANSFER_INFO));

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  // Enable TX pin port clock
  if (usart->io.tx) {
    Enable_GPIO_Clock (usart->io.tx->port);
  }

  // Enable RX pin port clock
  if (usart->io.rx) {
    Enable_GPIO_Clock (usart->io.rx->port);
  }

  // Enable CLK pin port clock
  if (usart->io.ck) {
    Enable_GPIO_Clock (usart->io.ck->port);
  }

  // Configure RTS pin
  if (usart->io.rts) {
    Enable_GPIO_Clock (usart->io.rts->port);
  }

  // Configure CTS pin
  if (usart->io.cts) {
    Enable_GPIO_Clock (usart->io.cts->port);
  }

#ifdef __USART_DMA
  if (usart->dma_rx) {
    // Initialize USART RX DMA Resources
    memset((void *)usart->dma_rx->hdma, 0U, sizeof(DMA_HandleTypeDef));
    usart->dma_rx->hdma->Instance             = usart->dma_rx->stream;
    usart->dma_rx->hdma->Init.Channel         = usart->dma_rx->channel;
    usart->dma_rx->hdma->Init.Direction       = DMA_PERIPH_TO_MEMORY;
    usart->dma_rx->hdma->Init.Mode            = DMA_NORMAL;
    usart->dma_rx->hdma->Init.Priority        = usart->dma_rx->priority;
    usart->dma_rx->hdma->Init.FIFOMode        = DMA_FIFOMODE_DISABLE;
    usart->dma_rx->hdma->Init.FIFOThreshold   = DMA_FIFO_THRESHOLD_FULL;
    usart->dma_rx->hdma->Init.MemBurst        = DMA_MBURST_SINGLE;
    usart->dma_rx->hdma->Init.PeriphBurst     = DMA_PBURST_SINGLE;
    usart->dma_rx->hdma->Parent               = NULL;
    usart->dma_rx->hdma->XferCpltCallback     = usart->dma_rx->cb_complete;
    usart->dma_rx->hdma->XferHalfCpltCallback = NULL;
    usart->dma_rx->hdma->XferM1CpltCallback   = NULL;
    usart->dma_rx->hdma->XferErrorCallback    = NULL;
  }

  if (usart->dma_tx) {
    // Initialize USART TX DMA Resources
    memset((void *)usart->dma_tx->hdma, 0U, sizeof(DMA_HandleTypeDef));
    usart->dma_tx->hdma->Instance             = usart->dma_tx->stream;
    usart->dma_tx->hdma->Init.Channel         = usart->dma_tx->channel;
    usart->dma_tx->hdma->Init.Direction       = DMA_MEMORY_TO_PERIPH;
    usart->dma_tx->hdma->Init.Mode            = DMA_NORMAL;
    usart->dma_tx->hdma->Init.Priority        = usart->dma_tx->priority;
    usart->dma_tx->hdma->Init.FIFOMode        = DMA_FIFOMODE_DISABLE;
    usart->dma_tx->hdma->Init.FIFOThreshold   = DMA_FIFO_THRESHOLD_FULL;
    usart->dma_tx->hdma->Init.MemBurst        = DMA_MBURST_SINGLE;
    usart->dma_tx->hdma->Init.PeriphBurst     = DMA_PBURST_SINGLE;
    usart->dma_tx->hdma->Parent               = NULL;
    usart->dma_tx->hdma->XferCpltCallback     = usart->dma_tx->cb_complete;
    usart->dma_tx->hdma->XferHalfCpltCallback = NULL;
    usart->dma_tx->hdma->XferM1CpltCallback   = NULL;
    usart->dma_tx->hdma->XferErrorCallback    = NULL;
  }

  // Enable DMA clock
  if ((usart->reg == USART1) || (usart->reg == USART6)) {
    // DMA2 used for USART1 and USART6
    __DMA2_CLK_ENABLE();
  } else {
    __DMA1_CLK_ENABLE();
  }
#endif
#else
  if (usart->dma_rx) {
    usart->dma_rx->hdma->XferCpltCallback = usart->dma_rx->cb_complete;
  }
  if (usart->dma_tx) {
    usart->dma_tx->hdma->XferCpltCallback = usart->dma_tx->cb_complete;
  }
#endif

  usart->info->flags = USART_FLAG_INITIALIZED;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USART_Uninitialize (const USART_RESOURCES *usart)
  \brief       De-initialize USART Interface.
  \param[in]   usart     Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_Uninitialize (const USART_RESOURCES *usart) {

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  // Unconfigure USART pins
  if (usart->io.tx)  { HAL_GPIO_DeInit(usart->io.tx->port,  usart->io.tx->pin);  }
  if (usart->io.rx)  { HAL_GPIO_DeInit(usart->io.rx->port,  usart->io.rx->pin);  }
  if (usart->io.ck)  { HAL_GPIO_DeInit(usart->io.ck->port,  usart->io.ck->pin);  }
  if (usart->io.rts) { HAL_GPIO_DeInit(usart->io.rts->port, usart->io.rts->pin); }
  if (usart->io.cts) { HAL_GPIO_DeInit(usart->io.cts->port, usart->io.cts->pin); }

  if (usart->dma_rx) { usart->dma_rx->hdma->Instance = NULL; }
  if (usart->dma_tx) { usart->dma_tx->hdma->Instance = NULL; }
#else

  switch (usart->vmode) {
#ifdef USART_ASYNC
    case VM_ASYNC:
      ((UART_HandleTypeDef*)usart->h)->Instance      = NULL;
      break;
#endif
#ifdef USART_SYNC
    case VM_SYNC:
      ((USART_HandleTypeDef*)usart->h)->Instance     = NULL;
      break;
#endif
#ifdef USART_IRDA
    case VM_IRDA:
      ((IRDA_HandleTypeDef*)usart->h)->Instance      = NULL;
      break;
#endif
#ifdef USART_SMARTCARD
    case VM_SMARTCARD:
      ((SMARTCARD_HandleTypeDef*)usart->h)->Instance = NULL;
      break;
#endif
  }
#endif

  // Reset USART status flags
  usart->info->flags = 0U;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USART_PowerControl (ARM_POWER_STATE state)
  \brief       Control USART Interface Power.
  \param[in]   state  Power state
  \param[in]   usart  Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_PowerControl (      ARM_POWER_STATE  state,
                                   const USART_RESOURCES *usart) {

  if ((state != ARM_POWER_OFF)  && 
      (state != ARM_POWER_FULL) && 
      (state != ARM_POWER_LOW)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  switch (state) {
    case ARM_POWER_OFF:
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
      HAL_NVIC_DisableIRQ (usart->irq_num);
#ifdef __USART_DMA
      if (usart->dma_rx) {
        if (usart->dma_rx->hdma->Instance) {
          if (usart->info->status.rx_busy != 0) {
            // DMA disable receiver
            usart->reg->CR3 &= ~USART_CR3_DMAR;
            // Abort RX DMA transfer
            HAL_DMA_Abort (usart->dma_rx->hdma);
          }
          // Disable DMA IRQ in NVIC
          HAL_NVIC_DisableIRQ (usart->dma_rx->irq_num);
          // Deinitialize DMA
          HAL_DMA_DeInit (usart->dma_rx->hdma);
        }
      }

      if (usart->dma_tx) {
        if (usart->dma_tx->hdma->Instance) {
          if (usart->info->status.tx_busy != 0) {
            // DMA disable transmitter
            usart->reg->CR3 &= ~USART_CR3_DMAT;
            // Abort TX DMA transfer
            HAL_DMA_Abort (usart->dma_tx->hdma);
          }
          // Disable DMA IRQ in NVIC
          HAL_NVIC_DisableIRQ (usart->dma_tx->irq_num);
          // Deinitialize DMA
          HAL_DMA_DeInit (usart->dma_tx->hdma);
        }
      }
      // USART peripheral reset
      USART_PeripheralReset (usart->reg);
#endif
#else
      // USART peripheral reset
      USART_PeripheralReset (usart->reg);
      switch (usart->vmode) {
#ifdef USART_ASYNC
        case VM_ASYNC:
          if (((UART_HandleTypeDef*)usart->h)->Instance != NULL) {
            HAL_UART_MspDeInit ((UART_HandleTypeDef*) usart->h);
          }
          break;
#endif
#ifdef USART_SYNC
        case VM_SYNC:
          if (((USART_HandleTypeDef*)usart->h)->Instance != NULL) {
            HAL_USART_MspDeInit ((USART_HandleTypeDef*) usart->h);
          }
          break;
#endif
#ifdef USART_IRDA
        case VM_IRDA:
          if (((IRDA_HandleTypeDef*)usart->h)->Instance != NULL) {
            HAL_IRDA_MspDeInit ((IRDA_HandleTypeDef*) usart->h);
          }
          break;
#endif
#ifdef USART_SMARTCARD
        case VM_SMARTCARD:
          if (((SMARTCARD_HandleTypeDef*)usart->h)->Instance != NULL) {
            HAL_SMARTCARD_MspDeInit ((SMARTCARD_HandleTypeDef*) usart->h);
          }
          break;
#endif
        default: break;
      }
#endif

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
      // Disable USART clock
      if      (usart->reg == USART1) { __HAL_RCC_USART1_CLK_DISABLE(); }
      else if (usart->reg == USART2) { __HAL_RCC_USART2_CLK_DISABLE(); }
      else if (usart->reg == USART3) { __HAL_RCC_USART3_CLK_DISABLE(); }
      else if (usart->reg == UART4)  { __HAL_RCC_UART4_CLK_DISABLE();  }
      else if (usart->reg == UART5)  { __HAL_RCC_UART5_CLK_DISABLE();  }
      else if (usart->reg == USART6) { __HAL_RCC_USART6_CLK_DISABLE(); }
      else if (usart->reg == UART7)  { __HAL_RCC_UART7_CLK_DISABLE();  }
      else if (usart->reg == UART8)  { __HAL_RCC_UART8_CLK_DISABLE();  }
#endif

      // Clear Status flags
      usart->info->status.tx_busy          = 0U;
      usart->info->status.rx_busy          = 0U;
      usart->info->status.tx_underflow     = 0U;
      usart->info->status.rx_overflow      = 0U;
      usart->info->status.rx_break         = 0U;
      usart->info->status.rx_framing_error = 0U;
      usart->info->status.rx_parity_error  = 0U;
      usart->xfer->send_active             = 0U;

      usart->info->flags &= ~USART_FLAG_POWERED;
      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_FULL:
      if ((usart->info->flags & USART_FLAG_INITIALIZED) == 0U) {
        return ARM_DRIVER_ERROR;
      }
      if ((usart->info->flags & USART_FLAG_POWERED)     != 0U) {
        return ARM_DRIVER_OK;
      }

      // Clear Status flags
      usart->info->status.tx_busy          = 0U;
      usart->info->status.rx_busy          = 0U;
      usart->info->status.tx_underflow     = 0U;
      usart->info->status.rx_overflow      = 0U;
      usart->info->status.rx_break         = 0U;
      usart->info->status.rx_framing_error = 0U;
      usart->info->status.rx_parity_error  = 0U;

      usart->xfer->send_active             = 0U;
      usart->xfer->def_val                 = 0U;
      usart->xfer->sync_mode               = 0U;
      usart->xfer->break_flag              = 0U;
      usart->info->mode                    = 0U;
      usart->info->flow_control            = 0U;

      usart->info->flags = USART_FLAG_POWERED | USART_FLAG_INITIALIZED;

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
      // Enable USART clock
      if      (usart->reg == USART1) { __HAL_RCC_USART1_CLK_ENABLE(); }
      else if (usart->reg == USART2) { __HAL_RCC_USART2_CLK_ENABLE(); }
      else if (usart->reg == USART3) { __HAL_RCC_USART3_CLK_ENABLE(); }
      else if (usart->reg == UART4)  { __HAL_RCC_UART4_CLK_ENABLE();  }
      else if (usart->reg == UART5)  { __HAL_RCC_UART5_CLK_ENABLE();  }
      else if (usart->reg == USART6) { __HAL_RCC_USART6_CLK_ENABLE(); }
      else if (usart->reg == UART7)  { __HAL_RCC_UART7_CLK_ENABLE();  }
      else if (usart->reg == UART8)  { __HAL_RCC_UART8_CLK_ENABLE();  }

      // Clear and Enable USART IRQ
      NVIC_ClearPendingIRQ(usart->irq_num);
      NVIC_EnableIRQ(usart->irq_num);

#ifdef __USART_DMA
      if (usart->dma_rx) {
        // Clear and Enable DMA IRQ in NVIC
        NVIC_ClearPendingIRQ(usart->dma_rx->irq_num);
        NVIC_EnableIRQ(usart->dma_rx->irq_num);
      }

      if (usart->dma_tx) {
        // Clear and Enable DMA IRQ in NVIC
        NVIC_ClearPendingIRQ(usart->dma_tx->irq_num);
        NVIC_EnableIRQ(usart->dma_tx->irq_num);
      }
#endif
#else
      switch (usart->vmode) {
#ifdef USART_ASYNC
        case VM_ASYNC:
          ((UART_HandleTypeDef*)usart->h)->Instance = usart->reg;
          HAL_UART_MspInit ((UART_HandleTypeDef*) usart->h);
          break;
#endif
#ifdef USART_SYNC
        case VM_SYNC:
          ((USART_HandleTypeDef*)usart->h)->Instance = usart->reg;
          HAL_USART_MspInit ((USART_HandleTypeDef*) usart->h);
          break;
#endif
#ifdef USART_IRDA
        case VM_IRDA:
          ((IRDA_HandleTypeDef*)usart->h)->Instance = usart->reg;
          HAL_IRDA_MspInit ((IRDA_HandleTypeDef*) usart->h);
          break;
#endif
#ifdef USART_SMARTCARD
        case VM_SMARTCARD:
          ((SMARTCARD_HandleTypeDef*)usart->h)->Instance = usart->reg;
          HAL_SMARTCARD_MspInit ((SMARTCARD_HandleTypeDef*) usart->h);
          break;
#endif
        default: break;
      }
#endif

      // USART peripheral reset
      USART_PeripheralReset (usart->reg);

    break;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USART_Send (const void            *data,
                                         uint32_t         num,
                                   const USART_RESOURCES *usart)
  \brief       Start sending data to USART transmitter.
  \param[in]   data  Pointer to buffer with data to send to USART transmitter
  \param[in]   num   Number of data items to send
  \param[in]   usart Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_Send (const void            *data,
                                 uint32_t         num,
                           const USART_RESOURCES *usart) {
  int32_t stat;

#ifdef __USART_DMA_TX
  uint32_t cr1;
  uint32_t source_inc = DMA_MINC_ENABLE;
  uint32_t dma_flag;
#endif

  if ((data == NULL) || (num == 0U)) {
    // Invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((usart->info->flags & USART_FLAG_CONFIGURED) == 0U) {
    // USART is not configured (mode not selected)
    return ARM_DRIVER_ERROR;
  }

  if (usart->xfer->send_active != 0U) {
    // Send is not completed yet
    return ARM_DRIVER_ERROR_BUSY;
  }

  // Set Send active flag
  usart->xfer->send_active = 1U;

  // Save transmit buffer info
  usart->xfer->tx_buf = data;
  usart->xfer->tx_num = num;
  usart->xfer->tx_cnt = 0U;

  // Synchronous mode
  if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {
    if (usart->xfer->sync_mode == 0U) {
      usart->xfer->sync_mode = USART_SYNC_MODE_TX;
      // Start dummy reads
      stat = USART_Receive (&usart->xfer->dump_val, num, usart);
      if (stat == ARM_DRIVER_ERROR_BUSY) { return ARM_DRIVER_ERROR_BUSY; }

#ifdef __USART_DMA_TX
    } else {
      if (usart->xfer->sync_mode == USART_SYNC_MODE_RX) {
        // Dummy DMA writes (do not increment source address)
        source_inc = DMA_MINC_DISABLE;
      }
#endif
    }
  }

#ifdef __USART_DMA_TX
  dma_flag = 0U;
  if (usart->dma_tx != NULL) {
#if (USART_DCACHE == 1U)
    // Is DCache enabled
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
      if ((((uint32_t)data & 0x1FU) == 0) && ((num & 0x1FU) == 0)) {
        // Data is 32 Byte aligned and size is n *32 -> DMA can be used
        dma_flag = 1U;
        // Clean data cache: ensure data coherency between DMA and CPU
        SCB_CleanDCache_by_Addr ((uint32_t *)((uint32_t)data), (int32_t)num);
      }
    } else {
      dma_flag = 1U;
    }
#else
    dma_flag = 1U;
#endif
  }

  // DMA mode
  if (dma_flag == 1U) {

    // Prepare DMA to send TX data
    usart->dma_tx->hdma->Init.PeriphInc             = DMA_PINC_DISABLE;
    usart->dma_tx->hdma->Init.MemInc                = source_inc;

    cr1 = usart->reg->CR1;
    if (((cr1 & USART_CR1_M) != 0U) && ((cr1 & USART_CR1_PCE) == 0U)) {
      // 9-bit data frame, no parity
      usart->dma_tx->hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
      usart->dma_tx->hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    } else {
      // 8-bit data frame
      usart->dma_tx->hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      usart->dma_tx->hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    }

    // Initialize and start USART TX DMA Stream
    if (HAL_DMA_Init     (usart->dma_tx->hdma) != HAL_OK) { return ARM_DRIVER_ERROR; }
    if (HAL_DMA_Start_IT (usart->dma_tx->hdma, (uint32_t)usart->xfer->tx_buf, (uint32_t)(&usart->reg->TDR), num) != HAL_OK) {
      return ARM_DRIVER_ERROR;
    }

    // DMA Enable transmitter
    usart->reg->CR3 |= USART_CR3_DMAT;
  } else
#endif
  // Interrupt mode
  {
    // TXE interrupt enable
    usart->reg->CR1 |= USART_CR1_TXEIE;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USART_Receive (      void            *data,
                                            uint32_t         num,
                                      const USART_RESOURCES *usart)
  \brief       Start receiving data from USART receiver.
  \param[out]  data  Pointer to buffer for data to receive from USART receiver
  \param[in]   num   Number of data items to receive
  \param[in]   usart Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_Receive (      void            *data,
                                    uint32_t         num,
                              const USART_RESOURCES *usart) {

  int32_t stat;
#ifdef __USART_DMA_RX
  uint32_t cr1;
  uint32_t dest_inc = DMA_MINC_ENABLE;
  uint8_t dma_flag;
#endif

  if ((data == NULL) || (num == 0U)) {
    // Invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((usart->info->flags & USART_FLAG_CONFIGURED) == 0U) {
    // USART is not configured (mode not selected)
    return ARM_DRIVER_ERROR;
  }

  // Check if receiver is busy
  if (usart->info->status.rx_busy == 1U) {
    return ARM_DRIVER_ERROR_BUSY;
  }

  // Disable RXNE Interrupt
  usart->reg->CR1 &= ~USART_CR1_RXNEIE;

  // Save number of data to be received
  usart->xfer->rx_num = num;

  // Clear RX statuses
  usart->info->status.rx_break          = 0U;
  usart->info->status.rx_framing_error  = 0U;
  usart->info->status.rx_overflow       = 0U;
  usart->info->status.rx_parity_error   = 0U;

  // Save receive buffer info
  usart->xfer->rx_buf = (uint8_t *)data;
  usart->xfer->rx_cnt =  0U;

  // Set RX busy flag
  usart->info->status.rx_busy = 1U;

#ifdef __USART_DMA_RX
  dma_flag = 0U;
  if (usart->dma_rx != NULL) {
#if (USART_DCACHE == 1U)
    // Is DCache enabled
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
      if ((((uint32_t)data & 0x1FU) == 0) && ((num & 0x1FU) == 0)) {
        // Data is 32 Byte aligned and size is n *32 -> DMA can be used
        dma_flag = 1U;
      }
    } else {
      dma_flag = 1U;
    }
#else
    dma_flag = 1U;
#endif
  }

  // DMA mode
  if (dma_flag == 1U) {

    // Synchronous mode
    if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {
      if (usart->xfer->sync_mode == USART_SYNC_MODE_TX) {
        // Dummy DMA reads (do not increment destination address)
        dest_inc = DMA_MINC_DISABLE;
      }
    }

    // Prepare DMA to send RX data
    usart->dma_rx->hdma->Init.PeriphInc             = DMA_PINC_DISABLE;
    usart->dma_rx->hdma->Init.MemInc                = dest_inc;

    cr1 = usart->reg->CR1;
    if (((cr1 & USART_CR1_M) != 0U) && ((cr1 & USART_CR1_PCE) == 0U)) {
      // 9-bit data frame, no parity
      usart->dma_rx->hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
      usart->dma_rx->hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    } else {
      // 8 - bit data frame
      usart->dma_rx->hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
      usart->dma_rx->hdma->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    }

    // Initialize and start USART RX DMA Stream
    if (HAL_DMA_Init     (usart->dma_rx->hdma) != HAL_OK) { return ARM_DRIVER_ERROR; }
    if (HAL_DMA_Start_IT (usart->dma_rx->hdma, (uint32_t)(&usart->reg->RDR), (uint32_t)usart->xfer->rx_buf, num) != HAL_OK) {
      return ARM_DRIVER_ERROR;
    }
    // DMA enable Receiver
    usart->reg->CR3 |= USART_CR3_DMAR;
    // Enable Receiver Timeout interrupt
    usart->reg->CR1 |= USART_CR1_RTOIE;
  } else
#endif
  {
    // Enable RXNE and RTO interrupt
    usart->reg->CR1 |= (USART_CR1_RXNEIE| USART_CR1_RTOIE);
  }  

  // Synchronous mode
  if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {
    if (usart->xfer->sync_mode == 0U) {
      usart->xfer->sync_mode = USART_SYNC_MODE_RX;
      // Send dummy data
      stat = USART_Send (&usart->xfer->def_val, num, usart);
      if (stat == ARM_DRIVER_ERROR_BUSY) { return ARM_DRIVER_ERROR_BUSY; }
    }
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USART_Transfer (const void             *data_out,
                                             void             *data_in,
                                             uint32_t          num,
                                       const USART_RESOURCES  *usart)
  \brief       Start sending/receiving data to/from USART transmitter/receiver.
  \param[in]   data_out  Pointer to buffer with data to send to USART transmitter
  \param[out]  data_in   Pointer to buffer for data to receive from USART receiver
  \param[in]   num       Number of data items to transfer
  \param[in]   usart     Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_Transfer (const void             *data_out,
                                     void             *data_in,
                                     uint32_t          num,
                               const USART_RESOURCES  *usart) {
  int32_t status;

  if ((data_out == NULL) || (data_in == NULL) || (num == 0U)) {
    // Invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((usart->info->flags & USART_FLAG_CONFIGURED) == 0U) {
    // USART is not configured
    return ARM_DRIVER_ERROR;
  }

  if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {

    // Set xfer mode
    usart->xfer->sync_mode = USART_SYNC_MODE_TX_RX;

    // Receive
    status = USART_Receive (data_in, num, usart);
    if (status != ARM_DRIVER_OK) { return status; }

    // Send
    status = USART_Send (data_out, num, usart);
    if (status != ARM_DRIVER_OK) { return status; }

  } else {
    // Only in synchronous mode
    return ARM_DRIVER_ERROR;
  }
  return ARM_DRIVER_OK;
}

/**
  \fn          uint32_t USART_GetTxCount (const USART_RESOURCES *usart)
  \brief       Get transmitted data count.
  \param[in]   usart     Pointer to USART resources
  \return      number of data items transmitted
*/
static uint32_t USART_GetTxCount (const USART_RESOURCES *usart) {

#ifdef __USART_DMA_TX
  if (usart->dma_tx) {
    return (usart->xfer->tx_num - __HAL_DMA_GET_COUNTER(usart->dma_tx->hdma));
  } else
#endif
  {
    return usart->xfer->tx_cnt;
  }
}

/**
  \fn          uint32_t USART_GetRxCount (const USART_RESOURCES *usart)
  \brief       Get received data count.
  \param[in]   usart     Pointer to USART resources
  \return      number of data items received
*/
static uint32_t USART_GetRxCount (const USART_RESOURCES *usart) {

#ifdef __USART_DMA_RX
  if (usart->dma_rx) {
    return (usart->xfer->rx_num - __HAL_DMA_GET_COUNTER(usart->dma_rx->hdma));
  } else
#endif
  {
    return usart->xfer->rx_cnt;
  }
}

/**
  \fn          int32_t USART_Control (      uint32_t          control,
                                            uint32_t          arg,
                                      const USART_RESOURCES  *usart)
  \brief       Control USART Interface.
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \param[in]   usart    Pointer to USART resources
  \return      common \ref execution_status and driver specific \ref usart_execution_status
*/
static int32_t USART_Control (      uint32_t          control,
                                    uint32_t          arg,
                              const USART_RESOURCES  *usart) {
  uint32_t val, mode, flow_control, br, i;
  uint32_t cr1, cr2, cr3;
  GPIO_InitTypeDef GPIO_InitStruct;

  if ((usart->info->flags & USART_FLAG_POWERED) == 0U) {
    // USART not powered
    return ARM_DRIVER_ERROR;
  }

  cr1  = 0U;
  cr2  = 0U;
  cr3  = 0U;

  switch (control & ARM_USART_CONTROL_Msk) {
     // Control break
    case ARM_USART_CONTROL_BREAK:
      if (arg) {
        if (usart->xfer->send_active != 0U) { return ARM_DRIVER_ERROR_BUSY; }

        // Set Send active and Break flag
        usart->xfer->send_active = 1U;
        usart->xfer->break_flag  = 1U;

        // Send break and enable TX interrupt
        usart->reg->CR1 |= USART_CR1_TXEIE;
        usart->reg->RQR |= USART_RQR_SBKRQ;
      } else {
          if (usart->xfer->break_flag) {
            // Disable TX interrupt
            usart->reg->CR1 &= ~USART_CR1_TXEIE;

            // Clear break and Send Active flag
            usart->xfer->break_flag  = 0U;
            usart->xfer->send_active = 0U;
          }
      }
      return ARM_DRIVER_OK;

    // Abort Send
    case ARM_USART_ABORT_SEND:
      // Disable TX and TC interrupt
      usart->reg->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_TCIE);

      // If DMA mode - disable DMA channel
      if ((usart->dma_tx != NULL) && (usart->xfer->send_active != 0)) {
        // DMA disable transmitter
        usart->reg->CR3 &= ~USART_CR3_DMAT;

        // Abort TX DMA transfer
        HAL_DMA_Abort (usart->dma_tx->hdma);
      }

      // Clear break flag
      usart->xfer->break_flag = 0U;

      // Clear Send active flag
      usart->xfer->send_active = 0U;
      return ARM_DRIVER_OK;

    // Abort receive
    case ARM_USART_ABORT_RECEIVE:
      // Disable RX interrupt
      usart->reg->CR1 &= ~USART_CR1_RXNEIE;

      // If DMA mode - disable DMA channel
      if ((usart->dma_rx != NULL) && (usart->info->status.rx_busy != 0)) {
        // DMA disable Receiver
        usart->reg->CR3 &= ~USART_CR3_DMAR;

        // Abort RX DMA transfer
        HAL_DMA_Abort (usart->dma_rx->hdma);
      }

      // Clear RX busy status
      usart->info->status.rx_busy = 0U;

      return ARM_DRIVER_OK;

    // Abort transfer
    case ARM_USART_ABORT_TRANSFER:
      // Disable TX, TC and RX interrupt
      usart->reg->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_RXNEIE);

      // If DMA mode - disable DMA channel
      if ((usart->dma_tx != NULL) && (usart->xfer->send_active != 0U)) {
        // DMA disable transmitter
        usart->reg->CR3 &= ~USART_CR3_DMAT;

        // Abort TX DMA transfer
        HAL_DMA_Abort (usart->dma_tx->hdma);
      }

      // If DMA mode - disable DMA channel
      if ((usart->dma_rx != NULL) && (usart->info->status.rx_busy != 0U)) {
        // DMA disable Receiver
        usart->reg->CR3 &= ~USART_CR3_DMAR;

        // Abort RX DMA transfer
        HAL_DMA_Abort (usart->dma_rx->hdma);
      }

      // Clear busy statuses
      usart->info->status.rx_busy = 0U;
      usart->xfer->send_active    = 0U;
      return ARM_DRIVER_OK;

    // Control TX
    case ARM_USART_CONTROL_TX:
      // Check if TX pin available
      if (usart->io.tx == NULL) { return ARM_DRIVER_ERROR; }
      if (arg) {
        if (usart->info->mode != ARM_USART_MODE_SMART_CARD) {
          // USART TX pin function selected
          GPIO_InitStruct.Pin       = usart->io.tx->pin;
          GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
          GPIO_InitStruct.Pull      = usart->io.tx->pupd;
          GPIO_InitStruct.Speed     = usart->io.tx->speed;
          GPIO_InitStruct.Alternate = usart->io.tx->af;
          HAL_GPIO_Init(usart->io.tx->port, &GPIO_InitStruct);
        }
        usart->info->flags |= USART_FLAG_TX_ENABLED;

        // Transmitter enable
        usart->reg->CR1 |= USART_CR1_TE;
      } else {
        // Transmitter disable
        usart->reg->CR1 &= ~USART_CR1_TE;

        usart->info->flags &= ~USART_FLAG_TX_ENABLED;

        if (usart->info->mode != ARM_USART_MODE_SMART_CARD) {
          // GPIO pin function selected
          HAL_GPIO_DeInit (usart->io.tx->port, usart->io.tx->pin);
        }
      }
      return ARM_DRIVER_OK;

    // Control RX
    case ARM_USART_CONTROL_RX:
      // Check if RX line available
      if (usart->io.rx == NULL) { return ARM_DRIVER_ERROR; }
      if (arg) {
        if ((usart->info->mode != ARM_USART_MODE_SMART_CARD)   &&
            (usart->info->mode != ARM_USART_MODE_SINGLE_WIRE )) {
          // USART RX pin function selected
          GPIO_InitStruct.Pin       = usart->io.rx->pin;
          GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
          GPIO_InitStruct.Pull      = usart->io.rx->pupd;
          GPIO_InitStruct.Speed     = usart->io.rx->speed;
          GPIO_InitStruct.Alternate = usart->io.rx->af;
          HAL_GPIO_Init(usart->io.rx->port, &GPIO_InitStruct);
        }
        usart->info->flags |= USART_FLAG_RX_ENABLED;

        // Enable Error interrupt,
        usart->reg->CR3 |= USART_CR3_EIE;

        // Break detection interrupt enable
        usart->reg->CR2 |= USART_CR2_LBDIE;

        // Enable Time Out interrupt
        usart->reg->CR1 |= USART_CR1_RTOIE;

        if (((usart->info->status.rx_busy != 0U) && (usart->dma_rx != NULL)) == false) {
          usart->reg->CR1 |= USART_CR1_RXNEIE;
        }

        // Receiver enable
        usart->reg->CR1 |= USART_CR1_RE;

      } else {
        // Receiver disable
        usart->reg->CR1 &= ~USART_CR1_RE;

        usart->info->flags &= ~USART_FLAG_RX_ENABLED;

        if ((usart->info->mode != ARM_USART_MODE_SMART_CARD)   &&
            (usart->info->mode != ARM_USART_MODE_SINGLE_WIRE )) {
          // GPIO pin function selected
          HAL_GPIO_DeInit (usart->io.rx->port, usart->io.rx->pin);
        }
      }
      return ARM_DRIVER_OK;
    default: break;
  }

  // Check if busy
  if ((usart->info->status.rx_busy != 0U) || (usart->xfer->send_active != 0U)) {
    return ARM_DRIVER_ERROR_BUSY;
  }

  if (((usart->reg->CR1 & USART_CR1_TE) != 0U) && ((usart->reg->ISR & USART_ISR_TC) == 0U)) {
    return ARM_DRIVER_ERROR_BUSY;
  }

  switch (control & ARM_USART_CONTROL_Msk) {
    case ARM_USART_MODE_ASYNCHRONOUS:
#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
      if (usart->vmode != VM_ASYNC) { return ARM_USART_ERROR_MODE; }
#endif
      mode = ARM_USART_MODE_ASYNCHRONOUS;
      break;
    case ARM_USART_MODE_SYNCHRONOUS_MASTER:
#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
      if (usart->vmode != VM_SYNC) { return ARM_USART_ERROR_MODE; }
#endif
      if (usart->capabilities.synchronous_master) {
        // Enable Clock pin
        cr2 |= USART_CR2_CLKEN;

        // Enable last bit clock pulse
        cr2 |= USART_CR2_LBCL;
      } else { return ARM_USART_ERROR_MODE; }
      mode = ARM_USART_MODE_SYNCHRONOUS_MASTER;
      break;
    case ARM_USART_MODE_SYNCHRONOUS_SLAVE:
      return ARM_USART_ERROR_MODE;
    case ARM_USART_MODE_SINGLE_WIRE:
      // Enable Half duplex
      cr3 |= USART_CR3_HDSEL;
      mode = ARM_USART_MODE_SINGLE_WIRE;
      break;
    case ARM_USART_MODE_IRDA:
#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
      if (usart->vmode != VM_IRDA) { return ARM_USART_ERROR_MODE; }
#endif
      // Enable IrDA mode
      cr3 |= USART_CR3_IREN;
      mode = ARM_USART_MODE_IRDA;
      break;
    case ARM_USART_MODE_SMART_CARD:
#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
      if (usart->vmode != VM_SMARTCARD) { return ARM_USART_ERROR_MODE; }
#endif
      if (usart->capabilities.smart_card) {
        // Enable Smart card mode
        cr3 |= USART_CR3_SCEN;
      } else { return ARM_USART_ERROR_MODE; }
      mode = ARM_USART_MODE_SMART_CARD;
      break;

    // Default TX value
    case ARM_USART_SET_DEFAULT_TX_VALUE:
      usart->xfer->def_val = (uint16_t)arg;
#ifdef __USART_DMA_TX
#if (USART_DCACHE == 1U)
      if (usart->dma_tx != NULL) {
        // Is DCache enabled
        if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
          // Clean data cache: ensure data coherency between DMA and CPU
          SCB_CleanDCache_by_Addr ((uint32_t *)usart->xfer, 32);
        }
      }
#endif
#endif
      return ARM_DRIVER_OK;

    // IrDA pulse
    case ARM_USART_SET_IRDA_PULSE:
      if (usart->info->mode == ARM_USART_MODE_IRDA) {
        if (arg != 0U) {
          // IrDa low-power
          usart->reg->CR3 |= USART_CR3_IRLP;

          // Get clock
          val = USART_GetClockFreq (usart->reg);

          // Calculate period in ns
          val = 1000000000U / val;
          for (i = 1U; i < 256U; i++) {
            if ((val * i) > arg) { break; }
          }
          if (i == 256U) { return ARM_DRIVER_ERROR; }
          usart->reg->GTPR = (usart->reg->GTPR & ~USART_GTPR_PSC) | i;
        }
      } else { return ARM_DRIVER_ERROR; }
      return ARM_DRIVER_OK;

    // SmartCard guard time
    case ARM_USART_SET_SMART_CARD_GUARD_TIME:
      if (usart->info->mode == ARM_USART_MODE_SMART_CARD) {
        if (arg > 255U) return ARM_DRIVER_ERROR;

        usart->reg->GTPR = (usart->reg->GTPR & ~USART_GTPR_GT) | arg;
      } else { return ARM_DRIVER_ERROR; }
      return ARM_DRIVER_OK;

    // SmartCard clock
    case ARM_USART_SET_SMART_CARD_CLOCK:
      if (usart->info->mode == ARM_USART_MODE_SMART_CARD) {
        // Get clock
        val = USART_GetClockFreq (usart->reg);

        // Calculate period in ns
        val = 1000000000U / val;
        for (i = 1U; i <64U; i++) {
          // if in +-2% tolerance
          if (((val * i * 2U * 100U) < (arg * 102U)) &&
              ((val * i * 2U * 100U) > (arg * 98U))    ) {
            break;
          }
        }
        if (i == 64U) { return ARM_DRIVER_ERROR; }

        usart->reg->GTPR = (usart->reg->GTPR & ~USART_GTPR_PSC) | i;
      } else { return ARM_DRIVER_ERROR; }
      return ARM_DRIVER_OK;

    // SmartCard NACK
    case ARM_USART_CONTROL_SMART_CARD_NACK:
      if (usart->info->mode == ARM_USART_MODE_SMART_CARD) {
        // SmartCard NACK Enable
        if (arg != 0U) { usart->reg->CR3 |= USART_CR3_NACK; }
      } else { return ARM_DRIVER_ERROR; }
      return ARM_DRIVER_OK;

    // Unsupported command
    default: { return ARM_DRIVER_ERROR_UNSUPPORTED; }
  }

  // USART Data bits
  switch (control & ARM_USART_DATA_BITS_Msk) {
    case ARM_USART_DATA_BITS_6:
      if ((control & ARM_USART_PARITY_Msk) == ARM_USART_PARITY_NONE) {
        return ARM_USART_ERROR_DATA_BITS;
      }
      // 6 data bits, 7. data bit is parity bit
      cr1 |= USART_CR1_M_1;
      break;
    case ARM_USART_DATA_BITS_7:
      if ((control & ARM_USART_PARITY_Msk) == ARM_USART_PARITY_NONE) {
        // 7 data bits, no parity
        cr1 |= USART_CR1_M_1;
      }
      break;
    case ARM_USART_DATA_BITS_8:
      if ((control & ARM_USART_PARITY_Msk) == ARM_USART_PARITY_NONE) {
        // 8-data bits, no parity
      } else {
        // 11-bit break detection
        cr2 |= USART_CR2_LBDL;

        // 8-data bits, 9. bit is parity bit
        cr1 |= USART_CR1_M_0;
      }
      break;
    case ARM_USART_DATA_BITS_9:
      if ((control & ARM_USART_PARITY_Msk) != ARM_USART_PARITY_NONE) {
        return ARM_USART_ERROR_DATA_BITS;
      }

      // 11-bit break detection
      cr2 |= USART_CR2_LBDL;

      // 9-data bits, no parity
      cr1 |= USART_CR1_M_0;
      break;
    default: return ARM_USART_ERROR_DATA_BITS;
  }

  // USART Parity
  switch (control & ARM_USART_PARITY_Msk) {
    case ARM_USART_PARITY_NONE:                              break;
    case ARM_USART_PARITY_EVEN:   cr1 |=  USART_CR1_PCE;     break;
    case ARM_USART_PARITY_ODD:    cr1 |= (USART_CR1_PCE | 
                                          USART_CR1_PS);     break;
    default: return ARM_USART_ERROR_PARITY;
  }

  // USART Stop bits
  switch (control & ARM_USART_STOP_BITS_Msk) {
    case ARM_USART_STOP_BITS_1:                              break;
    case ARM_USART_STOP_BITS_2:   cr2 |= USART_CR2_STOP_1;   break;
    case ARM_USART_STOP_BITS_1_5: cr2 |= USART_CR2_STOP_0 |
                                         USART_CR2_STOP_1;   break;
    case ARM_USART_STOP_BITS_0_5: cr2 |= USART_CR2_STOP_0;   break;
    default: return ARM_USART_ERROR_STOP_BITS;
  }

  // USART Flow control
  switch (control & ARM_USART_FLOW_CONTROL_Msk) {
    case ARM_USART_FLOW_CONTROL_NONE:
      flow_control = ARM_USART_FLOW_CONTROL_NONE;
      break;
    case ARM_USART_FLOW_CONTROL_RTS:
      if (usart->capabilities.flow_control_rts) {
        flow_control = ARM_USART_FLOW_CONTROL_RTS;
        // RTS Enable
        cr3 |= USART_CR3_RTSE;
      }
      else  { return ARM_USART_ERROR_FLOW_CONTROL; }
      break;
    case ARM_USART_FLOW_CONTROL_CTS:
      if (usart->capabilities.flow_control_cts) {
        flow_control = ARM_USART_FLOW_CONTROL_CTS;
        // CTS Enable, CTS interrupt enable
        cr3 |= USART_CR3_CTSE | USART_CR3_CTSIE;
      }
      else { return ARM_USART_ERROR_FLOW_CONTROL; }
      break;
    case ARM_USART_FLOW_CONTROL_RTS_CTS:
      if ((usart->capabilities.flow_control_rts != 0U) &&
          (usart->capabilities.flow_control_cts != 0U)) {
        flow_control = ARM_USART_FLOW_CONTROL_RTS_CTS;
        // RTS and CTS Enable, CTS interrupt enable
        cr3 |= (USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_CTSIE);
      } else { return ARM_USART_ERROR_FLOW_CONTROL; }
      break;
    default: return ARM_USART_ERROR_FLOW_CONTROL;
  }

  // Clock setting for synchronous mode
  if (mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {

    // Polarity
    switch (control & ARM_USART_CPOL_Msk) {
      case ARM_USART_CPOL0:
        break;
      case ARM_USART_CPOL1:
        cr2 |= USART_CR2_CPOL;
        break;
      default: return ARM_USART_ERROR_CPOL;
    }

    // Phase
    switch (control & ARM_USART_CPHA_Msk) {
      case ARM_USART_CPHA0:
        break;
      case ARM_USART_CPHA1:
        cr2 |= USART_CR2_CPHA;
        break;
      default: return ARM_USART_ERROR_CPHA;
    }
  }

  // USART Baudrate
  val = (uint32_t) (USART_BAUDRATE_DIVIDER(USART_GetClockFreq (usart->reg), arg));
  br = ((USART_GetClockFreq (usart->reg) << 4U) / (val & 0xFFFFU)) >> 4U;
  // If inside +/- 2% tolerance, baud rate configured correctly
  if (!(((br * 100U) < (arg * 102U)) && ((br * 100U) > (arg * 98U)))) {
    return ARM_USART_ERROR_BAUDRATE;
  }

  // USART Disable
  usart->reg->CR1 &= ~USART_CR1_UE;

  // Configure Baud rate register
  usart->reg->BRR = val;

  // Configuration is OK - Mode is valid
  usart->info->mode = mode;

  // Save flow control mode
  usart->info->flow_control = flow_control;

  // Configure TX pin regarding mode and transmitter state
  if (usart->io.tx) {
    switch (usart->info->mode) {
      case ARM_USART_MODE_SMART_CARD:
        // USART TX pin function selected
        GPIO_InitStruct.Pin       = usart->io.tx->pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = usart->io.tx->pupd;
        GPIO_InitStruct.Speed     = usart->io.tx->speed;
        GPIO_InitStruct.Alternate = usart->io.tx->af;
        HAL_GPIO_Init(usart->io.tx->port, &GPIO_InitStruct);
        break;
      default:
        // Synchronous master/slave, asynchronous, single-wire and IrDA mode
        if (usart->info->flags & USART_FLAG_TX_ENABLED) {
          // USART TX pin function selected
          GPIO_InitStruct.Pin       = usart->io.tx->pin;
          GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
          GPIO_InitStruct.Pull      = usart->io.tx->pupd;
          GPIO_InitStruct.Speed     = usart->io.tx->speed;
          GPIO_InitStruct.Alternate = usart->io.tx->af;
          HAL_GPIO_Init(usart->io.tx->port, &GPIO_InitStruct);
        } else {
          // GPIO pin function selected
          HAL_GPIO_DeInit (usart->io.tx->port, usart->io.tx->pin);
        }
    }
  }

  // Configure RX pin regarding mode and receiver state
  if (usart->io.rx) {
    switch (usart->info->mode) {
      case ARM_USART_MODE_SINGLE_WIRE:
      case ARM_USART_MODE_SMART_CARD:
        // GPIO pin function selected
        HAL_GPIO_DeInit (usart->io.rx->port, usart->io.rx->pin);
        break;
      default:
        // Synchronous master/slave, asynchronous and  IrDA mode
        if (usart->info->flags & USART_FLAG_RX_ENABLED) {
          // USART RX pin function selected
          GPIO_InitStruct.Pin       = usart->io.rx->pin;
          GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
          GPIO_InitStruct.Pull      = usart->io.rx->pupd;
          GPIO_InitStruct.Speed     = usart->io.rx->speed;
          GPIO_InitStruct.Alternate = usart->io.rx->af;
          HAL_GPIO_Init(usart->io.rx->port, &GPIO_InitStruct);
        } else {
         // GPIO pin function selected
         HAL_GPIO_DeInit (usart->io.rx->port, usart->io.rx->pin);
        }
        break;
    }
  }

  // Configure CLK pin regarding mode
  if (usart->io.ck) {
    switch (usart->info->mode) {
      case ARM_USART_MODE_SMART_CARD:
      case ARM_USART_MODE_SYNCHRONOUS_MASTER:
        // USART CK pin function selected
        GPIO_InitStruct.Pin       = usart->io.ck->pin;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = usart->io.ck->pupd;
        GPIO_InitStruct.Speed     = usart->io.ck->speed;
        GPIO_InitStruct.Alternate = usart->io.ck->af;
        HAL_GPIO_Init(usart->io.ck->port, &GPIO_InitStruct);
        break;
      default:
        // Asynchronous, Single-wire and IrDA mode
        // GPIO pin function selected
        HAL_GPIO_DeInit (usart->io.ck->port, usart->io.ck->pin);
    }
  }

  // Configure RTS pin regarding Flow control configuration
  if (usart->io.rts) {
    if ((flow_control == ARM_USART_FLOW_CONTROL_RTS) ||
        (flow_control == ARM_USART_FLOW_CONTROL_RTS_CTS)) {
      // USART RTS Alternate function
      GPIO_InitStruct.Pin       = usart->io.rts->pin;
      GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull      = usart->io.rts->pupd;
      GPIO_InitStruct.Speed     = usart->io.rts->speed;
      GPIO_InitStruct.Alternate = usart->io.rts->af;
      HAL_GPIO_Init(usart->io.rts->port, &GPIO_InitStruct);
    } else {
      // GPIO output
      GPIO_InitStruct.Pin       = usart->io.rts->pin;
      GPIO_InitStruct.Mode      = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull      = GPIO_NOPULL;
      GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
      HAL_GPIO_Init(usart->io.rts->port, &GPIO_InitStruct);
    }
  }

  // Configure CTS pin regarding Flow control configuration
  if (usart->io.cts) {
    if ((flow_control == ARM_USART_FLOW_CONTROL_CTS) ||
        (flow_control == ARM_USART_FLOW_CONTROL_RTS_CTS)) {
      // USART CTS Alternate function
      GPIO_InitStruct.Pin       = usart->io.cts->pin;
      GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull      = usart->io.cts->pupd;
      GPIO_InitStruct.Speed     = usart->io.cts->speed;
      GPIO_InitStruct.Alternate = usart->io.cts->af;
      HAL_GPIO_Init(usart->io.cts->port, &GPIO_InitStruct);
    } else {
      // GPIO input
      GPIO_InitStruct.Pin       = usart->io.cts->pin;
      GPIO_InitStruct.Mode      = GPIO_MODE_INPUT;
      GPIO_InitStruct.Pull      = GPIO_NOPULL;
      GPIO_InitStruct.Speed     = GPIO_SPEED_LOW;
      HAL_GPIO_Init(usart->io.cts->port, &GPIO_InitStruct);
    }
  }

  // Configure USART control registers
  usart->reg->CR1  = cr1;
  usart->reg->CR2  = cr2 | USART_CR2_RTOEN;
  usart->reg->CR3  = cr3;
  usart->reg->RTOR = usart->rx_timeout_val;

  // USART Enable
  usart->reg->CR1 |= USART_CR1_UE;

  // Set configured flag
  usart->info->flags |= USART_FLAG_CONFIGURED;

  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_USART_STATUS USART_GetStatus (const USART_RESOURCES *usart)
  \brief       Get USART status.
  \param[in]   usart     Pointer to USART resources
  \return      USART status \ref ARM_USART_STATUS
*/
static ARM_USART_STATUS USART_GetStatus (const USART_RESOURCES *usart) {
  ARM_USART_STATUS status;

  if (usart->xfer->send_active != 0U) {
    status.tx_busy        = 1U;
  } else {
    status.tx_busy        = ((usart->reg->ISR & USART_ISR_TC) ? (0U) : (1U));
  }
  status.rx_busy          = usart->info->status.rx_busy;
  status.tx_underflow     = usart->info->status.tx_underflow;
  status.rx_overflow      = usart->info->status.rx_overflow;
  status.rx_break         = usart->info->status.rx_break;
  status.rx_framing_error = usart->info->status.rx_framing_error;
  status.rx_parity_error  = usart->info->status.rx_parity_error;

  return status;
}

/**
  \fn          int32_t USART_SetModemControl (      ARM_USART_MODEM_CONTROL  control,
                                              const USART_RESOURCES         *usart)
  \brief       Set USART Modem Control line state.
  \param[in]   control   \ref ARM_USART_MODEM_CONTROL
  \param[in]   usart     Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_SetModemControl (      ARM_USART_MODEM_CONTROL  control,
                                      const USART_RESOURCES         *usart) {

  if ((control != ARM_USART_RTS_CLEAR) && 
      (control != ARM_USART_RTS_SET)   &&
      (control != ARM_USART_DTR_CLEAR) &&
      (control != ARM_USART_DTR_SET)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  switch (control) {
    case ARM_USART_RTS_CLEAR:
      if ((usart->info->flow_control == ARM_USART_FLOW_CONTROL_NONE) ||
          (usart->info->flow_control == ARM_USART_FLOW_CONTROL_CTS)) {
        if (usart->io.rts) {
          HAL_GPIO_WritePin (usart->io.rts->port, usart->io.rts->pin, GPIO_PIN_SET);
        }
      } else {
        // Hardware RTS
        return ARM_DRIVER_ERROR;
      }
      break;
    case ARM_USART_RTS_SET:
      if ((usart->info->flow_control == ARM_USART_FLOW_CONTROL_NONE) ||
          (usart->info->flow_control == ARM_USART_FLOW_CONTROL_CTS)) {
        if (usart->io.rts) {
          HAL_GPIO_WritePin (usart->io.rts->port, usart->io.rts->pin, GPIO_PIN_RESET);
        }
      } else {
        // Hardware RTS
        return ARM_DRIVER_ERROR;
      }
      break;
    case ARM_USART_DTR_CLEAR:
    case ARM_USART_DTR_SET:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_USART_MODEM_STATUS USART_GetModemStatus (const USART_RESOURCES *usart)
  \brief       Get USART Modem Status lines state.
  \param[in]   usart     Pointer to USART resources
  \return      modem status \ref ARM_USART_MODEM_STATUS
*/
static ARM_USART_MODEM_STATUS USART_GetModemStatus (const USART_RESOURCES *usart) {
  ARM_USART_MODEM_STATUS modem_status;

  modem_status.cts = 0U;
  if ((usart->info->flow_control == ARM_USART_FLOW_CONTROL_NONE) ||
      (usart->info->flow_control == ARM_USART_FLOW_CONTROL_RTS)) {
    if (usart->io.cts) {
      if (HAL_GPIO_ReadPin (usart->io.cts->port, usart->io.cts->pin) == GPIO_PIN_RESET) {
        modem_status.cts = 1U;
      }
    }
  }
  modem_status.dsr = 0U;
  modem_status.ri  = 0U;
  modem_status.dcd = 0U;

  return modem_status;
}


/**
  \fn          void USART_IRQHandler (const USART_RESOURCES *usart)
  \brief       USART Interrupt handler.
  \param[in]   usart     Pointer to USART resources
*/
void USART_IRQHandler (const USART_RESOURCES *usart) {
  uint32_t val, sr, event;
  uint16_t data;

  // Read USART status register
  sr = usart->reg->ISR;

  // Reset local variables
  val   = 0U;
  event = 0U;
  data  = 0U;

  // Read Data register not empty
  if (sr & USART_ISR_RXNE & usart->reg->CR1) {
    // Check for RX overflow
    if (usart->info->status.rx_busy == 0U) {
      // New receive has not been started
      // Dump RX data
      usart->reg->RDR;
      usart->info->status.rx_overflow = 1;
      event |= ARM_USART_EVENT_RX_OVERFLOW;
    } else {
      if ((usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) &&
          (usart->xfer->sync_mode == USART_SYNC_MODE_TX)) {
        // Dummy read in synchronous transmit only mode
        usart->reg->RDR;
      } else {
        // Read data from RX FIFO into receive buffer
        data = (uint16_t)usart->reg->RDR;
      }

      *(usart->xfer->rx_buf++) = (uint8_t)data;

      // If nine bit data, no parity
      val = usart->reg->CR1;
      if (((val & USART_CR1_PCE) == 0U) &&
          ((val & USART_CR1_M)   != 0U)) {
        *(usart->xfer->rx_buf++) = (uint8_t)(data >> 8U);
      }
      usart->xfer->rx_cnt++;

      // Check if requested amount of data is received
      if (usart->xfer->rx_cnt == usart->xfer->rx_num) {

        // Disable Receiver timeout interrupt
        usart->reg->CR1 &= ~USART_CR1_RTOIE;

        // Clear RX busy flag and set receive transfer complete event
        usart->info->status.rx_busy = 0U;
        if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {
          val = usart->xfer->sync_mode;
          usart->xfer->sync_mode = 0U;
          switch (val) {
            case USART_SYNC_MODE_TX:
              event |= ARM_USART_EVENT_SEND_COMPLETE;
              break;
            case USART_SYNC_MODE_RX:
              event |= ARM_USART_EVENT_RECEIVE_COMPLETE;
              break;
            case USART_SYNC_MODE_TX_RX:
              event |= ARM_USART_EVENT_TRANSFER_COMPLETE;
              break;
            default: break;
          }
        } else {
          event |= ARM_USART_EVENT_RECEIVE_COMPLETE;
        }
      }
    }
  }

  // Receiver Timeout
  if (sr & USART_ISR_RTOF) {
    if (usart->reg->CR1 & USART_CR1_RTOIE) {
      // Clear Receiver Timeout interrupt
      usart->reg->ICR = USART_ICR_RTOCF;
      event |= ARM_USART_EVENT_RX_TIMEOUT;
    }
  }

  // Transmit data register empty
  if (sr & USART_ISR_TXE & usart->reg->CR1) {

    // Break handling
    if (usart->xfer->break_flag) {
      // Send break
      usart->reg->RQR |= USART_RQR_SBKRQ;
    } else {
      if(usart->xfer->tx_num != usart->xfer->tx_cnt) {
        if ((usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) &&
             (usart->xfer->sync_mode == USART_SYNC_MODE_RX)) {
          // Dummy write in synchronous receive only mode
          data = usart->xfer->def_val;
        } else {
          // Write data to TX FIFO
          data = *(usart->xfer->tx_buf++);

          // If nine bit data, no parity
          val = usart->reg->CR1;
          if (((val & USART_CR1_PCE) == 0U) &&
              ((val & USART_CR1_M)   != 0U)) {
            data |= *(usart->xfer->tx_buf++) << 8U;
          }
        }
      }
      usart->xfer->tx_cnt++;

      // Write to data register
      usart->reg->TDR = data;

      // Check if all data is transmitted
      if (usart->xfer->tx_num == usart->xfer->tx_cnt) {
        // Disable TXE interrupt
        usart->reg->CR1 &= ~USART_CR1_TXEIE;

        // Enable TC interrupt
        usart->reg->CR1 |= USART_CR1_TCIE;

        usart->xfer->send_active = 0U;

        // Set send complete event
        if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {
          if ((usart->xfer->sync_mode == USART_SYNC_MODE_TX)    &&
              ((usart->info->flags & USART_FLAG_RX_ENABLED) == 0U)) {
            event |= ARM_USART_EVENT_SEND_COMPLETE;
          }
        } else {
          event |= ARM_USART_EVENT_SEND_COMPLETE;
        }
      }
    }
  }

  // Transmission complete
  if (sr & USART_ISR_TC & usart->reg->CR1) {
    // Disable transmission complete interrupt
    usart->reg->CR1 &= ~USART_CR1_TCIE;
    event |= ARM_USART_EVENT_TX_COMPLETE;
  }

  // RX Overrun
  if ((sr & USART_ISR_ORE) != 0U) {
    // Shift register has been overwritten
    // Clear the ORE flag
    usart->reg->ICR = USART_ICR_ORECF;
    usart->info->status.rx_overflow = 1U;
    event |= ARM_USART_EVENT_RX_OVERFLOW;
  }

  // Framing error
  if ((sr & USART_ISR_FE) != 0U) {
    // Dummy data read to clear the FE flag
    usart->reg->ICR = USART_ICR_FECF;
    usart->info->status.rx_framing_error = 1U;
    event |= ARM_USART_EVENT_RX_FRAMING_ERROR;
  }

  // Parity error
  if ((sr & USART_ISR_PE) != 0U) {
    // Clear the PE flag
    usart->reg->ICR = USART_ICR_PECF;
    usart->info->status.rx_parity_error = 1U;
    event |= ARM_USART_EVENT_RX_PARITY_ERROR;
  }

  // Noise error
  if ((sr & USART_ISR_NE) != 0U) {
    // Clear the NE flag
    usart->reg->ICR = USART_ICR_NCF;
  }

  // Break Detection
  if ((sr & USART_ISR_LBDF) != 0U) {
    // Clear Break detection flag
    usart->reg->ICR = USART_ICR_LBDCF;

    usart->info->status.rx_break = 1U;
    event |= ARM_USART_EVENT_RX_BREAK;
  }

  // CTS changed
  if ((sr & USART_ISR_CTSIF) != 0U) {
    usart->reg->ICR = USART_ICR_CTSCF;
    event |= ARM_USART_EVENT_CTS;
  }

  // Send Event
  if ((event && usart->info->cb_event) != 0U) {
    usart->info->cb_event (event);
  }
}

#ifdef __USART_DMA_TX
static void USART_TX_DMA_Complete(const USART_RESOURCES *usart) {

  if ((__HAL_DMA_GET_COUNTER(usart->dma_tx->hdma) != 0) && (usart->xfer->tx_num != 0)) {
    // TX DMA Complete caused by send/transfer abort
    return;
  }

  usart->xfer->tx_cnt = usart->xfer->tx_num;
  // Clear TX busy flag
  usart->xfer->send_active = 0U;

  // TC interrupt enable
  usart->reg->CR1 |= USART_CR1_TCIE; 

  // Set Send Complete event for asynchronous transfers
  if (usart->info->mode != ARM_USART_MODE_SYNCHRONOUS_MASTER) {
    if (usart->info->cb_event) {
      usart->info->cb_event (ARM_USART_EVENT_SEND_COMPLETE);
    }
  }
}
#endif

#ifdef __USART_DMA_RX
static void USART_RX_DMA_Complete(const USART_RESOURCES *usart) {
  uint32_t val, event;

  if ((__HAL_DMA_GET_COUNTER(usart->dma_rx->hdma) != 0) && (usart->xfer->rx_num != 0)) {
    // RX DMA Complete caused by receive/transfer abort
    return;
  }

  // Disable Receiver Timeout interrupt
  usart->reg->CR1 &= ~USART_CR1_RTOIE;

  event = 0U;

  if (usart->info->mode == ARM_USART_MODE_SYNCHRONOUS_MASTER) {
    val = usart->xfer->sync_mode;
    usart->xfer->sync_mode = 0U;
    switch (val) {
      case USART_SYNC_MODE_TX:
        event = ARM_USART_EVENT_SEND_COMPLETE;
        break;
      case USART_SYNC_MODE_RX:
        event = ARM_USART_EVENT_RECEIVE_COMPLETE;
        break;
      case USART_SYNC_MODE_TX_RX:
        event = ARM_USART_EVENT_TRANSFER_COMPLETE;
         break;
      default: break;
    }
  } else {
    event = ARM_USART_EVENT_RECEIVE_COMPLETE;
  }

#if (USART_DCACHE == 1U)
  // Is DCache enabled
  if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
    // Invalidate data cache: ensure data coherency between DMA and CPU
    SCB_InvalidateDCache_by_Addr ((uint32_t *)((uint32_t)usart->xfer->rx_buf), (int32_t)(usart->xfer->rx_num));
  }
#endif

  usart->xfer->rx_cnt = usart->xfer->rx_num;
  usart->info->status.rx_busy = 0U;

  // Enable RXNE interrupt to detect RX overrun
  usart->reg->CR1 |= USART_CR1_RXNEIE;

  if (usart->info->cb_event && event) { usart->info->cb_event (event); }
}
#endif



#ifdef MX_USART1
// USART1 Driver Wrapper functions
static ARM_USART_CAPABILITIES  USART1_GetCapabilities (void)                                                { return USART_GetCapabilities (&USART1_Resources); }
static int32_t                 USART1_Initialize      (ARM_USART_SignalEvent_t cb_event)                    { return USART_Initialize (cb_event, &USART1_Resources); }
static int32_t                 USART1_Uninitialize    (void)                                                { return USART_Uninitialize (&USART1_Resources); }
static int32_t                 USART1_PowerControl    (ARM_POWER_STATE state)                               { return USART_PowerControl (state, &USART1_Resources); }
static int32_t                 USART1_Send            (const void *data, uint32_t num)                      { return USART_Send (data, num, &USART1_Resources); }
static int32_t                 USART1_Receive         (void *data, uint32_t num)                            { return USART_Receive (data, num, &USART1_Resources); }
static int32_t                 USART1_Transfer        (const void *data_out, void *data_in, uint32_t num)   { return USART_Transfer (data_out, data_in, num, &USART1_Resources); }
static uint32_t                USART1_GetTxCount      (void)                                                { return USART_GetTxCount (&USART1_Resources); }
static uint32_t                USART1_GetRxCount      (void)                                                { return USART_GetRxCount (&USART1_Resources); }
static int32_t                 USART1_Control         (uint32_t control, uint32_t arg)                      { return USART_Control (control, arg, &USART1_Resources); }
static ARM_USART_STATUS        USART1_GetStatus       (void)                                                { return USART_GetStatus (&USART1_Resources); }
static int32_t                 USART1_SetModemControl (ARM_USART_MODEM_CONTROL control)                     { return USART_SetModemControl (control, &USART1_Resources); }
static ARM_USART_MODEM_STATUS  USART1_GetModemStatus  (void)                                                { return USART_GetModemStatus (&USART1_Resources); }
       void                    USART1_IRQHandler      (void)                                                {        USART_IRQHandler (&USART1_Resources); }

#ifdef MX_USART1_TX_DMA_Instance
static void                    USART1_TX_DMA_Complete (DMA_HandleTypeDef *hdma)                             { (void)hdma; USART_TX_DMA_Complete(&USART1_Resources); }

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
void USART1_TX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_USART1_TX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
}
#endif
#endif
#ifdef MX_USART1_RX_DMA_Instance
static void                    USART1_RX_DMA_Complete (DMA_HandleTypeDef *hdma)                             { (void)hdma; USART_RX_DMA_Complete(&USART1_Resources); }

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
void USART1_RX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_USART1_RX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
}
#endif
#endif

// USART1 Driver Control Block
ARM_DRIVER_USART Driver_USART1 = {
    USARTx_GetVersion,
    USART1_GetCapabilities,
    USART1_Initialize,
    USART1_Uninitialize,
    USART1_PowerControl,
    USART1_Send, 
    USART1_Receive,
    USART1_Transfer,
    USART1_GetTxCount,
    USART1_GetRxCount,
    USART1_Control,
    USART1_GetStatus,
    USART1_SetModemControl,
    USART1_GetModemStatus
};
#endif

#ifdef MX_USART2
// USART2 Driver Wrapper functions
static ARM_USART_CAPABILITIES  USART2_GetCapabilities (void)                                                { return USART_GetCapabilities (&USART2_Resources); }
static int32_t                 USART2_Initialize      (ARM_USART_SignalEvent_t cb_event)                    { return USART_Initialize (cb_event, &USART2_Resources); }
static int32_t                 USART2_Uninitialize    (void)                                                { return USART_Uninitialize (&USART2_Resources); }
static int32_t                 USART2_PowerControl    (ARM_POWER_STATE state)                               { return USART_PowerControl (state, &USART2_Resources); }
static int32_t                 USART2_Send            (const void *data, uint32_t num)                      { return USART_Send (data, num, &USART2_Resources); }
static int32_t                 USART2_Receive         (void *data, uint32_t num)                            { return USART_Receive (data, num, &USART2_Resources); }
static int32_t                 USART2_Transfer        (const void *data_out, void *data_in, uint32_t num)   { return USART_Transfer (data_out, data_in, num, &USART2_Resources); }
static uint32_t                USART2_GetTxCount      (void)                                                { return USART_GetTxCount (&USART2_Resources); }
static uint32_t                USART2_GetRxCount      (void)                                                { return USART_GetRxCount (&USART2_Resources); }
static int32_t                 USART2_Control         (uint32_t control, uint32_t arg)                      { return USART_Control (control, arg, &USART2_Resources); }
static ARM_USART_STATUS        USART2_GetStatus       (void)                                                { return USART_GetStatus (&USART2_Resources); }
static int32_t                 USART2_SetModemControl (ARM_USART_MODEM_CONTROL control)                     { return USART_SetModemControl (control, &USART2_Resources); }
static ARM_USART_MODEM_STATUS  USART2_GetModemStatus  (void)                                                { return USART_GetModemStatus (&USART2_Resources); }
       void                    USART2_IRQHandler      (void)                                                {        USART_IRQHandler (&USART2_Resources); }

#ifdef MX_USART2_TX_DMA_Instance
static void                    USART2_TX_DMA_Complete (DMA_HandleTypeDef *hdma)                             { (void)hdma; USART_TX_DMA_Complete(&USART2_Resources); }

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
void USART2_TX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_USART2_TX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
}
#endif
#endif
#ifdef MX_USART2_RX_DMA_Instance
static void                    USART2_RX_DMA_Complete (DMA_HandleTypeDef *hdma)                             { (void)hdma; USART_RX_DMA_Complete(&USART2_Resources); }

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
void USART2_RX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_USART2_RX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
}
#endif
#endif

// USART2 Driver Control Block
ARM_DRIVER_USART Driver_USART2 = {
    USARTx_GetVersion,
    USART2_GetCapabilities,
    USART2_Initialize,
    USART2_Uninitialize,
    USART2_PowerControl,
    USART2_Send, 
    USART2_Receive,
    USART2_Transfer,
    USART2_GetTxCount,
    USART2_GetRxCount,
    USART2_Control,
    USART2_GetStatus,
    USART2_SetModemControl,
    USART2_GetModemStatus
};
#endif

#ifdef MX_USART3
// USART3 Driver Wrapper functions
static ARM_USART_CAPABILITIES  USART3_GetCapabilities (void)                                                { return USART_GetCapabilities (&USART3_Resources); }
static int32_t                 USART3_Initialize      (ARM_USART_SignalEvent_t cb_event)                    { return USART_Initialize (cb_event, &USART3_Resources); }
static int32_t                 USART3_Uninitialize    (void)                                                { return USART_Uninitialize (&USART3_Resources); }
static int32_t                 USART3_PowerControl    (ARM_POWER_STATE state)                               { return USART_PowerControl (state, &USART3_Resources); }
static int32_t                 USART3_Send            (const void *data, uint32_t num)                      { return USART_Send (data, num, &USART3_Resources); }
static int32_t                 USART3_Receive         (void *data, uint32_t num)                            { return USART_Receive (data, num, &USART3_Resources); }
static int32_t                 USART3_Transfer        (const void *data_out, void *data_in, uint32_t num)   { return USART_Transfer (data_out, data_in, num, &USART3_Resources); }
static uint32_t                USART3_GetTxCount      (void)                                                { return USART_GetTxCount (&USART3_Resources); }
static uint32_t                USART3_GetRxCount      (void)                                                { return USART_GetRxCount (&USART3_Resources); }
static int32_t                 USART3_Control         (uint32_t control, uint32_t arg)                      { return USART_Control (control, arg, &USART3_Resources); }
static ARM_USART_STATUS        USART3_GetStatus       (void)                                                { return USART_GetStatus (&USART3_Resources); }
static int32_t                 USART3_SetModemControl (ARM_USART_MODEM_CONTROL control)                     { return USART_SetModemControl (control, &USART3_Resources); }
static ARM_USART_MODEM_STATUS  USART3_GetModemStatus  (void)                                                { return USART_GetModemStatus (&USART3_Resources); }
       void                    USART3_IRQHandler      (void)                                                {        USART_IRQHandler (&USART3_Resources); }

#ifdef MX_USART3_TX_DMA_Instance
static void                    USART3_TX_DMA_Complete (DMA_HandleTypeDef *hdma)                             { (void)hdma; USART_TX_DMA_Complete(&USART3_Resources); }

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
void USART3_TX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_USART3_TX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
}
#endif
#endif
#ifdef MX_USART3_RX_DMA_Instance
static void                    USART3_RX_DMA_Complete (DMA_HandleTypeDef *hdma)                             { (void)hdma; USART_RX_DMA_Complete(&USART3_Resources); }

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
void USART3_RX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_USART3_RX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
}
#endif
#endif

// USART3 Driver Control Block
ARM_DRIVER_USART Driver_USART3 = {
    USARTx_GetVersion,
    USART3_GetCapabilities,
    USART3_Initialize,
    USART3_Uninitialize,
    USART3_PowerControl,
    USART3_Send, 
    USART3_Receive,
    USART3_Transfer,
    USART3_GetTxCount,
    USART3_GetRxCount,
    USART3_Control,
    USART3_GetStatus,
    USART3_SetModemControl,
    USART3_GetModemStatus
};
#endif

#ifdef MX_UART4
// USART4 Driver Wrapper functions
static ARM_USART_CAPABILITIES  USART4_GetCapabilities (void)                                                { return USART_GetCapabilities (&USART4_Resources); }
static int32_t                 USART4_Initialize      (ARM_USART_SignalEvent_t cb_event)                    { return USART_Initialize (cb_event, &USART4_Resources); }
static int32_t                 USART4_Uninitialize    (void)                                                { return USART_Uninitialize (&USART4_Resources); }
static int32_t                 USART4_PowerControl    (ARM_POWER_STATE state)                               { return USART_PowerControl (state, &USART4_Resources); }
static int32_t                 USART4_Send            (const void *data, uint32_t num)                      { return USART_Send (data, num, &USART4_Resources); }
static int32_t                 USART4_Receive         (void *data, uint32_t num)                            { return USART_Receive (data, num, &USART4_Resources); }
static int32_t                 USART4_Transfer        (const void *data_out, void *data_in, uint32_t num)   { return USART_Transfer (data_out, data_in, num, &USART4_Resources); }
static uint32_t                USART4_GetTxCount      (void)                                                { return USART_GetTxCount (&USART4_Resources); }
static uint32_t                USART4_GetRxCount      (void)                                                { return USART_GetRxCount (&USART4_Resources); }
static int32_t                 USART4_Control         (uint32_t control, uint32_t arg)                      { return USART_Control (control, arg, &USART4_Resources); }
static ARM_USART_STATUS        USART4_GetStatus       (void)                                                { return USART_GetStatus (&USART4_Resources); }
static int32_t                 USART4_SetModemControl (ARM_USART_MODEM_CONTROL control)                     { return USART_SetModemControl (control, &USART4_Resources); }
static ARM_USART_MODEM_STATUS  USART4_GetModemStatus  (void)                                                { return USART_GetModemStatus (&USART4_Resources); }
       void                    UART4_IRQHandler       (void)                                                {        USART_IRQHandler (&USART4_Resources); }

#ifdef MX_UART4_TX_DMA_Instance
static void                    UART4_TX_DMA_Complete (DMA_HandleTypeDef *hdma)                              { (void)hdma; USART_TX_DMA_Complete(&USART4_Resources); }

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
void UART4_TX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_UART4_TX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
}
#endif
#endif
#ifdef MX_UART4_RX_DMA_Instance
static void                    UART4_RX_DMA_Complete (DMA_HandleTypeDef *hdma)                              { (void)hdma; USART_RX_DMA_Complete(&USART4_Resources); }

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
void UART4_RX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_UART4_RX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
}
#endif
#endif

// USART4 Driver Control Block
ARM_DRIVER_USART Driver_USART4 = {
    USARTx_GetVersion,
    USART4_GetCapabilities,
    USART4_Initialize,
    USART4_Uninitialize,
    USART4_PowerControl,
    USART4_Send, 
    USART4_Receive,
    USART4_Transfer,
    USART4_GetTxCount,
    USART4_GetRxCount,
    USART4_Control,
    USART4_GetStatus,
    USART4_SetModemControl,
    USART4_GetModemStatus
};
#endif

#ifdef MX_UART5
// USART5 Driver Wrapper functions
static ARM_USART_CAPABILITIES  USART5_GetCapabilities (void)                                                { return USART_GetCapabilities (&USART5_Resources); }
static int32_t                 USART5_Initialize      (ARM_USART_SignalEvent_t cb_event)                    { return USART_Initialize (cb_event, &USART5_Resources); }
static int32_t                 USART5_Uninitialize    (void)                                                { return USART_Uninitialize (&USART5_Resources); }
static int32_t                 USART5_PowerControl    (ARM_POWER_STATE state)                               { return USART_PowerControl (state, &USART5_Resources); }
static int32_t                 USART5_Send            (const void *data, uint32_t num)                      { return USART_Send (data, num, &USART5_Resources); }
static int32_t                 USART5_Receive         (void *data, uint32_t num)                            { return USART_Receive (data, num, &USART5_Resources); }
static int32_t                 USART5_Transfer        (const void *data_out, void *data_in, uint32_t num)   { return USART_Transfer (data_out, data_in, num, &USART5_Resources); }
static uint32_t                USART5_GetTxCount      (void)                                                { return USART_GetTxCount (&USART5_Resources); }
static uint32_t                USART5_GetRxCount      (void)                                                { return USART_GetRxCount (&USART5_Resources); }
static int32_t                 USART5_Control         (uint32_t control, uint32_t arg)                      { return USART_Control (control, arg, &USART5_Resources); }
static ARM_USART_STATUS        USART5_GetStatus       (void)                                                { return USART_GetStatus (&USART5_Resources); }
static int32_t                 USART5_SetModemControl (ARM_USART_MODEM_CONTROL control)                     { return USART_SetModemControl (control, &USART5_Resources); }
static ARM_USART_MODEM_STATUS  USART5_GetModemStatus  (void)                                                { return USART_GetModemStatus (&USART5_Resources); }
       void                    UART5_IRQHandler       (void)                                                {        USART_IRQHandler (&USART5_Resources); }

#ifdef MX_UART5_TX_DMA_Instance
static void                    UART5_TX_DMA_Complete (DMA_HandleTypeDef *hdma)                              { (void)hdma; USART_TX_DMA_Complete(&USART5_Resources); }

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
void UART5_TX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_UART5_TX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_uart5_tx);
}
#endif
#endif
#ifdef MX_UART5_RX_DMA_Instance
static void                    UART5_RX_DMA_Complete (DMA_HandleTypeDef *hdma)                              { (void)hdma; USART_RX_DMA_Complete(&USART5_Resources); }

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
void UART5_RX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_UART5_RX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_uart5_rx);
}
#endif
#endif

// USART5 Driver Control Block
ARM_DRIVER_USART Driver_USART5 = {
    USARTx_GetVersion,
    USART5_GetCapabilities,
    USART5_Initialize,
    USART5_Uninitialize,
    USART5_PowerControl,
    USART5_Send, 
    USART5_Receive,
    USART5_Transfer,
    USART5_GetTxCount,
    USART5_GetRxCount,
    USART5_Control,
    USART5_GetStatus,
    USART5_SetModemControl,
    USART5_GetModemStatus
};
#endif

#ifdef MX_USART6
// USART6 Driver Wrapper functions
static ARM_USART_CAPABILITIES  USART6_GetCapabilities (void)                                                { return USART_GetCapabilities (&USART6_Resources); }
static int32_t                 USART6_Initialize      (ARM_USART_SignalEvent_t cb_event)                    { return USART_Initialize (cb_event, &USART6_Resources); }
static int32_t                 USART6_Uninitialize    (void)                                                { return USART_Uninitialize (&USART6_Resources); }
static int32_t                 USART6_PowerControl    (ARM_POWER_STATE state)                               { return USART_PowerControl (state, &USART6_Resources); }
static int32_t                 USART6_Send            (const void *data, uint32_t num)                      { return USART_Send (data, num, &USART6_Resources); }
static int32_t                 USART6_Receive         (void *data, uint32_t num)                            { return USART_Receive (data, num, &USART6_Resources); }
static int32_t                 USART6_Transfer        (const void *data_out, void *data_in, uint32_t num)   { return USART_Transfer (data_out, data_in, num, &USART6_Resources); }
static uint32_t                USART6_GetTxCount      (void)                                                { return USART_GetTxCount (&USART6_Resources); }
static uint32_t                USART6_GetRxCount      (void)                                                { return USART_GetRxCount (&USART6_Resources); }
static int32_t                 USART6_Control         (uint32_t control, uint32_t arg)                      { return USART_Control (control, arg, &USART6_Resources); }
static ARM_USART_STATUS        USART6_GetStatus       (void)                                                { return USART_GetStatus (&USART6_Resources); }
static int32_t                 USART6_SetModemControl (ARM_USART_MODEM_CONTROL control)                     { return USART_SetModemControl (control, &USART6_Resources); }
static ARM_USART_MODEM_STATUS  USART6_GetModemStatus  (void)                                                { return USART_GetModemStatus (&USART6_Resources); }
       void                    USART6_IRQHandler      (void)                                                {        USART_IRQHandler (&USART6_Resources); }

#ifdef MX_USART6_TX_DMA_Instance
static void                    USART6_TX_DMA_Complete (DMA_HandleTypeDef *hdma)                             { (void)hdma; USART_TX_DMA_Complete(&USART6_Resources); }

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
void USART6_TX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_USART6_TX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_usart6_tx);
}
#endif
#endif
#ifdef MX_USART6_RX_DMA_Instance
static void                    USART6_RX_DMA_Complete (DMA_HandleTypeDef *hdma)                             { (void)hdma; USART_RX_DMA_Complete(&USART6_Resources); }

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
void USART6_RX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_USART6_RX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
}
#endif
#endif

// USART6 Driver Control Block
ARM_DRIVER_USART Driver_USART6 = {
    USARTx_GetVersion,
    USART6_GetCapabilities,
    USART6_Initialize,
    USART6_Uninitialize,
    USART6_PowerControl,
    USART6_Send, 
    USART6_Receive,
    USART6_Transfer,
    USART6_GetTxCount,
    USART6_GetRxCount,
    USART6_Control,
    USART6_GetStatus,
    USART6_SetModemControl,
    USART6_GetModemStatus
};
#endif

#ifdef MX_UART7
// USART7 Driver Wrapper functions
static ARM_USART_CAPABILITIES  USART7_GetCapabilities (void)                                                { return USART_GetCapabilities (&USART7_Resources); }
static int32_t                 USART7_Initialize      (ARM_USART_SignalEvent_t cb_event)                    { return USART_Initialize (cb_event, &USART7_Resources); }
static int32_t                 USART7_Uninitialize    (void)                                                { return USART_Uninitialize (&USART7_Resources); }
static int32_t                 USART7_PowerControl    (ARM_POWER_STATE state)                               { return USART_PowerControl (state, &USART7_Resources); }
static int32_t                 USART7_Send            (const void *data, uint32_t num)                      { return USART_Send (data, num, &USART7_Resources); }
static int32_t                 USART7_Receive         (void *data, uint32_t num)                            { return USART_Receive (data, num, &USART7_Resources); }
static int32_t                 USART7_Transfer        (const void *data_out, void *data_in, uint32_t num)   { return USART_Transfer (data_out, data_in, num, &USART7_Resources); }
static uint32_t                USART7_GetTxCount      (void)                                                { return USART_GetTxCount (&USART7_Resources); }
static uint32_t                USART7_GetRxCount      (void)                                                { return USART_GetRxCount (&USART7_Resources); }
static int32_t                 USART7_Control         (uint32_t control, uint32_t arg)                      { return USART_Control (control, arg, &USART7_Resources); }
static ARM_USART_STATUS        USART7_GetStatus       (void)                                                { return USART_GetStatus (&USART7_Resources); }
static int32_t                 USART7_SetModemControl (ARM_USART_MODEM_CONTROL control)                     { return USART_SetModemControl (control, &USART7_Resources); }
static ARM_USART_MODEM_STATUS  USART7_GetModemStatus  (void)                                                { return USART_GetModemStatus (&USART7_Resources); }
       void                    UART7_IRQHandler       (void)                                                {        USART_IRQHandler (&USART7_Resources); }

#ifdef MX_UART7_TX_DMA_Instance
static void                    UART7_TX_DMA_Complete (DMA_HandleTypeDef *hdma)                              { (void)hdma; USART_TX_DMA_Complete(&USART7_Resources); }

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
void UART7_TX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_UART7_TX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_uart7_tx);
}
#endif
#endif
#ifdef MX_UART7_RX_DMA_Instance
static void                    UART7_RX_DMA_Complete (DMA_HandleTypeDef *hdma)                              { (void)hdma; USART_RX_DMA_Complete(&USART7_Resources); }

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
void UART7_RX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_UART7_RX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_uart7_rx);
}
#endif
#endif

// USART7 Driver Control Block
ARM_DRIVER_USART Driver_USART7 = {
    USARTx_GetVersion,
    USART7_GetCapabilities,
    USART7_Initialize,
    USART7_Uninitialize,
    USART7_PowerControl,
    USART7_Send, 
    USART7_Receive,
    USART7_Transfer,
    USART7_GetTxCount,
    USART7_GetRxCount,
    USART7_Control,
    USART7_GetStatus,
    USART7_SetModemControl,
    USART7_GetModemStatus
};
#endif

#ifdef MX_UART8
// USART8 Driver Wrapper functions
static ARM_USART_CAPABILITIES  USART8_GetCapabilities (void)                                                { return USART_GetCapabilities (&USART8_Resources); }
static int32_t                 USART8_Initialize      (ARM_USART_SignalEvent_t cb_event)                    { return USART_Initialize (cb_event, &USART8_Resources); }
static int32_t                 USART8_Uninitialize    (void)                                                { return USART_Uninitialize (&USART8_Resources); }
static int32_t                 USART8_PowerControl    (ARM_POWER_STATE state)                               { return USART_PowerControl (state, &USART8_Resources); }
static int32_t                 USART8_Send            (const void *data, uint32_t num)                      { return USART_Send (data, num, &USART8_Resources); }
static int32_t                 USART8_Receive         (void *data, uint32_t num)                            { return USART_Receive (data, num, &USART8_Resources); }
static int32_t                 USART8_Transfer        (const void *data_out, void *data_in, uint32_t num)   { return USART_Transfer (data_out, data_in, num, &USART8_Resources); }
static uint32_t                USART8_GetTxCount      (void)                                                { return USART_GetTxCount (&USART8_Resources); }
static uint32_t                USART8_GetRxCount      (void)                                                { return USART_GetRxCount (&USART8_Resources); }
static int32_t                 USART8_Control         (uint32_t control, uint32_t arg)                      { return USART_Control (control, arg, &USART8_Resources); }
static ARM_USART_STATUS        USART8_GetStatus       (void)                                                { return USART_GetStatus (&USART8_Resources); }
static int32_t                 USART8_SetModemControl (ARM_USART_MODEM_CONTROL control)                     { return USART_SetModemControl (control, &USART8_Resources); }
static ARM_USART_MODEM_STATUS  USART8_GetModemStatus  (void)                                                { return USART_GetModemStatus (&USART8_Resources); }
       void                    UART8_IRQHandler       (void)                                                {        USART_IRQHandler (&USART8_Resources); }
#ifdef MX_UART8_TX_DMA_Instance
static void                    UART8_TX_DMA_Complete (DMA_HandleTypeDef *hdma)                              { (void)hdma; USART_TX_DMA_Complete(&USART8_Resources); }

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
void UART8_TX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_UART8_TX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_uart8_tx);
}
#endif
#endif
#ifdef MX_UART8_RX_DMA_Instance
static void                    UART8_RX_DMA_Complete (DMA_HandleTypeDef *hdma)                              { (void)hdma; USART_RX_DMA_Complete(&USART8_Resources); }

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
void UART8_RX_DMA_Handler (void) {
  HAL_NVIC_ClearPendingIRQ(MX_UART8_RX_DMA_IRQn);
  HAL_DMA_IRQHandler(&hdma_uart8_rx);
}
#endif
#endif

// USART8 Driver Control Block
ARM_DRIVER_USART Driver_USART8 = {
    USARTx_GetVersion,
    USART8_GetCapabilities,
    USART8_Initialize,
    USART8_Uninitialize,
    USART8_PowerControl,
    USART8_Send, 
    USART8_Receive,
    USART8_Transfer,
    USART8_GetTxCount,
    USART8_GetRxCount,
    USART8_Control,
    USART8_GetStatus,
    USART8_SetModemControl,
    USART8_GetModemStatus
};
#endif

/*! \endcond */
