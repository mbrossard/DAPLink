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
 * $Revision:    V1.8
 *
 * Project:      USART Driver definitions for ST STM32F7xx
 * -------------------------------------------------------------------------- */

#ifndef __USART_STM32F7XX_H
#define __USART_STM32F7XX_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "Driver_USART.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_ll_rcc.h"

#include "RTE_Components.h"
#if   defined(RTE_DEVICE_FRAMEWORK_CLASSIC)
#include "RTE_Device.h"
#elif defined(RTE_DEVICE_FRAMEWORK_CUBE_MX)
  #define  VM_ASYNC                      (1UL)
  #define  VM_SYNC                       (2UL)
  #define  VM_IRDA                       (3UL)
  #define  VM_SMARTCARD                  (4UL)
  #define  Asynchronous                  VM_ASYNC
  #define  IrDA                          VM_IRDA
#include "MX_Device.h"
#else
#error "::Device:STM32Cube Framework: not selected in RTE"
#endif

#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  #if ((defined(RTE_Drivers_USART1) || \
        defined(RTE_Drivers_USART2) || \
        defined(RTE_Drivers_USART3) || \
        defined(RTE_Drivers_UART4 ) || \
        defined(RTE_Drivers_UART5 ) || \
        defined(RTE_Drivers_USART6) || \
        defined(RTE_Drivers_UART7 ) || \
        defined(RTE_Drivers_UART8 ))   \
       && (RTE_USART1 == 0)            \
       && (RTE_USART2 == 0)            \
       && (RTE_USART3 == 0)            \
       && (RTE_UART4  == 0)            \
       && (RTE_UART5  == 0)            \
       && (RTE_USART6 == 0)            \
       && (RTE_UART7  == 0)            \
       && (RTE_UART8  == 0))
    #error "USART not configured in RTE_Device.h!"
  #endif

// RTE macros
#define _DMA_CHANNEL_x(x)               DMA_CHANNEL_##x
#define  DMA_CHANNEL_x(x)              _DMA_CHANNEL_x(x)

#define  DMA_PRIORITY(x)              ((x == 0) ? DMA_PRIORITY_LOW    : \
                                       (x == 1) ? DMA_PRIORITY_MEDIUM : \
                                       (x == 2) ? DMA_PRIORITY_HIGH   : \
                                                  DMA_PRIORITY_VERY_HIGH)

#define _DMAx_STREAMy(x, y)             DMA##x##_Stream##y
#define  DMAx_STREAMy(x, y)            _DMAx_STREAMy(x, y)

#define _DMAx_STREAMy_IRQ(x, y)         DMA##x##_Stream##y##_IRQHandler
#define  DMAx_STREAMy_IRQ(x, y)        _DMAx_STREAMy_IRQ(x, y)

#define _DMAx_STREAMy_IRQn(x, y)        DMA##x##_Stream##y##_IRQn
#define  DMAx_STREAMy_IRQn(x, y)       _DMAx_STREAMy_IRQn(x, y)

// USART1 configuration definitions
#if (RTE_USART1 == 1)
  #define MX_USART1

  #if (RTE_USART1_RX_DMA == 1)
    #define MX_USART1_RX_DMA_Instance DMAx_STREAMy(RTE_USART1_RX_DMA_NUMBER, RTE_USART1_RX_DMA_STREAM)
    #define MX_USART1_RX_DMA_IRQn     DMAx_STREAMy_IRQn(RTE_USART1_RX_DMA_NUMBER, RTE_USART1_RX_DMA_STREAM)
    #define MX_USART1_RX_DMA_Channel  DMA_CHANNEL_x(RTE_USART1_RX_DMA_CHANNEL)
    #define MX_USART1_RX_DMA_Priority DMA_PRIORITY(RTE_USART1_RX_DMA_PRIORITY)

    #define USART1_RX_DMA_Handler     DMAx_STREAMy_IRQ(RTE_USART1_RX_DMA_NUMBER, RTE_USART1_RX_DMA_STREAM)
  #endif
  #if (RTE_USART1_TX_DMA == 1)
    #define MX_USART1_TX_DMA_Instance DMAx_STREAMy(RTE_USART1_TX_DMA_NUMBER, RTE_USART1_TX_DMA_STREAM)
    #define MX_USART1_TX_DMA_IRQn     DMAx_STREAMy_IRQn(RTE_USART1_TX_DMA_NUMBER, RTE_USART1_TX_DMA_STREAM)
    #define MX_USART1_TX_DMA_Channel  DMA_CHANNEL_x(RTE_USART1_TX_DMA_CHANNEL)
    #define MX_USART1_TX_DMA_Priority DMA_PRIORITY(RTE_USART1_TX_DMA_PRIORITY)

    #define USART1_TX_DMA_Handler     DMAx_STREAMy_IRQ(RTE_USART1_TX_DMA_NUMBER, RTE_USART1_TX_DMA_STREAM)
  #endif

  #ifndef USART1_GPIO_SPEED_FREQ
    #define USART1_GPIO_SPEED_FREQ    GPIO_SPEED_FREQ_LOW
  #endif

  #if (RTE_USART1_TX == 1)
    #define MX_USART1_TX_Pin          1
    #define MX_USART1_TX_GPIOx        RTE_USART1_TX_PORT
    #define MX_USART1_TX_GPIO_Pin    (1U << RTE_USART1_TX_BIT)
    #define MX_USART1_TX_GPIO_PuPd    GPIO_NOPULL
    #define MX_USART1_TX_GPIO_Speed   USART1_GPIO_SPEED_FREQ
    #if   (RTE_USART1_TX_ID == 3)
      #define MX_USART1_TX_GPIO_AF    GPIO_AF4_USART1
    #else
      #define MX_USART1_TX_GPIO_AF    GPIO_AF7_USART1
    #endif
  #endif

  #if (RTE_USART1_RX == 1)
    #define MX_USART1_RX_Pin          1
    #define MX_USART1_RX_GPIOx        RTE_USART1_RX_PORT
    #define MX_USART1_RX_GPIO_Pin    (1U << RTE_USART1_RX_BIT)
    #define MX_USART1_RX_GPIO_PuPd    GPIO_NOPULL
    #define MX_USART1_RX_GPIO_Speed   USART1_GPIO_SPEED_FREQ
    #if   (RTE_USART1_RX_ID == 3)
      #define MX_USART1_RX_GPIO_AF    GPIO_AF4_USART1
    #else
      #define MX_USART1_RX_GPIO_AF    GPIO_AF7_USART1
    #endif
  #endif

  #if (RTE_USART1_CK == 1)
    #define MX_USART1_CK_Pin          1
    #define MX_USART1_CK_GPIOx        RTE_USART1_CK_PORT
    #define MX_USART1_CK_GPIO_Pin    (1U << RTE_USART1_CK_BIT)
    #define MX_USART1_CK_GPIO_PuPd    GPIO_NOPULL
    #define MX_USART1_CK_GPIO_Speed   USART1_GPIO_SPEED_FREQ
    #define MX_USART1_CK_GPIO_AF      GPIO_AF7_USART1
  #endif


  #if (RTE_USART1_RTS == 1)
    #define MX_USART1_RTS_Pin         1
    #define MX_USART1_RTS_GPIOx       RTE_USART1_RTS_PORT
    #define MX_USART1_RTS_GPIO_Pin   (1U << RTE_USART1_RTS_BIT)
    #define MX_USART1_RTS_GPIO_PuPd   GPIO_NOPULL
    #define MX_USART1_RTS_GPIO_Speed  USART1_GPIO_SPEED_FREQ
    #define MX_USART1_RTS_GPIO_AF     GPIO_AF7_USART1
  #endif

  #if (RTE_USART1_CTS == 1)
    #define MX_USART1_CTS_Pin         1
    #define MX_USART1_CTS_GPIOx       RTE_USART1_CTS_PORT
    #define MX_USART1_CTS_GPIO_Pin   (1U << RTE_USART1_CTS_BIT)
    #define MX_USART1_CTS_GPIO_PuPd   GPIO_NOPULL
    #define MX_USART1_CTS_GPIO_Speed  USART1_GPIO_SPEED_FREQ
    #define MX_USART1_CTS_GPIO_AF     GPIO_AF7_USART1
  #endif
#endif

// USART2 configuration definitions
#if (RTE_USART2 == 1)
  #define MX_USART2

  #if (RTE_USART2_RX_DMA == 1)
    #define MX_USART2_RX_DMA_Instance DMAx_STREAMy(RTE_USART2_RX_DMA_NUMBER, RTE_USART2_RX_DMA_STREAM)
    #define MX_USART2_RX_DMA_IRQn     DMAx_STREAMy_IRQn(RTE_USART2_RX_DMA_NUMBER, RTE_USART2_RX_DMA_STREAM)
    #define MX_USART2_RX_DMA_Channel  DMA_CHANNEL_x(RTE_USART2_RX_DMA_CHANNEL)
    #define MX_USART2_RX_DMA_Priority DMA_PRIORITY(RTE_USART2_RX_DMA_PRIORITY)

    #define USART2_RX_DMA_Handler     DMAx_STREAMy_IRQ(RTE_USART2_RX_DMA_NUMBER, RTE_USART2_RX_DMA_STREAM)
  #endif
  #if (RTE_USART2_TX_DMA == 1)
    #define MX_USART2_TX_DMA_Instance DMAx_STREAMy(RTE_USART2_TX_DMA_NUMBER, RTE_USART2_TX_DMA_STREAM)
    #define MX_USART2_TX_DMA_IRQn     DMAx_STREAMy_IRQn(RTE_USART2_TX_DMA_NUMBER, RTE_USART2_TX_DMA_STREAM)
    #define MX_USART2_TX_DMA_Channel  DMA_CHANNEL_x(RTE_USART2_TX_DMA_CHANNEL)
    #define MX_USART2_TX_DMA_Priority DMA_PRIORITY(RTE_USART2_TX_DMA_PRIORITY)

    #define USART2_TX_DMA_Handler     DMAx_STREAMy_IRQ(RTE_USART2_TX_DMA_NUMBER, RTE_USART2_TX_DMA_STREAM)
  #endif

  #ifndef USART2_GPIO_SPEED_FREQ
    #define USART2_GPIO_SPEED_FREQ    GPIO_SPEED_FREQ_LOW
  #endif

  #if (RTE_USART2_TX == 1)
    #define MX_USART2_TX_Pin          1
    #define MX_USART2_TX_GPIOx        RTE_USART2_TX_PORT
    #define MX_USART2_TX_GPIO_Pin    (1U << RTE_USART2_TX_BIT)
    #define MX_USART2_TX_GPIO_PuPd    GPIO_NOPULL
    #define MX_USART2_TX_GPIO_Speed   USART2_GPIO_SPEED_FREQ
    #define MX_USART2_TX_GPIO_AF      GPIO_AF7_USART2
  #endif

  #if (RTE_USART2_RX == 1)
    #define MX_USART2_RX_Pin          1
    #define MX_USART2_RX_GPIOx        RTE_USART2_RX_PORT
    #define MX_USART2_RX_GPIO_Pin    (1U << RTE_USART2_RX_BIT)
    #define MX_USART2_RX_GPIO_PuPd    GPIO_NOPULL
    #define MX_USART2_RX_GPIO_Speed   USART2_GPIO_SPEED_FREQ
    #define MX_USART2_RX_GPIO_AF      GPIO_AF7_USART2
  #endif

  #if (RTE_USART2_CK == 1)
    #define MX_USART2_CK_Pin          1
    #define MX_USART2_CK_GPIOx        RTE_USART2_CK_PORT
    #define MX_USART2_CK_GPIO_Pin    (1U << RTE_USART2_CK_BIT)
    #define MX_USART2_CK_GPIO_PuPd    GPIO_NOPULL
    #define MX_USART2_CK_GPIO_Speed   USART2_GPIO_SPEED_FREQ
    #define MX_USART2_CK_GPIO_AF      GPIO_AF7_USART2
  #endif


  #if (RTE_USART2_RTS == 1)
    #define MX_USART2_RTS_Pin         1
    #define MX_USART2_RTS_GPIOx       RTE_USART2_RTS_PORT
    #define MX_USART2_RTS_GPIO_Pin   (1U << RTE_USART2_RTS_BIT)
    #define MX_USART2_RTS_GPIO_PuPd   GPIO_NOPULL
    #define MX_USART2_RTS_GPIO_Speed  USART2_GPIO_SPEED_FREQ
    #define MX_USART2_RTS_GPIO_AF     GPIO_AF7_USART2
  #endif

  #if (RTE_USART2_CTS == 1)
    #define MX_USART2_CTS_Pin         1
    #define MX_USART2_CTS_GPIOx       RTE_USART2_CTS_PORT
    #define MX_USART2_CTS_GPIO_Pin   (1U << RTE_USART2_CTS_BIT)
    #define MX_USART2_CTS_GPIO_PuPd   GPIO_NOPULL
    #define MX_USART2_CTS_GPIO_Speed  USART2_GPIO_SPEED_FREQ
    #define MX_USART2_CTS_GPIO_AF     GPIO_AF7_USART2
  #endif
#endif

// USART3 configuration definitions
#if (RTE_USART3 == 1)

  #ifndef USART3
    #error "USART3 not available for selected device!"
  #endif

  #define MX_USART3

  #if (RTE_USART3_RX_DMA == 1)
    #define MX_USART3_RX_DMA_Instance DMAx_STREAMy(RTE_USART3_RX_DMA_NUMBER, RTE_USART3_RX_DMA_STREAM)
    #define MX_USART3_RX_DMA_IRQn     DMAx_STREAMy_IRQn(RTE_USART3_RX_DMA_NUMBER, RTE_USART3_RX_DMA_STREAM)
    #define MX_USART3_RX_DMA_Channel  DMA_CHANNEL_x(RTE_USART3_RX_DMA_CHANNEL)
    #define MX_USART3_RX_DMA_Priority DMA_PRIORITY(RTE_USART3_RX_DMA_PRIORITY)

    #define USART3_RX_DMA_Handler     DMAx_STREAMy_IRQ(RTE_USART3_RX_DMA_NUMBER, RTE_USART3_RX_DMA_STREAM)
  #endif
  #if (RTE_USART3_TX_DMA == 1)
    #if ((RTE_USART3_TX_DMA_STREAM == 3) && (RTE_USART3_TX_DMA_CHANNEL == 7))
      #error "USART3 TX DMA Configuration error: Stream 3 requires Channel 4."
    #endif
    #if ((RTE_USART3_TX_DMA_STREAM == 4) && (RTE_USART3_TX_DMA_CHANNEL == 4))
      #error "USART3 TX DMA Configuration error: Stream 4 requires Channel 7."
    #endif
    #define MX_USART3_TX_DMA_Instance DMAx_STREAMy(RTE_USART3_TX_DMA_NUMBER, RTE_USART3_TX_DMA_STREAM)
    #define MX_USART3_TX_DMA_IRQn     DMAx_STREAMy_IRQn(RTE_USART3_TX_DMA_NUMBER, RTE_USART3_TX_DMA_STREAM)
    #define MX_USART3_TX_DMA_Channel  DMA_CHANNEL_x(RTE_USART3_TX_DMA_CHANNEL)
    #define MX_USART3_TX_DMA_Priority DMA_PRIORITY(RTE_USART3_TX_DMA_PRIORITY)

    #define USART3_TX_DMA_Handler     DMAx_STREAMy_IRQ(RTE_USART3_TX_DMA_NUMBER, RTE_USART3_TX_DMA_STREAM)
  #endif

  #ifndef USART3_GPIO_SPEED_FREQ
    #define USART3_GPIO_SPEED_FREQ    GPIO_SPEED_FREQ_LOW
  #endif

  #if (RTE_USART3_TX == 1)
    #define MX_USART3_TX_Pin          1
    #define MX_USART3_TX_GPIOx        RTE_USART3_TX_PORT
    #define MX_USART3_TX_GPIO_Pin    (1U << RTE_USART3_TX_BIT)
    #define MX_USART3_TX_GPIO_PuPd    GPIO_NOPULL
    #define MX_USART3_TX_GPIO_Speed   USART3_GPIO_SPEED_FREQ
    #define MX_USART3_TX_GPIO_AF      GPIO_AF7_USART3
  #endif

  #if (RTE_USART3_RX == 1)
    #define MX_USART3_RX_Pin          1
    #define MX_USART3_RX_GPIOx        RTE_USART3_RX_PORT
    #define MX_USART3_RX_GPIO_Pin    (1U << RTE_USART3_RX_BIT)
    #define MX_USART3_RX_GPIO_PuPd    GPIO_NOPULL
    #define MX_USART3_RX_GPIO_Speed   USART3_GPIO_SPEED_FREQ
    #define MX_USART3_RX_GPIO_AF      GPIO_AF7_USART3
  #endif

  #if (RTE_USART3_CK == 1)
    #define MX_USART3_CK_Pin          1
    #define MX_USART3_CK_GPIOx        RTE_USART3_CK_PORT
    #define MX_USART3_CK_GPIO_Pin    (1U << RTE_USART3_CK_BIT)
    #define MX_USART3_CK_GPIO_PuPd    GPIO_NOPULL
    #define MX_USART3_CK_GPIO_Speed   USART3_GPIO_SPEED_FREQ
    #define MX_USART3_CK_GPIO_AF      GPIO_AF7_USART3
  #endif


  #if (RTE_USART3_RTS == 1)
    #define MX_USART3_RTS_Pin         1
    #define MX_USART3_RTS_GPIOx       RTE_USART3_RTS_PORT
    #define MX_USART3_RTS_GPIO_Pin   (1U << RTE_USART3_RTS_BIT)
    #define MX_USART3_RTS_GPIO_PuPd   GPIO_NOPULL
    #define MX_USART3_RTS_GPIO_Speed  USART3_GPIO_SPEED_FREQ
    #define MX_USART3_RTS_GPIO_AF     GPIO_AF7_USART3
  #endif

  #if (RTE_USART3_CTS == 1)
    #define MX_USART3_CTS_Pin         1
    #define MX_USART3_CTS_GPIOx       RTE_USART3_CTS_PORT
    #define MX_USART3_CTS_GPIO_Pin   (1U << RTE_USART3_CTS_BIT)
    #define MX_USART3_CTS_GPIO_PuPd   GPIO_NOPULL
    #define MX_USART3_CTS_GPIO_Speed  USART3_GPIO_SPEED_FREQ
    #define MX_USART3_CTS_GPIO_AF     GPIO_AF7_USART3
  #endif
#endif

// UART4 configuration definitions
#if (RTE_UART4 == 1)

  #ifndef UART4
    #error "UART4 not available for selected device!"
  #endif

  #define MX_UART4

  #if (RTE_UART4_RX_DMA == 1)
    #define MX_UART4_RX_DMA_Instance  DMAx_STREAMy(RTE_UART4_RX_DMA_NUMBER, RTE_UART4_RX_DMA_STREAM)
    #define MX_UART4_RX_DMA_IRQn      DMAx_STREAMy_IRQn(RTE_UART4_RX_DMA_NUMBER, RTE_UART4_RX_DMA_STREAM)
    #define MX_UART4_RX_DMA_Channel   DMA_CHANNEL_x(RTE_UART4_RX_DMA_CHANNEL)
    #define MX_UART4_RX_DMA_Priority  DMA_PRIORITY(RTE_UART4_RX_DMA_PRIORITY)

    #define UART4_RX_DMA_Handler      DMAx_STREAMy_IRQ(RTE_UART4_RX_DMA_NUMBER, RTE_UART4_RX_DMA_STREAM)
  #endif
  #if (RTE_UART4_TX_DMA == 1)
    #define MX_UART4_TX_DMA_Instance  DMAx_STREAMy(RTE_UART4_TX_DMA_NUMBER, RTE_UART4_TX_DMA_STREAM)
    #define MX_UART4_TX_DMA_IRQn      DMAx_STREAMy_IRQn(RTE_UART4_TX_DMA_NUMBER, RTE_UART4_TX_DMA_STREAM)
    #define MX_UART4_TX_DMA_Channel   DMA_CHANNEL_x(RTE_UART4_TX_DMA_CHANNEL)
    #define MX_UART4_TX_DMA_Priority  DMA_PRIORITY(RTE_UART4_TX_DMA_PRIORITY)

    #define UART4_TX_DMA_Handler      DMAx_STREAMy_IRQ(RTE_UART4_TX_DMA_NUMBER, RTE_UART4_TX_DMA_STREAM)
  #endif

  #ifndef UART4_GPIO_SPEED_FREQ
    #define UART4_GPIO_SPEED_FREQ     GPIO_SPEED_FREQ_LOW
  #endif

  #if (RTE_UART4_TX == 1)
    #define MX_UART4_TX_Pin           1
    #define MX_UART4_TX_GPIOx         RTE_UART4_TX_PORT
    #define MX_UART4_TX_GPIO_Pin     (1U << RTE_UART4_TX_BIT)
    #define MX_UART4_TX_GPIO_PuPd     GPIO_NOPULL
    #define MX_UART4_TX_GPIO_Speed    UART4_GPIO_SPEED_FREQ
    #if   (RTE_UART4_TX_ID == 3)
      #define MX_UART4_TX_GPIO_AF     GPIO_AF6_UART4
    #else
      #define MX_UART4_TX_GPIO_AF     GPIO_AF8_UART4
    #endif
  #endif

  #if (RTE_UART4_RX == 1)
    #define MX_UART4_RX_Pin           1
    #define MX_UART4_RX_GPIOx         RTE_UART4_RX_PORT
    #define MX_UART4_RX_GPIO_Pin     (1U << RTE_UART4_RX_BIT)
    #define MX_UART4_RX_GPIO_PuPd     GPIO_NOPULL
    #define MX_UART4_RX_GPIO_Speed    UART4_GPIO_SPEED_FREQ
    #if   (RTE_UART4_RX_ID == 3)
      #define MX_UART4_RX_GPIO_AF     GPIO_AF6_UART4
    #else
      #define MX_UART4_RX_GPIO_AF     GPIO_AF8_UART4
    #endif
  #endif

  #if (RTE_UART4_RTS == 1)
    #define MX_UART4_RTS_Pin          1
    #define MX_UART4_RTS_GPIOx        RTE_UART4_RTS_PORT
    #define MX_UART4_RTS_GPIO_Pin    (1U << RTE_UART4_RTS_BIT)
    #define MX_UART4_RTS_GPIO_PuPd    GPIO_NOPULL
    #define MX_UART4_RTS_GPIO_Speed   UART4_GPIO_SPEED_FREQ
    #define MX_UART4_RTS_GPIO_AF      GPIO_AF8_UART4
  #endif

  #if (RTE_UART4_CTS == 1)
    #define MX_UART4_CTS_Pin          1
    #define MX_UART4_CTS_GPIOx        RTE_UART4_CTS_PORT
    #define MX_UART4_CTS_GPIO_Pin    (1U << RTE_UART4_CTS_BIT)
    #define MX_UART4_CTS_GPIO_PuPd    GPIO_NOPULL
    #define MX_UART4_CTS_GPIO_Speed   UART4_GPIO_SPEED_FREQ
    #define MX_UART4_CTS_GPIO_AF      GPIO_AF8_UART4
  #endif
#endif

// UART5 configuration definitions
#if (RTE_UART5 == 1)

  #ifndef UART5
    #error "UART5 not available for selected device!"
  #endif

  #define MX_UART5

  #if (RTE_UART5_RX_DMA == 1)
    #define MX_UART5_RX_DMA_Instance  DMAx_STREAMy(RTE_UART5_RX_DMA_NUMBER, RTE_UART5_RX_DMA_STREAM)
    #define MX_UART5_RX_DMA_IRQn      DMAx_STREAMy_IRQn(RTE_UART5_RX_DMA_NUMBER, RTE_UART5_RX_DMA_STREAM)
    #define MX_UART5_RX_DMA_Channel   DMA_CHANNEL_x(RTE_UART5_RX_DMA_CHANNEL)
    #define MX_UART5_RX_DMA_Priority  DMA_PRIORITY(RTE_UART5_RX_DMA_PRIORITY)

    #define UART5_RX_DMA_Handler      DMAx_STREAMy_IRQ(RTE_UART5_RX_DMA_NUMBER, RTE_UART5_RX_DMA_STREAM)
  #endif
  #if (RTE_UART5_TX_DMA == 1)
    #define MX_UART5_TX_DMA_Instance  DMAx_STREAMy(RTE_UART5_TX_DMA_NUMBER, RTE_UART5_TX_DMA_STREAM)
    #define MX_UART5_TX_DMA_IRQn      DMAx_STREAMy_IRQn(RTE_UART5_TX_DMA_NUMBER, RTE_UART5_TX_DMA_STREAM)
    #define MX_UART5_TX_DMA_Channel   DMA_CHANNEL_x(RTE_UART5_TX_DMA_CHANNEL)
    #define MX_UART5_TX_DMA_Priority  DMA_PRIORITY(RTE_UART5_TX_DMA_PRIORITY)

    #define UART5_TX_DMA_Handler      DMAx_STREAMy_IRQ(RTE_UART5_TX_DMA_NUMBER, RTE_UART5_TX_DMA_STREAM)
  #endif

  #ifndef UART5_GPIO_SPEED_FREQ
    #define UART5_GPIO_SPEED_FREQ     GPIO_SPEED_FREQ_LOW
  #endif

  #if (RTE_UART5_TX == 1)
    #define MX_UART5_TX_Pin           1
    #define MX_UART5_TX_GPIOx         RTE_UART5_TX_PORT
    #define MX_UART5_TX_GPIO_Pin     (1U << RTE_UART5_TX_BIT)
    #define MX_UART5_TX_GPIO_PuPd     GPIO_NOPULL
    #define MX_UART5_TX_GPIO_Speed    UART5_GPIO_SPEED_FREQ
    #if   (RTE_UART5_TX_ID == 2)
      #define MX_UART5_TX_GPIO_AF     GPIO_AF1_UART5
    #elif (RTE_UART5_TX_ID == 3)
      #define MX_UART5_TX_GPIO_AF     GPIO_AF7_UART5
    #else
      #define MX_UART5_TX_GPIO_AF     GPIO_AF8_UART5
    #endif
  #endif
  
  #if (RTE_UART5_RX == 1)
    #define MX_UART5_RX_Pin           1
    #define MX_UART5_RX_GPIOx         RTE_UART5_RX_PORT
    #define MX_UART5_RX_GPIO_Pin     (1U << RTE_UART5_RX_BIT)
    #define MX_UART5_RX_GPIO_PuPd     GPIO_NOPULL
    #define MX_UART5_RX_GPIO_Speed    UART5_GPIO_SPEED_FREQ
    #if   (RTE_UART5_RX_ID == 2)
      #define MX_UART5_RX_GPIO_AF     GPIO_AF1_UART5
    #elif (RTE_UART5_RX_ID == 3)
      #define MX_UART5_RX_GPIO_AF     GPIO_AF7_UART5
    #else
      #define MX_UART5_RX_GPIO_AF     GPIO_AF8_UART5
    #endif
  #endif

  #if (RTE_UART5_RTS == 1)
    #define MX_UART5_RTS_Pin          1
    #define MX_UART5_RTS_GPIOx        RTE_UART5_RTS_PORT
    #define MX_UART5_RTS_GPIO_Pin    (1U << RTE_UART5_RTS_BIT)
    #define MX_UART5_RTS_GPIO_PuPd    GPIO_NOPULL
    #define MX_UART5_RTS_GPIO_Speed   UART5_GPIO_SPEED_FREQ
    #define MX_UART5_RTS_GPIO_AF      GPIO_AF7_UART5
  #endif

  #if (RTE_UART5_CTS == 1)
    #define MX_UART5_CTS_Pin          1
    #define MX_UART5_CTS_GPIOx        RTE_UART5_CTS_PORT
    #define MX_UART5_CTS_GPIO_Pin    (1U << RTE_UART5_CTS_BIT)
    #define MX_UART5_CTS_GPIO_PuPd    GPIO_NOPULL
    #define MX_UART5_CTS_GPIO_Speed   UART5_GPIO_SPEED_FREQ
    #define MX_UART5_CTS_GPIO_AF      GPIO_AF7_UART5
  #endif
#endif

// USART6 configuration definitions
#if (RTE_USART6 == 1)
  #define MX_USART6

  #if (RTE_USART6_RX_DMA == 1)
    #define MX_USART6_RX_DMA_Instance DMAx_STREAMy(RTE_USART6_RX_DMA_NUMBER, RTE_USART6_RX_DMA_STREAM)
    #define MX_USART6_RX_DMA_IRQn     DMAx_STREAMy_IRQn(RTE_USART6_RX_DMA_NUMBER, RTE_USART6_RX_DMA_STREAM)
    #define MX_USART6_RX_DMA_Channel  DMA_CHANNEL_x(RTE_USART6_RX_DMA_CHANNEL)
    #define MX_USART6_RX_DMA_Priority DMA_PRIORITY(RTE_USART6_RX_DMA_PRIORITY)

    #define USART6_RX_DMA_Handler     DMAx_STREAMy_IRQ(RTE_USART6_RX_DMA_NUMBER, RTE_USART6_RX_DMA_STREAM)
  #endif
  #if (RTE_USART6_TX_DMA == 1)
    #define MX_USART6_TX_DMA_Instance DMAx_STREAMy(RTE_USART6_TX_DMA_NUMBER, RTE_USART6_TX_DMA_STREAM)
    #define MX_USART6_TX_DMA_IRQn     DMAx_STREAMy_IRQn(RTE_USART6_TX_DMA_NUMBER, RTE_USART6_TX_DMA_STREAM)
    #define MX_USART6_TX_DMA_Channel  DMA_CHANNEL_x(RTE_USART6_TX_DMA_CHANNEL)
    #define MX_USART6_TX_DMA_Priority DMA_PRIORITY(RTE_USART6_TX_DMA_PRIORITY)

    #define USART6_TX_DMA_Handler     DMAx_STREAMy_IRQ(RTE_USART6_TX_DMA_NUMBER, RTE_USART6_TX_DMA_STREAM)
  #endif

  #ifndef USART6_GPIO_SPEED_FREQ
    #define USART6_GPIO_SPEED_FREQ    GPIO_SPEED_FREQ_LOW
  #endif

  #if (RTE_USART6_TX == 1)
    #define MX_USART6_TX_Pin          1
    #define MX_USART6_TX_GPIOx        RTE_USART6_TX_PORT
    #define MX_USART6_TX_GPIO_Pin    (1U << RTE_USART6_TX_BIT)
    #define MX_USART6_TX_GPIO_PuPd    GPIO_NOPULL
    #define MX_USART6_TX_GPIO_Speed   USART6_GPIO_SPEED_FREQ
    #define MX_USART6_TX_GPIO_AF      GPIO_AF8_USART6
  #endif

  #if (RTE_USART6_RX == 1)
    #define MX_USART6_RX_Pin          1
    #define MX_USART6_RX_GPIOx        RTE_USART6_RX_PORT
    #define MX_USART6_RX_GPIO_Pin    (1U << RTE_USART6_RX_BIT)
    #define MX_USART6_RX_GPIO_PuPd    GPIO_NOPULL
    #define MX_USART6_RX_GPIO_Speed   USART6_GPIO_SPEED_FREQ
    #define MX_USART6_RX_GPIO_AF      GPIO_AF8_USART6
  #endif

  #if (RTE_USART6_CK == 1)
    #define MX_USART6_CK_Pin          1
    #define MX_USART6_CK_GPIOx        RTE_USART6_CK_PORT
    #define MX_USART6_CK_GPIO_Pin    (1U << RTE_USART6_CK_BIT)
    #define MX_USART6_CK_GPIO_PuPd    GPIO_NOPULL
    #define MX_USART6_CK_GPIO_Speed   USART6_GPIO_SPEED_FREQ
    #define MX_USART6_CK_GPIO_AF      GPIO_AF8_USART6
  #endif

  #if (RTE_USART6_RTS == 1)
    #define MX_USART6_RTS_Pin         1
    #define MX_USART6_RTS_GPIOx       RTE_USART6_RTS_PORT
    #define MX_USART6_RTS_GPIO_Pin   (1U << RTE_USART6_RTS_BIT)
    #define MX_USART6_RTS_GPIO_PuPd   GPIO_NOPULL
    #define MX_USART6_RTS_GPIO_Speed  USART6_GPIO_SPEED_FREQ
    #define MX_USART6_RTS_GPIO_AF     GPIO_AF8_USART6
  #endif

  #if (RTE_USART6_CTS == 1)
    #define MX_USART6_CTS_Pin         1
    #define MX_USART6_CTS_GPIOx       RTE_USART6_CTS_PORT
    #define MX_USART6_CTS_GPIO_Pin   (1U << RTE_USART6_CTS_BIT)
    #define MX_USART6_CTS_GPIO_PuPd   GPIO_NOPULL
    #define MX_USART6_CTS_GPIO_Speed  USART6_GPIO_SPEED_FREQ
    #define MX_USART6_CTS_GPIO_AF     GPIO_AF8_USART6
  #endif
#endif

// UART7 configuration definitions
#if (RTE_UART7 == 1)

  #ifndef UART7
    #error "UART7 not available for selected device!"
  #endif

  #define MX_UART7

  #if (RTE_UART7_RX_DMA == 1)
    #define MX_UART7_RX_DMA_Instance  DMAx_STREAMy(RTE_UART7_RX_DMA_NUMBER, RTE_UART7_RX_DMA_STREAM)
    #define MX_UART7_RX_DMA_IRQn      DMAx_STREAMy_IRQn(RTE_UART7_RX_DMA_NUMBER, RTE_UART7_RX_DMA_STREAM)
    #define MX_UART7_RX_DMA_Channel   DMA_CHANNEL_x(RTE_UART7_RX_DMA_CHANNEL)
    #define MX_UART7_RX_DMA_Priority  DMA_PRIORITY(RTE_UART7_RX_DMA_PRIORITY)

    #define UART7_RX_DMA_Handler      DMAx_STREAMy_IRQ(RTE_UART7_RX_DMA_NUMBER, RTE_UART7_RX_DMA_STREAM)
  #endif
  #if (RTE_UART7_TX_DMA == 1)
    #define MX_UART7_TX_DMA_Instance  DMAx_STREAMy(RTE_UART7_TX_DMA_NUMBER, RTE_UART7_TX_DMA_STREAM)
    #define MX_UART7_TX_DMA_IRQn      DMAx_STREAMy_IRQn(RTE_UART7_TX_DMA_NUMBER, RTE_UART7_TX_DMA_STREAM)
    #define MX_UART7_TX_DMA_Channel   DMA_CHANNEL_x(RTE_UART7_TX_DMA_CHANNEL)
    #define MX_UART7_TX_DMA_Priority  DMA_PRIORITY(RTE_UART7_TX_DMA_PRIORITY)

    #define UART7_TX_DMA_Handler      DMAx_STREAMy_IRQ(RTE_UART7_TX_DMA_NUMBER, RTE_UART7_TX_DMA_STREAM)
  #endif

  #ifndef UART7_GPIO_SPEED_FREQ
    #define UART7_GPIO_SPEED_FREQ     GPIO_SPEED_FREQ_LOW
  #endif

  #if (RTE_UART7_TX == 1)
    #define MX_UART7_TX_Pin           1
    #define MX_UART7_TX_GPIOx         RTE_UART7_TX_PORT
    #define MX_UART7_TX_GPIO_Pin     (1U << RTE_UART7_TX_BIT)
    #define MX_UART7_TX_GPIO_PuPd     GPIO_NOPULL
    #define MX_UART7_TX_GPIO_Speed    UART7_GPIO_SPEED_FREQ
    #if   (RTE_UART7_TX_ID == 3) || (RTE_UART7_TX_ID == 4)
      #define MX_UART7_TX_GPIO_AF     GPIO_AF12_UART7
    #else
      #define MX_UART7_TX_GPIO_AF     GPIO_AF8_UART7
    #endif
  #endif

  #if (RTE_UART7_RX == 1)
    #define MX_UART7_RX_Pin           1
    #define MX_UART7_RX_GPIOx         RTE_UART7_RX_PORT
    #define MX_UART7_RX_GPIO_Pin     (1U << RTE_UART7_RX_BIT)
    #define MX_UART7_RX_GPIO_PuPd     GPIO_NOPULL
    #define MX_UART7_RX_GPIO_Speed    UART7_GPIO_SPEED_FREQ
    #if   (RTE_UART7_RX_ID == 3) || (RTE_UART7_RX_ID == 4)
      #define MX_UART7_RX_GPIO_AF     GPIO_AF12_UART7
    #else
      #define MX_UART7_RX_GPIO_AF     GPIO_AF8_UART7
    #endif
  #endif

  #if (RTE_UART7_RTS == 1)
    #define MX_UART7_RTS_Pin          1
    #define MX_UART7_RTS_GPIOx        RTE_UART7_RTS_PORT
    #define MX_UART7_RTS_GPIO_Pin    (1U << RTE_UART7_RTS_BIT)
    #define MX_UART7_RTS_GPIO_PuPd    GPIO_NOPULL
    #define MX_UART7_RTS_GPIO_Speed   UART7_GPIO_SPEED_FREQ
    #define MX_UART7_RTS_GPIO_AF      GPIO_AF8_UART7
  #endif

  #if (RTE_UART7_CTS == 1)
    #define MX_UART7_CTS_Pin          1
    #define MX_UART7_CTS_GPIOx        RTE_UART7_CTS_PORT
    #define MX_UART7_CTS_GPIO_Pin    (1U << RTE_UART7_CTS_BIT)
    #define MX_UART7_CTS_GPIO_PuPd    GPIO_NOPULL
    #define MX_UART7_CTS_GPIO_Speed   UART7_GPIO_SPEED_FREQ
    #define MX_UART7_CTS_GPIO_AF      GPIO_AF8_UART7
  #endif
#endif

// UART8 configuration definitions
#if (RTE_UART8 == 1)

  #ifndef UART8
    #error "UART8 not available for selected device!"
  #endif

  #define MX_UART8

  #if (RTE_UART8_RX_DMA == 1)
    #define MX_UART8_RX_DMA_Instance  DMAx_STREAMy(RTE_UART8_RX_DMA_NUMBER, RTE_UART8_RX_DMA_STREAM)
    #define MX_UART8_RX_DMA_IRQn      DMAx_STREAMy_IRQn(RTE_UART8_RX_DMA_NUMBER, RTE_UART8_RX_DMA_STREAM)
    #define MX_UART8_RX_DMA_Channel   DMA_CHANNEL_x(RTE_UART8_RX_DMA_CHANNEL)
    #define MX_UART8_RX_DMA_Priority  DMA_PRIORITY(RTE_UART8_RX_DMA_PRIORITY)

    #define UART8_RX_DMA_Handler      DMAx_STREAMy_IRQ(RTE_UART8_RX_DMA_NUMBER, RTE_UART8_RX_DMA_STREAM)
  #endif
  #if (RTE_UART8_TX_DMA == 1)
    #define MX_UART8_TX_DMA_Instance  DMAx_STREAMy(RTE_UART8_TX_DMA_NUMBER, RTE_UART8_TX_DMA_STREAM)
    #define MX_UART8_TX_DMA_IRQn      DMAx_STREAMy_IRQn(RTE_UART8_TX_DMA_NUMBER, RTE_UART8_TX_DMA_STREAM)
    #define MX_UART8_TX_DMA_Channel   DMA_CHANNEL_x(RTE_UART8_TX_DMA_CHANNEL)
    #define MX_UART8_TX_DMA_Priority  DMA_PRIORITY(RTE_UART8_TX_DMA_PRIORITY)

    #define UART8_TX_DMA_Handler      DMAx_STREAMy_IRQ(RTE_UART8_TX_DMA_NUMBER, RTE_UART8_TX_DMA_STREAM)
  #endif

  #ifndef UART8_GPIO_SPEED_FREQ
    #define UART8_GPIO_SPEED_FREQ     GPIO_SPEED_FREQ_LOW
  #endif

  #if (RTE_UART8_TX == 1)
    #define MX_UART8_TX_Pin           1
    #define MX_UART8_TX_GPIOx         RTE_UART8_TX_PORT
    #define MX_UART8_TX_GPIO_Pin     (1U << RTE_UART8_TX_BIT)
    #define MX_UART8_TX_GPIO_PuPd     GPIO_NOPULL
    #define MX_UART8_TX_GPIO_Speed    UART8_GPIO_SPEED_FREQ
    #define MX_UART8_TX_GPIO_AF       GPIO_AF8_UART8
  #endif

  #if (RTE_UART8_RX == 1)
    #define MX_UART8_RX_Pin           1
    #define MX_UART8_RX_GPIOx         RTE_UART8_RX_PORT
    #define MX_UART8_RX_GPIO_Pin     (1U << RTE_UART8_RX_BIT)
    #define MX_UART8_RX_GPIO_PuPd     GPIO_NOPULL
    #define MX_UART8_RX_GPIO_Speed    UART8_GPIO_SPEED_FREQ
    #define MX_UART8_RX_GPIO_AF       GPIO_AF8_UART8
  #endif

  #if (RTE_UART8_RTS == 1)
    #define MX_UART8_RTS_Pin          1
    #define MX_UART8_RTS_GPIOx        RTE_UART8_RTS_PORT
    #define MX_UART8_RTS_GPIO_Pin    (1U << RTE_UART8_RTS_BIT)
    #define MX_UART8_RTS_GPIO_PuPd    GPIO_NOPULL
    #define MX_UART8_RTS_GPIO_Speed   UART8_GPIO_SPEED_FREQ
    #define MX_UART8_RTS_GPIO_AF      GPIO_AF8_UART8
  #endif

  #if (RTE_UART8_CTS == 1)
    #define MX_UART8_CTS_Pin          1
    #define MX_UART8_CTS_GPIOx        RTE_UART8_CTS_PORT
    #define MX_UART8_CTS_GPIO_Pin    (1U << RTE_UART8_CTS_BIT)
    #define MX_UART8_CTS_GPIO_PuPd    GPIO_NOPULL
    #define MX_UART8_CTS_GPIO_Speed   UART8_GPIO_SPEED_FREQ
    #define MX_UART8_CTS_GPIO_AF      GPIO_AF8_UART8
  #endif
#endif

#endif /* RTE_DEVICE_FRAMEWORK_CLASSIC */

#if defined(RTE_DEVICE_FRAMEWORK_CUBE_MX)
  #if ((defined(RTE_Drivers_USART1) || \
        defined(RTE_Drivers_USART2) || \
        defined(RTE_Drivers_USART3) || \
        defined(RTE_Drivers_UART4 ) || \
        defined(RTE_Drivers_UART5 ) || \
        defined(RTE_Drivers_USART6) || \
        defined(RTE_Drivers_UART7 ) || \
        defined(RTE_Drivers_UART8 ))   \
        && (!defined (MX_USART1))      \
        && (!defined (MX_USART2))      \
        && (!defined (MX_USART3))      \
        && (!defined (MX_UART4 ))      \
        && (!defined (MX_UART5 ))      \
        && (!defined (MX_USART6))      \
        && (!defined (MX_UART7 ))      \
        && (!defined (MX_UART8 )))
    #error "USART not configured in STM32CubeMX!"
  #endif

// MX macros
#ifdef MX_USART1

#if (MX_USART1_VM == VM_ASYNC)
#ifndef USART_ASYNC
#define USART_ASYNC                      (1UL)
#endif
#elif (MX_USART1_VM == VM_SYNC)
#ifndef USART_SYNC
#define USART_SYNC                       (1UL)
#endif
#elif (MX_USART1_VM == VM_IRDA)
#ifndef USART_IRDA
#define USART_IRDA                       (1UL)
#endif
#elif (MX_USART1_VM == VM_SMARTCARD)
#ifndef USART_SMARTCARD
#define USART_SMARTCARD                  (1UL)
#endif
#endif
#endif


#ifdef MX_USART2

#if (MX_USART2_VM == VM_ASYNC)
#ifndef USART_ASYNC
#define USART_ASYNC                      (1UL)
#endif
#elif (MX_USART2_VM == VM_SYNC)
#ifndef USART_SYNC
#define USART_SYNC                       (1UL)
#endif
#elif (MX_USART2_VM == VM_IRDA)
#ifndef USART_IRDA
#define USART_IRDA                       (1UL)
#endif
#elif (MX_USART2_VM == VM_SMARTCARD)
#ifndef USART_SMARTCARD
#define USART_SMARTCARD                  (1UL)
#endif
#endif
#endif


#ifdef MX_USART3

#if (MX_USART3_VM == VM_ASYNC)
#ifndef USART_ASYNC
#define USART_ASYNC                      (1UL)
#endif
#elif (MX_USART3_VM == VM_SYNC)
#ifndef USART_SYNC
#define USART_SYNC                       (1UL)
#endif
#elif (MX_USART3_VM == VM_IRDA)
#ifndef USART_IRDA
#define USART_IRDA                       (1UL)
#endif
#elif (MX_USART3_VM == VM_SMARTCARD)
#ifndef USART_SMARTCARD
#define USART_SMARTCARD                  (1UL)
#endif
#endif
#endif


#ifdef MX_UART4

#if (MX_UART4_VM == VM_ASYNC)
#ifndef USART_ASYNC
#define USART_ASYNC                      (1UL)
#endif
#elif (MX_UART4_VM == VM_IRDA)
#ifndef USART_IRDA
#define USART_IRDA                       (1UL)
#endif
#endif
#endif


#ifdef MX_UART5

#if (MX_UART5_VM == VM_ASYNC)
#ifndef USART_ASYNC
#define USART_ASYNC                      (1UL)
#endif
#elif (MX_UART5_VM == VM_IRDA)
#ifndef USART_IRDA
#define USART_IRDA                       (1UL)
#endif
#endif
#endif


#ifdef MX_USART6

#if (MX_USART6_VM == VM_ASYNC)
#ifndef USART_ASYNC
#define USART_ASYNC                      (1UL)
#endif
#elif (MX_USART6_VM == VM_SYNC)
#ifndef USART_SYNC
#define USART_SYNC                       (1UL)
#endif
#elif (MX_USART6_VM == VM_IRDA)
#ifndef USART_IRDA
#define USART_IRDA                       (1UL)
#endif
#elif (MX_USART6_VM == VM_SMARTCARD)
#ifndef USART_SMARTCARD
#define USART_SMARTCARD                  (1UL)
#endif
#endif
#endif


#ifdef MX_UART7

#if (MX_UART7_VM == VM_ASYNC)
#ifndef USART_ASYNC
#define USART_ASYNC                      (1UL)
#endif
#elif (MX_UART7_VM == VM_IRDA)
#ifndef USART_IRDA
#define USART_IRDA                       (1UL)
#endif
#endif
#endif


#ifdef MX_UART8

#if (MX_UART8_VM == VM_ASYNC)
#ifndef USART_ASYNC
#define USART_ASYNC                      (1UL)
#endif
#elif (MX_UART8_VM == VM_IRDA)
#ifndef USART_IRDA
#define USART_IRDA                       (1UL)
#endif
#endif
#endif

#endif  /* RTE_DEVICE_FRAMEWORK_CUBE_MX */

#ifdef MX_USART1
#if (defined(MX_USART1_RX_DMA_Instance) || defined(MX_USART1_TX_DMA_Instance))
#ifndef MX_USART1_RX_DMA_Instance
  #error "USART1 using DMA requires Rx and Tx DMA channel enabled in RTE_Device.h or MX_Device.h!"
#endif
#ifndef MX_USART1_TX_DMA_Instance
  #error "USART1 using DMA requires Rx and Tx DMA channel enabled in RTE_Device.h or MX_Device.h!"
#endif
#endif
#endif

#ifdef MX_USART2
#if (defined(MX_USART2_RX_DMA_Instance) || defined(MX_USART2_TX_DMA_Instance))
#ifndef MX_USART2_RX_DMA_Instance
  #error "USART2 using DMA requires Rx and Tx DMA channel enabled in RTE_Device.h or MX_Device.h!"
#endif
#ifndef MX_USART2_TX_DMA_Instance
  #error "USART2 using DMA requires Rx and Tx DMA channel enabled in RTE_Device.h or MX_Device.h!"
#endif
#endif
#endif

#ifdef MX_USART3
#if (defined(MX_USART3_RX_DMA_Instance) || defined(MX_USART3_TX_DMA_Instance))
#ifndef MX_USART3_RX_DMA_Instance
  #error "USART3 using DMA requires Rx and Tx DMA channel enabled in RTE_Device.h or MX_Device.h!"
#endif
#ifndef MX_USART3_TX_DMA_Instance
  #error "USART3 using DMA requires Rx and Tx DMA channel enabled in RTE_Device.h or MX_Device.h!"
#endif
#endif
#endif

#ifdef MX_UART4
#if (defined(MX_UART4_RX_DMA_Instance) || defined(MX_UART4_TX_DMA_Instance))
#ifndef MX_UART4_RX_DMA_Instance
  #error "UART4 using DMA requires Rx and Tx DMA channel enabled in RTE_Device.h or MX_Device.h!"
#endif
#ifndef MX_UART4_TX_DMA_Instance
  #error "UART4 using DMA requires Rx and Tx DMA channel enabled in RTE_Device.h or MX_Device.h!"
#endif
#endif
#endif

#ifdef MX_UART5
#if (defined(MX_UART5_RX_DMA_Instance) || defined(MX_UART5_TX_DMA_Instance))
#ifndef MX_UART5_RX_DMA_Instance
  #error "UART5 using DMA requires Rx and Tx DMA channel enabled in RTE_Device.h or MX_Device.h!"
#endif
#ifndef MX_UART5_TX_DMA_Instance
  #error "UART5 using DMA requires Rx and Tx DMA channel enabled in RTE_Device.h or MX_Device.h!"
#endif
#endif
#endif

#ifdef MX_USART6
#if (defined(MX_USART6_RX_DMA_Instance) || defined(MX_USART6_TX_DMA_Instance))
#ifndef MX_USART6_RX_DMA_Instance
  #error "USART6 using DMA requires Rx and Tx DMA channel enabled in RTE_Device.h or MX_Device.h!"
#endif
#ifndef MX_USART6_TX_DMA_Instance
  #error "USART6 using DMA requires Rx and Tx DMA channel enabled in RTE_Device.h or MX_Device.h!"
#endif
#endif
#endif

#ifdef MX_UART7
#if (defined(MX_UART7_RX_DMA_Instance) || defined(MX_UART7_TX_DMA_Instance))
#ifndef MX_UART7_RX_DMA_Instance
  #error "UART7 using DMA requires Rx and Tx DMA channel enabled in RTE_Device.h or MX_Device.h!"
#endif
#ifndef MX_UART7_TX_DMA_Instance
  #error "UART7 using DMA requires Rx and Tx DMA channel enabled in RTE_Device.h or MX_Device.h!"
#endif
#endif
#endif

#ifdef MX_UART8
#if (defined(MX_UART8_RX_DMA_Instance) || defined(MX_UART8_TX_DMA_Instance))
#ifndef MX_UART8_RX_DMA_Instance
  #error "UART using DMA requires Rx and Tx DMA channel enabled in RTE_Device.h or MX_Device.h!"
#endif
#ifndef MX_UART8_TX_DMA_Instance
  #error "UART8 using DMA requires Rx and Tx DMA channel enabled in RTE_Device.h or MX_Device.h!"
#endif
#endif
#endif


#if (defined(MX_USART1_RX_DMA_Instance) || \
     defined(MX_USART2_RX_DMA_Instance) || \
     defined(MX_USART3_RX_DMA_Instance) || \
     defined(MX_UART4_RX_DMA_Instance ) || \
     defined(MX_UART5_RX_DMA_Instance ) || \
     defined(MX_USART6_RX_DMA_Instance) || \
     defined(MX_UART7_RX_DMA_Instance ) || \
     defined(MX_UART8_RX_DMA_Instance))
#define __USART_DMA_RX
#endif
#if (defined(MX_USART1_TX_DMA_Instance) || \
     defined(MX_USART2_TX_DMA_Instance) || \
     defined(MX_USART3_TX_DMA_Instance) || \
     defined(MX_UART4_TX_DMA_Instance ) || \
     defined(MX_UART5_TX_DMA_Instance ) || \
     defined(MX_USART6_TX_DMA_Instance) || \
     defined(MX_UART7_TX_DMA_Instance ) || \
     defined(MX_UART8_TX_DMA_Instance))
#define __USART_DMA_TX
#endif
#if (defined(__USART_DMA_RX) && defined(__USART_DMA_TX))
#define __USART_DMA
#endif

// USART BRR macro
#define USART_BAUDRATE_DIVIDER(__PCLK__, __BAUD__)  (((__PCLK__) + ((__BAUD__)/2U)) / (__BAUD__))

// USART flags
#define USART_FLAG_INITIALIZED      ((uint8_t)(1U))
#define USART_FLAG_POWERED          ((uint8_t)(1U << 1))
#define USART_FLAG_CONFIGURED       ((uint8_t)(1U << 2))
#define USART_FLAG_TX_ENABLED       ((uint8_t)(1U << 3))
#define USART_FLAG_RX_ENABLED       ((uint8_t)(1U << 4))

// USART synchronous xfer modes
#define USART_SYNC_MODE_TX           ( 1UL )
#define USART_SYNC_MODE_RX           ( 2UL )
#define USART_SYNC_MODE_TX_RX        (USART_SYNC_MODE_TX | \
                                      USART_SYNC_MODE_RX)

// DMA Callback functions
typedef void (*DMA_Callback_t) (DMA_HandleTypeDef *hdma);

// USART DMA
typedef const struct {
  DMA_HandleTypeDef    *hdma;           // DMA handle
  DMA_Callback_t        cb_complete;    // DMA complete callback
#ifdef RTE_DEVICE_FRAMEWORK_CLASSIC
  DMA_Stream_TypeDef   *stream;         // Stream register interface
  uint32_t              channel;        // DMA channel
  uint32_t              priority;       // DMA channel priority
  IRQn_Type             irq_num;        // Stream IRQ number
  uint8_t               reserved[3];    // Reserved
#endif
} USART_DMA;

// USART pin
typedef const struct {
  GPIO_TypeDef         *port;           // Port
  uint16_t              pin;            // Pin
  uint16_t              af;             // Alternate function
  uint16_t              pupd;           // Pull up/down
  uint16_t              speed;          // Speed
} USART_PIN;

// USART Input/Output Configuration
typedef const struct {
  USART_PIN            *tx;             // TX  Pin identifier
  USART_PIN            *rx;             // RX  Pin identifier
  USART_PIN            *ck;             // CLK Pin identifier
  USART_PIN            *rts;            // RTS Pin identifier
  USART_PIN            *cts;            // CTS Pin identifier
} USART_IO;

// USART Transfer Information (Run-Time)
// USART_TRANSFER_INFO must be aligned to 32Byte due to DCache handling on def_val
typedef struct __ALIGNED(32) {
        uint32_t        rx_num;         // Total number of receive data
        uint32_t        tx_num;         // Total number of transmit data
        uint8_t        *rx_buf;         // Pointer to in data buffer
  const uint8_t        *tx_buf;         // Pointer to out data buffer
        uint32_t        rx_cnt;         // Number of data received
        uint32_t        tx_cnt;         // Number of data sent
        uint16_t        dump_val;       // Variable for dumping DMA data
        uint16_t        def_val;        // Default transfer value
        uint32_t        sync_mode;      // Synchronous mode flag
        uint8_t         break_flag;     // Transmit break flag
        uint8_t         send_active;    // Send active flag
        uint16_t        reserved;       // Reserved
} USART_TRANSFER_INFO;

typedef struct {
  uint8_t tx_busy;                      // Transmitter busy flag
  uint8_t rx_busy;                      // Receiver busy flag
  uint8_t tx_underflow;                 // Transmit data underflow detected (cleared on start of next send operation)
  uint8_t rx_overflow;                  // Receive data overflow detected (cleared on start of next receive operation)
  uint8_t rx_break;                     // Break detected on receive (cleared on start of next receive operation)
  uint8_t rx_framing_error;             // Framing error detected on receive (cleared on start of next receive operation)
  uint8_t rx_parity_error;              // Parity error detected on receive (cleared on start of next receive operation)
} USART_STATUS;

// USART Information (Run-time)
typedef struct {
  ARM_USART_SignalEvent_t cb_event;            // Event Callback
  USART_STATUS            status;              // Status flags
  uint8_t                 flags;               // Current USART flags
  uint32_t                mode;                // Current USART mode
  uint32_t                flow_control;        // Flow control
} USART_INFO;

// USART Resources definition
typedef const struct {
#ifdef RTE_DEVICE_FRAMEWORK_CUBE_MX
  void                   *h;                   // USART Handle
  uint32_t                vmode;               // Virtual mode
#endif
  ARM_USART_CAPABILITIES  capabilities;        // Capabilities
  USART_TypeDef          *reg;                 // USART peripheral pointer
  USART_IO                io;                  // USART Input/Output pins
  IRQn_Type               irq_num;             // USART IRQ Number
  uint8_t                 reserved[3];         // Reserved
  USART_DMA               *dma_tx;             // Transmit stream register interface
  USART_DMA               *dma_rx;             // Receive stream register interface
  uint32_t                 rx_timeout_val;     // Receive timeout value
  USART_INFO              *info;               // Run-Time Information
  USART_TRANSFER_INFO     *xfer;               // USART transfer information
} USART_RESOURCES;

// Global functions and variables exported by driver .c module
#ifdef MX_USART1
extern void USART1_IRQHandler     (void);
extern void USART1_TX_DMA_Handler (void);
extern void USART1_RX_DMA_Handler (void);

extern ARM_DRIVER_USART Driver_USART1;
#endif

#ifdef MX_USART2
extern void USART2_IRQHandler     (void);
extern void USART2_TX_DMA_Handler (void);
extern void USART2_RX_DMA_Handler (void);

extern ARM_DRIVER_USART Driver_USART2;
#endif

#ifdef MX_USART3
extern void USART3_IRQHandler     (void);
extern void USART3_TX_DMA_Handler (void);
extern void USART3_RX_DMA_Handler (void);

extern ARM_DRIVER_USART Driver_USART3;
#endif

#ifdef MX_UART4
extern void UART4_IRQHandler      (void);
extern void UART4_TX_DMA_Handler  (void);
extern void UART4_RX_DMA_Handler  (void);

extern ARM_DRIVER_USART Driver_USART4;
#endif

#ifdef MX_UART5
extern void UART5_IRQHandler      (void);
extern void UART5_TX_DMA_Handler  (void);
extern void UART5_RX_DMA_Handler  (void);

extern ARM_DRIVER_USART Driver_USART5;
#endif

#ifdef MX_USART6
extern void USART6_IRQHandler     (void);
extern void USART6_TX_DMA_Handler (void);
extern void USART6_RX_DMA_Handler (void);

extern ARM_DRIVER_USART Driver_USART6;
#endif

#ifdef MX_UART7
extern void UART7_IRQHandler      (void);
extern void UART7_TX_DMA_Handler  (void);
extern void UART7_RX_DMA_Handler  (void);

extern ARM_DRIVER_USART Driver_USART7;
#endif

#ifdef MX_UART8
extern void UART8_IRQHandler      (void);
extern void UART8_TX_DMA_Handler  (void);
extern void UART8_RX_DMA_Handler  (void);

extern ARM_DRIVER_USART Driver_USART8;
#endif

#endif /* __USART_STM32F7XX_H */
