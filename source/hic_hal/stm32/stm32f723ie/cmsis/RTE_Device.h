/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2019 Arm Limited (or its affiliates). All 
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
 * $Date:        17. June 2019
 * $Revision:    V1.5.1
 *
 * Project:      RTE Device Configuration for ST STM32F7xx
 * -------------------------------------------------------------------------- */

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef __RTE_DEVICE_H
#define __RTE_DEVICE_H


#define GPIO_PORT0      GPIOA
#define GPIO_PORT1      GPIOB
#define GPIO_PORT2      GPIOC
#define GPIO_PORT3      GPIOD
#define GPIO_PORT4      GPIOE
#define GPIO_PORT5      GPIOF
#define GPIO_PORT6      GPIOG
#define GPIO_PORT7      GPIOH
#define GPIO_PORT8      GPIOI
#define GPIO_PORT9      GPIOJ
#define GPIO_PORT10     GPIOK

#define GPIO_PORT(num)  GPIO_PORT##num


// <e> USART1 (Universal synchronous asynchronous receiver transmitter) [Driver_USART1]
// <i> Configuration settings for Driver_USART1 in component ::CMSIS Driver:USART
#define RTE_USART1                      0

//   <o> USART1_TX Pin <0=>Not Used <1=>PA9 <2=>PB6 <3=>PB14
#define RTE_USART1_TX_ID                0
#if    (RTE_USART1_TX_ID == 0)
#define RTE_USART1_TX                   0
#elif  (RTE_USART1_TX_ID == 1)
#define RTE_USART1_TX                   1
#define RTE_USART1_TX_PORT              GPIOA
#define RTE_USART1_TX_BIT               9
#elif  (RTE_USART1_TX_ID == 2)
#define RTE_USART1_TX                   1
#define RTE_USART1_TX_PORT              GPIOB
#define RTE_USART1_TX_BIT               6
#elif  (RTE_USART1_TX_ID == 3)
#define RTE_USART1_TX                   1
#define RTE_USART1_TX_PORT              GPIOB
#define RTE_USART1_TX_BIT               14
#else
#error "Invalid USART1_TX Pin Configuration!"
#endif

//   <o> USART1_RX Pin <0=>Not Used <1=>PA10 <2=>PB7 <3=>PB15
#define RTE_USART1_RX_ID                0
#if    (RTE_USART1_RX_ID == 0)
#define RTE_USART1_RX                   0
#elif  (RTE_USART1_RX_ID == 1)
#define RTE_USART1_RX                   1
#define RTE_USART1_RX_PORT              GPIOA
#define RTE_USART1_RX_BIT               10
#elif  (RTE_USART1_RX_ID == 2)
#define RTE_USART1_RX                   1
#define RTE_USART1_RX_PORT              GPIOB
#define RTE_USART1_RX_BIT               7
#elif  (RTE_USART1_RX_ID == 3)
#define RTE_USART1_RX                   1
#define RTE_USART1_RX_PORT              GPIOB
#define RTE_USART1_RX_BIT               15
#else
#error "Invalid USART1_RX Pin Configuration!"
#endif

//   <o> USART1_CK Pin <0=>Not Used <1=>PA8
#define RTE_USART1_CK_ID                0
#if    (RTE_USART1_CK_ID == 0)
#define RTE_USART1_CK                   0
#elif  (RTE_USART1_CK_ID == 1)
#define RTE_USART1_CK                   1
#define RTE_USART1_CK_PORT              GPIOA
#define RTE_USART1_CK_BIT               8
#else
#error "Invalid USART1_CK Pin Configuration!"
#endif

//   <o> USART1_CTS Pin <0=>Not Used <1=>PA11
#define RTE_USART1_CTS_ID               0
#if    (RTE_USART1_CTS_ID == 0)
#define RTE_USART1_CTS                  0
#elif  (RTE_USART1_CTS_ID == 1)
#define RTE_USART1_CTS                  1
#define RTE_USART1_CTS_PORT             GPIOA
#define RTE_USART1_CTS_BIT              11
#else
#error "Invalid USART1_CTS Pin Configuration!"
#endif

//   <o> USART1_RTS Pin <0=>Not Used <1=>PA12
#define RTE_USART1_RTS_ID               0
#if    (RTE_USART1_RTS_ID == 0)
#define RTE_USART1_RTS                  0
#elif  (RTE_USART1_RTS_ID == 1)
#define RTE_USART1_RTS                  1
#define RTE_USART1_RTS_PORT             GPIOA
#define RTE_USART1_RTS_BIT              12
#else
#error "Invalid USART1_RTS Pin Configuration!"
#endif

//   <e> DMA Rx
//     <o1> Number <2=>2
//     <i>  Selects DMA Number (only DMA2 can be used)
//     <o2> Stream <2=>2 <5=>5
//     <i>  Selects DMA Stream (only Stream 2 or 5 can be used)
//     <o3> Channel <4=>4
//     <i>  Selects DMA Channel (only Channel 4 can be used)
//     <o4> Priority <0=>Low <1=>Medium <2=>High <3=>Very High
//     <i>  Selects DMA Priority
//   </e>
#define RTE_USART1_RX_DMA               0
#define RTE_USART1_RX_DMA_NUMBER        2
#define RTE_USART1_RX_DMA_STREAM        2
#define RTE_USART1_RX_DMA_CHANNEL       4
#define RTE_USART1_RX_DMA_PRIORITY      0

//   <e> DMA Tx
//     <o1> Number <2=>2
//     <i>  Selects DMA Number (only DMA2 can be used)
//     <o2> Stream <7=>7
//     <i>  Selects DMA Stream (only Stream 7 can be used)
//     <o3> Channel <4=>4
//     <i>  Selects DMA Channel (only Channel 4 can be used)
//     <o4> Priority <0=>Low <1=>Medium <2=>High <3=>Very High
//     <i>  Selects DMA Priority
//   </e>
#define RTE_USART1_TX_DMA               0
#define RTE_USART1_TX_DMA_NUMBER        2
#define RTE_USART1_TX_DMA_STREAM        7
#define RTE_USART1_TX_DMA_CHANNEL       4
#define RTE_USART1_TX_DMA_PRIORITY      0

// </e>


// <e> USART2 (Universal synchronous asynchronous receiver transmitter) [Driver_USART2]
// <i> Configuration settings for Driver_USART2 in component ::CMSIS Driver:USART
#define RTE_USART2                      0

//   <o> USART2_TX Pin <0=>Not Used <1=>PA2 <2=>PD5
#define RTE_USART2_TX_ID                0
#if    (RTE_USART2_TX_ID == 0)
#define RTE_USART2_TX                   0
#elif  (RTE_USART2_TX_ID == 1)
#define RTE_USART2_TX                   1
#define RTE_USART2_TX_PORT              GPIOA
#define RTE_USART2_TX_BIT               2
#elif  (RTE_USART2_TX_ID == 2)
#define RTE_USART2_TX                   1
#define RTE_USART2_TX_PORT              GPIOD
#define RTE_USART2_TX_BIT               5
#else
#error "Invalid USART2_TX Pin Configuration!"
#endif

//   <o> USART2_RX Pin <0=>Not Used <1=>PA3 <2=>PD6
#define RTE_USART2_RX_ID                0
#if    (RTE_USART2_RX_ID == 0)
#define RTE_USART2_RX                   0
#elif  (RTE_USART2_RX_ID == 1)
#define RTE_USART2_RX                   1
#define RTE_USART2_RX_PORT              GPIOA
#define RTE_USART2_RX_BIT               3
#elif  (RTE_USART2_RX_ID == 2)
#define RTE_USART2_RX                   1
#define RTE_USART2_RX_PORT              GPIOD
#define RTE_USART2_RX_BIT               6
#else
#error "Invalid USART2_RX Pin Configuration!"
#endif

//   <o> USART2_CK Pin <0=>Not Used <1=>PA4 <2=>PD7
#define RTE_USART2_CK_ID                0
#if    (RTE_USART2_CK_ID == 0)  
#define RTE_USART2_CK                   0
#elif  (RTE_USART2_CK_ID == 1)
#define RTE_USART2_CK                   1
#define RTE_USART2_CK_PORT              GPIOA
#define RTE_USART2_CK_BIT               4
#elif  (RTE_USART2_CK_ID == 2)
#define RTE_USART2_CK                   1
#define RTE_USART2_CK_PORT              GPIOD
#define RTE_USART2_CK_BIT               7
#else
#error "Invalid USART2_CK Pin Configuration!"
#endif

//   <o> USART2_CTS Pin <0=>Not Used <1=>PA0 <2=>PD3
#define RTE_USART2_CTS_ID               0
#if    (RTE_USART2_CTS_ID == 0)
#define RTE_USART2_CTS                  0
#elif  (RTE_USART2_CTS_ID == 1)
#define RTE_USART2_CTS                  1
#define RTE_USART2_CTS_PORT             GPIOA
#define RTE_USART2_CTS_BIT              0
#elif  (RTE_USART2_CTS_ID == 2)
#define RTE_USART2_CTS                  1
#define RTE_USART2_CTS_PORT             GPIOD
#define RTE_USART2_CTS_BIT              3
#else
#error "Invalid USART2_CTS Pin Configuration!"
#endif

//   <o> USART2_RTS Pin <0=>Not Used <1=>PA1 <2=>PD4
#define RTE_USART2_RTS_ID               0
#if    (RTE_USART2_RTS_ID == 0)
#define RTE_USART2_RTS                  0
#elif  (RTE_USART2_RTS_ID == 1)
#define RTE_USART2_RTS                  1
#define RTE_USART2_RTS_PORT             GPIOA
#define RTE_USART2_RTS_BIT              1
#elif  (RTE_USART2_RTS_ID == 2)
#define RTE_USART2_RTS                  1
#define RTE_USART2_RTS_PORT             GPIOD
#define RTE_USART2_RTS_BIT              4
#else
#error "Invalid USART2_RTS Pin Configuration!"
#endif

//   <e> DMA Rx
//     <o1> Number <1=>1
//     <i>  Selects DMA Number (only DMA1 can be used)
//     <o2> Stream <5=>5
//     <i>  Selects DMA Stream (only Stream 5 can be used)
//     <o3> Channel <4=>4
//     <i>  Selects DMA Channel (only Channel 4 can be used)
//     <o4> Priority <0=>Low <1=>Medium <2=>High <3=>Very High
//     <i>  Selects DMA Priority
//   </e>
#define RTE_USART2_RX_DMA               0
#define RTE_USART2_RX_DMA_NUMBER        1
#define RTE_USART2_RX_DMA_STREAM        5
#define RTE_USART2_RX_DMA_CHANNEL       4
#define RTE_USART2_RX_DMA_PRIORITY      0

//   <e> DMA Tx
//     <o1> Number <1=>1
//     <i>  Selects DMA Number (only DMA1 can be used)
//     <o2> Stream <6=>6
//     <i>  Selects DMA Stream (only Stream 6 can be used)
//     <o3> Channel <4=>4
//     <i>  Selects DMA Channel (only Channel 4 can be used)
//     <o4> Priority <0=>Low <1=>Medium <2=>High <3=>Very High
//     <i>  Selects DMA Priority
//   </e>
#define RTE_USART2_TX_DMA               0
#define RTE_USART2_TX_DMA_NUMBER        1
#define RTE_USART2_TX_DMA_STREAM        6
#define RTE_USART2_TX_DMA_CHANNEL       4
#define RTE_USART2_TX_DMA_PRIORITY      0

// </e>


// <e> USART3 (Universal synchronous asynchronous receiver transmitter) [Driver_USART3]
// <i> Configuration settings for Driver_USART3 in component ::CMSIS Driver:USART
#define RTE_USART3                      0

//   <o> USART3_TX Pin <0=>Not Used <1=>PB10 <2=>PC10 <3=>PD8
#define RTE_USART3_TX_ID                0
#if    (RTE_USART3_TX_ID == 0)
#define RTE_USART3_TX                   0
#elif  (RTE_USART3_TX_ID == 1)
#define RTE_USART3_TX                   1
#define RTE_USART3_TX_PORT              GPIOB
#define RTE_USART3_TX_BIT               10
#elif  (RTE_USART3_TX_ID == 2)
#define RTE_USART3_TX                   1
#define RTE_USART3_TX_PORT              GPIOC
#define RTE_USART3_TX_BIT               10
#elif  (RTE_USART3_TX_ID == 3)
#define RTE_USART3_TX                   1
#define RTE_USART3_TX_PORT              GPIOD
#define RTE_USART3_TX_BIT               8
#else
#error "Invalid USART3_TX Pin Configuration!"
#endif

//   <o> USART3_RX Pin <0=>Not Used <1=>PB11 <2=>PC11 <3=>PD9
#define RTE_USART3_RX_ID                0
#if    (RTE_USART3_RX_ID == 0)
#define RTE_USART3_RX                   0
#elif  (RTE_USART3_RX_ID == 1)
#define RTE_USART3_RX                   1
#define RTE_USART3_RX_PORT              GPIOB
#define RTE_USART3_RX_BIT               11
#elif  (RTE_USART3_RX_ID == 2)
#define RTE_USART3_RX                   1
#define RTE_USART3_RX_PORT              GPIOC
#define RTE_USART3_RX_BIT               11
#elif  (RTE_USART3_RX_ID == 3)
#define RTE_USART3_RX                   1
#define RTE_USART3_RX_PORT              GPIOD
#define RTE_USART3_RX_BIT               9
#else
#error "Invalid USART3_RX Pin Configuration!"
#endif

//   <o> USART3_CK Pin <0=>Not Used <1=>PB12 <2=>PC12 <3=>PD10
#define RTE_USART3_CK_ID                0
#if    (RTE_USART3_CK_ID == 0)
#define RTE_USART3_CK                   0
#elif  (RTE_USART3_CK_ID == 1)
#define RTE_USART3_CK                   1
#define RTE_USART3_CK_PORT              GPIOB
#define RTE_USART3_CK_BIT               12
#elif  (RTE_USART3_CK_ID == 2)
#define RTE_USART3_CK                   1
#define RTE_USART3_CK_PORT              GPIOC
#define RTE_USART3_CK_BIT               12
#elif  (RTE_USART3_CK_ID == 3)
#define RTE_USART3_CK                   1
#define RTE_USART3_CK_PORT              GPIOD
#define RTE_USART3_CK_BIT               10
#else
#error "Invalid USART3_CK Pin Configuration!"
#endif

//   <o> USART3_CTS Pin <0=>Not Used <1=>PB13 <2=>PD11
#define RTE_USART3_CTS_ID               0
#if    (RTE_USART3_CTS_ID == 0)
#define RTE_USART3_CTS                  0
#elif  (RTE_USART3_CTS_ID == 1)
#define RTE_USART3_CTS                  1
#define RTE_USART3_CTS_PORT             GPIOB
#define RTE_USART3_CTS_BIT              13
#elif  (RTE_USART3_CTS_ID == 2)
#define RTE_USART3_CTS                  1
#define RTE_USART3_CTS_PORT             GPIOD
#define RTE_USART3_CTS_BIT              11
#else
#error "Invalid USART3_CTS Pin Configuration!"
#endif

//   <o> USART3_RTS Pin <0=>Not Used <1=>PB14 <2=>PD12
#define RTE_USART3_RTS_ID               0
#if    (RTE_USART3_RTS_ID == 0)
#define RTE_USART3_RTS                  0
#elif  (RTE_USART3_RTS_ID == 1)
#define RTE_USART3_RTS                  1
#define RTE_USART3_RTS_PORT             GPIOB
#define RTE_USART3_RTS_BIT              14
#elif  (RTE_USART3_RTS_ID == 2)
#define RTE_USART3_RTS                  1
#define RTE_USART3_RTS_PORT             GPIOD
#define RTE_USART3_RTS_BIT              12
#else
#error "Invalid USART3_RTS Pin Configuration!"
#endif

//   <e> DMA Rx
//     <o1> Number <1=>1
//     <i>  Selects DMA Number (only DMA1 can be used)
//     <o2> Stream <1=>1
//     <i>  Selects DMA Stream (only Stream 1 can be used)
//     <o3> Channel <4=>4
//     <i>  Selects DMA Channel (only Channel 4 can be used)
//     <o4> Priority <0=>Low <1=>Medium <2=>High <3=>Very High
//     <i>  Selects DMA Priority
//   </e>
#define RTE_USART3_RX_DMA               0
#define RTE_USART3_RX_DMA_NUMBER        1
#define RTE_USART3_RX_DMA_STREAM        1
#define RTE_USART3_RX_DMA_CHANNEL       4
#define RTE_USART3_RX_DMA_PRIORITY      0

//   <e> DMA Tx
//     <o1> Number <1=>1
//     <i>  Selects DMA Number (only DMA1 can be used)
//     <o2> Stream <3=>3 <4=>4 
//     <i>  Selects DMA Stream (only Stream 3 or 4 can be used)
//     <o3> Channel <4=>4 <7=>7
//     <i>  Selects DMA Channel (only Channel 4 or 7 can be used)
//     <o4> Priority <0=>Low <1=>Medium <2=>High <3=>Very High
//     <i>  Selects DMA Priority
//   </e>
#define RTE_USART3_TX_DMA               0
#define RTE_USART3_TX_DMA_NUMBER        1
#define RTE_USART3_TX_DMA_STREAM        3
#define RTE_USART3_TX_DMA_CHANNEL       4
#define RTE_USART3_TX_DMA_PRIORITY      0

// </e>


// <e> UART4 (Universal asynchronous receiver transmitter) [Driver_USART4]
// <i> Configuration settings for Driver_USART4 in component ::CMSIS Driver:USART
#define RTE_UART4                       0

//   <o> UART4_TX Pin <0=>Not Used <1=>PA0 <2=>PC10 <3=>PA12 <4=>PD1 <5=>PH13
#define RTE_UART4_TX_ID                 0
#if    (RTE_UART4_TX_ID == 0)
#define RTE_UART4_TX                    0
#elif  (RTE_UART4_TX_ID == 1)
#define RTE_UART4_TX                    1
#define RTE_UART4_TX_PORT               GPIOA
#define RTE_UART4_TX_BIT                0
#elif  (RTE_UART4_TX_ID == 2)
#define RTE_UART4_TX                    1
#define RTE_UART4_TX_PORT               GPIOC
#define RTE_UART4_TX_BIT                10
#elif  (RTE_UART4_TX_ID == 3)
#define RTE_UART4_TX                    1
#define RTE_UART4_TX_PORT               GPIOA
#define RTE_UART4_TX_BIT                12
#elif  (RTE_UART4_TX_ID == 4)
#define RTE_UART4_TX                    1
#define RTE_UART4_TX_PORT               GPIOD
#define RTE_UART4_TX_BIT                1
#elif  (RTE_UART4_TX_ID == 5)
#define RTE_UART4_TX                    1
#define RTE_UART4_TX_PORT               GPIOH
#define RTE_UART4_TX_BIT                13
#else
#error "Invalid UART4_TX Pin Configuration!"
#endif

//   <o> UART4_RX Pin <0=>Not Used <1=>PA1 <2=>PC11 <3=>PA11 <4=>PD0 <5=>PH14 <6=>PI9
#define RTE_UART4_RX_ID                 0
#if    (RTE_UART4_RX_ID == 0)
#define RTE_UART4_RX                    0
#elif  (RTE_UART4_RX_ID == 1)
#define RTE_UART4_RX                    1
#define RTE_UART4_RX_PORT               GPIOA
#define RTE_UART4_RX_BIT                1
#elif  (RTE_UART4_RX_ID == 2)
#define RTE_UART4_RX                    1
#define RTE_UART4_RX_PORT               GPIOC
#define RTE_UART4_RX_BIT                11
#elif  (RTE_UART4_RX_ID == 3)
#define RTE_UART4_RX                    1
#define RTE_UART4_RX_PORT               GPIOA
#define RTE_UART4_RX_BIT                11
#elif  (RTE_UART4_RX_ID == 4)
#define RTE_UART4_RX                    1
#define RTE_UART4_RX_PORT               GPIOD
#define RTE_UART4_RX_BIT                0
#elif  (RTE_UART4_RX_ID == 5)
#define RTE_UART4_RX                    1
#define RTE_UART4_RX_PORT               GPIOH
#define RTE_UART4_RX_BIT                14
#elif  (RTE_UART4_RX_ID == 6)
#define RTE_UART4_RX                    1
#define RTE_UART4_RX_PORT               GPIOI
#define RTE_UART4_RX_BIT                9
#else
#error "Invalid UART4_RX Pin Configuration!"
#endif

//   <o> UART4_CTS Pin <0=>Not Used <1=>PB0 <2=>PB15
#define RTE_UART4_CTS_ID                0
#if    (RTE_UART4_CTS_ID == 0)
#define RTE_UART4_CTS                   0
#elif  (RTE_UART4_CTS_ID == 1)
#define RTE_UART4_CTS                   1
#define RTE_UART4_CTS_PORT              GPIOB
#define RTE_UART4_CTS_BIT               0
#elif  (RTE_UART4_CTS_ID == 2)
#define RTE_UART4_CTS                   1
#define RTE_UART4_CTS_PORT              GPIOB
#define RTE_UART4_CTS_BIT               15
#else
#error "Invalid UART4_CTS Pin Configuration!"
#endif

//   <o> UART4_RTS Pin <0=>Not Used <1=>PA15 <2=>PB14
#define RTE_UART4_RTS_ID                0
#if    (RTE_UART4_RTS_ID == 0)
#define RTE_UART4_RTS                   0
#elif  (RTE_UART4_RTS_ID == 1)
#define RTE_UART4_RTS                   1
#define RTE_UART4_RTS_PORT              GPIOA
#define RTE_UART4_RTS_BIT               15
#elif  (RTE_UART4_RTS_ID == 2)
#define RTE_UART4_RTS                   1
#define RTE_UART4_RTS_PORT              GPIOB
#define RTE_UART4_RTS_BIT               14
#else
#error "Invalid UART4_RTS Pin Configuration!"
#endif

//   <e> DMA Rx
//     <o1> Number <1=>1
//     <i>  Selects DMA Number (only DMA1 can be used)
//     <o2> Stream <2=>2
//     <i>  Selects DMA Stream (only Stream 2 can be used)
//     <o3> Channel <4=>4
//     <i>  Selects DMA Channel (only Channel 4 can be used)
//     <o4> Priority <0=>Low <1=>Medium <2=>High <3=>Very High
//     <i>  Selects DMA Priority
//   </e>
#define RTE_UART4_RX_DMA                0
#define RTE_UART4_RX_DMA_NUMBER         1
#define RTE_UART4_RX_DMA_STREAM         2
#define RTE_UART4_RX_DMA_CHANNEL        4
#define RTE_UART4_RX_DMA_PRIORITY       0

//   <e> DMA Tx
//     <o1> Number <1=>1
//     <i>  Selects DMA Number (only DMA1 can be used)
//     <o2> Stream <4=>4
//     <i>  Selects DMA Stream (only Stream 4 can be used)
//     <o3> Channel <4=>4
//     <i>  Selects DMA Channel (only Channel 4 can be used)
//     <o4> Priority <0=>Low <1=>Medium <2=>High <3=>Very High
//     <i>  Selects DMA Priority
//   </e>
#define RTE_UART4_TX_DMA                0
#define RTE_UART4_TX_DMA_NUMBER         1
#define RTE_UART4_TX_DMA_STREAM         4
#define RTE_UART4_TX_DMA_CHANNEL        4
#define RTE_UART4_TX_DMA_PRIORITY       0

// </e>


// <e> UART5 (Universal asynchronous receiver transmitter) [Driver_USART5]
// <i> Configuration settings for Driver_USART5 in component ::CMSIS Driver:USART
#define RTE_UART5                       0

//   <o> UART5_TX Pin <0=>Not Used <1=>PC12 <2=>PB6 <3=>PB9 <4=>PB13
#define RTE_UART5_TX_ID                 0
#if    (RTE_UART5_TX_ID == 0)
#define RTE_UART5_TX                    0
#elif  (RTE_UART5_TX_ID == 1)
#define RTE_UART5_TX                    1
#define RTE_UART5_TX_PORT               GPIOC
#define RTE_UART5_TX_BIT                12
#elif  (RTE_UART5_TX_ID == 2)
#define RTE_UART5_TX                    1
#define RTE_UART5_TX_PORT               GPIOB
#define RTE_UART5_TX_BIT                6
#elif  (RTE_UART5_TX_ID == 3)
#define RTE_UART5_TX                    1
#define RTE_UART5_TX_PORT               GPIOB
#define RTE_UART5_TX_BIT                9
#elif  (RTE_UART5_TX_ID == 4)
#define RTE_UART5_TX                    1
#define RTE_UART5_TX_PORT               GPIOB
#define RTE_UART5_TX_BIT                13
#else
#error "Invalid UART5_TX Pin Configuration!"
#endif

//   <o> UART5_RX Pin <0=>Not Used <1=>PD2 <2=>PB5 <3=>PB8 <4=>PB12
#define RTE_UART5_RX_ID                 0
#if    (RTE_UART5_RX_ID == 0)
#define RTE_UART5_RX                    0
#elif  (RTE_UART5_RX_ID == 1)
#define RTE_UART5_RX                    1
#define RTE_UART5_RX_PORT               GPIOD
#define RTE_UART5_RX_BIT                2
#elif  (RTE_UART5_RX_ID == 2)
#define RTE_UART5_RX                    1
#define RTE_UART5_RX_PORT               GPIOB
#define RTE_UART5_RX_BIT                5
#elif  (RTE_UART5_RX_ID == 3)
#define RTE_UART5_RX                    1
#define RTE_UART5_RX_PORT               GPIOB
#define RTE_UART5_RX_BIT                8
#elif  (RTE_UART5_RX_ID == 4)
#define RTE_UART5_RX                    1
#define RTE_UART5_RX_PORT               GPIOB
#define RTE_UART5_RX_BIT                12
#else
#error "Invalid UART5_RX Pin Configuration!"
#endif

//   <o> UART5_CTS Pin <0=>Not Used <1=>PC9
#define RTE_UART5_CTS_ID                0
#if    (RTE_UART5_CTS_ID == 0)
#define RTE_UART5_CTS                   0
#elif  (RTE_UART5_CTS_ID == 1)
#define RTE_UART5_CTS                   1
#define RTE_UART5_CTS_PORT              GPIOC
#define RTE_UART5_CTS_BIT               9
#else
#error "Invalid UART5_CTS Pin Configuration!"
#endif

//   <o> UART5_RTS Pin <0=>Not Used <1=>PC8
#define RTE_UART5_RTS_ID                0
#if    (RTE_UART5_RTS_ID == 0)
#define RTE_UART5_RTS                   0
#elif  (RTE_UART5_RTS_ID == 1)
#define RTE_UART5_RTS                   1
#define RTE_UART5_RTS_PORT              GPIOC
#define RTE_UART5_RTS_BIT               8
#else
#error "Invalid UART5_RTS Pin Configuration!"
#endif

//   <e> DMA Rx
//     <o1> Number <1=>1
//     <i>  Selects DMA Number (only DMA1 can be used)
//     <o2> Stream <0=>0
//     <i>  Selects DMA Stream (only Stream 0 can be used)
//     <o3> Channel <4=>4
//     <i>  Selects DMA Channel (only Channel 4 can be used)
//     <o4> Priority <0=>Low <1=>Medium <2=>High <3=>Very High
//     <i>  Selects DMA Priority
//   </e>
#define RTE_UART5_RX_DMA                0
#define RTE_UART5_RX_DMA_NUMBER         1
#define RTE_UART5_RX_DMA_STREAM         0
#define RTE_UART5_RX_DMA_CHANNEL        4
#define RTE_UART5_RX_DMA_PRIORITY       0

//   <e> DMA Tx
//     <o1> Number <1=>1
//     <i>  Selects DMA Number (only DMA1 can be used)
//     <o2> Stream <7=>7
//     <i>  Selects DMA Stream (only Stream 7 can be used)
//     <o3> Channel <4=>4
//     <i>  Selects DMA Channel (only Channel 4 can be used)
//     <o4> Priority <0=>Low <1=>Medium <2=>High <3=>Very High
//     <i>  Selects DMA Priority
//   </e>
#define RTE_UART5_TX_DMA                0
#define RTE_UART5_TX_DMA_NUMBER         1
#define RTE_UART5_TX_DMA_STREAM         7
#define RTE_UART5_TX_DMA_CHANNEL        4
#define RTE_UART5_TX_DMA_PRIORITY       0

// </e>


// <e> USART6 (Universal synchronous asynchronous receiver transmitter) [Driver_USART6]
// <i> Configuration settings for Driver_USART6 in component ::CMSIS Driver:USART
#define RTE_USART6                      0

//   <o> USART6_TX Pin <0=>Not Used <1=>PC6 <2=>PG14
#define RTE_USART6_TX_ID                0
#if    (RTE_USART6_TX_ID == 0)
#define RTE_USART6_TX                   0
#elif  (RTE_USART6_TX_ID == 1)
#define RTE_USART6_TX                   1
#define RTE_USART6_TX_PORT              GPIOC
#define RTE_USART6_TX_BIT               6
#elif  (RTE_USART6_TX_ID == 2)
#define RTE_USART6_TX                   1
#define RTE_USART6_TX_PORT              GPIOG
#define RTE_USART6_TX_BIT               14
#else
#error "Invalid USART6_TX Pin Configuration!"
#endif

//   <o> USART6_RX Pin <0=>Not Used <1=>PC7 <2=>PG9
#define RTE_USART6_RX_ID                0
#if    (RTE_USART6_RX_ID == 0)
#define RTE_USART6_RX                   0
#elif  (RTE_USART6_RX_ID == 1)
#define RTE_USART6_RX                   1
#define RTE_USART6_RX_PORT              GPIOC
#define RTE_USART6_RX_BIT               7
#elif  (RTE_USART6_RX_ID == 2)
#define RTE_USART6_RX                   1
#define RTE_USART6_RX_PORT              GPIOG
#define RTE_USART6_RX_BIT               9
#else
#error "Invalid USART6_RX Pin Configuration!"
#endif

//   <o> USART6_CK Pin <0=>Not Used <1=>PC8 <2=>PG7
#define RTE_USART6_CK_ID                0
#if    (RTE_USART6_CK_ID == 0)
#define RTE_USART6_CK                   0
#elif  (RTE_USART6_CK_ID == 1)
#define RTE_USART6_CK                   1
#define RTE_USART6_CK_PORT              GPIOC
#define RTE_USART6_CK_BIT               8
#elif  (RTE_USART6_CK_ID == 2)
#define RTE_USART6_CK                   1
#define RTE_USART6_CK_PORT              GPIOG
#define RTE_USART6_CK_BIT               7
#else
#error "Invalid USART6_CK Pin Configuration!"
#endif

//   <o> USART6_CTS Pin <0=>Not Used <1=>PG13 <2=>PG15
#define RTE_USART6_CTS_ID               0
#if    (RTE_USART6_CTS_ID == 0)
#define RTE_USART6_CTS                  0
#elif  (RTE_USART6_CTS_ID == 1)
#define RTE_USART6_CTS                  1
#define RTE_USART6_CTS_PORT             GPIOG
#define RTE_USART6_CTS_BIT              13
#elif  (RTE_USART6_CTS_ID == 2)
#define RTE_USART6_CTS                  1
#define RTE_USART6_CTS_PORT             GPIOG
#define RTE_USART6_CTS_BIT              15
#else
#error "Invalid USART6_CTS Pin Configuration!"
#endif

//   <o> USART6_RTS Pin <0=>Not Used <1=>PG8 <2=>PG12
#define RTE_USART6_RTS_ID               0
#if    (RTE_USART6_RTS_ID == 0)
#define RTE_USART6_RTS                  0
#elif  (RTE_USART6_RTS_ID == 1)
#define RTE_USART6_RTS                  1
#define RTE_USART6_RTS_PORT             GPIOG
#define RTE_USART6_RTS_BIT              8
#elif  (RTE_USART6_RTS_ID == 2)
#define RTE_USART6_RTS                  1
#define RTE_USART6_RTS_PORT             GPIOG
#define RTE_USART6_RTS_BIT              12
#else
#error "Invalid USART6_RTS Pin Configuration!"
#endif

//   <e> DMA Rx
//     <o1> Number <2=>2
//     <i>  Selects DMA Number (only DMA2 can be used)
//     <o2> Stream <1=>1 <2=>2
//     <i>  Selects DMA Stream (only Stream 1 or 2 can be used)
//     <o3> Channel <5=>5
//     <i>  Selects DMA Channel (only Channel 5 can be used)
//     <o4> Priority <0=>Low <1=>Medium <2=>High <3=>Very High
//     <i>  Selects DMA Priority
//   </e>
#define RTE_USART6_RX_DMA               0
#define RTE_USART6_RX_DMA_NUMBER        2
#define RTE_USART6_RX_DMA_STREAM        1
#define RTE_USART6_RX_DMA_CHANNEL       5
#define RTE_USART6_RX_DMA_PRIORITY      0

//   <e> DMA Tx
//     <o1> Number <2=>2
//     <i>  Selects DMA Number (only DMA2 can be used)
//     <o2> Stream <6=>6 <7=>7
//     <i>  Selects DMA Stream (only Stream 6 or 7 can be used)
//     <o3> Channel <5=>5
//     <i>  Selects DMA Channel (only Channel 5 can be used)
//     <o4> Priority <0=>Low <1=>Medium <2=>High <3=>Very High
//     <i>  Selects DMA Priority
//   </e>
#define RTE_USART6_TX_DMA               0
#define RTE_USART6_TX_DMA_NUMBER        2
#define RTE_USART6_TX_DMA_STREAM        6
#define RTE_USART6_TX_DMA_CHANNEL       5
#define RTE_USART6_TX_DMA_PRIORITY      0

// </e>

// <e> UART7 (Universal asynchronous receiver transmitter) [Driver_USART7]
// <i> Configuration settings for Driver_USART7 in component ::CMSIS Driver:USART
#define RTE_UART7                       0

//   <o> UART7_TX Pin <0=>Not Used <1=>PF7 <2=>PE8 <3=>PA15 <4=>PB4
#define RTE_UART7_TX_ID                 0
#if    (RTE_UART7_TX_ID == 0)
#define RTE_UART7_TX                    0
#elif  (RTE_UART7_TX_ID == 1)
#define RTE_UART7_TX                    1
#define RTE_UART7_TX_PORT               GPIOF
#define RTE_UART7_TX_BIT                7
#elif  (RTE_UART7_TX_ID == 2)
#define RTE_UART7_TX                    1
#define RTE_UART7_TX_PORT               GPIOE
#define RTE_UART7_TX_BIT                8
#elif  (RTE_UART7_TX_ID == 3)
#define RTE_UART7_TX                    1
#define RTE_UART7_TX_PORT               GPIOA
#define RTE_UART7_TX_BIT                15
#elif  (RTE_UART7_TX_ID == 4)
#define RTE_UART7_TX                    1
#define RTE_UART7_TX_PORT               GPIOB
#define RTE_UART7_TX_BIT                4
#else
#error "Invalid UART7_TX Pin Configuration!"
#endif

//   <o> UART7_RX Pin <0=>Not Used <1=>PF6 <2=>PE7 <3=>PA8 <4=>PB3
#define RTE_UART7_RX_ID                 0
#if    (RTE_UART7_RX_ID == 0)
#define RTE_UART7_RX                    0
#elif  (RTE_UART7_RX_ID == 1)
#define RTE_UART7_RX                    1
#define RTE_UART7_RX_PORT               GPIOF
#define RTE_UART7_RX_BIT                6
#elif  (RTE_UART7_RX_ID == 2)
#define RTE_UART7_RX                    1
#define RTE_UART7_RX_PORT               GPIOE
#define RTE_UART7_RX_BIT                7
#elif  (RTE_UART7_RX_ID == 3)
#define RTE_UART7_RX                    1
#define RTE_UART7_RX_PORT               GPIOA
#define RTE_UART7_RX_BIT                8
#elif  (RTE_UART7_RX_ID == 4)
#define RTE_UART7_RX                    1
#define RTE_UART7_RX_PORT               GPIOB
#define RTE_UART7_RX_BIT                3
#else
#error "Invalid UART7_RX Pin Configuration!"
#endif

//   <o> UART7_CTS Pin <0=>Not Used <1=>PF9 <2=>PE10
#define RTE_UART7_CTS_ID                0
#if    (RTE_UART7_CTS_ID == 0)
#define RTE_UART7_CTS                   0
#elif  (RTE_UART7_CTS_ID == 1)
#define RTE_UART7_CTS                   1
#define RTE_UART7_CTS_PORT              GPIOF
#define RTE_UART7_CTS_BIT               9
#elif  (RTE_UART7_CTS_ID == 2)
#define RTE_UART7_CTS                   1
#define RTE_UART7_CTS_PORT              GPIOE
#define RTE_UART7_CTS_BIT               10
#else
#error "Invalid UART7_CTS Pin Configuration!"
#endif

//   <o> UART7_RTS Pin <0=>Not Used <1=>PF8 <2=>PE9
#define RTE_UART7_RTS_ID                0
#if    (RTE_UART7_RTS_ID == 0)
#define RTE_UART7_RTS                   0
#elif  (RTE_UART7_RTS_ID == 1)
#define RTE_UART7_RTS                   1
#define RTE_UART7_RTS_PORT              GPIOF
#define RTE_UART7_RTS_BIT               8
#elif  (RTE_UART7_RTS_ID == 2)
#define RTE_UART7_RTS                   1
#define RTE_UART7_RTS_PORT              GPIOE
#define RTE_UART7_RTS_BIT               9
#else
#error "Invalid UART7_RTS Pin Configuration!"
#endif

//   <e> DMA Rx
//     <o1> Number <1=>1
//     <i>  Selects DMA Number (only DMA1 can be used)
//     <o2> Stream <3=>3
//     <i>  Selects DMA Stream (only Stream 3 can be used)
//     <o3> Channel <5=>5
//     <i>  Selects DMA Channel (only Channel 5 can be used)
//     <o4> Priority <0=>Low <1=>Medium <2=>High <3=>Very High
//     <i>  Selects DMA Priority
//   </e>
#define RTE_UART7_RX_DMA                0
#define RTE_UART7_RX_DMA_NUMBER         1
#define RTE_UART7_RX_DMA_STREAM         3
#define RTE_UART7_RX_DMA_CHANNEL        5
#define RTE_UART7_RX_DMA_PRIORITY       0

//   <e> DMA Tx
//     <o1> Number <1=>1
//     <i>  Selects DMA Number (only DMA1 can be used)
//     <o2> Stream <1=>1
//     <i>  Selects DMA Stream (only Stream 1 can be used)
//     <o3> Channel <5=>5
//     <i>  Selects DMA Channel (only Channel 5 can be used)
//     <o4> Priority <0=>Low <1=>Medium <2=>High <3=>Very High
//     <i>  Selects DMA Priority
//   </e>
#define RTE_UART7_TX_DMA                0
#define RTE_UART7_TX_DMA_NUMBER         1
#define RTE_UART7_TX_DMA_STREAM         1
#define RTE_UART7_TX_DMA_CHANNEL        5
#define RTE_UART7_TX_DMA_PRIORITY       0

// </e>

// <e> UART8 (Universal asynchronous receiver transmitter) [Driver_USART8]
// <i> Configuration settings for Driver_USART8 in component ::CMSIS Driver:USART
#define RTE_UART8                       0

//   <o> UART8_TX Pin <0=>Not Used <1=>PE1
#define RTE_UART8_TX_ID                 0
#if    (RTE_UART8_TX_ID == 0)
#define RTE_UART8_TX                    0
#elif  (RTE_UART8_TX_ID == 1)
#define RTE_UART8_TX                    1
#define RTE_UART8_TX_PORT               GPIOE
#define RTE_UART8_TX_BIT                1
#else
#error "Invalid UART8_TX Pin Configuration!"
#endif

//   <o> UART8_RX Pin <0=>Not Used <1=>PE0
#define RTE_UART8_RX_ID                 0
#if    (RTE_UART8_RX_ID == 0)
#define RTE_UART8_RX                    0
#elif  (RTE_UART8_RX_ID == 1)
#define RTE_UART8_RX                    1
#define RTE_UART8_RX_PORT               GPIOE
#define RTE_UART8_RX_BIT                0
#else
#error "Invalid UART8_RX Pin Configuration!"
#endif

//   <o> UART8_CTS Pin <0=>Not Used <1=>PD14
#define RTE_UART8_CTS_ID                0
#if    (RTE_UART8_CTS_ID == 0)
#define RTE_UART8_CTS                   0
#elif  (RTE_UART8_CTS_ID == 1)
#define RTE_UART8_CTS                   1
#define RTE_UART8_CTS_PORT              GPIOD
#define RTE_UART8_CTS_BIT               14
#else
#error "Invalid UART8_CTS Pin Configuration!"
#endif

//   <o> UART8_RTS Pin <0=>Not Used <1=>PD15
#define RTE_UART8_RTS_ID                0
#if    (RTE_UART8_RTS_ID == 0)
#define RTE_UART8_RTS                   0
#elif  (RTE_UART8_RTS_ID == 1)
#define RTE_UART8_RTS                   1
#define RTE_UART8_RTS_PORT              GPIOD
#define RTE_UART8_RTS_BIT               15
#else
#error "Invalid UART8_RTS Pin Configuration!"
#endif

//   <e> DMA Rx
//     <o1> Number <1=>1
//     <i>  Selects DMA Number (only DMA1 can be used)
//     <o2> Stream <6=>6
//     <i>  Selects DMA Stream (only Stream 6 can be used)
//     <o3> Channel <5=>5
//     <i>  Selects DMA Channel (only Channel 5 can be used)
//     <o4> Priority <0=>Low <1=>Medium <2=>High <3=>Very High
//     <i>  Selects DMA Priority
//   </e>
#define RTE_UART8_RX_DMA                0
#define RTE_UART8_RX_DMA_NUMBER         1
#define RTE_UART8_RX_DMA_STREAM         6
#define RTE_UART8_RX_DMA_CHANNEL        5
#define RTE_UART8_RX_DMA_PRIORITY       0

//   <e> DMA Tx
//     <o1> Number <1=>1
//     <i>  Selects DMA Number (only DMA1 can be used)
//     <o2> Stream <0=>0
//     <i>  Selects DMA Stream (only Stream 0 can be used)
//     <o3> Channel <5=>5
//     <i>  Selects DMA Channel (only Channel 5 can be used)
//     <o4> Priority <0=>Low <1=>Medium <2=>High <3=>Very High
//     <i>  Selects DMA Priority
//   </e>
#define RTE_UART8_TX_DMA                0
#define RTE_UART8_TX_DMA_NUMBER         1
#define RTE_UART8_TX_DMA_STREAM         0
#define RTE_UART8_TX_DMA_CHANNEL        5
#define RTE_UART8_TX_DMA_PRIORITY       0

// </e>

#endif  /* __RTE_DEVICE_H */
