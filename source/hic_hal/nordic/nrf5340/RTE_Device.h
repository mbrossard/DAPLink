/* -----------------------------------------------------------------------------
 * Copyright (c) 2013-2021 Arm Limited (or its affiliates). All
 * rights reserved.
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
 *
 * $Date:        16. July 2021
 * $Revision:    V1.0
 *
 * Project:      RTE Device Configuration for Nordic Semiconductor nRF52820
 * -------------------------------------------------------------------------- */

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef __RTE_DEVICE_H
#define __RTE_DEVICE_H

#include "IO_Config.h"

// <e> USART0 (Universal synchronous asynchronous receiver transmitter) [Driver_USART0]
// <i> Configuration settings for Driver_USART0 in component ::CMSIS Driver:USART
#define RTE_USART0                      1

//   <e> TX Pin
//     <o1.0..4> Pin number
#define RTE_USART0_TX_EN                1
#define RTE_USART0_TX_PIN_NUM           UART_TX_PIN
//   </e>
//   <e> RX Pin
//     <o1.0..4> Pin number
#define RTE_USART0_RX_EN                1
#define RTE_USART0_RX_PIN_NUM           UART_RX_PIN
//   </e>
//   <e> CTS Pin
//     <o1.0..4> Pin number
#define RTE_USART0_CTS_EN               0
#define RTE_USART0_CTS_PIN_NUM          UART_CTS_PIN
//   </e>
//   <e> RTS Pin
//     <o1.0..4> Pin number
#define RTE_USART0_RTS_EN               0
#define RTE_USART0_RTS_PIN_NUM          UART_RTS_PIN
//   </e>

#endif /* __RTE_DEVICE_H */
