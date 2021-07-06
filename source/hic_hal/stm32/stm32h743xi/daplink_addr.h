/**
 * @file    daplink_addr.h
 * @brief   Address map for stm32h743xx HIC
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2021, Arm Limited
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

#ifndef DAPLINK_ADDR_H
#define DAPLINK_ADDR_H

/* Device sizes */

#define DAPLINK_ROM_START               
#define DAPLINK_ROM_SIZE                

#define DAPLINK_RAM_START               
#define DAPLINK_RAM_SIZE                

/* ROM sizes */

#define DAPLINK_ROM_BL_START            
#define DAPLINK_ROM_BL_SIZE             

#define DAPLINK_ROM_IF_START            
#define DAPLINK_ROM_IF_SIZE             

#define DAPLINK_ROM_CONFIG_USER_START   
#define DAPLINK_ROM_CONFIG_USER_SIZE    

/* RAM sizes */

#define DAPLINK_RAM_APP_START           
#define DAPLINK_RAM_APP_SIZE            

#define DAPLINK_RAM_SHARED_START        
#define DAPLINK_RAM_SHARED_SIZE         

/* USB RAM */
#define DAPLINK_USB_RAM_START           
#define DAPLINK_USB_RAM_SIZE            

/* Flash Programming Info */

#define DAPLINK_SECTOR_SIZE             
#define DAPLINK_MIN_WRITE_SIZE          

/* Current build */

#if defined(DAPLINK_BL)

#define DAPLINK_ROM_APP_START            DAPLINK_ROM_BL_START
#define DAPLINK_ROM_APP_SIZE             DAPLINK_ROM_BL_SIZE
#define DAPLINK_ROM_UPDATE_START         DAPLINK_ROM_IF_START
#define DAPLINK_ROM_UPDATE_SIZE          DAPLINK_ROM_IF_SIZE

#elif defined(DAPLINK_IF)

#define DAPLINK_ROM_APP_START            DAPLINK_ROM_IF_START
#define DAPLINK_ROM_APP_SIZE             DAPLINK_ROM_IF_SIZE
#define DAPLINK_ROM_UPDATE_START         DAPLINK_ROM_BL_START
#define DAPLINK_ROM_UPDATE_SIZE          DAPLINK_ROM_BL_SIZE

#else

#error "Build must be either bootloader or interface"

#endif

#endif
