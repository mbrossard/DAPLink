/**
 * @file    target_reset.c
 * @brief   Target reset for the lpc812
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2019, ARM Limited, All Rights Reserved
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
#include "target_family.h"
#include "swd_host.h"
#include "cmsis_os2.h"

#define DBG_Addr                (0xE000EDF0)
#define DWT_COMP0               (0xE0001020)
#define DWT_FUNCTION0           (0xE0001028)
#define DWT_FUNCTION_MATCH      (0x4 << 0)
#define DWT_FUNCTION_ACTION     (0x1 << 4)
#define DWT_FUNCTION_DATAVSIZE  (0x2 << 10)

/* Debug mailbox AP registers */
// #define DEBUGMB_CSW  0x02000000
// #define DEBUGMB_REQ  0x02000004
// #define DEBUGMB_RET  0x02000008
// #define DEBUGMB_ID   0x020000FC

#define DEBUGMB_CSW  0x00
#define DEBUGMB_REQ  0x04
#define DEBUGMB_RET  0x08
#define DEBUGMB_ID   0x5009C00C

#define CSW_RESYNCH_REQ 1 << 0
#define CSW_REQ_PENDING 1 << 1
#define CSW_DBG_OR_ERR 1 << 2
#define CSW_AHB_OR_ERR 1 << 3
#define CSW_SOFT_RESET 1 << 4
#define CSW_CHIP_RESET_REQ 1 << 5

////! @brief States into which the target can be placed.
////!
////! These enums are passed to target_set_state() and indicate the desired state into which
////! the target should be reset.
//typedef enum _target_state {
//    RESET_HOLD,              //!< Hold target in reset
//    RESET_PROGRAM,           //!< Reset target and setup for flash programming
//    RESET_RUN,               //!< Reset target and run normally
//    NO_DEBUG,                //!< Disable debug on running target
//    DEBUG,                   //!< Enable debug on running target
//    HALT,                    //!< Halt the target without resetting it
//    RUN,                     //!< Resume the target without resetting it
//    POST_FLASH_RESET,        //!< Reset target after flash programming
//    POWER_ON,                //!< Poweron the target
//    SHUTDOWN,                //!< Poweroff the target
//} target_state_t;


uint8_t swd_read_ap_(uint32_t adr, uint32_t *val)
{
    uint8_t tmp_in, ack;
    uint8_t tmp_out[4];
    uint32_t tmp;
    uint32_t bank_sel = adr & APBANKSEL;


    tmp_in = SWD_REG_AP | SWD_REG_R | SWD_REG_ADR(adr << 2);
    // first dummy read
    swd_transfer_retry(tmp_in, (uint32_t *)tmp_out);
    ack = swd_transfer_retry(tmp_in, (uint32_t *)tmp_out);
    *val = 0;
    tmp = tmp_out[3];
    *val |= (tmp << 24);
    tmp = tmp_out[2];
    *val |= (tmp << 16);
    tmp = tmp_out[1];
    *val |= (tmp << 8);
    tmp = tmp_out[0];
    *val |= (tmp << 0);
    return (ack == 0x01);
}

// Write access port register
uint8_t swd_write_ap_(uint32_t adr, uint32_t val)
{
    uint8_t data[4];
    uint8_t req, ack;
    uint32_t bank_sel = adr & APBANKSEL;

    req = SWD_REG_AP | SWD_REG_W | SWD_REG_ADR(adr << 2);
    int2array(data, val, 4);

    if (swd_transfer_retry(req, (uint32_t *)data) != 0x01) {
        return 0;
    }

    req = SWD_REG_DP | SWD_REG_R | SWD_REG_ADR(DP_RDBUFF);
    ack = swd_transfer_retry(req, NULL);
}


static uint8_t lpc55s6x_target_set_state(target_state_t state)
{
    uint32_t val;
    int8_t ap_retries = 2;

    volatile static target_state_t st = 0;
    volatile static uint16_t count = 0;
    st = state;
    count++;

    if (state <= 2) {

        if (!swd_init_debug()) {
            return 0;
        }

    	swd_write_dp(2 << 2, 0x020000F0);

        swd_read_ap_(3, &val);
        if( val != 0x002A0000)
        {
        	val = 0;
        }

        // Select DM AP index 2
        swd_write_dp(2 << 2, 0x02000000);

        swd_write_ap_(0, CSW_RESYNCH_REQ | CSW_CHIP_RESET_REQ);
        // Poll CSW register (0) for zero return, indicating success
        val = 0xFFFFFF;
        while (val != 0) {
        	swd_read_ap_(0, &val);
        }

        // Write DM START_DBG_SESSION to REQUEST register (1)
        swd_write_ap_(1, 0x07);
        // Poll RETURN register (2) for zero return
        val = 0xFFFFFF;
        while ((val & 0xFFFF) != 0x0) {
        	swd_read_ap_(2, &val);
        }

        return 1;

    } else {
        return swd_set_target_state_sw(state);
    }
}

const target_family_descriptor_t g_target_family_lpc55S6X = {
    .family_id = kNXP_LPC55xx_FamilyID, //ID not maching the predefined family ids
    .target_set_state = lpc55s6x_target_set_state,
};

const target_family_descriptor_t *g_target_family = &g_target_family_lpc55S6X;
