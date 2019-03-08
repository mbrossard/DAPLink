/* File Version : 4.1.0.517
 * Flash OS Routines (Automagically Generated)
 * Copyright (c) 2009-2015 ARM Limited
 *
 * Copyright (c) 2020 Cypress Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#if !defined(CY8C6XXA_FLASH_BLOB_C)
#define CY8C6XXA_FLASH_BLOB_C

#include "flash_blob.h"


// coverity[misra_c_2012_rule_8_9_violation]
static const uint32_t CY8C6xxA_flash_prog_blob[] = {
    0xE00ABE00u, 0x062D780Du, 0x24084068u, 0xD3000040u, 0x1E644058u, 0x1C49D1FAu, 0x2A001E52u, 0x4770D1F2u,
    0xf000b510u, 0xf000f95bu, 0xbd10f801u, 0xf000b510u, 0xf000f991u, 0x2000f99eu, 0xb510bd10u, 0xf990f000u,
    0xbd102000u, 0xf000b510u, 0xbd10fa9bu, 0xf000b510u, 0xbd10faa0u, 0xf000b510u, 0xbd10fab2u, 0x4605b570u,
    0x2000460cu, 0x4628e008u, 0xfa93f000u, 0xd1052800u, 0x1e6435ffu, 0x350235ffu, 0xd1f42c00u, 0xb510bd70u,
    0x48134604u, 0x02492101u, 0xf0004448u, 0x4910fb2du, 0x44494620u, 0xfaa2f000u, 0xb530bd10u, 0xe0042300u,
    0x5cc45cd5u, 0xd10242a5u, 0x428b1c5bu, 0x18c0d3f8u, 0xb530bd30u, 0x20004604u, 0xe0054603u, 0x42955ce5u,
    0x2001d001u, 0x1c5bbd30u, 0xd3f7428bu, 0x0000bd30u, 0x00000044u, 0xf7ffb510u, 0xbd10ffb0u, 0xf7ffb510u,
    0xbd10ffb1u, 0xf7ffb510u, 0xbd10ffb1u, 0x4611b510u, 0xffb0f7ffu, 0xb510bd10u, 0xffcff7ffu, 0xb510bd10u,
    0xffd7f7ffu, 0x0000bd10u, 0x68004827u, 0x0f000500u, 0x2000d001u, 0x20014770u, 0xb5704770u, 0x444c4c23u,
    0x28007820u, 0xf7ffd10bu, 0x2501ffefu, 0x72204068u, 0xf0004620u, 0x2800fa02u, 0x2000d004u, 0x481b7020u,
    0xbd704448u, 0x26007860u, 0xd11d2801u, 0x280078a0u, 0x2802d006u, 0x2805d009u, 0x280ed00cu, 0xe013d00fu,
    0x28e278e0u, 0x7266d110u, 0x78e0e00eu, 0xd10b28e4u, 0xe0082002u, 0x28e778e0u, 0x2005d106u, 0x78e0e003u,
    0xd10128eau, 0x7260200eu, 0x28007a20u, 0xf000d005u, 0x2800f9b2u, 0x72a5d001u, 0x72a6e000u, 0xe7cd2001u,
    0x44484802u, 0x47707800u, 0x40200000u, 0x00000008u, 0x4c61b570u, 0x444c4961u, 0x1e486925u, 0x60651828u,
    0xfa96f000u, 0x7020217du, 0x1e4800c9u, 0xf0001828u, 0x60a0fa8fu, 0x60e003c0u, 0xb570bd70u, 0x68204c58u,
    0x48570701u, 0x008a0f09u, 0x18133880u, 0x07426c18u, 0xd0050f52u, 0x2a014853u, 0x2a04d00fu, 0xe001d10du,
    0xe00a4851u, 0x06c26818u, 0x0ed2207du, 0x2a100200u, 0x2a11d003u, 0x2001d101u, 0x290003c0u, 0x2901d002u,
    0xe04ad024u, 0x69114a49u, 0x689107cbu, 0x00890fdbu, 0x29030f89u, 0x2500d005u, 0xd0152900u, 0xd0132901u,
    0x2501e000u, 0x40192100u, 0xd0364329u, 0x038e6811u, 0x0bb66851u, 0x681204c9u, 0x01d20cc9u, 0x1c6d0fd5u,
    0xfa46f000u, 0xe0254370u, 0xe7ec2101u, 0x31c04937u, 0x07cb6809u, 0x0fdb4935u, 0x680a3180u, 0x0f920092u,
    0xd0052a03u, 0x2a002500u, 0x2a01d026u, 0xe000d024u, 0x22002501u, 0x432a401au, 0x680ad00fu, 0x680a0653u,
    0x04d20e5bu, 0x0ed26809u, 0x0ecd02c9u, 0x46114358u, 0xfa1ef000u, 0xf0004629u, 0x6821fa1bu, 0x06894c1eu,
    0x40c80f89u, 0x4922444cu, 0x69096160u, 0x1c490e09u, 0xfa0ef000u, 0xbd7061a0u, 0xe7db2201u, 0x4c16b510u,
    0x7860444cu, 0xd00a2800u, 0xff0ef7ffu, 0xd0022800u, 0xf9aaf000u, 0xf000e001u, 0x2000f9c1u, 0xbd107060u,
    0x4d0db570u, 0x46042601u, 0x03f6444du, 0x68e8e003u, 0xf98af000u, 0x42b41ba4u, 0x68a8d8f9u, 0xf0004360u,
    0xbd70f983u, 0xb5104904u, 0x78094449u, 0xf0004348u, 0xbd10f97bu, 0xe7f5e7e4u, 0x00000014u, 0x000f4240u,
    0x40260380u, 0x016e3600u, 0x007a1200u, 0x40260580u, 0x40210000u, 0x688048a9u, 0x444949a9u, 0x47706008u,
    0x688248a6u, 0x444949a6u, 0x42916809u, 0x6081d000u, 0xb5104770u, 0xfed1f7ffu, 0x28007a00u, 0x2001d000u,
    0x30102101u, 0x40814a9du, 0x4b9d6890u, 0x444b4208u, 0xd1016018u, 0x60904308u, 0xb5f7bd10u, 0x4999460cu,
    0x25000140u, 0x27011846u, 0x0fc169f0u, 0x43084620u, 0x43884021u, 0xd1074078u, 0x1c6d9902u, 0xd203428du,
    0xf7ff2001u, 0xe7efff9du, 0xbdfe4078u, 0x460eb570u, 0x2400498cu, 0x18450140u, 0x0fc06828u, 0x1c64d106u,
    0xd20342b4u, 0xf7ff2001u, 0xe7f5ff8bu, 0x40482101u, 0xb5f8bd70u, 0x460f4615u, 0x24004606u, 0x60286830u,
    0x280a0f00u, 0x2000d007u, 0x42bc1c64u, 0x2001d204u, 0xff76f7ffu, 0x2001e7f2u, 0x40482101u, 0xb5f3bdf8u,
    0x4605b081u, 0xfebcf7ffu, 0xd0032800u, 0xfe75f7ffu, 0xe0037a00u, 0xfe68f7ffu, 0x40482101u, 0xd0032800u,
    0xd0042801u, 0xbdfe2001u, 0x4c6e2700u, 0x4c6de002u, 0x34202701u, 0x00c9217du, 0xf7ff4638u, 0x2800ffb7u,
    0x07e8d1f1u, 0x1c7617c6u, 0x4d67d001u, 0x60e5444du, 0x60a02001u, 0x21004a65u, 0xf7ff4638u, 0x2800ff8eu,
    0x21ffd1e1u, 0x9a0231f5u, 0xd0012e00u, 0xe0014628u, 0x300c4620u, 0xffadf7ffu, 0xb5ffbdfeu, 0x460fb085u,
    0x4616461du, 0x9c102004u, 0xf7ff990eu, 0x6020ff8fu, 0xd11e2800u, 0x98054669u, 0x804f8008u, 0x4d519503u,
    0x96024668u, 0x60e83580u, 0x60a82020u, 0x20042100u, 0xf7ff9a0fu, 0x6020ff62u, 0xd10a2800u, 0x684168e8u,
    0xd0012900u, 0x60212101u, 0xd0022800u, 0xb0096880u, 0x2001bdf0u, 0xb51fe7fbu, 0xa9032002u, 0x23009000u,
    0x90019102u, 0x2106461au, 0xf7ff2004u, 0x9803ffc6u, 0xd0022800u, 0xb0042000u, 0x2001bd10u, 0xb57fe7fbu,
    0x94032400u, 0xae03247du, 0x00e44d38u, 0x95019602u, 0xf7ff9400u, 0xb004ffb2u, 0xb51cbd70u, 0x46694604u,
    0xf7ff2001u, 0x2800ff6cu, 0x9800d119u, 0x0a00a901u, 0x98007060u, 0x980070a0u, 0x71600c00u, 0x300220ffu,
    0xff5df7ffu, 0xd10a2800u, 0x0a099901u, 0x990170e1u, 0x030a7121u, 0x02090f12u, 0x0f0971a2u, 0xbd1c71e1u,
    0xb5084921u, 0x44494822u, 0x46696008u, 0xff47f7ffu, 0x491dbd08u, 0x4602b508u, 0x481e4449u, 0x604a6008u,
    0xf7ff4669u, 0xbd08ff3cu, 0xb5084917u, 0x44494602u, 0x60084819u, 0x4669604au, 0xff31f7ffu, 0xb538bd08u,
    0x46034a11u, 0x4815444au, 0x24ff6010u, 0x60543407u, 0x60d16093u, 0xf7ff4669u, 0xbd38ff22u, 0x4a0ab538u,
    0x444a4603u, 0x6010480eu, 0x340724ffu, 0x60936054u, 0x466960d1u, 0xff13f7ffu, 0x0000bd38u, 0x40221000u,
    0x00000004u, 0x40220000u, 0x00000030u, 0x00003a98u, 0x0a000100u, 0x1c000100u, 0x14000100u, 0x06000100u,
    0x05000100u, 0x00000000u, 0x08801c80u, 0x1c40d003u, 0xd1fc1e80u, 0x4770bf00u, 0x8010f3efu, 0x4770b672u,
    0x8810f380u, 0x00004770u, 0xf7ffb510u, 0x4808fdc6u, 0x04006900u, 0x48070e01u, 0x44481c49u, 0xf0006800u,
    0x4905f847u, 0x60084449u, 0xfda2f7ffu, 0x0000bd10u, 0x40210000u, 0x0000002cu, 0x00000024u, 0xf7ffb510u,
    0x4808fdacu, 0x04006900u, 0x48070e01u, 0x44481c49u, 0xf0006800u, 0x4905f82du, 0x60084449u, 0xfd88f7ffu,
    0x0000bd10u, 0x40210080u, 0x00000028u, 0x00000024u, 0xc004e001u, 0x29041f09u, 0x078bd2fbu, 0x8002d501u,
    0x07c91c80u, 0x7002d000u, 0x29004770u, 0x07c3d00bu, 0x7002d002u, 0x1e491c40u, 0xd3042902u, 0xd5020783u,
    0x1c808002u, 0xe7e31e89u, 0xe7ee2200u, 0xe7df2200u, 0x09032200u, 0xd32c428bu, 0x428b0a03u, 0x2300d311u,
    0xe04e469cu, 0x430b4603u, 0x2200d43cu, 0x428b0843u, 0x0903d331u, 0xd31c428bu, 0x428b0a03u, 0x4694d301u,
    0x09c3e03fu, 0xd301428bu, 0x1ac001cbu, 0x09834152u, 0xd301428bu, 0x1ac0018bu, 0x09434152u, 0xd301428bu,
    0x1ac0014bu, 0x09034152u, 0xd301428bu, 0x1ac0010bu, 0x08c34152u, 0xd301428bu, 0x1ac000cbu, 0x08834152u,
    0xd301428bu, 0x1ac0008bu, 0x08434152u, 0xd301428bu, 0x1ac0004bu, 0x1a414152u, 0x4601d200u, 0x46104152u,
    0xe05d4770u, 0xd0000fcau, 0x10034249u, 0x4240d300u, 0x22004053u, 0x0903469cu, 0xd32d428bu, 0x428b0a03u,
    0x22fcd312u, 0xba120189u, 0x428b0a03u, 0x0189d30cu, 0x428b1192u, 0x0189d308u, 0x428b1192u, 0x0189d304u,
    0x1192d03au, 0x0989e000u, 0x428b09c3u, 0x01cbd301u, 0x41521ac0u, 0x428b0983u, 0x018bd301u, 0x41521ac0u,
    0x428b0943u, 0x014bd301u, 0x41521ac0u, 0x428b0903u, 0x010bd301u, 0x41521ac0u, 0x428b08c3u, 0x00cbd301u,
    0x41521ac0u, 0x428b0883u, 0x008bd301u, 0x41521ac0u, 0x0843d2d9u, 0xd301428bu, 0x1ac0004bu, 0x1a414152u,
    0x4601d200u, 0x41524663u, 0x4610105bu, 0x4240d301u, 0xd5002b00u, 0x47704249u, 0x105b4663u, 0x4240d300u,
    0x2000b501u, 0x46c046c0u, 0x0000bd02u, 0x00000000u, 0x00000001u, 0xffffff00u, 0xffffffffu, 0x0000ffffu,
    0x00000104u, 0x003d0900u, 0x00000fa0u, 0x07d00000u, 0x003d0900u, 0x007a1200u, 0x003d0900u
};

// coverity[misra_c_2012_rule_8_9_violation]
static const program_target_t CY8C6xxA_flash_prog =
    {
        .init =         PSOC6_SRAM_START_ADDR + 0x00000021u, // Init
        .uninit =       PSOC6_SRAM_START_ADDR + 0x000000d5u, // UnInit
        .erase_chip =   PSOC6_SRAM_START_ADDR + 0x000000ddu, // EraseChip
        .erase_sector = PSOC6_SRAM_START_ADDR + 0x000000e5u, // EraseSector
        .program_page = PSOC6_SRAM_START_ADDR + 0x000000edu, // ProgramPage
        .verify =       PSOC6_SRAM_START_ADDR + 0x000000f7u, // Verify
        // BKPT : start of blob + 1
        // RSB  : blob start + header + rw data offset
        // RSP  : stack pointer
        .sys_call_s =
            {
                PSOC6_SRAM_START_ADDR + 0x00000001u,
                PSOC6_SRAM_START_ADDR + 0x0000084cu,
                PSOC6_SRAM_START_ADDR + 0x000014fcu
            },
        .program_buffer = PSOC6_SRAM_START_ADDR + 0x00001500u,      // mem buffer location
        .algo_start = PSOC6_SRAM_START_ADDR,                       // location to write prog_blob in target RAM
        .algo_size = sizeof(CY8C6xxA_flash_prog_blob),  // prog_blob size
        .algo_blob = CY8C6xxA_flash_prog_blob,          // address of prog_blob
        .program_buffer_size = 0x00000200u,              // ram_to_flash_bytes_to_be_written
        .algo_flags = (kAlgoVerifyReturnsAddress | kAlgoSingleInitType)
    };
#endif
