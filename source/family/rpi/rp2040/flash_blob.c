/* Flash algorithm for Rasperry Pi RP2040
 *
 * DAPLink Interface Firmware
 * Copyright (c) 2009-2022 Arm Limited, All Rights Reserved
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

// Generated from 'flash-algo' (Rasperry Pi RP2040)
// digest = 57a4e56426e6e65c26144ab1c216f8f256be18727222d42925c95d4385e82b70, file size = 12028
// algo version = 0x1, algo size = 628 (0x274)
static const uint32_t flash-algo_flash_prog_blob[] = {
    0xe7fdbe00,
    0xaf03b5f0, 0x4836b087, 0x68064478, 0x28007830, 0x447d4d34, 0x682cd004, 0x47806920, 0x47806960,
    0x90062001, 0x20147030, 0x1e818800, 0x884b4a28, 0xd04c2b00, 0x46141d09, 0x42a334f7, 0x9505d1f7,
    0x88091e83, 0x49239104, 0x2c00885c, 0x1d1bd03f, 0x42ac1c8d, 0x9603d1f8, 0x881e1e84, 0x2b008863,
    0x1d24d035, 0xd1f94293, 0x88251e82, 0x88544b1a, 0xd02c2c00, 0x429c1d12, 0x9502d1f9, 0x88141e83,
    0x885d4a16, 0xd0222d00, 0x42951d1b, 0x9401d1f9, 0x1e804635, 0x9200881a, 0x2a008842, 0x1d00d017,
    0xd1f9428a, 0x9e048804, 0x47a847b0, 0x68009805, 0x60456006, 0x60819902, 0x60c19901, 0x61019900,
    0x98036144, 0x70019906, 0xb0072000, 0xdefebdf0, 0x00004552, 0x00005843, 0x00005052, 0x00004346,
    0x00000264, 0x0000025e, 0xaf02b5b0, 0x44784808, 0x78206804, 0xd0092800, 0x44784806, 0x69286805,
    0x69684780, 0x20004780, 0xbdb07020, 0xbdb02001, 0x0000017e, 0x00000176, 0x44784805, 0x78006800,
    0xd0012800, 0x47704801, 0x47702001, 0x000070d0, 0x00000152, 0xaf02b5d0, 0x4479490a, 0x78096809,
    0xd00d2900, 0x0709210f, 0x49071840, 0x68094479, 0x2201688c, 0x04120311, 0x47a023d8, 0xbdd02000,
    0xbdd02001, 0x00000132, 0x00000124, 0xaf02b5d0, 0x490a460b, 0x68094479, 0x29007809, 0x210fd00b,
    0x18400709, 0x44794906, 0x68cc6809, 0x461a4611, 0x200047a0, 0x2001bdd0, 0x46c0bdd0, 0x000000f8,
    0x000000ea, 0xd4d4d400, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x61520001, 0x72657073, 0x50207972, 0x50522069, 0x30343032, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00050000, 0x10000000, 0x01000000, 0x00000100, 0x00000000, 0x000000ff, 0x000001f4, 0x00001388,
    0x00001000, 0x10000000, 0xffffffff, 0xffffffff
};

// Start address of flash
static const uint32_t flash_start = 0x10000000;
// Size of flash
static const uint32_t flash_size = 0x01000000;

/**
* List of start and size for each size of flash sector - even indexes are start, odd are size
* The size will apply to all sectors between the listed address and the next address
* in the list.
* The last pair in the list will have sectors starting at that address and ending
* at address flash_start + flash_size.
*/
static const sector_info_t sectors_info[] = {
    {0x20000000, 0x00001000},
};

static const program_target_t flash = {
    0x20000005, // Init
    0x200000ed, // UnInit
    0x2000011d, // EraseChip
    0x20000139, // EraseSector
    0x20000171, // ProgramPage
    0x00000000, // Verify

    // BKPT : start of blob + 1
    // RSB  : blob start + header + rw data offset
    // RSP  : stack pointer
    {
        0x20000001,
        0x200001c4,
        0x20000b00
    },

    // mem buffer location
    0x20000b00,
    // location to write prog_blob in target RAM
    0x20000000,
    // prog_blob size
    sizeof(flash-algo_flash_prog_blob),
    // address of prog_blob
    flash-algo_flash_prog_blob,
    // ram_to_flash_bytes_to_be_written
    0x00000100
};
