/* Flash OS Routines (Automagically Generated)
 * Copyright (c) 2009-2015 ARM Limited
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

#include "flash_blob.h"

static const uint32_t STM32G0Bx_512_flash_prog_blob[] = {
    0xE00ABE00, 
    0x4a6ab672, 0x608a496a, 0x608a4a6a, 0xbf00e000, 0x0392690a, 0xd1fa0f92, 0x444a4a67, 0x48676050,
    0x04008c00, 0x60900980, 0x60d00840, 0x68004864, 0x48640503, 0x18180d1b, 0x280ad01f, 0x2810d01d,
    0x2001d01b, 0x6a086010, 0x0fc00280, 0x6a086110, 0xd40803c0, 0x4a5c485d, 0x10526002, 0x22066002,
    0x4a5b6042, 0x6a086082, 0xd4040300, 0x4959485a, 0x217f6041, 0x20006001, 0x20004770, 0x484ee7e2,
    0x68404448, 0x484a6801, 0xd0041c49, 0x22016801, 0x43910412, 0x69416001, 0x07d22201, 0x61414311,
    0x8f4ff3bf, 0x47702000, 0x47702001, 0x4a4b4840, 0x13c16102, 0x69416141, 0x041b2301, 0x61414319,
    0x8f4ff3bf, 0xbf00e000, 0x03896901, 0xd1fa0f89, 0x42116901, 0x6102d002, 0x47702001, 0x47702000,
    0x4b35b530, 0x444b2401, 0x04e4681a, 0xd10c2a01, 0x29016919, 0x6899d002, 0xd10642a1, 0x685968dd,
    0x4285194d, 0x2101d801, 0x2100e000, 0xd0012a01, 0xe006689a, 0x2a01691a, 0x689ad002, 0xd10042a2,
    0x4b2e68da, 0x40101e52, 0x0ac04a21, 0x00c06113, 0x1c800349, 0x61504308, 0x21016950, 0x43080409,
    0xf3bf6150, 0xe0008f4f, 0x6910bf00, 0x0f800380, 0x6910d1fa, 0xd0014018, 0x20016113, 0xb530bd30,
    0x4b131dc9, 0x4d1d08c9, 0x611d00c9, 0x615c2401, 0x6814e014, 0x68546004, 0xf3bf6044, 0xe0008f4f,
    0x691cbf00, 0x0fa403a4, 0x691cd1fa, 0xd002422c, 0x2001611d, 0x3008bd30, 0x32083908, 0xd1e82900,
    0x08406958, 0x61580040, 0xbd302000, 0x45670123, 0x40022000, 0xcdef89ab, 0x00000004, 0x1fff75c0,
    0x40015800, 0xfffffbaa, 0x0000aaaa, 0x40003000, 0x00000fff, 0x000001ff, 0x40002c00, 0x0000c3fa,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000
};

// Start address of flash
static const uint32_t flash_start = 0x08000000;
// Size of flash
static const uint32_t flash_size = 0x00080000;

/**
* List of start and size for each size of flash sector - even indexes are start, odd are size
* The size will apply to all sectors between the listed address and the next address
* in the list.
* The last pair in the list will have sectors starting at that address and ending
* at address flash_start + flash_size.
*/
static const sector_info_t sectors_info[] = {
    {0x08000000, 0x00000800},
};

static const program_target_t flash = {
    0x20000005, // Init
    0x20000083, // UnInit
    0x200000b1, // EraseChip
    0x200000e5, // EraseSector
    0x20000163, // ProgramPage
    0x0, // Verify

    // BKPT : start of blob + 1
    // RSB  : blob start + header + rw data offset
    // RSP  : stack pointer
    {
        0x20000001,
        0x200001e4,
        0x20000400
    },

    0x20000000 + 0x00000A00,  // mem buffer location
    0x20000000,               // location to write prog_blob in target RAM
    sizeof(STM32G0Bx_512_flash_prog_blob),   // prog_blob size
    STM32G0Bx_512_flash_prog_blob,           // address of prog_blob
    0x00000400       // ram_to_flash_bytes_to_be_written
};