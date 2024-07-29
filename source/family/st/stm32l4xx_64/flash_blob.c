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

static const uint32_t STM32L4xx_64_flash_prog_blob[] = {
    0xE00ABE00, 
    0x8f4ff3bf, 0x487c4770, 0xf3c06800, 0xf5a0000b, 0x38356080, 0x282dd005, 0x282fd003, 0x2001d001,
    0x20004770, 0x48754770, 0xf3c06a00, 0x47705040, 0x4601b500, 0xffe7f7ff, 0xd1082801, 0xfff3f7ff,
    0xd1042801, 0x4281486e, 0x2001d301, 0x2000bd00, 0xb500bd00, 0xf7ff4601, 0x2801ffd6, 0xf3c1d002,
    0xbd0020c9, 0x30dff240, 0x20d1ea00, 0xd5f803c9, 0x7080f500, 0x4861bd00, 0x60814962, 0x60814962,
    0x60012100, 0x31faf24c, 0x6a006101, 0xd40803c0, 0xf245485e, 0x60015155, 0x60412106, 0x71fff640,
    0x20006081, 0x49554770, 0x4000f04f, 0x20006148, 0x20014770, 0x48514770, 0x31faf24c, 0x13c16101,
    0x69416141, 0x3180f441, 0xf64a6141, 0x4a4f21aa, 0x6011e000, 0x03db6903, 0x2100d4fb, 0x46086141,
    0xb5104770, 0xf7ff4603, 0x4602ffa3, 0xf7ff4618, 0x4942ffb0, 0x34faf24c, 0x2302610c, 0x00c0eb03,
    0x20c2ea40, 0x69486148, 0x3080f440, 0xf3bf6148, 0xf64a8f4f, 0x4a3d20aa, 0x6010e000, 0x03db690b,
    0x2000d4fb, 0x69086148, 0xd0014020, 0x2001610c, 0xb5f0bd10, 0xb0884e31, 0x37faf24c, 0x23006137,
    0x46ec6173, 0x2301e054, 0x29086173, 0x6813d314, 0x68536003, 0xf3bf6043, 0xf64a8f4f, 0x4c2b23aa,
    0x6023e000, 0x03ed6935, 0x6933d4fb, 0xd13b423b, 0x39083008, 0xe03b3208, 0xe0042300, 0x4b01f812,
    0x4023f84c, 0x428b1c5b, 0x2300d3f8, 0xf1c124ff, 0xe0040508, 0x0e01eb03, 0xf84c1c5b, 0x429d402e,
    0xe9ddd8f8, 0xeb011300, 0x9b022103, 0x041b9c03, 0x6304eb03, 0x91004419, 0x1304e9dd, 0x2103eb01,
    0x9c079b06, 0xeb03041b, 0x44196304, 0x99009101, 0x99016001, 0xf3bf6041, 0x21008f4f, 0x03db6933,
    0x6933d4fc, 0xd003423b, 0x20016137, 0xbdf0b008, 0xd1a82900, 0xe7f92000, 0xe0042000, 0x40022000,
    0x08010000, 0x45670123, 0xcdef89ab, 0x40003000, 0x00000000
};

// Start address of flash
static const uint32_t flash_start = 0x08000000;
// Size of flash
static const uint32_t flash_size = 0x00010000;

/**
* List of start and size for each size of flash sector - even indexes are start, odd are size
* The size will apply to all sectors between the listed address and the next address
* in the list.
* The last pair in the list will have sectors starting at that address and ending
* at address flash_start + flash_size.
*/
static const uint32_t sectors_info[] = {
    0x08000000, 0x00000800,
};

static const program_target_t flash = {
    0x2000007b, // Init
    0x200000ab, // UnInit
    0x200000bb, // EraseChip
    0x200000e7, // EraseSector
    0x20000137, // ProgramPage
    0x120000003, // Verify

    // BKPT : start of blob + 1
    // RSB  : blob start + header + rw data offset
    // RSP  : stack pointer
    {
        0x20000001,
        0x20000214,
        0x20000500
    },

    0x20000000 + 0x00000A00,  // mem buffer location
    0x20000000,               // location to write prog_blob in target RAM
    sizeof(STM32L4xx_64_flash_prog_blob),   // prog_blob size
    STM32L4xx_64_flash_prog_blob,           // address of prog_blob
    0x00000400       // ram_to_flash_bytes_to_be_written
};