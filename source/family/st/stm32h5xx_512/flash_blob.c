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

static const uint32_t STM32H5xx_512K_0800_flash_prog_blob[] = {
    0xE00ABE00, 
    0xf242b5b0, 0xf2c40320, 0x6d5a0302, 0x1eabf648, 0x427ff002, 0x1123f240, 0x0c00f64f, 0x4f34f1b2,
    0x0214f240, 0x5eeff6cc, 0x5167f2c4, 0x0cfff6c0, 0x0200f2c0, 0xf240d11d, 0xf1030510, 0xf2c0040c,
    0xf8490500, 0x1d1c4005, 0x050cf240, 0x4002f849, 0x0414f103, 0x0500f2c0, 0x4005f849, 0x1c18f843,
    0xec18f843, 0x8f4ff3bf, 0x07c96859, 0xbf00d01f, 0xf240e7fa, 0xf1030510, 0xf2c00408, 0xf8490500,
    0xf2404005, 0xf103050c, 0xf2c00410, 0xf8490500, 0xf8493002, 0xf8434005, 0xf8431c1c, 0xf3bfec1c,
    0xbf008f4f, 0x07c96819, 0xbf00d020, 0xf04fe7fa, 0xf8c331ff, 0xf8c31080, 0xf8c31084, 0xf8c31088,
    0xf8c3108c, 0xf8c31180, 0xf8c31184, 0xf8c31188, 0xf64e118c, 0xf2ce51d0, 0x23000100, 0xf10c608b,
    0xf8c103e1, 0x610bc00c, 0x600b2301, 0x1002f859, 0x07c96809, 0xbf00d006, 0xf859bf00, 0x68091002,
    0xd1f907c9, 0x0104f240, 0x0100f2c0, 0x0001f849, 0x000cf8dc, 0x4100f64f, 0x31fff2c0, 0x0208f240,
    0x2180ea01, 0x0200f2c0, 0xf8492000, 0xbdb01002, 0x0010f240, 0x0000f2c0, 0x0000f859, 0x60012101,
    0x0014f240, 0x0000f2c0, 0x8f4ff3bf, 0x1000f859, 0x07c96809, 0xbf00d006, 0xf859bf00, 0x68091000,
    0xd1f907c9, 0x47702000, 0x47702001, 0x000cf240, 0x0000f2c0, 0x0000f859, 0x01fcf44f, 0xf2406001,
    0xf2c00010, 0xf8590000, 0xf44f0000, 0x60014100, 0xf0416801, 0x60010120, 0x0014f240, 0x0000f2c0,
    0x8f4ff3bf, 0x1000f859, 0x07c96809, 0xbf00d006, 0xf859bf00, 0x68091000, 0xd1f907c9, 0x47702000,
    0xf240b510, 0xf2400104, 0xf2c00208, 0xf2c00100, 0xf8590200, 0xf8591001, 0xf04f2002, 0xeb013cff,
    0xf2400e52, 0xf2c00114, 0xf8590100, 0xeb0c3001, 0x681b0252, 0x07db4002, 0xbf00d006, 0xf859bf00,
    0x681b3001, 0xd1f907db, 0x0c0cf240, 0x0c00f2c0, 0x300cf859, 0x04fcf44f, 0xf06f601c, 0xea03033f,
    0x230412d2, 0xf2404586, 0xf2c80010, 0xf2c00300, 0xbf880000, 0xf8592304, 0x441a0000, 0x68026002,
    0x0220f042, 0xf3bf6002, 0xbf008f4f, 0x0001f859, 0x07d26802, 0xbf00d001, 0x6800e7f8, 0x0ffcf410,
    0x2000bf0f, 0x000cf859, 0x01fcf44f, 0xbf186001, 0xbd102001, 0xf240b5b0, 0xf2c00314, 0xf8590300,
    0x310f4003, 0xf0216824, 0x07e4010f, 0xbf00d006, 0xf859bf00, 0x68244003, 0xd1f907e4, 0x0c0cf240,
    0x0c00f2c0, 0xe00cf859, 0x04fcf44f, 0x4000f8ce, 0x0e10f240, 0x0e00f2c0, 0x400ef859, 0x60252502,
    0xbf00b1e9, 0x60046814, 0x60446854, 0x60846894, 0x60c468d4, 0x8f4ff3bf, 0x4003f859, 0x07ed6825,
    0xbf00d002, 0xbf00e7f8, 0xf4146824, 0xd1090ffc, 0x39103010, 0x0210f102, 0xf859d1e4, 0x2000400e,
    0xbdb06020, 0x000cf859, 0x01fcf44f, 0x20016001, 0x0000bdb0, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000
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
    {0x08000000, 0x00002000},
};

static const program_target_t flash = {
    0x20000005, // Init
    0x20000135, // UnInit
    0x20000171, // EraseChip
    0x200001c5, // EraseSector
    0x20000279, // ProgramPage
    0x0, // Verify

    // BKPT : start of blob + 1
    // RSB  : blob start + header + rw data offset
    // RSP  : stack pointer
    {
        0x20000001,
        0x20000318,
        0x20000600
    },

    0x20000000 + 0x00000A00,  // mem buffer location
    0x20000000,               // location to write prog_blob in target RAM
    sizeof(STM32H5xx_512K_0800_flash_prog_blob),   // prog_blob size
    STM32H5xx_512K_0800_flash_prog_blob,           // address of prog_blob
    0x00000400       // ram_to_flash_bytes_to_be_written
};