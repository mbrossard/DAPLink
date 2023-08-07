#include <stdint.h>
#include "target_family.h"
#include "target_board.h"

const target_family_descriptor_t *g_target_family = NULL;

uint16_t board_id_hex_default = 0x9903;
#if defined(INTERFACE_KL27Z)
uint16_t board_id_hex = 0x9904;
#elif defined(INTERFACE_NRF52820)
uint16_t board_id_hex = 0x9905;
#else
#error "microbitv2 Target requires INTERFACE_KL27Z or INTERFACE_NRF52820"
#endif

static const uint32_t nRF52833_flash_algo[] = { 0xE00ABE00 };
static const sector_info_t sectors_info_nrf52[] = { {0, 4096} };

static const program_target_t flash_nrf52833 = {
    .init = 0x20000039,
    .uninit = 0x2000003d,
    .erase_chip = 0x20000041,
    .erase_sector = 0x2000006d,
    .program_page = 0x200000a1,
    .verify = 0x0,
    {
        .breakpoint = 0x20000001,
        .static_base = 0x200000e4,
        .stack_pointer = 0x20000300
    },
    .program_buffer = 0x20000000 + 0x00000A00,
    .algo_start = 0x20000000,
    sizeof(nRF52833_flash_algo),
    .algo_blob = nRF52833_flash_algo,
    .program_buffer_size = 512 // should be USBD_MSC_BlockSize
};

// target information for nRF52833 with 128 KB RAM / 512 KB Flash
target_cfg_t target_device_nrf52833 __ALIGNED(8) = {
    .version                        = kTargetConfigVersion,
    .sectors_info                   = sectors_info_nrf52,
    .sector_info_length             = (sizeof(sectors_info_nrf52))/(sizeof(sector_info_t)),
    .flash_regions[0].start         = 0,
    .flash_regions[0].end           = KB(512),
    .flash_regions[0].flags         = kRegionIsDefault,
    .flash_regions[0].flash_algo    = (program_target_t *) &flash_nrf52833,
    .ram_regions[0].start           = 0x20000000,
    .ram_regions[0].end             = 0x20020000,
    .erase_reset                    = 1,
    .target_vendor                  = "NordicSemiconductor",
    .target_part_number             = "nRF52833_xxAA",
#if defined(INTERFACE_KL27Z)
    .rt_board_id                    = "9904",
#elif defined(INTERFACE_NRF52820)
    .rt_board_id                    = "9905",
#endif
};

const board_info_t g_board_info __ALIGNED(8) = {
    .info_version = kBoardInfoVersion,
    .family_id = kNordic_Nrf52_FamilyID,
    .daplink_url_name = "MICROBITHTM",
    .daplink_drive_name = "MICROBIT",
    .daplink_target_url = "https://microbit.org/device/?id=@B&v=@V",
    .target_cfg = &target_device_nrf52833,
    .board_vendor = "Micro:bit Educational Foundation",
    .board_name = "BBC micro:bit V2",
};

uint32_t target_flash_addr = 0x00000000;
uint32_t target_flash_size = KB(512);
uint8_t target_flash_byte[KB(512)];
