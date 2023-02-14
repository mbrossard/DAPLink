#include "target_config.h"

// The file flash_blob.c must only be included in target.c
#include "flash_blob.c"

target_cfg_t target_device_nrf9160 = {
    .version                        = kTargetConfigVersion,
    .sectors_info                   = sectors_info_nrf91,
    .sector_info_length             = (sizeof(sectors_info_nrf91))/(sizeof(sector_info_t)),
    .flash_regions[0].start         = flash_start_nrf91,
    .flash_regions[0].end           = flash_size_nrf91,
    .flash_regions[0].flags         = kRegionIsSecure,
    .flash_regions[0].flash_algo    = (program_target_t *) &flash_nrf91,
    .flash_regions[1].start         = flash_start_nrf91_uicr,
    .flash_regions[1].end           = flash_size_nrf91_uicr,
    .flash_regions[1].flags         = kRegionIsSecure,
    .flash_regions[1].flash_algo    = (program_target_t *) &flash_nrf91_uicr,
    .ram_regions[0].start           = 0x20000000,
    .ram_regions[0].end             = 0x2003E000,
    .erase_reset                    = 1,
    .target_vendor                  = "NordicSemiconductor",
    .target_part_number             = "nRF9160_xxAA",
};
