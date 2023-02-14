#include "target_config.h"

// The file flash_blob.c must only be included in target.c
#include "flash_blob.c"

target_cfg_t target_device_nrf9160 = {
    .version                        = kTargetConfigVersion,
    .sectors_info                   = sectors_info_nrf53,
    .sector_info_length             = (sizeof(sectors_info_nrf53))/(sizeof(sector_info_t)),
    .flash_regions[0].start         = flash_start_nrf53xx_application,
    .flash_regions[0].end           = flash_size_nrf53xx_application,
    .flash_regions[0].flags         = kRegionIsSecure,
    .flash_regions[0].flash_algo    = (program_target_t *) &flash_nrf53xx_application,
    .flash_regions[1].start         = flash_start_nrf53xx_network,
    .flash_regions[1].end           = flash_size_nrf53xx_network,
    .flash_regions[1].flags         = kRegionIsSecure,
    .flash_regions[1].flash_algo    = (program_target_t *) &flash_nrf53xx_network,
    .flash_regions[2].start         = flash_start_nrf53xx_application_uicr,
    .flash_regions[2].end           = flash_size_nrf53xx_application_uicr,
    .flash_regions[2].flags         = kRegionIsSecure,
    .flash_regions[2].flash_algo    = (program_target_t *) &flash_nrf53xx_application_uicr,
    .flash_regions[3].start         = flash_start_nrf53xx_network_uicr,
    .flash_regions[3].end           = flash_size_nrf53xx_network_uicr,
    .flash_regions[3].flags         = kRegionIsSecure,
    .flash_regions[3].flash_algo    = (program_target_t *) &flash_nrf53xx_network_uicr,
    .ram_regions[0].start           = 0x21000000, // RAM
    .ram_regions[0].end             = 0x21010000,
    .ram_regions[1].start           = 0x20000000, // APP_RAM0
    .ram_regions[1].end             = 0x20040000,
    .ram_regions[2].start           = 0x20040000, // APP_RAM1
    .ram_regions[3].end             = 0x20080000,
    .erase_reset                    = 1,
    .target_vendor                  = "NordicSemiconductor",
    .target_part_number             = "nRF5340_xxAA",
};
