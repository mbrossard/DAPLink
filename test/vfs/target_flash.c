#include <string.h>
#include <stdio.h>

#include "target_config.h"
#include "intelhex.h"
#include "flash_intf.h"
#include "util.h"
#include "settings.h"
#include "target_family.h"
#include "target_board.h"
#include "daplink_debug.h"

extern uint32_t target_flash_addr;
extern uint32_t target_flash_size;
extern uint8_t target_flash_byte[];

#define DEFAULT_PROGRAM_PAGE_MIN_SIZE   (256u)

typedef enum {
    STATE_CLOSED,
    STATE_OPEN,
    STATE_ERROR
} state_t;

static error_t target_flash_init(void);
static error_t target_flash_uninit(void);
static error_t target_flash_program_page(uint32_t adr, const uint8_t *buf, uint32_t size);
static error_t target_flash_erase_sector(uint32_t addr);
static error_t target_flash_erase_chip(void);
static uint32_t target_flash_program_page_min_size(uint32_t addr);
static uint32_t target_flash_erase_sector_size(uint32_t addr);
static uint8_t target_flash_busy(void);
static error_t target_flash_set(uint32_t addr);

static const flash_intf_t flash_intf = {
    target_flash_init,
    target_flash_uninit,
    target_flash_program_page,
    target_flash_erase_sector,
    target_flash_erase_chip,
    target_flash_program_page_min_size,
    target_flash_erase_sector_size,
    target_flash_busy,
    target_flash_set,
};

static state_t state = STATE_CLOSED;

const flash_intf_t *const flash_intf_target = &flash_intf;

//saved flash algo
static program_target_t * current_flash_algo = NULL;

//saved default region for default flash algo
static region_info_t * default_region = NULL;

//saved flash start from flash algo
static uint32_t flash_start = 0;

static program_target_t * get_flash_algo(uint32_t addr)
{
    region_info_t * flash_region = g_board_info.target_cfg->flash_regions;

    for (; flash_region->start != 0 || flash_region->end != 0; ++flash_region) {
        if (addr >= flash_region->start && addr <= flash_region->end) {
            flash_start = flash_region->start; //save the flash start
            if (flash_region->flash_algo) {
                return flash_region->flash_algo;
            } else {
                return NULL;
            }
        }
    }

    //could not find a flash algo for the region; use default
    if (default_region) {
        flash_start = default_region->start;
        return default_region->flash_algo;
    } else {
        return NULL;
    }
}

static error_t target_flash_set(uint32_t addr)
{
    program_target_t * new_flash_algo = get_flash_algo(addr);
    if (new_flash_algo == NULL) {
        return ERROR_ALGO_MISSING;
    }
    if(current_flash_algo != new_flash_algo) {
        current_flash_algo = new_flash_algo;
    }
    return ERROR_SUCCESS;
}

static error_t target_flash_init()
{
    debug_msg("target_flash_init()\n");
    if (g_board_info.target_cfg) {
        current_flash_algo = NULL;

        //get default region
        region_info_t * flash_region = g_board_info.target_cfg->flash_regions;
        for (; flash_region->start != 0 || flash_region->end != 0; ++flash_region) {
            if (flash_region->flags & kRegionIsDefault) {
                default_region = flash_region;
                break;
            }
        }

        state = STATE_OPEN;
        return ERROR_SUCCESS;
    } else {
        return ERROR_FAILURE;
    }
}

static error_t target_flash_uninit(void)
{
    debug_msg("target_flash_uninit()\n");
    if (g_board_info.target_cfg) {
        if (config_get_auto_rst()) {
            // Resume the target if configured to do so
            debug_msg("target_set_state(RESET_RUN)\n");
        } else {
            // Leave the target halted until a reset occurs
            debug_msg("target_set_state(RESET_PROGRAM)\n");
        }
        // Check to see if anything needs to be done after programming.
        // This is usually a no-op for most targets.
        debug_msg("target_set_state(POST_FLASH_RESET)\n");

        state = STATE_CLOSED;
        return ERROR_SUCCESS;
    } else {
        return ERROR_FAILURE;
    }
}

static error_t target_flash_program_page(uint32_t addr, const uint8_t *buf, uint32_t size)
{
    debug_msg("target_flash_program_page(0x%08x, %d bytes)\n", addr, size);
    if (g_board_info.target_cfg) {
        program_target_t * flash = current_flash_algo;

        if (!flash) {
            return ERROR_INTERNAL;
        }

        // check if security bits were set
        if (g_target_family && g_target_family->security_bits_set){
            if (1 == g_target_family->security_bits_set(addr, (uint8_t *)buf, size)) {
                return ERROR_SECURITY_BITS;
            }
        }

        while (size > 0) {
            uint32_t write_size = MIN(size, flash->program_buffer_size);
            debug_msg("target_flash_program_page() flashing %d bytes at 0x%08x\n", write_size, addr);
            if ((addr >= target_flash_addr) &&
                ((addr + write_size) <= (target_flash_addr + target_flash_size))) {
                memcpy(&(target_flash_byte[addr - target_flash_addr]), buf, write_size);
            }

            addr += write_size;
            buf += write_size;
            size -= write_size;
        }

        return ERROR_SUCCESS;
    } else {
        return ERROR_FAILURE;
    }
}

static error_t target_flash_erase_sector(uint32_t addr)
{
    debug_msg("target_flash_erase_sector(0x%08x)\n", addr);
    if (g_board_info.target_cfg) {
        program_target_t * flash = current_flash_algo;
        uint32_t erase_sector_size = target_flash_erase_sector_size(addr);
        if (!flash) {
            return ERROR_INTERNAL;
        }

        // Check to make sure the address is on a sector boundary
        if ((addr % erase_sector_size) != 0) {
            return ERROR_ERASE_SECTOR;
        } else {
            if ((addr >= target_flash_addr) &&
                ((addr + erase_sector_size) <= (target_flash_addr + target_flash_size))) {
                memset(&(target_flash_byte[addr - target_flash_addr]), 0xFF, erase_sector_size);
            }
            debug_msg("target_flash_erase_sector: sector at 0x%08x has %d bytes size\n",
                      addr, erase_sector_size);
        }

        return ERROR_SUCCESS;
    } else {
        return ERROR_FAILURE;
    }
}

static error_t target_flash_erase_chip(void)
{
    debug_msg("target_flash_erase_chip()\n");
    if (g_board_info.target_cfg){
        error_t status = ERROR_SUCCESS;
        region_info_t * flash_region = g_board_info.target_cfg->flash_regions;

        for (; flash_region->start != 0 || flash_region->end != 0; ++flash_region) {
            program_target_t *new_flash_algo = get_flash_algo(flash_region->start);
            if ((new_flash_algo != NULL) && ((new_flash_algo->algo_flags & kAlgoSkipChipErase) != 0)) {
                // skip flash region
                continue;
            }
            debug_msg("target_flash_erase_chip: Calling erase_chip() on 0x%08x\n", flash_region->start);
            if(flash_region->start == target_flash_addr) {
                memset(target_flash_byte, 0xFF, target_flash_size);
            }
        }

        // Reset and re-initialize the target after the erase if required
        if (g_board_info.target_cfg->erase_reset) {
            status = target_flash_init();
        }

        return status;
    } else {
        return ERROR_FAILURE;
    }
}

static uint32_t target_flash_program_page_min_size(uint32_t addr)
{
    if (g_board_info.target_cfg){
        uint32_t size = DEFAULT_PROGRAM_PAGE_MIN_SIZE;
        if (size > target_flash_erase_sector_size(addr)) {
            size = target_flash_erase_sector_size(addr);
        }
        return size;
    } else {
        return 0;
    }
}

static uint32_t target_flash_erase_sector_size(uint32_t addr)
{
    if (g_board_info.target_cfg) {
        if(g_board_info.target_cfg->sector_info_length > 0) {
            int sector_index = g_board_info.target_cfg->sector_info_length - 1;
            for (; sector_index >= 0; sector_index--) {
                if (addr >= g_board_info.target_cfg->sectors_info[sector_index].start) {
                    return g_board_info.target_cfg->sectors_info[sector_index].size;
                }
            }
        }
        //sector information should be in sector_info
        util_assert(0);
        return 0;
    } else {
        return 0;
    }
}

static uint8_t target_flash_busy(void) {
    return (state == STATE_OPEN);
}
