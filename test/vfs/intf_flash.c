#include <stdbool.h>
#include <stdio.h>
#include "flash_hal.h"
#include "daplink_addr.h"
#include "daplink_debug.h"
#include "compiler.h"
#include "util.h"

uint32_t intf_flash_addr = DAPLINK_ROM_START;
uint32_t intf_flash_size = DAPLINK_ROM_SIZE;
uint8_t intf_flash_byte[DAPLINK_ROM_SIZE];

uint32_t  flash_erase_sector(uint32_t addr)
{
    return EraseSector(addr);
}

uint32_t  flash_program_page(uint32_t adr, uint32_t sz, uint8_t *buf)
{
    return ProgramPage(adr, sz, (uint32_t *)buf);
}

// Default implementation. May be overridden by HIC support.
__WEAK bool flash_is_readable(uint32_t addr, uint32_t length)
{
    uint32_t end_addr = addr + length - 1;
    return (addr >= DAPLINK_ROM_START && addr < (DAPLINK_ROM_START + DAPLINK_ROM_SIZE))
        && (end_addr >= DAPLINK_ROM_START && end_addr < (DAPLINK_ROM_START + DAPLINK_ROM_SIZE));
}

uint32_t Init(uint32_t adr, uint32_t clk, uint32_t fnc)
{
    debug_msg("Init(0x%08x, 0x%08x, 0x%08x)\n", adr, clk, fnc);
    return 0; //
}

uint32_t UnInit(uint32_t fnc)
{
    debug_msg("UnInit(0x%08x)\n", fnc);
    return 0; //
}

uint32_t EraseChip(void)
{
    debug_msg("EraseChip()\n");
    memset(intf_flash_byte, 0xFF, intf_flash_size);
    return 0;
}

uint32_t EraseSector(uint32_t adr)
{
    debug_msg("EraseSector(0x%08x, %d (0x%x))\n", adr, DAPLINK_SECTOR_SIZE, DAPLINK_SECTOR_SIZE);
    util_assert((adr >= intf_flash_addr) &&
                ((adr + DAPLINK_SECTOR_SIZE) <= (intf_flash_addr + intf_flash_size)));
    util_assert((adr % DAPLINK_SECTOR_SIZE) == 0);
    memset(&(intf_flash_byte[adr - intf_flash_addr]), 0xFF, DAPLINK_SECTOR_SIZE);
    return 0;
}

uint32_t ProgramPage(uint32_t adr, uint32_t sz, uint32_t *buf)
{
    debug_msg("ProgramPage(0x%08x, %d (0x%x))\n", adr, sz, sz);
    util_assert((adr >= intf_flash_addr) &&
                ((adr + sz) <= (intf_flash_addr + intf_flash_size)));
    debug_msg("ProgramPage(0x%08x, %d (0x%x))\n", adr, sz, sz);
    memcpy(&(intf_flash_byte[adr - intf_flash_addr]), (void *)buf, sz);
    return 0;
}
