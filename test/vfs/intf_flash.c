#include <stdbool.h>
#include <stdio.h>
#include "flash_hal.h"
#include "daplink_addr.h"
#include "daplink_debug.h"
#include "compiler.h"

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
    return 0; //
}

uint32_t EraseSector(uint32_t adr)
{
    debug_msg("EraseSector(0x%08x)\n", adr);
    return 0; //
}

uint32_t ProgramPage(uint32_t adr, uint32_t sz, uint32_t *buf)
{
    debug_msg("ProgramPage(0x%08x, %d)\n", adr, sz);
    return 0; //
}
