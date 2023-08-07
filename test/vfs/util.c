#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "cmsis_os2.h"
/*
uint32_t DAPLINK_ROM_START;
uint32_t DAPLINK_ROM_SIZE;
uint32_t DAPLINK_RAM_START;
uint32_t DAPLINK_RAM_SIZE;
uint32_t DAPLINK_ROM_BL_START;
uint32_t DAPLINK_ROM_BL_SIZE;
uint32_t DAPLINK_ROM_IF_START;
uint32_t DAPLINK_ROM_IF_SIZE;
uint32_t DAPLINK_ROM_CONFIG_USER_START;
uint32_t DAPLINK_ROM_CONFIG_USER_SIZE;
uint32_t DAPLINK_RAM_APP_START;
uint32_t DAPLINK_RAM_APP_SIZE;
uint32_t DAPLINK_RAM_SHARED_START;
uint32_t DAPLINK_RAM_SHARED_SIZE;
uint32_t DAPLINK_MIN_WRITE_SIZE;
uint32_t DAPLINK_ROM_APP_START;
uint32_t DAPLINK_ROM_APP_SIZE;
uint32_t DAPLINK_ROM_UPDATE_START;
uint32_t DAPLINK_ROM_UPDATE_SIZE;
uint32_t DAPLINK_HIC_ID = 0x6E052820;
*/

bool config_get_auto_rst()
{
    return true;
}

bool config_get_automation_allowed(void)
{
    return true;
}

bool config_get_overflow_detect()
{
    return true;
}

bool config_get_detect_incompatible_target()
{
    return true;
}

void config_ram_set_page_erase(bool page_erase_enable)
{
    //
}

void info_crc_compute()
{
    //
}

osThreadId_t osThreadGetId (void)
{
    return (osThreadId_t)1;
}

void _util_assert(bool expression, const char *filename, uint16_t line)
{
    if (expression) {
        return;
    }

    printf("Assert %s:%d\n", filename, line);

    exit(-1);
}
