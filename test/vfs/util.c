#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#include "compiler.h"
#include "cmsis_os2.h"
#include "target_family.h"

__WEAK const target_family_descriptor_t *g_target_family = NULL;

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
