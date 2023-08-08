#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "error.h"
#include "file_stream.h"
#include "daplink_addr.h"
#include "daplink_debug.h"
#include "util.h"

typedef enum {
    TARGET_FLASH, INTF_FLASH_BL, INTF_FLASH_IF,
} flash_location_t;

typedef struct {
    char *input_file;
    char *ref_file;
    flash_location_t location;
} vfs_tests_t;

#if defined(INTERFACE_NRF52820)
#if defined(DAPLINK_BL)
vfs_tests_t tests[] = {
    {
        "files/nrf52820_microbit_if_crc.hex",
        "files/nrf52820_microbit_if_crc.bin",
        INTF_FLASH_IF
    }
};
#elif defined(DAPLINK_IF)
vfs_tests_t tests[] = {
    {
        "files/nrf52820_microbit_bl_crc.hex",
        "files/nrf52820_microbit_bl_crc.bin",
        INTF_FLASH_BL
    },
    {
        "files/Out of box experience.hex",
        "files/OOBE-9903.bin",
        TARGET_FLASH
    },
};
#endif
#endif

#if defined(INTERFACE_KL27Z)
#if defined(DAPLINK_BL)
vfs_tests_t tests[] = {
    {
        "files/kl27z_microbit_if_crc.hex",
        "files/kl27z_microbit_if_crc.bin",
        INTF_FLASH_IF
    }
};
#elif defined(DAPLINK_IF)
vfs_tests_t tests[] = {
    {
        "files/kl27z_microbit_bl_crc.hex",
        "files/kl27z_microbit_bl_crc.bin",
        INTF_FLASH_BL
    },
    {
        "files/Out of box experience.hex",
        "files/OOBE-9903.bin",
        TARGET_FLASH
    },
};
#endif
#endif

#if defined(INTERFACE_KL26Z)
vfs_tests_t tests[] = {
    {
        "files/Out of box experience.hex",
        "files/OOBE-9900.bin",
        TARGET_FLASH
    },
};
#endif

extern uint32_t target_flash_addr;
extern uint32_t target_flash_size;
extern uint8_t target_flash_byte[];

extern uint32_t intf_flash_addr;
extern uint32_t intf_flash_size;
extern uint8_t intf_flash_byte[];

int main(int argc, char **argv)
{
    uint8_t buffer[512];

    for (uint32_t t = 0; t < sizeof(tests) / sizeof(vfs_tests_t); t++) {
        uint32_t i = 0, l = 0, to_read = sizeof(buffer);
        bool first_block = true;
        FILE *f = fopen(tests[t].input_file, "rb");
        if (f == NULL) {
            fprintf(stderr, "File not '%s' found\r\n", tests[t].input_file);
            exit(-1);
        }

        error_t status;
        while ((l = fread(buffer, 1, to_read, f)) > 0) {
            fprintf(stderr, "Block %i, size=%i\r\n", i, l);
            if (first_block) {
            stream_type_t stream = stream_start_identify(buffer, l);
                status = stream_open(stream);
                if (status != ERROR_SUCCESS) {
                    fprintf(stderr, "Error opening stream (%i): %s\r\n", status, error_get_string(status));
                    exit(-1);
                }
                first_block = false;
            }
            status = stream_write(buffer, l);
            if(status != ERROR_SUCCESS_DONE && status != ERROR_SUCCESS_DONE_OR_CONTINUE && status != ERROR_SUCCESS) {
                fprintf(stderr, "Error (%i): %s\r\n", status, error_get_string(status));
                exit(-1);
                break;
            } else if (ERROR_SUCCESS_DONE == status) {
                break;
            }
            i++;
        }

        fclose(f);
        status = stream_close();

        if (status == ERROR_SUCCESS) {
            FILE *f = fopen(tests[t].ref_file, "rb");
            if (f == NULL) {
                fprintf(stderr, "File not '%s' found\r\n", tests[t].ref_file);
                exit(-1);
            }

            i = 0;
            int pos = 0, check = 0;
            while ((l = fread(buffer, 1, to_read, f)) > 0) {
                fprintf(stderr, "Check block %i, size=%i\r\n", i, l);
                uint8_t *ptr = 0;
                switch (tests[t].location) {
#if defined(DAPLINK_IF)
                case TARGET_FLASH:
                    ptr = &(target_flash_byte[pos]);
                    break;
#endif
                case INTF_FLASH_BL:
                    util_assert((pos + l) <= DAPLINK_ROM_BL_SIZE);
                    ptr = &(intf_flash_byte[(DAPLINK_ROM_BL_START - DAPLINK_ROM_START) + pos]);
                    break;
                case INTF_FLASH_IF:
                    util_assert((pos + l) <= DAPLINK_ROM_IF_SIZE);
                    ptr = &(intf_flash_byte[(DAPLINK_ROM_IF_SIZE - DAPLINK_ROM_START) + pos]);
                    break;
                default:
                    fprintf(stderr, "Unexpected");
                    exit(-1);
                }
                check = memcmp(ptr, buffer, l);
                if (check != 0) {
                    fprintf(stderr, "Mismatch at offset 0x%08x\r\n", pos + (check < 0 ? -check : check));
                }
                pos += l;
                i++;
            }
            fprintf(stderr, "Success\r\n");
        } else {
            fprintf(stderr, "Error closing stream: %s\r\n", error_get_string(status));
            exit(-1);
        }

    }
    fprintf(stdout, "Success\r\n");

    return 0;
}