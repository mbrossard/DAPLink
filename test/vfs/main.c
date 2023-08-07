#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "error.h"
#include "file_stream.h"
#include "daplink_debug.h"

#if defined(INTERFACE_NRF52820)
#if defined(DAPLINK_BL)
char *file_name = "files/nrf52820_microbit_if_crc.hex";
#elif defined(DAPLINK_IF)
// char *file_name = "files/nrf52820_microbit_bl_crc.hex";
char *file_name = "files/Out of box experience.hex";
#endif
#endif

#if defined(INTERFACE_KL27Z)
#if defined(DAPLINK_BL)
char *file_name = "files/kl27z_microbit_if_crc.hex";
#elif defined(DAPLINK_IF)
// char *file_name = "files/kl27z_microbit_bl_crc.hex";
char *file_name = "files/Out of box experience.hex";
#endif
#endif

#if defined(INTERFACE_KL26Z)
char *file_name = "files/Out of box experience.hex";
#endif

int main(int argc, char **argv)
{
    uint8_t buffer[512];
    uint32_t i = 0, l = 0, to_read = sizeof(buffer);
    bool first_block = true;
    FILE *f = fopen(file_name, "r");
    if (f == NULL) {
        debug_msg("File not found\r\n");
        exit(-1);
    }

    // to_read = 64;

    error_t status;
    while ((l = fread(buffer, 1, to_read, f)) > 0) {
        debug_msg("Block %i, size=%i\r\n", i, l);
        if (first_block) {
        stream_type_t stream = stream_start_identify(buffer, l);
            status = stream_open(stream);
            if (status != ERROR_SUCCESS) {
                debug_msg("Error opening stream (%i): %s\r\n", status, error_get_string(status));
                exit(-1);
            }
            first_block = false;
        }
        status = stream_write(buffer, l);
        if(status != ERROR_SUCCESS_DONE && status != ERROR_SUCCESS_DONE_OR_CONTINUE && status != ERROR_SUCCESS) {
            debug_msg("Error (%i): %s\r\n", status, error_get_string(status));
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
        debug_msg("Success\r\n");
    } else {
        debug_msg("Error closing stream: %s\r\n", error_get_string(status));
        exit(-1);
    }

    return 0;
}