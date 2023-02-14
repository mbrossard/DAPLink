#include "DAP_config.h"
#include "target_family.h"
#include "target_board.h"
#include "swd_host.h"

static void swd_set_target_reset_nrf(uint8_t asserted)
{
    uint32_t ap_index_return;

    if (asserted) {
        swd_init_debug();

        swd_read_ap(0x010000FC, &ap_index_return);
        if (ap_index_return == 0x12880000) {
            // Have CTRL-AP
            swd_write_ap(0x01000000, 1);  // CTRL-AP reset hold
        }
        else {
            // No CTRL-AP - Perform a soft reset
            // 0x05FA0000 = VECTKEY, 0x4 = SYSRESETREQ
            uint32_t swd_mem_write_data = 0x05FA0000 | 0x4;
            swd_write_memory(0xE000ED0C, (uint8_t *) &swd_mem_write_data, 4);
        }
        if(g_board_info.swd_set_target_reset){ //aditional reset
            g_board_info.swd_set_target_reset(asserted);
        }
    } else {
        swd_read_ap(0x010000FC, &ap_index_return);
        if (ap_index_return == 0x12880000) {
            // Device has CTRL-AP
            swd_write_ap(0x01000000, 0);  // CTRL-AP reset release
        }
        else {
            // No CTRL-AP - Soft reset has been performed
        }
        if(g_board_info.swd_set_target_reset){
            g_board_info.swd_set_target_reset(asserted);
        }
    }
}

const target_family_descriptor_t g_nordic_nrf91 = {
    .family_id = kNordic_Nrf91_FamilyID,
    .default_reset_type = kSoftwareReset,
    .soft_reset_type = SYSRESETREQ,
    .swd_set_target_reset = swd_set_target_reset_nrf,
};

const target_family_descriptor_t g_nordic_nrf53 = {
    .family_id = kNordic_Nrf53_FamilyID,
    .default_reset_type = kSoftwareReset,
    .soft_reset_type = SYSRESETREQ,
    .swd_set_target_reset = swd_set_target_reset_nrf,
};
