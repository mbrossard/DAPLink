#if defined(INTERFACE_NRF52820)
#define DAPLINK_HIC_ID 0x6E052820
#include "../../../source/hic_hal/nordic/nrf52820/daplink_addr.h"
#endif

#if defined(INTERFACE_KL27Z)
#define DAPLINK_HIC_ID 0x9796990B
#include "../../../source/hic_hal/freescale/kl27z/daplink_addr.h"
#endif

#if defined(INTERFACE_KL26Z)
#define DAPLINK_HIC_ID 0x97969901
#include "../../../source/hic_hal/freescale/kl26z/daplink_addr.h"
#endif
