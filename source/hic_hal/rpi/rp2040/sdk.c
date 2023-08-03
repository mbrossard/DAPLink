#include "tusb.h"
#include "pico-sdk/hardware/irq.h"

void sdk_init(void)
{
}

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+

extern void USBD_SignalHandler(void);

void dcd_daplink_irq(void)
{
#if !defined(TINYDAP) && defined(TUSB_DAPLINK_DEFERRED)
    irq_set_enabled(USBCTRL_IRQ, false);
    USBD_SignalHandler();
#else
    tud_int_handler(0);
#if !defined(TINYDAP) && defined(TUSB_DAPLINK_DEFERRED_PARTIAL)
    USBD_SignalHandler();
#endif
#endif
}

void USBD_IntEnable(void)
{
    irq_set_enabled(USBCTRL_IRQ, true);
}

void daplink_tusb_init(void)
{
    irq_set_exclusive_handler(USBCTRL_IRQ, dcd_daplink_irq);
}
