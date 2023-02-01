#include "device.h"
#include "IO_Config.h"
#include "daplink.h"
#include "compiler.h"
#include "util.h"
#include "nrfx.h"
#include "nrfx_clock.h"


void sdk_init()
{
	if (!nrfx_clock_hfclk_is_running()) {
		nrfx_clock_hfclk_start();
	}
	if (!nrfx_clock_lfclk_is_running()) {
		nrfx_clock_lfclk_start();
	}
}
