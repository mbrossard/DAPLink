#include "RP20xx.h"
#include "gpio.h"

void GPIO_Init(uint32_t gpio_no, eGPIO_IO_Type io_type, eGPIO_Pull pu_pd, eGPIO_Speed speed, uint32_t alternate_function)
{
    __IOM uint32_t* IO_BANK_CTRL_REG = (uint32_t*)(IO_BANK0_BASE + 0x04 + (0x08 * gpio_no));
    __IOM uint32_t* PADS_BANK_CTRL_REG = (uint32_t*)(PADS_BANK0_BASE + 0x04 + (0x04 * gpio_no));

    *PADS_BANK_CTRL_REG = (uint32_t)pu_pd | PADS_BANK0_GPIO0_SCHMITT_Msk |
        (uint32_t)speed | PADS_BANK0_GPIO0_IE_Msk;

    *IO_BANK_CTRL_REG = io_type | (alternate_function << IO_BANK0_GPIO0_CTRL_FUNCSEL_Pos);

    if (io_type == GPIO_IO_TYPE_INPUT) {
        SET_BIT(*PADS_BANK_CTRL_REG, PADS_BANK0_GPIO0_OD_Msk);
    } else {
        CLEAR_BIT(*PADS_BANK_CTRL_REG, PADS_BANK0_GPIO0_OD_Msk);
    }
}