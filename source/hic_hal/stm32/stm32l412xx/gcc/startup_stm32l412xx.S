/**
 * @file    startup_stm32l412xx.s
 * @brief
 *
 * DAPLink Interface Firmware
 * Copyright (c) 1997 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2017 NXP
 * Copyright (c) 2009-2016, ARM Limited, All Rights Reserved
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*****************************************************************************/
/* Version: GCC for ARM Embedded Processors                                  */
/*****************************************************************************/
    .syntax unified
    .arch armv7e-m

    .section .isr_vector, "a"
    .align 2
    .globl __isr_vector
__isr_vector:
    .long   __StackTop                                      /* Top of Stack */
    .long   Reset_Handler                                   /* Reset Handler */
    .long   NMI_Handler                                     /* NMI Handler*/
    .long   HardFault_Handler                               /* Hard Fault Handler*/
    .long   MemManage_Handler                               /* MPU Fault Handler*/
    .long   BusFault_Handler                                /* Bus Fault Handler*/
    .long   UsageFault_Handler                              /* Usage Fault Handler*/
    .long   0                                               /* Reserved*/
    .long   DAPLINK_BUILD_KEY                               /* DAPLINK: Build type (BL/IF)*/
    .long   DAPLINK_HIC_ID                                  /* DAPLINK: Compatibility*/
    .long   DAPLINK_VERSION                                 /* DAPLINK: Version*/
    .long   SVC_Handler                                     /* SVCall Handler*/
    .long   DebugMon_Handler                                /* Debug Monitor Handler*/
    .long   g_board_info                                    /* DAPLINK: Pointer to board/family/target info*/
    .long   PendSV_Handler                                  /* PendSV Handler*/
    .long   SysTick_Handler                                 /* SysTick Handler*/

    /* External Interrupts*/
    .long   WWDG_IRQHandler
    .long   PVD_PVM_IRQHandler
    .long   TAMP_STAMP_IRQHandler
    .long   RTC_WKUP_IRQHandler
    .long   FLASH_IRQHandler
    .long   RCC_IRQHandler
    .long   EXTI0_IRQHandler
    .long   EXTI1_IRQHandler
    .long   EXTI2_IRQHandler
    .long   EXTI3_IRQHandler
    .long   EXTI4_IRQHandler
    .long   DMA1_Channel1_IRQHandler
    .long   DMA1_Channel2_IRQHandler
    .long   DMA1_Channel3_IRQHandler
    .long   DMA1_Channel4_IRQHandler
    .long   DMA1_Channel5_IRQHandler
    .long   DMA1_Channel6_IRQHandler
    .long   DMA1_Channel7_IRQHandler
    .long   ADC1_2_IRQHandler
    .long   0
    .long   0
    .long   0
    .long   0
    .long   EXTI9_5_IRQHandler
    .long   TIM1_BRK_TIM15_IRQHandler
    .long   TIM1_UP_TIM16_IRQHandler
    .long   TIM1_TRG_COM_IRQHandler
    .long   TIM1_CC_IRQHandler
    .long   TIM2_IRQHandler
    .long   0
    .long   0
    .long   I2C1_EV_IRQHandler
    .long   I2C1_ER_IRQHandler
    .long   I2C2_EV_IRQHandler
    .long   I2C2_ER_IRQHandler
    .long   SPI1_IRQHandler
    .long   SPI2_IRQHandler
    .long   USART1_IRQHandler
    .long   USART2_IRQHandler
    .long   USART3_IRQHandler
    .long   EXTI15_10_IRQHandler
    .long   RTC_Alarm_IRQHandler
    .long   0
    .long   0
    .long   0
    .long   0
    .long   0
    .long   0
    .long   0
    .long   0
    .long   0
    .long   0
    .long   0
    .long   0
    .long   TIM6_IRQHandler
    .long   0
    .long   DMA2_Channel1_IRQHandler
    .long   DMA2_Channel2_IRQHandler
    .long   DMA2_Channel3_IRQHandler
    .long   DMA2_Channel4_IRQHandler
    .long   DMA2_Channel5_IRQHandler
    .long   0
    .long   0
    .long   0
    .long   COMP_IRQHandler
    .long   LPTIM1_IRQHandler
    .long   LPTIM2_IRQHandler
    .long   USB_IRQHandler
    .long   DMA2_Channel6_IRQHandler
    .long   DMA2_Channel7_IRQHandler
    .long   LPUART1_IRQHandler
    .long   QUADSPI_IRQHandler
    .long   I2C3_EV_IRQHandler
    .long   I2C3_ER_IRQHandler
    .long   0
    .long   0
    .long   0
    .long   TSC_IRQHandler
    .long   0
    .long   0
    .long   RNG_IRQHandler
    .long   FPU_IRQHandler
    .long   CRS_IRQHandler

    .size    __isr_vector, . - __isr_vector

    .text
    .thumb

/* Reset Handler */

    .thumb_func
    .align 2
    .globl   Reset_Handler
    .weak    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
    cpsid   i               /* Mask interrupts */
    .equ    VTOR, 0xE000ED08
    ldr     r0, =VTOR
    ldr     r1, =__isr_vector
    str     r1, [r0]
    ldr     r2, [r1]
    msr     msp, r2
#ifndef __NO_SYSTEM_INIT
    ldr   r0,=SystemInit
    blx   r0
#endif
/*     Loop to copy data from read only memory to RAM. The ranges
 *      of copy from/to are specified by following symbols evaluated in
 *      linker script.
 *      __etext: End of code section, i.e., begin of data sections to copy from.
 *      __data_start__/__data_end__: RAM address range that data should be
 *      copied to. Both must be aligned to 4 bytes boundary.  */

    ldr    r1, =__etext
    ldr    r2, =__data_start__
    ldr    r3, =__data_end__

#if 1
/* Here are two copies of loop implemenations. First one favors code size
 * and the second one favors performance. Default uses the first one.
 * Change to "#if 0" to use the second one */
.LC0:
    cmp     r2, r3
    ittt    lt
    ldrlt   r0, [r1], #4
    strlt   r0, [r2], #4
    blt    .LC0
#else
    subs    r3, r2
    ble    .LC1
.LC0:
    subs    r3, #4
    ldr    r0, [r1, r3]
    str    r0, [r2, r3]
    bgt    .LC0
.LC1:
#endif

#ifdef __STARTUP_CLEAR_BSS
/*     This part of work usually is done in C library startup code. Otherwise,
 *     define this macro to enable it in this startup.
 *
 *     Loop to zero out BSS section, which uses following symbols
 *     in linker script:
 *      __bss_start__: start of BSS section. Must align to 4
 *      __bss_end__: end of BSS section. Must align to 4
 */
    ldr r1, =__bss_start__
    ldr r2, =__bss_end__

    movs    r0, 0
.LC2:
    cmp     r1, r2
    itt    lt
    strlt   r0, [r1], #4
    blt    .LC2
#endif /* __STARTUP_CLEAR_BSS */

    cpsie   i               /* Unmask interrupts */
#ifndef __START
#define __START _start
#endif
#ifndef __ATOLLIC__
    ldr   r0,=__START
    blx   r0
#else
    ldr   r0,=__libc_init_array
    blx   r0
    ldr   r0,=main
    bx    r0
#endif
    .pool
    .size Reset_Handler, . - Reset_Handler

    .align  1
    .thumb_func
    .weak DefaultISR
    .type DefaultISR, %function
DefaultISR:
    b DefaultISR
    .size DefaultISR, . - DefaultISR

    .align 1
    .thumb_func
    .weak NMI_Handler
    .type NMI_Handler, %function
NMI_Handler:
    ldr   r0,=NMI_Handler
    bx    r0
    .size NMI_Handler, . - NMI_Handler

    .align 1
    .thumb_func
    .weak HardFault_Handler
    .type HardFault_Handler, %function
HardFault_Handler:
    ldr   r0,=HardFault_Handler
    bx    r0
    .size HardFault_Handler, . - HardFault_Handler

    .align 1
    .thumb_func
    .weak MemManage_Handler
    .type MemManage_Handler, %function
MemManage_Handler:
    ldr   r0,=MemManage_Handler
    bx    r0
    .size MemManage_Handler, . - MemManage_Handler

    .align 1
    .thumb_func
    .weak BusFault_Handler
    .type BusFault_Handler, %function
BusFault_Handler:
    ldr   r0,=BusFault_Handler
    bx    r0
    .size BusFault_Handler, . - BusFault_Handler

    .align 1
    .thumb_func
    .weak UsageFault_Handler
    .type UsageFault_Handler, %function
UsageFault_Handler:
    ldr   r0,=UsageFault_Handler
    bx    r0
    .size UsageFault_Handler, . - UsageFault_Handler

    .align 1
    .thumb_func
    .weak SVC_Handler
    .type SVC_Handler, %function
SVC_Handler:
    ldr   r0,=SVC_Handler
    bx    r0
    .size SVC_Handler, . - SVC_Handler

    .align 1
    .thumb_func
    .weak DebugMon_Handler
    .type DebugMon_Handler, %function
DebugMon_Handler:
    ldr   r0,=DebugMon_Handler
    bx    r0
    .size DebugMon_Handler, . - DebugMon_Handler

    .align 1
    .thumb_func
    .weak PendSV_Handler
    .type PendSV_Handler, %function
PendSV_Handler:
    ldr   r0,=PendSV_Handler
    bx    r0
    .size PendSV_Handler, . - PendSV_Handler

    .align 1
    .thumb_func
    .weak SysTick_Handler
    .type SysTick_Handler, %function
SysTick_Handler:
    ldr   r0,=SysTick_Handler
    bx    r0
    .size SysTick_Handler, . - SysTick_Handler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro def_irq_handler  handler_name
    .weak \handler_name
    .set  \handler_name, DefaultISR
    .endm

/* Exception Handlers */
    def_irq_handler WWDG_IRQHandler             /* Window Watchdog */
    def_irq_handler PVD_PVM_IRQHandler          /* PVD through EXTI Line detect */
    def_irq_handler TAMP_STAMP_IRQHandler       /* Tamper */
    def_irq_handler RTC_WKUP_IRQHandler         /* RTC */
    def_irq_handler FLASH_IRQHandler            /* Flash */
    def_irq_handler RCC_IRQHandler              /* RCC */
    def_irq_handler EXTI0_IRQHandler            /* EXTI Line 0 */
    def_irq_handler EXTI1_IRQHandler            /* EXTI Line 1 */
    def_irq_handler EXTI2_IRQHandler            /* EXTI Line 2 */
    def_irq_handler EXTI3_IRQHandler            /* EXTI Line 3 */
    def_irq_handler EXTI4_IRQHandler            /* EXTI Line 4 */
    def_irq_handler DMA1_Channel1_IRQHandler    /* DMA1 Channel 1 */
    def_irq_handler DMA1_Channel2_IRQHandler    /* DMA1 Channel 2 */
    def_irq_handler DMA1_Channel3_IRQHandler    /* DMA1 Channel 3 */
    def_irq_handler DMA1_Channel4_IRQHandler    /* DMA1 Channel 4 */
    def_irq_handler DMA1_Channel5_IRQHandler    /* DMA1 Channel 5 */
    def_irq_handler DMA1_Channel6_IRQHandler    /* DMA1 Channel 6 */
    def_irq_handler DMA1_Channel7_IRQHandler    /* DMA1 Channel 7 */
    def_irq_handler ADC1_2_IRQHandler           /* ADC1_2 */
    def_irq_handler EXTI9_5_IRQHandler          /* EXTI Line 9..5 */
    def_irq_handler TIM1_BRK_TIM15_IRQHandler   /* TIM1 Break TIM15 */
    def_irq_handler TIM1_UP_TIM16_IRQHandler    
    def_irq_handler TIM1_TRG_COM_IRQHandler
    def_irq_handler TIM1_CC_IRQHandler
    def_irq_handler TIM2_IRQHandler
    def_irq_handler I2C1_EV_IRQHandler
    def_irq_handler I2C1_ER_IRQHandler
    def_irq_handler I2C2_EV_IRQHandler
    def_irq_handler I2C2_ER_IRQHandler
    def_irq_handler SPI1_IRQHandler
    def_irq_handler SPI2_IRQHandler
    def_irq_handler USART1_IRQHandler
    def_irq_handler USART2_IRQHandler
    def_irq_handler USART3_IRQHandler
    def_irq_handler EXTI15_10_IRQHandler
    def_irq_handler RTC_Alarm_IRQHandler
    def_irq_handler TIM6_IRQHandler
    def_irq_handler DMA2_Channel1_IRQHandler
    def_irq_handler DMA2_Channel2_IRQHandler
    def_irq_handler DMA2_Channel3_IRQHandler
    def_irq_handler DMA2_Channel4_IRQHandler
    def_irq_handler DMA2_Channel5_IRQHandler
    def_irq_handler COMP_IRQHandler
    def_irq_handler LPTIM1_IRQHandler
    def_irq_handler LPTIM2_IRQHandler
    def_irq_handler USB_IRQHandler              /* USB */
    def_irq_handler DMA2_Channel6_IRQHandler
    def_irq_handler DMA2_Channel7_IRQHandler
    def_irq_handler LPUART1_IRQHandler
    def_irq_handler QUADSPI_IRQHandler
    def_irq_handler I2C3_EV_IRQHandler
    def_irq_handler I2C3_ER_IRQHandler
    def_irq_handler TSC_IRQHandler
    def_irq_handler RNG_IRQHandler
    def_irq_handler FPU_IRQHandler
    def_irq_handler CRS_IRQHandler
    .end
