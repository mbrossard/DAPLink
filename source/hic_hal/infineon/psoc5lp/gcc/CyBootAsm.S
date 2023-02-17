/***************************************************************************//**
* \file cy_syslib_gcc.S
* \version 2.70
*
* \brief Assembly routines for GNU GCC.
*
********************************************************************************
* \copyright
* Copyright 2016-2020 Cypress Semiconductor Corporation
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

.syntax unified
.text
.thumb


/*******************************************************************************
* Function Name: CyDelayCycles
****************************************************************************//**
*
* Delays for the specified number of cycles.
*
* \param uint32_t cycles: The number of cycles to delay.
*
*******************************************************************************/
/* void CyDelayCycles(uint32_t cycles) */
.align 3                    /* Align to 8 byte boundary (2^n) */
.global CyDelayCycles
.func CyDelayCycles, CyDelayCycles
.type CyDelayCycles, %function
.thumb_func
CyDelayCycles:      /* cycles bytes */

    ADDS r0, r0, #2         /*    1    2    Round to nearest multiple of 4 */
    LSRS r0, r0, #2         /*    1    2    Divide by 4 and set flags */
    BEQ Cy_DelayCycles_done /*    2    2    Skip if 0 */

Cy_DelayCycles_loop:
    ADDS r0, r0, #1         /*    1    2    Increment counter */
    SUBS r0, r0, #2         /*    1    2    Decrement counter by 2 */
    BNE Cy_DelayCycles_loop /*   (1)2  2    2 CPU cycles (if branch is taken) */
    NOP                     /*    1    2    Loop alignment padding */

Cy_DelayCycles_done:
    NOP                     /*    1    2    Loop alignment padding */
    BX lr                   /*    3    2 */

.endfunc

/*******************************************************************************
* Function Name: CyEnterCriticalSection
****************************************************************************//**
*
* CyEnterCriticalSection disables interrupts and returns a value
* indicating whether interrupts were previously enabled.
*
* Note Implementation of CyEnterCriticalSection manipulates the IRQ
* enable bit with interrupts still enabled.
*
* \return Returns 0 if interrupts were previously enabled or 1 if interrupts
* were previously disabled.
*
*******************************************************************************/
/* uint8_t CyEnterCriticalSection(void) */
.global CyEnterCriticalSection
.func CyEnterCriticalSection, CyEnterCriticalSection
.type CyEnterCriticalSection, %function
.thumb_func

CyEnterCriticalSection:
    MRS r0, PRIMASK         /* Save and return interrupt state */
    cpsid i                 /* Disable interrupts */
    BX lr

.endfunc


/*******************************************************************************
* Function Name: CyExitCriticalSection
****************************************************************************//**
*
*  Re-enables interrupts if they were enabled before
*  CyEnterCriticalSection() was called. The argument should be the value
*  returned from \ref CyEnterCriticalSection().
*
*  \param uint8_t savedIntrStatus:
*   Saved interrupt status returned by the \ref CyEnterCriticalSection().
*
*******************************************************************************/
/* void CyExitCriticalSection(uint8_t savedIntrStatus) */
.global CyExitCriticalSection
.func CyExitCriticalSection, CyExitCriticalSection
.type CyExitCriticalSection, %function
.thumb_func

CyExitCriticalSection:
    MSR PRIMASK, r0         /* Restore interrupt state */
    BX lr

.endfunc

.end

/* [] END OF FILE */
