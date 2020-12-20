/**
 * DAPLink Interface Firmware
 * Copyright (c) 2020 Arm Limited, All Rights Reserved
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

#include "DAP_config.h"
#include "DAP.h"
#include "fsl_ctimer.h"
#include "fsl_inputmux.h"
#include "fsl_dma.h"
#include "fsl_reset.h"

//! @brief Dma channel constants.
enum _dma_channels {
    kDmaWriteChannel = 0,
    kDmaWriteChannelMask = 1 << kDmaWriteChannel,
    kDmaReadChannel = 1,
    kDmaReadChannelMask = 1 << kDmaReadChannel,

    kFirstFreeDescriptor = FSL_FEATURE_DMA_NUMBER_OF_CHANNELS,
};

//! @brief SWD bit counts and buffer sizes.
enum _swd_bit_counts {
    kSwdRequestBits = 8,
    kMaxTurnaroundBits = 3,
    kAckBits = 3,
    kDataBits = 33,

    kMaxSWJBits = 256,

    //! Number of words in the SWD write buffer. Must allow enough room for the maximum length
    //! of SWJ_Sequence transfers, plus the final write to restore signals to high state.
    //! Multiply by 2 for both clock edges.
    kWriteBufferLength = 2 * (kMaxSWJBits + 1),

    //! Number of bytes in the ACK read buffer.
    //! Multiply by 2 for both clock edges.
    kAckBufferLength = 2 * (kMaxTurnaroundBits * 2 + kAckBits),

    //! Number of bytes in the data read buffer.
    //! Multiply by 2 for both clock edges.
    kReadBufferLength = 2 * (kAckBits + kDataBits + 2 * kMaxTurnaroundBits),
};

enum _swd_timeout {
    //! 1 second timeout per SWD transfer.
    kSwdTimeout_ms = 1000,

    //! Cycles per iteration of the DMA IRQ polling loop.
    kCyclesPerPollLoop = 5,
};

/*!
 * @brief Runtime context data for DMA-based SWD.
 */
typedef struct _swd_dma_context {
    uint32_t last_clock; //!< Most recent configured clock frequency.
    uint32_t dirsetclr_mask; //!< Value written by DMA to GPIO.DIR{SET,CLR}.
    uint32_t write_data[kWriteBufferLength]; //!< DMA source buffer.
    uint8_t read_data[kWriteBufferLength]; //!< DMA destination buffer for
} swd_dma_context_t;

// Buffers and DMA descriptors are placed in a separate RAM instance so
// no other master is generating bus transactions to that memory.

//! @brief Global context data for the DMA-based SWD.
static swd_dma_context_t s_context __attribute__((section(".dmaram")));

//! @brief Array of DMA descriptors.
//!
//! DMA0's descriptor base address is set to this array, so the first N=22 descriptors are technically
//! allocated for the main descriptor for each channel.
static dma_descriptor_t s_dma_descriptors[FSL_FEATURE_DMA_NUMBER_OF_CHANNELS + 8]
        __ALIGNED(FSL_FEATURE_DMA_DESCRIPTOR_ALIGN_SIZE) __attribute__((section(".dmaram")));

__STATIC_FORCEINLINE uint32_t * restrict swd_dma_append_cycle(uint32_t * restrict pins, uint32_t other_pins);
__STATIC_FORCEINLINE uint32_t * restrict swd_dma_append_cycle(uint32_t * restrict pins, uint32_t bit);
uint32_t * restrict swd_dma_append_request(uint32_t * restrict pins, uint32_t request);
/*static*/ void swd_dma_perform_transfer(void);
/*static*/ uint32_t swd_dma_prepare_read(uint32_t request);
/*static*/ uint32_t swd_dma_prepare_write(uint32_t request, uint32_t data);
/*static*/ void swd_dma_setup(void);

//! @brief Add a SWCLK cycle with SWDIO value to a pin buffer.
#define SW_APPEND_BIT(bit) \
            do {                                                    \
                uint32_t _b = ((bit) & 1U) << PIN_TMS_SWDIO;        \
                *pins++ = _b | PIN_TCK_SWCLK_MASK;                  \
                *pins++ = _b | 0;                                   \
            } while (0)

//! @brief Add a SWCLK cycle with other pin values to a pin buffer.
//!
//! Target samples on the rising edge of the clock (ADIv6 B4.3.1).
uint32_t * restrict swd_dma_append_cycle(uint32_t * restrict pins, uint32_t other_pins)
{
    *pins++ = other_pins | 0; // clock falling edge
    *pins++ = other_pins | PIN_TCK_SWCLK_MASK; // clock rising edge
    return pins;
}

//! @brief Add a SWCLK cycle with SWDIO value to a pin buffer.
uint32_t * restrict swd_dma_append_write(uint32_t * restrict pins, uint32_t bit)
{
    uint32_t b = ((bit) & 1U) << PIN_TMS_SWDIO;
    return swd_dma_append_cycle(pins, b | PIN_TMS_SWDIO_TXEN_MASK);
}

//! @brief DMA descriptor XFERCFG for writing SWDIO.
#define DMA_WRITE_XFERCFG(sz) \
            DMA_CHANNEL_XFERCFG_CFGVALID(1)                                         \
            | DMA_CHANNEL_XFERCFG_RELOAD(0)                                         \
            | DMA_CHANNEL_XFERCFG_SWTRIG(0)                                         \
            | DMA_CHANNEL_XFERCFG_CLRTRIG(1)                                        \
            | DMA_CHANNEL_XFERCFG_SETINTA(1)                                        \
            | DMA_CHANNEL_XFERCFG_SETINTB(0)                                        \
            | DMA_CHANNEL_XFERCFG_WIDTH(2) /* 32-bit width */                       \
            | DMA_CHANNEL_XFERCFG_SRCINC(1) /* 1 x width increment (4 byte) */      \
            | DMA_CHANNEL_XFERCFG_DSTINC(0) /* no increment */                      \
            /* x2 for clock, +1 for final write, -1 for xfercount */                \
            | DMA_CHANNEL_XFERCFG_XFERCOUNT((sz * 2 + 1) - 1)

//! @brief DMA descriptor XFERCFG for reading SWDIO.
#define DMA_READ_XFERCFG(sz) \
            DMA_CHANNEL_XFERCFG_CFGVALID(1)                                         \
            | DMA_CHANNEL_XFERCFG_RELOAD(0)                                         \
            | DMA_CHANNEL_XFERCFG_SWTRIG(0)                                         \
            | DMA_CHANNEL_XFERCFG_CLRTRIG(1)                                        \
            | DMA_CHANNEL_XFERCFG_SETINTA(0)                                        \
            | DMA_CHANNEL_XFERCFG_SETINTB(1)                                        \
            | DMA_CHANNEL_XFERCFG_WIDTH(0) /* 8-bit width */                        \
            | DMA_CHANNEL_XFERCFG_SRCINC(0) /* no increment */                      \
            | DMA_CHANNEL_XFERCFG_DSTINC(1) /* 1 x width increment (1 byte) */      \
            /* x2 for clock, -1 for xfercount */                \
            | DMA_CHANNEL_XFERCFG_XFERCOUNT((sz * 2) - 1)

// Generate SWJ Sequence
//   count:  sequence bit count
//   data:   pointer to sequence bit data
//   return: none
void SWJ_Sequence(uint32_t count, const uint8_t *data)
{
    uint32_t fill_count = count;
    uint8_t val = 0U;
    uint32_t n = 0U;
    uint32_t *pins = &s_context.write_data[0];

    // Reinit peripherals used for DMA.
    swd_dma_setup();

    // Copy SWJ sequence into write buffer while generating the clock signal.
    while (fill_count--) {
        if (n == 0U) {
            val = *data++;
            n = 8U;
        }
        pins = swd_dma_append_write(pins, val);
        val >>= 1;
        n--;
    }

    // Add a final write to return SWCLK and SWDIO to 1.
    *pins = PIN_TCK_SWCLK_MASK | PIN_TMS_SWDIO_MASK | PIN_TMS_SWDIO_TXEN_MASK;

    // Make sure SWDIO is being driven to target.
    PIN_SWDIO_OUT_ENABLE();

    // Make sure the channel isn't already active.
    if (DMA0->COMMON[0].ACTIVE & kDmaWriteChannelMask) {
        DMA0->COMMON[0].ENABLECLR = kDmaWriteChannelMask;
        DMA0->COMMON[0].ABORT = kDmaWriteChannelMask;
    }

    // Configure the DMA transfer.
    s_dma_descriptors[kDmaWriteChannel] = (dma_descriptor_t){
            .srcEndAddr = pins,
            .dstEndAddr = (void *)&GPIO->MPIN[0],
        };
    DMA0->CHANNEL[kDmaWriteChannel].XFERCFG = DMA_WRITE_XFERCFG(count);

    // Invoke transfer and wait for completion.
    swd_dma_perform_transfer();
}

//! @brief Perform a preconfigured DMA transfer.
//!
//! This function does not return until the transfer is completed.
void swd_dma_perform_transfer(void)
{
    uint32_t timeout = MSEC_TO_COUNT(kSwdTimeout_ms, SystemCoreClock) / kCyclesPerPollLoop;

    // Reset timer and make certain the match flag is cleared.
    CTIMER_Reset(CTIMER0);
    CTIMER_ClearStatusFlags(CTIMER0, kCTIMER_Match0Flag);

    // Kick off transfer by enabling the DMA channel and starting the timer triggering.
    DMA0->COMMON[0].ENABLESET = kDmaWriteChannelMask;
    CTIMER_StartTimer(CTIMER0);

    // Wait for transfer to complete.
    uint32_t status;
    do {
        status = DMA0->COMMON[0].INTA;
    } while (((status & kDmaWriteChannelMask) == 0) && --timeout);

    // Clear IRQ.
    DMA0->COMMON[0].INTA = kDmaWriteChannelMask;

    // Disable channel and timers.
    CTIMER_StopTimer(CTIMER0);
    DMA0->COMMON[0].ENABLECLR = kDmaWriteChannelMask;
}

void DMA0_IRQHandler(void)
{
}

/*!
 * Request:     `{Start(1) + APnDP + RnW + A[2:3] + WParity + Stop(0) + Park(1)}`
 *
 * Read:        `{Request + Trn + Ack[0:2] [+ RData[0:31] + RParity] + Trn}`
 *
 * Write:       `{Request + Trn + Ack[0:2] + Trn [+ WData[0:31] + WParity]}`
 *
 * Multi-bit values are read/written LSb first.
 *
 * Normally, an Ack != 3'b001 will abort the data phase (this appears on SWDIO LSb first, as 3'b100). If
 * DAP_Data.swd_conf.data_phase is set, the data phase is always read/written with discarded/dummy data.
 * In addition, a protocol error will trigger 33 idle cycles to account for the data phase in case the
 * target is still expected it.
 *
 * Note that SWDIO sampling on the host happens on the opposite clock edge from sampling done by the
 * target. The target samples on the rising edge *and* drives output on the rising edge. The host both
 * samples and drives on the falling edge.
 *
 * DMA setup:
 * - use GPIO->MASK[0] to select which pins are written by MPORTP
 * - write to GPIO->MPIN[0]
 * - read SWDIO from GPIO->B[0][2]
 * - write data is constructed in a word array -> 2x SWD frequency
 * - read data is written to byte array
 *
 * Two DMA channels are used; one for GPIO writing, and one for GPIO reads. The write channel writes
 * SWCLK to generate the clock signal plus SWDIO and SWDIO_TXEN. The read channel only reads from SWDIO.
 *
 * - read request
 *   - W Desc 0: write Request; CLRTRIG=1
 *   - W Desc 1: write GPIO->DIRCLR to set SWDIO to input; CLRTRIG=0
 *   - W desc 2: write clocks for Ack read
 *   - R Desc 0: read Trn + Ack[0:2], IRQ, pause chain
 *      - if DAP_Data.swd_conf.data_phase, disable IRQ and pause
 *   - Desc 3: read RData[0:31] + Parity, IRQ
 *   - manually write: SWDIO=1, DIRSET to change SWDIO to output
 *
 * - write request
 *   - Desc 0: write Request
 *   - Desc 1: write GPIO->DIRCLR to set SWDIO to input
 *   - Desc 2: read Trn + Ack[0:2] + Trn, IRQ, pause chain
 *      - if DAP_Data.swd_conf.data_phase, disable IRQ and pause
 *   - Desc 1: write GPIO->DIRSET to set SWDIO to output
 *   - Desc 3: write WData[0:31] + Parity, IRQ
 *   - manually write: SWDIO=1
 *
 * @todo
 * - post-transfer idle cycles (DAP_Data.transfer.idle_cycles)
 * - DAP_Data.swd_conf.data_phase
 * - data phase on protocol error
 */
uint8_t SWD_Transfer(uint32_t request, uint32_t *data)
{
    // Reinit peripherals used for DMA.
    swd_dma_setup();

    uint32_t start_timeout = MSEC_TO_COUNT(kSwdTimeout_ms, SystemCoreClock) / kCyclesPerPollLoop;
    uint32_t timeout;
    uint32_t count;
    uint32_t offset;
    uint32_t status;
    uint32_t ack_offset;
    uint32_t data_offset;
    bool is_write = (request & DAP_TRANSFER_RnW) == 0;

    // Clear read data.
    memset(s_context.read_data, 0xff, sizeof(s_context.read_data));

    // Fill in write buffer with clock and any write data.
    if (!is_write) {
        count = swd_dma_prepare_read(request);
    }
    else {
        count = swd_dma_prepare_write(request, *data);
    }

    // Make sure the channels aren't already active.
    if (DMA0->COMMON[0].ACTIVE & (kDmaWriteChannelMask | kDmaReadChannelMask)) {
        DMA0->COMMON[0].ENABLECLR = kDmaWriteChannelMask | kDmaReadChannelMask;
        DMA0->COMMON[0].ABORT = kDmaWriteChannelMask | kDmaReadChannelMask;
    }

    // Configure write channel descriptors.
    count = 2 * kSwdRequestBits;
    offset = count;
    s_dma_descriptors[kDmaWriteChannel] = (dma_descriptor_t){
            .srcEndAddr = s_context.write_data + offset - 1,
            .dstEndAddr = (void *)&GPIO->MPIN[0],
            .linkToNextDesc = &s_dma_descriptors[kFirstFreeDescriptor],
        };
    DMA0->CHANNEL[kDmaWriteChannel].XFERCFG =
            DMA_CHANNEL_XFERCFG_CFGVALID(1)
            | DMA_CHANNEL_XFERCFG_RELOAD(1)
            | DMA_CHANNEL_XFERCFG_CLRTRIG(1)
            | DMA_CHANNEL_XFERCFG_WIDTH(2) // 32-bit width
            | DMA_CHANNEL_XFERCFG_SRCINC(1) // 1 x width increment (4 byte)
            | DMA_CHANNEL_XFERCFG_DSTINC(0) // no increment
            | DMA_CHANNEL_XFERCFG_XFERCOUNT(count - 1); // -1 for xfercount

    // Descriptor to switch SWDIO to input
    s_dma_descriptors[kFirstFreeDescriptor] = (dma_descriptor_t){
            .xfercfg = DMA_CHANNEL_XFERCFG_CFGVALID(1)
                        | DMA_CHANNEL_XFERCFG_RELOAD(1)
                        | DMA_CHANNEL_XFERCFG_CLRTRIG(0)
                        | DMA_CHANNEL_XFERCFG_WIDTH(2) // 32-bit width
                        | DMA_CHANNEL_XFERCFG_SRCINC(0) // no increment
                        | DMA_CHANNEL_XFERCFG_DSTINC(0) // no increment
                        | DMA_CHANNEL_XFERCFG_XFERCOUNT((1) - 1), // -1 for xfercount
            .srcEndAddr = &s_context.dirsetclr_mask,
            .dstEndAddr = (void *)&GPIO->DIRCLR[0],
            .linkToNextDesc = &s_dma_descriptors[kFirstFreeDescriptor + 1],
        };

    // Descriptor to drive clock while reading Ack.
    count = 2 * (DAP_Data.swd_conf.turnaround + kAckBits);
    if (is_write) {
        // Add another turnaround period for writes.
        count += 2 * DAP_Data.swd_conf.turnaround;
    }
    offset += count;
    s_dma_descriptors[kFirstFreeDescriptor + 1] = (dma_descriptor_t){
            .xfercfg = DMA_CHANNEL_XFERCFG_CFGVALID(1)
                        | DMA_CHANNEL_XFERCFG_RELOAD(1)
                        | DMA_CHANNEL_XFERCFG_CLRTRIG(1)
                        | DMA_CHANNEL_XFERCFG_WIDTH(2) // 32-bit width
                        | DMA_CHANNEL_XFERCFG_SRCINC(1) // 1 x width increment (4 byte)
                        | DMA_CHANNEL_XFERCFG_DSTINC(0) // no increment
                        | DMA_CHANNEL_XFERCFG_XFERCOUNT(count - 1), // -1 for xfercount
            .srcEndAddr = s_context.write_data + offset - 1,
            .dstEndAddr = (void *)&GPIO->MPIN[0],
            .linkToNextDesc = &s_dma_descriptors[kFirstFreeDescriptor + (is_write ? 2 : 3)],
        };

    // Descriptor to switch SWDIO to output.
    if (is_write) {
        s_dma_descriptors[kFirstFreeDescriptor + 2] = (dma_descriptor_t){
                .xfercfg = DMA_CHANNEL_XFERCFG_CFGVALID(1)
                            | DMA_CHANNEL_XFERCFG_RELOAD(1)
                            | DMA_CHANNEL_XFERCFG_CLRTRIG(0)
                            | DMA_CHANNEL_XFERCFG_WIDTH(2) // 32-bit width
                            | DMA_CHANNEL_XFERCFG_SRCINC(0) // no increment
                            | DMA_CHANNEL_XFERCFG_DSTINC(0) // no increment
                            | DMA_CHANNEL_XFERCFG_XFERCOUNT((1) - 1), // -1 for xfercount
                .srcEndAddr = &s_context.dirsetclr_mask,
                .dstEndAddr = (void *)&GPIO->DIRSET[0],
                .linkToNextDesc = &s_dma_descriptors[kFirstFreeDescriptor + 3],
            };
    }

    // Descriptor to write data or drive clock while reading data.
    count = 2 * (kDataBits);
    if (!is_write) {
        count += 2 * DAP_Data.swd_conf.turnaround;
    }
    offset += count;
    s_dma_descriptors[kFirstFreeDescriptor + 3] = (dma_descriptor_t){
            .xfercfg = DMA_CHANNEL_XFERCFG_CFGVALID(0)
                        | DMA_CHANNEL_XFERCFG_RELOAD(0)
                        | DMA_CHANNEL_XFERCFG_CLRTRIG(1)
                        | DMA_CHANNEL_XFERCFG_WIDTH(2) // 32-bit width
                        | DMA_CHANNEL_XFERCFG_SRCINC(1) // 1 x width increment (4 byte)
                        | DMA_CHANNEL_XFERCFG_DSTINC(0) // no increment
                        | DMA_CHANNEL_XFERCFG_XFERCOUNT(count - 1), // -1 for xfercount
            .srcEndAddr = s_context.write_data + offset - 1,
            .dstEndAddr = (void *)&GPIO->MPIN[0],
        };

    // Configure read channel descriptors.
    //
    // In order to simplify synchronization of the read and write DMA channels, we go ahead and read during
    // the request phase, even though that's clearly not necessary (the data is discarded). For writes, the
    // read descriptor chain stops after the ack phase. For reads, it obviously must carry through to data
    // plus parity. Optimization may be possible, particularly by using DMA channel output triggers.
    count = 2 * kSwdRequestBits;
    offset = count;
    s_dma_descriptors[kDmaReadChannel] = (dma_descriptor_t){
            .srcEndAddr = (void *)&GPIO->B[0][PIN_TMS_SWDIO],
            .dstEndAddr = s_context.read_data + offset - 1,
            .linkToNextDesc = &s_dma_descriptors[kFirstFreeDescriptor + 4],
        };
    DMA0->CHANNEL[kDmaReadChannel].XFERCFG =
            DMA_CHANNEL_XFERCFG_CFGVALID(1)
            | DMA_CHANNEL_XFERCFG_RELOAD(1)
            | DMA_CHANNEL_XFERCFG_CLRTRIG(1)
            | DMA_CHANNEL_XFERCFG_WIDTH(0) // 8-bit width
            | DMA_CHANNEL_XFERCFG_SRCINC(0) // no increment
            | DMA_CHANNEL_XFERCFG_DSTINC(1) // 1 x width increment (1 byte)
            | DMA_CHANNEL_XFERCFG_XFERCOUNT(count - 1); // -1 for xfercount

    // Descriptor to read Ack.
    count = 2 * (DAP_Data.swd_conf.turnaround + kAckBits);
    if (is_write) {
        // Add another turnaround period for writes.
        count += 2 * DAP_Data.swd_conf.turnaround;
    }
    ack_offset = offset + 2 * DAP_Data.swd_conf.turnaround;
    offset += count;
    s_dma_descriptors[kFirstFreeDescriptor + 4] = (dma_descriptor_t){
            .xfercfg = DMA_CHANNEL_XFERCFG_CFGVALID(1)
                        | DMA_CHANNEL_XFERCFG_RELOAD((uint32_t)(!is_write))
                        | DMA_CHANNEL_XFERCFG_CLRTRIG(1)
                        | DMA_CHANNEL_XFERCFG_SETINTA(1)
                        | DMA_CHANNEL_XFERCFG_WIDTH(0) // 8-bit width
                        | DMA_CHANNEL_XFERCFG_SRCINC(0) // no increment
                        | DMA_CHANNEL_XFERCFG_DSTINC(1) // 1 x width increment (1 byte)
                        | DMA_CHANNEL_XFERCFG_XFERCOUNT(count - 1), // -1 for xfercount
            .srcEndAddr = (void *)&GPIO->B[0][PIN_TMS_SWDIO],
            .dstEndAddr = s_context.read_data + offset - 1,
            .linkToNextDesc = is_write ? NULL : &s_dma_descriptors[kFirstFreeDescriptor + 5],
        };

    if (!is_write) {
        // Descriptor to read data.
        count = 2 * (kDataBits + DAP_Data.swd_conf.turnaround);
        data_offset = offset;
        offset += count;
        s_dma_descriptors[kFirstFreeDescriptor + 5] = (dma_descriptor_t){
                .xfercfg = DMA_CHANNEL_XFERCFG_CFGVALID(0)
                            | DMA_CHANNEL_XFERCFG_RELOAD(0)
                            | DMA_CHANNEL_XFERCFG_CLRTRIG(1)
                            | DMA_CHANNEL_XFERCFG_SETINTA(1)
                            | DMA_CHANNEL_XFERCFG_WIDTH(0) // 8-bit width
                            | DMA_CHANNEL_XFERCFG_SRCINC(0) // no increment
                            | DMA_CHANNEL_XFERCFG_DSTINC(1) // 1 x width increment (1 byte)
                            | DMA_CHANNEL_XFERCFG_XFERCOUNT(count - 1), // -1 for xfercount
                .srcEndAddr = (void *)&GPIO->B[0][PIN_TMS_SWDIO],
                .dstEndAddr = s_context.read_data + offset - 1,
            };
    }

    // Invoke transfer and wait for completion.

    // Make sure SWDIO is being driven to target.
    PIN_SWDIO_OUT_ENABLE();

    // Reset timer and make certain the match flag is cleared.
    CTIMER_Reset(CTIMER0);
    CTIMER_ClearStatusFlags(CTIMER0, kCTIMER_Match0Flag | kCTIMER_Match1Flag);

    // Kick off transfer by enabling the DMA channel and starting the timer triggering.
    DMA0->COMMON[0].ENABLESET = kDmaWriteChannelMask | kDmaReadChannelMask;
    CTIMER_StartTimer(CTIMER0);

    // Wait for transfer to complete.
    timeout = start_timeout;
    do {
        status = DMA0->COMMON[0].INTA;
    } while (((status & kDmaReadChannelMask) == 0) && --timeout);

    // Clear IRQ.
    DMA0->COMMON[0].INTA = kDmaReadChannelMask;

    // Extract Ack.
    uint32_t ack = 0;
    uint8_t * bit_ptr = &s_context.read_data[ack_offset + 1];
    ack = (bit_ptr[0] << 0)
            | (bit_ptr[2] << 1)
            | (bit_ptr[4] << 2);

    if (ack == DAP_TRANSFER_OK) {
        // Start data transfer.
        uint32_t next_dma_mask = is_write ? kDmaWriteChannelMask : (kDmaWriteChannelMask | kDmaReadChannelMask);
        DMA0->COMMON[0].SETVALID = next_dma_mask;

        // Wait for transfer to complete.
        timeout = start_timeout;
        do {
            status = DMA0->COMMON[0].INTA;
        } while (((status & kDmaReadChannelMask) == 0) && --timeout);

        // Clear IRQ.
        DMA0->COMMON[0].INTA = kDmaReadChannelMask;
    }

    // Disable channel and timers.
    CTIMER_StopTimer(CTIMER0);
    DMA0->COMMON[0].ENABLECLR = kDmaWriteChannelMask | kDmaReadChannelMask;

    // Capture timestamp.
    if (request & DAP_TRANSFER_TIMESTAMP) {
      DAP_Data.timestamp = TIMESTAMP_GET();
    }

    // Restore SWDIO value and direction.
    PIN_SWDIO_OUT(1);
    PIN_SWDIO_OUT_ENABLE();

    if (!is_write) {
        // Extract 32-bit word from read data and compute parity.
        uint32_t result = 0;
        uint8_t parity = 0;
        uint32_t i = 0;
        bit_ptr = &s_context.read_data[data_offset + 1];
        for (; i < 32; ++i) {
            uint8_t b = *bit_ptr;
            parity += b;
            result |= b << i;
            bit_ptr += 2;
        }
        *data = result;

        uint8_t received_parity = s_context.read_data[data_offset + 33];
        if ((parity & 1) != received_parity) {
            ack = DAP_TRANSFER_ERROR;
        }
    }

    return ack;
}

uint32_t * restrict swd_dma_append_request(uint32_t * restrict pins, uint32_t request)
{
    uint8_t parity;
    uint8_t bit0;
    uint8_t bit1;
    uint8_t bit2;
    uint8_t bit3;

    // Packet Request: [Start(1), APnDP, RnW, A2, A3, Parity, Stop(0), Park(1)]

    // Compute parity from request (APnDP, RnW, A2, A3).
    bit0 = request >> 0;
    bit1 = request >> 1;
    bit2 = request >> 2;
    bit3 = request >> 3;
    parity = bit0 + bit1 + bit2 + bit3;

    pins = swd_dma_append_write(pins, 1U);      // Start Bit
    pins = swd_dma_append_write(pins, bit0);    // APnDP Bit
    pins = swd_dma_append_write(pins, bit1);    // RnW Bit
    pins = swd_dma_append_write(pins, bit2);    // A2 Bit
    pins = swd_dma_append_write(pins, bit3);    // A3 Bit
    pins = swd_dma_append_write(pins, parity);  // Parity Bit
    pins = swd_dma_append_write(pins, 0U);      // Stop Bit
    pins = swd_dma_append_write(pins, 1U);      // Park Bit

    return pins;
}

uint32_t swd_dma_prepare_read(uint32_t request)
{
    uint32_t i = 0;
    uint32_t *pins = &s_context.write_data[0];

    // Request
    pins = swd_dma_append_request(pins, request);

    // FIXME: memcpy of clock sequence would be more efficient here
    // Turnaround
    for (i = DAP_Data.swd_conf.turnaround; i; i--) {
        pins = swd_dma_append_cycle(pins, 0U);
    }

    // Ack
    for (i = kAckBits; i; i--) {
        pins = swd_dma_append_cycle(pins, 0U);
    }

    // 32 data bits plus parity
    for (i = kDataBits; i; i--) {
        pins = swd_dma_append_cycle(pins, 0U);
    }

    // Add a final write to return SWCLK and SWDIO to 1.
    *pins++ = PIN_TCK_SWCLK_MASK | PIN_TMS_SWDIO_MASK | PIN_TMS_SWDIO_TXEN_MASK;

    return kSwdRequestBits + DAP_Data.swd_conf.turnaround + kAckBits + kDataBits + 1;
}

uint32_t swd_dma_prepare_write(uint32_t request, uint32_t data)
{
    uint32_t i = 0;
    uint32_t *pins = &s_context.write_data[0];

    // Request
    pins = swd_dma_append_request(pins, request);

    // Turnaround
    for (i = DAP_Data.swd_conf.turnaround; i; i--) {
        pins = swd_dma_append_cycle(pins, 0U);
    }

    // Ack
    for (i = kAckBits; i; i--) {
        pins = swd_dma_append_cycle(pins, 0U);
    }

    // Turnaround
    for (i = DAP_Data.swd_conf.turnaround; i; i--) {
        pins = swd_dma_append_cycle(pins, 0U);
    }

    // 32 data bits plus parity
    uint32_t parity = 0;
    for (i = kDataBits - 1; i; i--) {
        parity += data;
        pins = swd_dma_append_write(pins, data);
        data >>= 1;
    }
    pins = swd_dma_append_write(pins, parity);

    // Add a final write to return SWCLK and SWDIO to 1.
    *pins++ = PIN_TCK_SWCLK_MASK | PIN_TMS_SWDIO_MASK | PIN_TMS_SWDIO_TXEN_MASK;

    return kSwdRequestBits + 2 * DAP_Data.swd_conf.turnaround + kAckBits + kDataBits + 1;
}

//! @brief Update timer match value based on requested SWD frequency.
//!
//! The timer is configured to generate DMA triggers at twice the SWD frequency, so
//! we can generate both the rising and falling edges of the clock signal.
void swd_dma_update_clock(void) {
    if (s_context.last_clock != DAP_Data.nominal_clock) {
        uint32_t count = SystemCoreClock / (DAP_Data.nominal_clock * 2);
        CTIMER0->MR[0] = count;
        CTIMER0->MR[1] = count;

        s_context.last_clock = DAP_Data.nominal_clock;
    }
}

void swd_dma_setup(void)
{
    // Configure timer.
    // All zeros is timer mode with 0 prescaler.
    ctimer_config_t timer_config = {0};
    CTIMER_Init(CTIMER0, &timer_config);

    ctimer_match_config_t match_config = {
            .matchValue = 0,
            .enableCounterReset = true,
            .enableCounterStop = false,
            .outControl = kCTIMER_Output_NoAction,
            .outPinInitState = false,
            .enableInterrupt = false,
        };
    CTIMER_SetupMatch(CTIMER0, kCTIMER_Match_0, &match_config);
    match_config.enableCounterReset = false;
    CTIMER_SetupMatch(CTIMER0, kCTIMER_Match_1, &match_config);

    s_context.last_clock = 0;
    swd_dma_update_clock();

    // Init inputmux.
    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, kDmaWriteChannel, kINPUTMUX_Ctimer0M0ToDma0);
    INPUTMUX_AttachSignal(INPUTMUX, kDmaReadChannel, kINPUTMUX_Ctimer0M1ToDma0);

    // Init DMA0.
    RESET_PeripheralReset(kDMA0_RST_SHIFT_RSTn);
    DMA0->SRAMBASE = (uint32_t)&s_dma_descriptors;
    DMA0->CTRL |= DMA_CTRL_ENABLE_MASK;

    DMA0->CHANNEL[kDmaWriteChannel].CFG = DMA_CHANNEL_CFG_PERIPHREQEN(0)
                                            | DMA_CHANNEL_CFG_HWTRIGEN(1)
                                            | DMA_CHANNEL_CFG_TRIGPOL(1) // rising edge
                                            | DMA_CHANNEL_CFG_TRIGTYPE(0) // edge triggered
                                            | DMA_CHANNEL_CFG_TRIGBURST(1) // each trigger transfers a burst
                                            | DMA_CHANNEL_CFG_BURSTPOWER(0) // burst size is 1 transfer
                                            | DMA_CHANNEL_CFG_SRCBURSTWRAP(0)
                                            | DMA_CHANNEL_CFG_DSTBURSTWRAP(0)
                                            | DMA_CHANNEL_CFG_CHPRIORITY(0); // 0 is highest priority

    DMA0->CHANNEL[kDmaReadChannel].CFG = DMA_CHANNEL_CFG_PERIPHREQEN(0)
                                            | DMA_CHANNEL_CFG_HWTRIGEN(1)
                                            | DMA_CHANNEL_CFG_TRIGPOL(1) // rising edge
                                            | DMA_CHANNEL_CFG_TRIGTYPE(0) // edge triggered
                                            | DMA_CHANNEL_CFG_TRIGBURST(1) // each trigger transfers a burst
                                            | DMA_CHANNEL_CFG_BURSTPOWER(0) // burst size is 1 transfer
                                            | DMA_CHANNEL_CFG_SRCBURSTWRAP(0)
                                            | DMA_CHANNEL_CFG_DSTBURSTWRAP(0)
                                            | DMA_CHANNEL_CFG_CHPRIORITY(1); // 0 is highest priority
}

//! Initialize context and peripherals.
void swd_dma_init(void)
{
    // Clear context and descriptors.
    memset(&s_context, 0, sizeof(s_context));
    memset(s_dma_descriptors, 0, sizeof(s_dma_descriptors));

    // Init context members.
    s_context.dirsetclr_mask = PIN_TMS_SWDIO_MASK;

    // Setup clocks.
    CLOCK_AttachClk(kMAIN_CLK_to_CTIMER0);
    CLOCK_EnableClock(kCLOCK_Dma0);

    // Set up pin mask.
    GPIO->MASK[0] = ~(PIN_TCK_SWCLK_MASK | PIN_TMS_SWDIO_MASK | PIN_TMS_SWDIO_TXEN_MASK);
}

