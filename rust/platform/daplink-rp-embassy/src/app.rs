use embassy_executor::{InterruptExecutor, SpawnError, SpawnToken, Spawner};
use embassy_rp::interrupt;
use embassy_rp::interrupt::{InterruptExt, Priority};
use embassy_usb::class::cdc_acm::{ControlChanged, Receiver, Sender};
use embassy_usb::UsbDevice;

use daplink_embassy_usb::{
    dap_reader_task, dap_sessions as run_dap_sessions, dap_writer_task, DapExecHalf, DapPipeline,
    DapReadHalf, DapV2Class, DapV2Reader, DapV2Writer, DapWriteHalf,
};
use daplink_rs::daplink_core::DEFAULT_VCOM_BAUD;

use crate::vcom::vcom_uart;
use crate::{reboot_watch, usb_task, vcom_task, UsbDrv, VcomIo};

/// Per-image queue state, `DEPTH` packets of `SLOT` bytes, reset at every USB session boundary.
pub struct AppState<const DEPTH: usize, const SLOT: usize> {
    pipeline: DapPipeline<DEPTH, SLOT>,
}

impl<const DEPTH: usize, const SLOT: usize> AppState<DEPTH, SLOT> {
    /// Create empty application state.
    pub const fn new() -> Self {
        Self {
            pipeline: DapPipeline::new(),
        }
    }

    /// The queue this image's sessions run on.
    pub fn pipeline(&mut self) -> &mut DapPipeline<DEPTH, SLOT> {
        &mut self.pipeline
    }

    /// The three pipeline sides, run at two priorities by [`dap_sessions`].
    fn split(
        &mut self,
    ) -> (
        DapReadHalf<'_, DEPTH, SLOT>,
        DapExecHalf<'_, DEPTH, SLOT>,
        DapWriteHalf<'_, DEPTH, SLOT>,
    ) {
        self.pipeline.split()
    }
}

impl<const DEPTH: usize, const SLOT: usize> Default for AppState<DEPTH, SLOT> {
    fn default() -> Self {
        Self::new()
    }
}

/// Spawn the USB, reboot, and VCOM services common to RP probe applications.
pub fn spawn_probe_services(
    spawner: Spawner,
    usb: UsbDevice<'static, UsbDrv>,
    cdc_tx: Sender<'static, UsbDrv>,
    cdc_rx: Receiver<'static, UsbDrv>,
    control: ControlChanged<'static>,
    vcom_io: VcomIo,
    led: &'static mut dyn daplink_rs::Activity,
) {
    spawner.spawn(usb_task(usb).expect("usb_task spawns once"));
    spawner.spawn(reboot_watch().expect("reboot_watch spawns once"));
    spawner.spawn(
        vcom_task(
            cdc_tx,
            cdc_rx,
            control,
            vcom_uart(vcom_io),
            DEFAULT_VCOM_BAUD,
            led,
        )
        .expect("vcom_task spawns once"),
    );
}

/// Runs the two DAP endpoint tasks above thread mode, on a software interrupt below USB, timer and DMA.
static DAP_EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn SWI_IRQ_0() {
    // SAFETY: the executor's own vector, started in `dap_sessions` before the line is enabled.
    unsafe { DAP_EXECUTOR.on_interrupt() }
}

/// The endpoint tasks for a queue shape; implemented for 8 packets of 64 or 512 bytes.
pub trait DapShape<const DEPTH: usize, const SLOT: usize> {
    /// The OUT endpoint task.
    fn read_task(
        half: DapReadHalf<'static, DEPTH, SLOT>,
        reader: DapV2Reader<UsbDrv>,
    ) -> Result<SpawnToken<impl Send>, SpawnError>;
    /// The IN endpoint task.
    fn write_task(
        half: DapWriteHalf<'static, DEPTH, SLOT>,
        writer: DapV2Writer<UsbDrv>,
    ) -> Result<SpawnToken<impl Send>, SpawnError>;
}

/// The queue shapes [`dap_sessions`] runs.
pub struct Shapes;

macro_rules! endpoint_tasks {
    ($depth:literal, $slot:literal) => {
        /// Empty the OUT endpoint and re-arm it, whatever thread mode is doing.
        #[embassy_executor::task]
        async fn dap_read_task(
            half: DapReadHalf<'static, $depth, $slot>,
            reader: DapV2Reader<UsbDrv>,
        ) -> ! {
            dap_reader_task(half, reader).await
        }

        /// Refill the IN endpoint the moment the host drains it.
        #[embassy_executor::task]
        async fn dap_write_task(
            half: DapWriteHalf<'static, $depth, $slot>,
            writer: DapV2Writer<UsbDrv>,
        ) -> ! {
            dap_writer_task(half, writer).await
        }

        impl DapShape<$depth, $slot> for Shapes {
            #[inline(always)]
            fn read_task(
                half: DapReadHalf<'static, $depth, $slot>,
                reader: DapV2Reader<UsbDrv>,
            ) -> Result<SpawnToken<impl Send>, SpawnError> {
                dap_read_task(half, reader)
            }

            #[inline(always)]
            fn write_task(
                half: DapWriteHalf<'static, $depth, $slot>,
                writer: DapV2Writer<UsbDrv>,
            ) -> Result<SpawnToken<impl Send>, SpawnError> {
                dap_write_task(half, writer)
            }
        }
    };
}

endpoint_tasks!(8, 64);

mod packet_512 {
    use super::*;
    endpoint_tasks!(8, 512);
}

/// Run RP DAP sessions: endpoints served from the `SWI_IRQ_0` executor, commands in thread mode.
pub async fn dap_sessions<const COUNT: bool, const DEPTH: usize, const SLOT: usize, C, E, S, F>(
    class: DapV2Class<UsbDrv>,
    state: &'static mut AppState<DEPTH, SLOT>,
    context: &mut C,
    execute: E,
    suspend: S,
    finished: F,
) -> !
where
    Shapes: DapShape<DEPTH, SLOT>,
    E: FnMut(&mut C, &[u8], &mut [u8]) -> usize,
    S: FnMut(&mut C),
    F: FnMut(&mut C, u32),
{
    // The priority must be set before the executor enables the line.
    interrupt::SWI_IRQ_0.set_priority(Priority::P3);
    let spawner = DAP_EXECUTOR.start(interrupt::SWI_IRQ_0);
    let (reader, writer) = class.split();
    let (read_half, mut exec_half, write_half) = state.split();
    spawner.spawn(Shapes::read_task(read_half, reader).expect("dap_read_task spawns once"));
    spawner.spawn(Shapes::write_task(write_half, writer).expect("dap_write_task spawns once"));
    run_dap_sessions::<COUNT, _, _, _, _, _>(&mut exec_half, context, execute, suspend, finished)
        .await
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn session_start_clears_queued_packets() {
        let mut state = AppState::<8, 64>::new();
        let queue = state.pipeline().queue();
        queue.submit(&[0x11]).expect("queue has room");
        queue.post_response(&[0x22]).expect("queue has room");
        let generation = queue.generation();

        // What `PipelineSession::begin_session` does at every boundary.
        state.pipeline().reset();
        let reset = state.pipeline().queue();
        assert_eq!(reset.execute_next_with(|_, _| 0), Ok(None));
        assert_eq!(reset.pending_response(), None);
        assert_ne!(reset.generation(), generation);
    }
}
