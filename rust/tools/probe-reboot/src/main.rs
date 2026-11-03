//! `probe-reboot` -- enter a probe's ROM bootloader via the `0x9F` vendor command.

mod proto;

use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Context, Result};
use clap::Parser;
use nusb::descriptors::TransferType;
use nusb::transfer::{Buffer, Bulk, Direction, In, Out};
use nusb::{DeviceInfo, Endpoint, MaybeFuture};

use daplink_core::{DAPLINK_PID, DAPLINK_VID};

/// Product-string marker, matched as a substring: the board name follows it.
const PRODUCT_MARKER: &str = "CMSIS-DAP";
/// `bInterfaceClass` for a vendor-specific interface (what CMSIS-DAP v2 advertises).
const VENDOR_SPECIFIC_CLASS: u8 = 0xFF;
/// Timeout for a single bulk transfer.
const TRANSFER_TIMEOUT: Duration = Duration::from_secs(2);
/// How long to wait for the probe to detach after a reboot.
const DETACH_TIMEOUT: Duration = Duration::from_secs(10);
/// Reply-size hint; reads round it up to a whole number of max-size packets.
const RESPONSE_BUFFER_HINT: usize = 64;

/// Put a daplink-rs probe into its ROM bootloader for firmware self-update.
#[derive(Parser, Debug)]
#[command(name = "probe-reboot", version, about)]
struct Cli {
    /// Serial number of the probe to use, when more than one is attached.
    #[arg(long)]
    serial: Option<String>,
    /// Arm the ROM bootloader entry after discovery (default: discovery only).
    #[arg(long)]
    reboot: bool,
}

fn main() -> Result<()> {
    let cli = Cli::parse();

    let info = find_probe(cli.serial.as_deref())?;
    println!(
        "found daplink-rs probe {:04x}:{:04x}{}",
        info.vendor_id(),
        info.product_id(),
        info.serial_number()
            .map(|s| format!(" serial {s}"))
            .unwrap_or_default()
    );

    let device = info.open().wait().context("opening the probe")?;
    let (iface_num, ep_out, ep_in) = find_dap_v2_interface(&device)?;
    let interface = device
        .claim_interface(iface_num)
        .wait()
        .context("claiming the CMSIS-DAP v2 interface")?;
    let mut out = interface
        .endpoint::<Bulk, Out>(ep_out)
        .context("opening bulk OUT endpoint")?;
    let mut in_ = interface
        .endpoint::<Bulk, In>(ep_in)
        .context("opening bulk IN endpoint")?;

    let reply =
        xfer(&mut out, &mut in_, &proto::discovery_cmd()).context("discovery transfer failed")?;
    let disc = proto::parse_discovery(&reply)?;
    println!(
        "vendor protocol v{}, bootloader: {}",
        disc.version,
        disc.kind.label()
    );

    if !cli.reboot {
        return Ok(());
    }

    require_reboot(disc.kind)?;

    let request = reboot_transaction(
        || submit(&mut out, &proto::reboot_cmd()),
        || receive(&mut in_),
    );
    finish_reboot(
        request,
        || {
            drop(out);
            drop(in_);
            drop(interface);
            drop(device);
        },
        || wait_for_detach(&info, DETACH_TIMEOUT),
    )?;
    println!(
        "reboot armed and probe detached; {}",
        disc.kind.followup_hint()
    );
    Ok(())
}

enum RebootRequest {
    SubmissionFailed(anyhow::Error),
    Armed,
    Refused(proto::Status),
    Unconfirmed(anyhow::Error),
}

fn require_reboot(kind: proto::Kind) -> Result<()> {
    if !kind.supports_reboot() {
        bail!("reboot unsupported: {}", kind.followup_hint());
    }
    Ok(())
}

fn reboot_transaction(
    submit: impl FnOnce() -> Result<()>,
    receive: impl FnOnce() -> Result<Vec<u8>>,
) -> RebootRequest {
    if let Err(error) = submit() {
        return RebootRequest::SubmissionFailed(error);
    }
    match receive().and_then(|reply| proto::parse_status(&reply)) {
        Ok(proto::Status::Armed) => RebootRequest::Armed,
        Ok(proto::Status::Unknown(value)) => {
            RebootRequest::Unconfirmed(anyhow!("unknown acknowledgement status {value:#04x}"))
        }
        Ok(status) => RebootRequest::Refused(status),
        Err(error) => RebootRequest::Unconfirmed(error),
    }
}

fn finish_reboot(
    request: RebootRequest,
    release: impl FnOnce(),
    poll: impl FnOnce() -> Result<bool>,
) -> Result<()> {
    // Endpoint and interface handles must be released before enumeration polling.
    release();
    match request {
        RebootRequest::SubmissionFailed(error) => {
            Err(error.context("reboot OUT submission failed"))
        }
        RebootRequest::Refused(status) => bail!("reboot refused: {}", status.label()),
        RebootRequest::Armed => {
            if poll().context("reboot armed; detach observation failed")? {
                Ok(())
            } else {
                bail!("reboot armed, but probe still present after {DETACH_TIMEOUT:?}");
            }
        }
        RebootRequest::Unconfirmed(error) => {
            let detached = poll().with_context(|| {
                format!("UNCONFIRMED reboot ({error:#}); detach observation failed")
            })?;
            Err(error.context(if detached {
                "UNCONFIRMED reboot: probe detached without affirmative arming; verify the expected post-reboot identity before recovery"
            } else {
                "UNCONFIRMED reboot: no affirmative arming and probe still present"
            }))
        }
    }
}

/// Find a daplink-rs probe by DAPLink VID/PID and product marker, optionally by serial.
fn find_probe(serial: Option<&str>) -> Result<DeviceInfo> {
    let mut matches = nusb::list_devices()
        .wait()
        .context("enumerating USB devices")?
        .filter(|d| d.vendor_id() == DAPLINK_VID && d.product_id() == DAPLINK_PID)
        .filter(|d| product_matches(d.product_string()))
        .filter(|d| match serial {
            Some(want) => d.serial_number() == Some(want),
            None => true,
        });

    let first = matches.next().context(
        "no daplink-rs probe found (looked for a CMSIS-DAP device at DAPLink 0d28:0204)",
    )?;
    if serial.is_none() && matches.next().is_some() {
        bail!("multiple probes found; pass --serial <SERIAL> to pick one");
    }
    Ok(first)
}

fn product_matches(product: Option<&str>) -> bool {
    product.is_some_and(|p| p.contains(PRODUCT_MARKER))
}

/// Locate the CMSIS-DAP v2 vendor interface and its bulk endpoint addresses.
fn find_dap_v2_interface(device: &nusb::Device) -> Result<(u8, u8, u8)> {
    let config = device
        .active_configuration()
        .context("reading the active configuration")?;
    for group in config.interfaces() {
        let alt = group.first_alt_setting();
        if alt.class() != VENDOR_SPECIFIC_CLASS {
            continue;
        }
        let mut ep_out = None;
        let mut ep_in = None;
        for ep in alt.endpoints() {
            if ep.transfer_type() != TransferType::Bulk {
                continue;
            }
            match ep.direction() {
                Direction::Out if ep_out.is_none() => ep_out = Some(ep.address()),
                Direction::In if ep_in.is_none() => ep_in = Some(ep.address()),
                _ => {}
            }
        }
        if let (Some(out), Some(in_)) = (ep_out, ep_in) {
            return Ok((alt.interface_number(), out, in_));
        }
    }
    bail!("no CMSIS-DAP v2 bulk interface found on the probe")
}

fn xfer(
    out: &mut Endpoint<Bulk, Out>,
    in_: &mut Endpoint<Bulk, In>,
    cmd: &[u8],
) -> Result<Vec<u8>> {
    submit(out, cmd)?;
    receive(in_)
}

fn submit(out: &mut Endpoint<Bulk, Out>, cmd: &[u8]) -> Result<()> {
    let completion = out.transfer_blocking(Buffer::from(cmd), TRANSFER_TIMEOUT);
    completion.status.context("bulk OUT")?;
    check_submission_length(completion.actual_len, cmd.len())
}

fn check_submission_length(actual: usize, expected: usize) -> Result<()> {
    if actual != expected {
        bail!("bulk OUT incomplete: submitted {actual} of {expected} bytes");
    }
    Ok(())
}

fn receive(in_: &mut Endpoint<Bulk, In>) -> Result<Vec<u8>> {
    let read_len = round_up(RESPONSE_BUFFER_HINT, in_.max_packet_size().max(1));
    let reply = in_
        .transfer_blocking(Buffer::new(read_len), TRANSFER_TIMEOUT)
        .into_result()
        .context("bulk IN")?;
    Ok(reply.into_vec())
}

fn round_up(len: usize, granularity: usize) -> usize {
    if granularity == 0 {
        return len;
    }
    len.div_ceil(granularity) * granularity
}

/// Poll until the probe is no longer enumerated, returning `true` on a clean detach.
fn wait_for_detach(info: &DeviceInfo, timeout: Duration) -> Result<bool> {
    let deadline = Instant::now() + timeout;
    loop {
        let still_present = nusb::list_devices()
            .wait()
            .context("enumerating USB devices")?
            .any(|d| {
                d.vendor_id() == info.vendor_id()
                    && d.product_id() == info.product_id()
                    && d.serial_number() == info.serial_number()
            });
        if !still_present {
            return Ok(true);
        }
        if Instant::now() >= deadline {
            return Ok(false);
        }
        std::thread::sleep(Duration::from_millis(200));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;

    fn transaction(
        submission_ok: bool,
        reply: Result<Vec<u8>>,
        detached: Result<bool>,
    ) -> (Result<()>, Vec<&'static str>) {
        let calls = RefCell::new(Vec::new());
        let request = reboot_transaction(
            || {
                calls.borrow_mut().push("submit");
                if submission_ok {
                    Ok(())
                } else {
                    bail!("OUT fixture")
                }
            },
            || {
                calls.borrow_mut().push("receive");
                reply
            },
        );
        let result = finish_reboot(
            request,
            || calls.borrow_mut().push("release"),
            || {
                calls.borrow_mut().push("poll");
                detached
            },
        );
        (result, calls.into_inner())
    }

    #[test]
    fn out_failure_never_reads_or_polls_and_releases_once() {
        let (result, calls) = transaction(false, Ok(vec![0x9F, 0]), Ok(true));
        assert_eq!(calls, ["submit", "release"]);
        let message = format!("{:#}", result.unwrap_err());
        assert!(message.contains("OUT submission failed"));
        assert!(message.contains("OUT fixture"));
    }

    #[test]
    fn partial_out_is_not_submission_success() {
        for actual in 0..proto::reboot_cmd().len() {
            let request = reboot_transaction(
                || check_submission_length(actual, proto::reboot_cmd().len()),
                || panic!("partial OUT must not read"),
            );
            assert!(finish_reboot(request, || {}, || panic!("partial OUT must not poll")).is_err());
        }
        assert!(check_submission_length(6, 6).is_ok());
    }

    #[test]
    fn unsupported_and_unknown_kinds_fail_requested_reboot() {
        assert!(require_reboot(proto::Kind::None).is_err());
        assert!(require_reboot(proto::Kind::Unknown(127)).is_err());
        for kind in [proto::Kind::Uf2, proto::Kind::StDfu, proto::Kind::NxpIsp] {
            assert!(require_reboot(kind).is_ok());
        }
    }

    #[test]
    fn armed_requires_detach_and_releases_before_polling() {
        for detached in [true, false] {
            let (result, calls) = transaction(true, Ok(vec![0x9F, 0]), Ok(detached));
            assert_eq!(calls, ["submit", "receive", "release", "poll"]);
            assert_eq!(result.is_ok(), detached);
        }
    }

    #[test]
    fn explicit_refusals_never_poll_or_succeed() {
        for status in [1, 2, 3] {
            let (result, calls) = transaction(true, Ok(vec![0x9F, status]), Ok(true));
            assert_eq!(calls, ["submit", "receive", "release"]);
            assert!(format!("{:#}", result.unwrap_err()).contains("reboot refused"));
        }
    }

    #[test]
    fn missing_malformed_and_unknown_acknowledgements_remain_unconfirmed() {
        for detached in [true, false] {
            for reply in [
                Err(anyhow!("IN fixture")),
                Ok(vec![]),
                Ok(vec![0x9F]),
                Ok(vec![0xFF, 0]),
                Ok(vec![0x9F, 127]),
            ] {
                let (result, calls) = transaction(true, reply, Ok(detached));
                assert_eq!(calls, ["submit", "receive", "release", "poll"]);
                let message = format!("{:#}", result.unwrap_err());
                assert!(message.contains("UNCONFIRMED"));
                assert!(message.contains(if detached {
                    "detached without affirmative"
                } else {
                    "still present"
                }));
            }
        }
    }

    #[test]
    fn enumeration_failure_is_not_detach() {
        for reply in [Ok(vec![0x9F, 0]), Err(anyhow!("IN fixture"))] {
            let (result, calls) = transaction(true, reply, Err(anyhow!("enumeration fixture")));
            assert_eq!(calls, ["submit", "receive", "release", "poll"]);
            let message = format!("{:#}", result.unwrap_err());
            assert!(message.contains("detach observation failed"));
            assert!(message.contains("enumeration fixture"));
        }
    }

    #[test]
    fn matches_fleet_product_strings() {
        assert!(product_matches(Some("daplink-rs CMSIS-DAP (rp2040)")));
        assert!(product_matches(Some("daplink-rs CMSIS-DAP (stm32f103)")));
    }

    #[test]
    fn matches_bare_marker() {
        assert!(product_matches(Some("CMSIS-DAP")));
    }

    #[test]
    fn rejects_foreign_product_string() {
        assert!(!product_matches(Some("Bus Pirate 6")));
        assert!(!product_matches(Some("STLink-V3")));
    }

    #[test]
    fn rejects_missing_product_string() {
        assert!(!product_matches(None));
    }
}
