//! `probe-reboot` -- enter a probe's ROM bootloader via the `0x9F` vendor command.

mod proto;

use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use clap::Parser;
use nusb::descriptors::TransferType;
use nusb::transfer::{Buffer, Bulk, Direction, In, Out};
use nusb::{DeviceInfo, Endpoint, MaybeFuture};

use daplink_core::{DAPLINK_PID, DAPLINK_VID};

/// Product-string marker every daplink-rs probe advertises. `daplink-core` documents it as a
/// substring, not a prefix: the board name follows it (`daplink-rs CMSIS-DAP (rp2040)`).
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

    // Discovery: confirm identity and learn the bootloader kind.
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

    if !disc.kind.supports_reboot() {
        println!("reboot skipped: {}", disc.kind.followup_hint());
        return Ok(());
    }

    // The probe detaches shortly after answering, so a failed read here is success too.
    match xfer(&mut out, &mut in_, &proto::reboot_cmd()) {
        Ok(reply) => match proto::parse_status(&reply) {
            Ok(status) => println!("reboot: {}", status.label()),
            Err(e) => println!("reboot: sent (unparsable status: {e})"),
        },
        Err(_) => println!("reboot: sent (probe detached before acknowledging)"),
    }

    // Drop the endpoints/interface/device so the OS can re-enumerate cleanly.
    drop(out);
    drop(in_);
    drop(interface);
    drop(device);

    if wait_for_detach(&info, DETACH_TIMEOUT)? {
        println!("probe detached; {}", disc.kind.followup_hint());
    } else {
        println!("warning: probe still present after {DETACH_TIMEOUT:?}");
    }
    Ok(())
}

/// Find a daplink-rs probe by DAPLink VID/PID and the `CMSIS-DAP` product-string marker,
/// optionally filtered by serial.
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

/// Whether a USB product string identifies a daplink-rs probe: the `CMSIS-DAP` marker may appear
/// anywhere in it. [`find_probe`] has already restricted enumeration to DAPLink's VID/PID.
fn product_matches(product: Option<&str>) -> bool {
    product.is_some_and(|p| p.contains(PRODUCT_MARKER))
}

/// Locate the CMSIS-DAP v2 interface (a vendor-specific interface with a bulk IN + bulk OUT
/// pair) and its endpoint addresses.
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

/// Send a command on the bulk OUT endpoint and read the reply from the bulk IN endpoint.
fn xfer(
    out: &mut Endpoint<Bulk, Out>,
    in_: &mut Endpoint<Bulk, In>,
    cmd: &[u8],
) -> Result<Vec<u8>> {
    out.transfer_blocking(Buffer::from(cmd), TRANSFER_TIMEOUT)
        .into_result()
        .context("bulk OUT")?;
    // Bulk IN reads must be a whole number of the endpoint's max packet size.
    let read_len = round_up(RESPONSE_BUFFER_HINT, in_.max_packet_size().max(1));
    let reply = in_
        .transfer_blocking(Buffer::new(read_len), TRANSFER_TIMEOUT)
        .into_result()
        .context("bulk IN")?;
    Ok(reply.into_vec())
}

/// Round `len` up to the next multiple of `granularity`.
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
    use super::{product_matches, round_up};

    #[test]
    fn round_up_keeps_a_full_speed_read_at_one_packet() {
        assert_eq!(round_up(64, 64), 64);
    }

    #[test]
    fn round_up_pads_to_a_whole_high_speed_packet() {
        assert_eq!(round_up(64, 512), 512);
    }

    #[test]
    fn round_up_is_a_no_op_at_zero_granularity() {
        assert_eq!(round_up(64, 0), 64);
    }

    /// `CMSIS-DAP` sits in the middle of a real product string, where a prefix match misses it.
    #[test]
    fn matches_fleet_product_strings() {
        assert!(product_matches(Some("daplink-rs CMSIS-DAP (rp2040)")));
        assert!(product_matches(Some("daplink-rs CMSIS-DAP (stm32f103)")));
    }

    /// A bare `"CMSIS-DAP"` still contains the marker, so it matches under the substring rule.
    #[test]
    fn matches_bare_marker() {
        assert!(product_matches(Some("CMSIS-DAP")));
    }

    /// Something clearly foreign -- no marker at all -- must not match.
    #[test]
    fn rejects_foreign_product_string() {
        assert!(!product_matches(Some("Bus Pirate 6")));
        assert!(!product_matches(Some("STLink-V3")));
    }

    /// A device with no readable product string descriptor must not match.
    #[test]
    fn rejects_missing_product_string() {
        assert!(!product_matches(None));
    }
}
