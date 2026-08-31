//! Target-identity probes: a positive fingerprint of the silicon over plain memory reads.

use crate::error::Error;

/// One positive identity check: `read(addr) & mask == expected` on the genuine device.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IdentityProbe {
    /// Address of the identity word to read (a chip-ID register, a bootrom magic word, ...).
    pub addr: u32,
    /// Mask applied to the read word before comparison (isolates the significant field).
    pub mask: u32,
    /// Value the masked word must equal on the genuine device.
    pub expected: u32,
}

impl IdentityProbe {
    /// Construct a probe from its address, mask and expected (masked) value.
    pub const fn new(addr: u32, mask: u32, expected: u32) -> Self {
        Self {
            addr,
            mask,
            expected,
        }
    }

    /// Whether `read` (the raw word at [`addr`](Self::addr)) matches this probe.
    pub const fn matches(&self, read: u32) -> bool {
        read & self.mask == self.expected
    }
}

/// Run every probe in `probes` via the injected `read_word`. `Ok(())` if all match (or `probes`
/// is empty); on the first mismatch, [`Error::IdentityMismatch`].
pub fn check_identity<F>(probes: &[IdentityProbe], mut read_word: F) -> Result<(), Error>
where
    F: FnMut(u32) -> Result<u32, Error>,
{
    for probe in probes {
        let read = read_word(probe.addr)?;
        if !probe.matches(read) {
            return Err(Error::IdentityMismatch {
                addr: probe.addr,
                expected: probe.expected,
                read: read & probe.mask,
            });
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;
    use std::vec;
    use std::vec::Vec;

    #[test]
    fn empty_probe_list_is_ok_and_reads_nothing() {
        let mut reads: Vec<u32> = Vec::new();
        let r = check_identity(&[], |a| {
            reads.push(a);
            Ok(0)
        });
        assert!(r.is_ok());
        assert!(reads.is_empty(), "an empty descriptor performs no reads");
    }

    #[test]
    fn all_probes_match() {
        let probes = [
            IdentityProbe::new(0x0000_0010, 0x00FF_FFFF, 0x0001_754D),
            IdentityProbe::new(0xE004_2000, 0x0000_0FFF, 0x0000_0410),
        ];
        let r = check_identity(&probes, |a| match a {
            0x0000_0010 => Ok(0xAB01_754D),
            0xE004_2000 => Ok(0x1000_0410),
            other => panic!("unexpected read {other:#x}"),
        });
        assert!(r.is_ok());
    }

    #[test]
    fn first_mismatch_aborts_with_details() {
        let probes = [
            IdentityProbe::new(0x0000_0010, 0x00FF_FFFF, 0x0001_754D),
            IdentityProbe::new(0xE004_2000, 0x0000_0FFF, 0x0000_0410),
        ];
        let err = check_identity(&probes, |a| match a {
            0x0000_0010 => Ok(0x0001_754D),
            0xE004_2000 => Ok(0x1000_0412),
            other => panic!("unexpected read {other:#x}"),
        })
        .unwrap_err();
        assert_eq!(
            err,
            Error::IdentityMismatch {
                addr: 0xE004_2000,
                expected: 0x0000_0410,
                read: 0x0000_0412,
            }
        );
    }

    #[test]
    fn stops_at_first_mismatch() {
        let probes = [
            IdentityProbe::new(0x10, 0xFFFF_FFFF, 0x1234),
            IdentityProbe::new(0x20, 0xFFFF_FFFF, 0x5678),
        ];
        let mut reads: Vec<u32> = Vec::new();
        let err = check_identity(&probes, |a| {
            reads.push(a);
            Ok(0)
        })
        .unwrap_err();
        assert_eq!(reads, vec![0x10]);
        assert!(matches!(err, Error::IdentityMismatch { addr: 0x10, .. }));
    }

    #[test]
    fn read_error_propagates() {
        let probes = [IdentityProbe::new(0x10, 0xFFFF_FFFF, 0x1234)];
        let err = check_identity(&probes, |_| Err(Error::Timeout)).unwrap_err();
        assert_eq!(err, Error::Timeout);
    }
}
