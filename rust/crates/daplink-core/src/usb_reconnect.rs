//! USB re-enumeration seam: the mechanism a volume remount uses to flush a host's cached view.

/// Hardware mechanism that detaches the probe from the USB bus and reattaches it. The caller owns
/// the timing -- detach, wait, attach -- and both operations must be cheap and non-blocking.
pub trait UsbReconnect {
    /// Drop off the bus (e.g. disable the D+ pull-up): the host sees a disconnect.
    fn detach(&mut self);
    /// Rejoin the bus: the host re-enumerates the device and re-reads everything fresh.
    fn attach(&mut self);
}

/// No-op [`UsbReconnect`]: the remount republishes through UNIT ATTENTION alone.
pub struct NoUsbReconnect;

impl UsbReconnect for NoUsbReconnect {
    fn detach(&mut self) {}
    fn attach(&mut self) {}
}
