//! Command ids, `DAP_Info` ids, capability bits and transfer request bits, as CMSIS-DAP numbers them.

/// `DAP_Info`.
pub const INFO: u8 = 0x00;
/// `DAP_HostStatus`.
pub const HOST_STATUS: u8 = 0x01;
/// `DAP_Connect`.
pub const CONNECT: u8 = 0x02;
/// `DAP_Disconnect`.
pub const DISCONNECT: u8 = 0x03;
/// `DAP_TransferConfigure`.
pub const TRANSFER_CONFIGURE: u8 = 0x04;
/// `DAP_Transfer`.
pub const TRANSFER: u8 = 0x05;
/// `DAP_TransferBlock`.
pub const TRANSFER_BLOCK: u8 = 0x06;
/// `DAP_TransferAbort`.
pub const TRANSFER_ABORT: u8 = 0x07;
/// `DAP_WriteABORT`.
pub const WRITE_ABORT: u8 = 0x08;
/// `DAP_Delay`.
pub const DELAY: u8 = 0x09;
/// `DAP_ResetTarget`.
pub const RESET_TARGET: u8 = 0x0A;
/// `DAP_SWJ_Pins`.
pub const SWJ_PINS: u8 = 0x10;
/// `DAP_SWJ_Clock`.
pub const SWJ_CLOCK: u8 = 0x11;
/// `DAP_SWJ_Sequence`.
pub const SWJ_SEQUENCE: u8 = 0x12;
/// `DAP_SWD_Configure`.
pub const SWD_CONFIGURE: u8 = 0x13;
/// `DAP_JTAG_Sequence`.
pub const JTAG_SEQUENCE: u8 = 0x14;
/// `DAP_JTAG_Configure`.
pub const JTAG_CONFIGURE: u8 = 0x15;
/// `DAP_JTAG_IDCODE`.
pub const JTAG_IDCODE: u8 = 0x16;
/// `DAP_SWO_Transport`.
pub const SWO_TRANSPORT: u8 = 0x17;
/// `DAP_SWO_Mode`.
pub const SWO_MODE: u8 = 0x18;
/// `DAP_SWO_Baudrate`.
pub const SWO_BAUDRATE: u8 = 0x19;
/// `DAP_SWO_Control`.
pub const SWO_CONTROL: u8 = 0x1A;
/// `DAP_SWO_Status`.
pub const SWO_STATUS: u8 = 0x1B;
/// `DAP_SWO_Data`.
pub const SWO_DATA: u8 = 0x1C;
/// `DAP_SWD_Sequence`.
pub const SWD_SEQUENCE: u8 = 0x1D;
/// `DAP_SWO_ExtendedStatus`.
pub const SWO_EXTENDED_STATUS: u8 = 0x1E;
/// `DAP_UART_Transport`.
pub const UART_TRANSPORT: u8 = 0x1F;
/// `DAP_UART_Configure`.
pub const UART_CONFIGURE: u8 = 0x20;
/// `DAP_UART_Transfer`.
pub const UART_TRANSFER: u8 = 0x21;
/// `DAP_UART_Control`.
pub const UART_CONTROL: u8 = 0x22;
/// `DAP_UART_Status`.
pub const UART_STATUS: u8 = 0x23;
/// `DAP_QueueCommands`.
pub const QUEUE_COMMANDS: u8 = 0x7E;
/// `DAP_ExecuteCommands`.
pub const EXECUTE_COMMANDS: u8 = 0x7F;
/// The reply to a command the probe does not implement; never a command.
pub const UNIMPLEMENTED: u8 = 0xFF;

/// Every assigned id and its name, in id order.
pub const COMMANDS: [(u8, &str); 33] = [
    (INFO, "DAP_Info"),
    (HOST_STATUS, "DAP_HostStatus"),
    (CONNECT, "DAP_Connect"),
    (DISCONNECT, "DAP_Disconnect"),
    (TRANSFER_CONFIGURE, "DAP_TransferConfigure"),
    (TRANSFER, "DAP_Transfer"),
    (TRANSFER_BLOCK, "DAP_TransferBlock"),
    (TRANSFER_ABORT, "DAP_TransferAbort"),
    (WRITE_ABORT, "DAP_WriteABORT"),
    (DELAY, "DAP_Delay"),
    (RESET_TARGET, "DAP_ResetTarget"),
    (SWJ_PINS, "DAP_SWJ_Pins"),
    (SWJ_CLOCK, "DAP_SWJ_Clock"),
    (SWJ_SEQUENCE, "DAP_SWJ_Sequence"),
    (SWD_CONFIGURE, "DAP_SWD_Configure"),
    (JTAG_SEQUENCE, "DAP_JTAG_Sequence"),
    (JTAG_CONFIGURE, "DAP_JTAG_Configure"),
    (JTAG_IDCODE, "DAP_JTAG_IDCODE"),
    (SWO_TRANSPORT, "DAP_SWO_Transport"),
    (SWO_MODE, "DAP_SWO_Mode"),
    (SWO_BAUDRATE, "DAP_SWO_Baudrate"),
    (SWO_CONTROL, "DAP_SWO_Control"),
    (SWO_STATUS, "DAP_SWO_Status"),
    (SWO_DATA, "DAP_SWO_Data"),
    (SWD_SEQUENCE, "DAP_SWD_Sequence"),
    (SWO_EXTENDED_STATUS, "DAP_SWO_ExtendedStatus"),
    (UART_TRANSPORT, "DAP_UART_Transport"),
    (UART_CONFIGURE, "DAP_UART_Configure"),
    (UART_TRANSFER, "DAP_UART_Transfer"),
    (UART_CONTROL, "DAP_UART_Control"),
    (UART_STATUS, "DAP_UART_Status"),
    (QUEUE_COMMANDS, "DAP_QueueCommands"),
    (EXECUTE_COMMANDS, "DAP_ExecuteCommands"),
];

/// The vendor command range, `0x80`-`0x9F`, as Arm's reference firmware assigns it.
pub fn is_vendor(id: u8) -> bool {
    (0x80..=0x9F).contains(&id)
}

/// The command's name; `None` for a vendor or unassigned id.
pub fn name(id: u8) -> Option<&'static str> {
    COMMANDS.iter().find(|c| c.0 == id).map(|c| c.1)
}

/// `DAP_Info` ids.
pub mod info {
    /// Vendor name, a string.
    pub const VENDOR: u8 = 0x01;
    /// Product name, a string.
    pub const PRODUCT: u8 = 0x02;
    /// Serial number, a string.
    pub const SERIAL: u8 = 0x03;
    /// CMSIS-DAP protocol version, a string.
    pub const PROTOCOL_VERSION: u8 = 0x04;
    /// Target device vendor, a string.
    pub const DEVICE_VENDOR: u8 = 0x05;
    /// Target device name, a string.
    pub const DEVICE_NAME: u8 = 0x06;
    /// Target board vendor, a string.
    pub const BOARD_VENDOR: u8 = 0x07;
    /// Target board name, a string.
    pub const BOARD_NAME: u8 = 0x08;
    /// Product firmware version, a string.
    pub const FIRMWARE_VERSION: u8 = 0x09;
    /// Capabilities: one or two bytes of `capability` bits.
    pub const CAPABILITIES: u8 = 0xF0;
    /// Test Domain Timer frequency.
    pub const TIMER_FREQUENCY: u8 = 0xF1;
    /// UART receive buffer size, a WORD.
    pub const UART_RX_BUFFER: u8 = 0xFB;
    /// UART transmit buffer size, a WORD.
    pub const UART_TX_BUFFER: u8 = 0xFC;
    /// SWO trace buffer size, a WORD.
    pub const SWO_BUFFER: u8 = 0xFD;
    /// Maximum packet count, a BYTE.
    pub const PACKET_COUNT: u8 = 0xFE;
    /// Maximum packet size, a SHORT.
    pub const PACKET_SIZE: u8 = 0xFF;
}

/// `DAP_Info` capability bits: `Info0` in the low byte, `Info1` in the high byte.
pub mod capability {
    /// The SWD commands.
    pub const SWD: u16 = 1 << 0;
    /// The JTAG commands.
    pub const JTAG: u16 = 1 << 1;
    /// SWO in UART mode.
    pub const SWO_UART: u16 = 1 << 2;
    /// SWO in Manchester mode.
    pub const SWO_MANCHESTER: u16 = 1 << 3;
    /// `DAP_QueueCommands` and `DAP_ExecuteCommands`.
    pub const ATOMIC: u16 = 1 << 4;
    /// The Test Domain Timer.
    pub const TIMER: u16 = 1 << 5;
    /// SWO streaming trace on its own endpoint.
    pub const SWO_STREAM: u16 = 1 << 6;
    /// The UART communication port.
    pub const UART: u16 = 1 << 7;
    /// The UART routed to a USB COM port.
    pub const UART_COM_PORT: u16 = 1 << 8;
}

/// `DAP_Transfer` request bits; `DAP_TransferBlock` uses the low four.
pub mod request {
    /// APnDP: an AP register.
    pub const AP: u8 = 1 << 0;
    /// RnW: a read.
    pub const READ: u8 = 1 << 1;
    /// `A[3:2]`.
    pub const A32: u8 = 3 << 2;
    /// Read until the value matches.
    pub const VALUE_MATCH: u8 = 1 << 4;
    /// Write the match mask.
    pub const MATCH_MASK: u8 = 1 << 5;
    /// Return a Test Domain Timer value.
    pub const TIMESTAMP: u8 = 1 << 7;
}
