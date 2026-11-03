//! The JTAG-DP: its instructions, its ACK by protocol, and the ReadResult an xPACC scan returns.

/// Length of the DPACC, APACC and ABORT scan registers.
pub const XPACC_LEN: u8 = 35;

/// The data register a JTAG-DP instruction selects.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Instruction {
    /// ABORT.
    Abort,
    /// DPACC.
    Dpacc,
    /// APACC.
    Apacc,
    /// IDCODE.
    Idcode,
    /// BYPASS.
    Bypass,
    /// Boundary scan.
    Boundary,
}

/// What `ir` selects in a JTAG-DP TAP of IR length `ir_len`.
pub fn instruction(ir: u32, ir_len: u8) -> Instruction {
    let ones = (1u32 << ir_len) - 1;
    if ir == ones {
        return Instruction::Bypass;
    }
    // An 8-bit IR puts the 4-bit codes under 0xF0; a clear MSB is boundary scan.
    if ir >> (ir_len - 1) == 0 {
        return Instruction::Boundary;
    }
    if ir >> 4 != ones >> 4 {
        return Instruction::Bypass;
    }
    match ir & 0xF {
        0x8 => Instruction::Abort,
        0xA => Instruction::Dpacc,
        0xB => Instruction::Apacc,
        0xE => Instruction::Idcode,
        _ => Instruction::Bypass,
    }
}

/// The IDCODE instruction for IR length `ir_len`.
pub fn idcode_ir(ir_len: u8) -> u32 {
    (1 << ir_len) - 2
}

/// The JTAG-DP protocol: 0 answers OK and FAULT alike, 1 tells them apart.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Protocol {
    /// ADIv5.
    Zero,
    /// ADIv6.
    One,
    /// Not yet known: protocol 1 once an OK (`0b100`) is seen.
    Unknown,
}

impl Protocol {
    /// The protocol after an ACK of `bits`: only protocol 1 sends `0b100`.
    pub fn learn(self, bits: u8) -> Protocol {
        match (self, bits) {
            (Protocol::Unknown, 0b100) => Protocol::One,
            _ => self,
        }
    }
}

/// The JTAG-DP's DP version: what DPACC reads at `0x0` and `0x8`, and what ABORT takes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum DpVersion {
    /// No DPIDR, SELECT reads back, ABORT takes DAPABORT only.
    #[default]
    V0,
    /// DPIDR, `0x8` read UNPREDICTABLE.
    V1,
    /// As DPv1.
    V2,
    /// DPIDR, `0x8` read RES0.
    V3,
}

/// An xPACC scan's ACK.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JtagAck {
    /// WAIT.
    Wait,
    /// OK/FAULT, protocol 0.
    OkFault,
    /// OK, protocol 1.
    Ok,
    /// FAULT, protocol 1.
    Fault,
    /// No ACK the protocol defines: these bits.
    Invalid(u8),
}

impl JtagAck {
    /// The three ACK bits, `ACK[0]` first on TDO.
    pub fn bits(self) -> u64 {
        match self {
            JtagAck::Wait => 0b001,
            JtagAck::OkFault | JtagAck::Fault => 0b010,
            JtagAck::Ok => 0b100,
            JtagAck::Invalid(bits) => u64::from(bits & 7),
        }
    }
}

/// The ACK `bits` mean under `protocol`; an unknown protocol reads `0b010` as OK/FAULT.
pub fn decode_ack(bits: u8, protocol: Protocol) -> JtagAck {
    match (bits & 7, protocol) {
        (0b001, _) => JtagAck::Wait,
        (0b010, Protocol::One) => JtagAck::Fault,
        (0b010, _) => JtagAck::OkFault,
        (0b100, Protocol::One | Protocol::Unknown) => JtagAck::Ok,
        (bits, _) => JtagAck::Invalid(bits),
    }
}

/// Why ReadResult holds no access's result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Unknown {
    /// Test-Logic-Reset or a protocol switch.
    Reset,
    /// An instruction other than DPACC, APACC or BYPASS was loaded.
    Instruction,
    /// The last executed access was a write.
    Write,
    /// The last capture answered FAULT.
    Fault,
}

/// What the next xPACC capture returns in its data bits.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReadResult {
    /// The last read's result.
    Valid(u32),
    /// UNKNOWN.
    Unknown(Unknown),
}

/// Which access the next ACK reports on, and whether ReadResult is valid.
#[derive(Clone, Debug, Default)]
pub struct Pipeline {
    pub(crate) previous: Option<Instruction>,
    pub(crate) unknown: Option<Unknown>,
    held: u32,
}

impl Pipeline {
    /// No previous access, and ReadResult UNKNOWN.
    pub fn forget(&mut self, why: Unknown) {
        self.previous = None;
        self.unknown = Some(why);
    }

    /// A capture's ACK: WAIT while `busy`; on protocol 1, FAULT for `sticky` unless after a DPACC.
    pub fn ack(&mut self, protocol: Protocol, busy: bool, sticky: bool) -> JtagAck {
        let ack = if busy {
            JtagAck::Wait
        } else if protocol == Protocol::Zero {
            JtagAck::OkFault
        } else if sticky && self.previous != Some(Instruction::Dpacc) {
            JtagAck::Fault
        } else {
            JtagAck::Ok
        };
        self.acked(ack);
        ack
    }

    /// A capture answered `ack`: a FAULT leaves ReadResult UNKNOWN.
    pub fn acked(&mut self, ack: JtagAck) {
        if ack == JtagAck::Fault {
            self.unknown = Some(Unknown::Fault);
        }
    }

    /// What the next capture returns.
    pub fn result(&self) -> ReadResult {
        match self.unknown {
            Some(why) => ReadResult::Unknown(why),
            None => ReadResult::Valid(self.held),
        }
    }

    /// The last value an access left, valid or not.
    pub fn held(&self) -> u32 {
        self.held
    }

    /// An executed DPACC or APACC scan: a read leaves ReadResult to [`Pipeline::set`].
    pub fn execute(&mut self, access: Instruction, read: bool) {
        self.previous = Some(access);
        self.unknown = (!read).then_some(Unknown::Write);
    }

    /// The executed access's result.
    pub fn set(&mut self, value: u32) {
        self.held = value;
    }
}

/// One DPACC or APACC scan, as the JTAG-DP saw it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Xpacc {
    /// The JTAG-DP's TAP index.
    pub tap: usize,
    /// DPACC or APACC.
    pub access: Instruction,
    /// RnW.
    pub read: bool,
    /// `A[3:2]`.
    pub a: u8,
    /// The data shifted in.
    pub data: u32,
    /// The ACK captured.
    pub ack: JtagAck,
    /// The ReadResult captured.
    pub result: ReadResult,
    /// Whether Update-DR performed the access; not after WAIT.
    pub executed: bool,
}
