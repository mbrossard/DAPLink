//! Requests decoded, and replies decoded against the request they answer.

use super::check::Finding;
use super::ids::*;
use super::layout::{self, Length};
use super::status::Status;
use crate::session::Item;

/// A decoded request; fields in layout order.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Command {
    /// `DAP_Info`: the info id.
    Info(u8),
    /// `DAP_HostStatus`: type and status.
    HostStatus(u8, u8),
    /// `DAP_Connect`: port.
    Connect(u8),
    /// `DAP_Disconnect`.
    Disconnect,
    /// `DAP_TransferConfigure`: idle cycles, WAIT retries, match retries.
    TransferConfigure(u8, u16, u16),
    /// `DAP_Transfer`: DAP index and items.
    Transfer(u8, Vec<Item>),
    /// `DAP_TransferBlock`: DAP index, count, request byte, write data.
    TransferBlock(u8, u16, u8, Vec<u32>),
    /// `DAP_TransferAbort`.
    TransferAbort,
    /// `DAP_WriteABORT`: DAP index and word.
    WriteAbort(u8, u32),
    /// `DAP_Delay`: microseconds.
    Delay(u16),
    /// `DAP_ResetTarget`.
    ResetTarget,
    /// `DAP_SWJ_Pins`: output, select, wait.
    SwjPins(u8, u8, u32),
    /// `DAP_SWJ_Clock`: Hz.
    SwjClock(u32),
    /// `DAP_SWJ_Sequence`: bit count (0 is 256) and data.
    SwjSequence(u8, Vec<u8>),
    /// `DAP_SWD_Configure`.
    SwdConfigure(u8),
    /// `DAP_SWD_Sequence`: info bytes, each with its output data.
    SwdSequence(Vec<(u8, Vec<u8>)>),
    /// `DAP_JTAG_Sequence`: info bytes, each with its TDI data.
    JtagSequence(Vec<(u8, Vec<u8>)>),
    /// `DAP_JTAG_Configure`: IR lengths.
    JtagConfigure(Vec<u8>),
    /// `DAP_JTAG_IDCODE`: TAP index.
    JtagIdcode(u8),
    /// Any other id and the bytes after it: as its layout sizes them, else the whole report.
    Other(u8, Vec<u8>),
}

struct Cursor<'a>(&'a [u8]);

impl Cursor<'_> {
    fn bytes(&mut self, n: usize) -> Vec<u8> {
        let (head, rest) = self.0.split_at(n);
        self.0 = rest;
        head.to_vec()
    }

    fn u8(&mut self) -> u8 {
        self.bytes(1)[0]
    }

    fn u16(&mut self) -> u16 {
        u16::from_le_bytes([self.u8(), self.u8()])
    }

    fn u32(&mut self) -> u32 {
        u32::from_le_bytes([self.u8(), self.u8(), self.u8(), self.u8()])
    }

    fn sequences(&mut self, data: impl Fn(u8) -> usize) -> Vec<(u8, Vec<u8>)> {
        (0..self.u8())
            .map(|_| {
                let info = self.u8();
                (info, self.bytes(data(info)))
            })
            .collect()
    }
}

impl Command {
    /// Decode the command starting `report`; bytes past its layout are ignored.
    pub fn decode(report: &[u8]) -> Result<Command, Finding> {
        let len = match layout::request(report) {
            Length::Len(n) if n <= report.len() => n,
            Length::Unsized => report.len(),
            _ => return Err(Finding::ShortRequest),
        };
        let id = report[0];
        let mut c = Cursor(&report[1..len]);
        Ok(match id {
            INFO => Command::Info(c.u8()),
            HOST_STATUS => Command::HostStatus(c.u8(), c.u8()),
            CONNECT => Command::Connect(c.u8()),
            DISCONNECT => Command::Disconnect,
            TRANSFER_CONFIGURE => Command::TransferConfigure(c.u8(), c.u16(), c.u16()),
            TRANSFER => {
                let index = c.u8();
                let items = (0..c.u8())
                    .map(|_| {
                        let req = c.u8();
                        let word = req & request::READ == 0 || req & request::VALUE_MATCH != 0;
                        Item {
                            req,
                            word: word.then(|| c.u32()),
                        }
                    })
                    .collect();
                Command::Transfer(index, items)
            }
            TRANSFER_BLOCK => {
                let (index, count, req) = (c.u8(), c.u16(), c.u8());
                let words = if req & request::READ == 0 { count } else { 0 };
                Command::TransferBlock(index, count, req, (0..words).map(|_| c.u32()).collect())
            }
            TRANSFER_ABORT => Command::TransferAbort,
            WRITE_ABORT => Command::WriteAbort(c.u8(), c.u32()),
            DELAY => Command::Delay(c.u16()),
            RESET_TARGET => Command::ResetTarget,
            SWJ_PINS => Command::SwjPins(c.u8(), c.u8(), c.u32()),
            SWJ_CLOCK => Command::SwjClock(c.u32()),
            SWJ_SEQUENCE => {
                let n = c.u8();
                Command::SwjSequence(n, c.bytes(len - 2))
            }
            SWD_CONFIGURE => Command::SwdConfigure(c.u8()),
            SWD_SEQUENCE => Command::SwdSequence(c.sequences(|info| {
                if info & 0x80 == 0 {
                    layout::sequence_bits(info).div_ceil(8)
                } else {
                    0
                }
            })),
            JTAG_SEQUENCE => {
                Command::JtagSequence(c.sequences(|info| layout::sequence_bits(info).div_ceil(8)))
            }
            JTAG_CONFIGURE => {
                let n = c.u8();
                Command::JtagConfigure(c.bytes(usize::from(n)))
            }
            JTAG_IDCODE => Command::JtagIdcode(c.u8()),
            _ => Command::Other(id, c.bytes(len - 1)),
        })
    }

    /// The command id.
    pub fn id(&self) -> u8 {
        match self {
            Command::Info(_) => INFO,
            Command::HostStatus(..) => HOST_STATUS,
            Command::Connect(_) => CONNECT,
            Command::Disconnect => DISCONNECT,
            Command::TransferConfigure(..) => TRANSFER_CONFIGURE,
            Command::Transfer(..) => TRANSFER,
            Command::TransferBlock(..) => TRANSFER_BLOCK,
            Command::TransferAbort => TRANSFER_ABORT,
            Command::WriteAbort(..) => WRITE_ABORT,
            Command::Delay(_) => DELAY,
            Command::ResetTarget => RESET_TARGET,
            Command::SwjPins(..) => SWJ_PINS,
            Command::SwjClock(_) => SWJ_CLOCK,
            Command::SwjSequence(..) => SWJ_SEQUENCE,
            Command::SwdConfigure(_) => SWD_CONFIGURE,
            Command::SwdSequence(_) => SWD_SEQUENCE,
            Command::JtagSequence(_) => JTAG_SEQUENCE,
            Command::JtagConfigure(_) => JTAG_CONFIGURE,
            Command::JtagIdcode(_) => JTAG_IDCODE,
            Command::Other(id, _) => *id,
        }
    }

    /// The request bytes.
    pub fn encode(&self) -> Vec<u8> {
        let mut r = vec![self.id()];
        let sequences = |r: &mut Vec<u8>, seqs: &[(u8, Vec<u8>)]| {
            r.push(seqs.len() as u8);
            for (info, data) in seqs {
                r.push(*info);
                r.extend(data);
            }
        };
        match self {
            Command::Info(b)
            | Command::Connect(b)
            | Command::SwdConfigure(b)
            | Command::JtagIdcode(b) => r.push(*b),
            Command::HostStatus(a, b) => r.extend([*a, *b]),
            Command::Disconnect | Command::TransferAbort | Command::ResetTarget => {}
            Command::TransferConfigure(idle, wait, retries) => {
                r.push(*idle);
                r.extend(wait.to_le_bytes());
                r.extend(retries.to_le_bytes());
            }
            Command::Transfer(index, items) => {
                r.extend([*index, items.len() as u8]);
                for item in items {
                    r.push(item.req);
                    r.extend(item.word.map(u32::to_le_bytes).into_iter().flatten());
                }
            }
            Command::TransferBlock(index, count, req, data) => {
                r.push(*index);
                r.extend(count.to_le_bytes());
                r.push(*req);
                r.extend(data.iter().flat_map(|w| w.to_le_bytes()));
            }
            Command::WriteAbort(index, word) => {
                r.push(*index);
                r.extend(word.to_le_bytes());
            }
            Command::Delay(us) => r.extend(us.to_le_bytes()),
            Command::SwjPins(out, select, wait) => {
                r.extend([*out, *select]);
                r.extend(wait.to_le_bytes());
            }
            Command::SwjClock(hz) => r.extend(hz.to_le_bytes()),
            Command::SwjSequence(n, data) => {
                r.push(*n);
                r.extend(data);
            }
            Command::SwdSequence(seqs) | Command::JtagSequence(seqs) => sequences(&mut r, seqs),
            Command::JtagConfigure(lengths) => {
                r.push(lengths.len() as u8);
                r.extend(lengths);
            }
            Command::Other(_, bytes) => r.extend(bytes),
        }
        r
    }
}

/// Whether a count after a failure includes the transfer that failed; CMSIS-DAP leaves it open.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Counting {
    /// Completed transfers, a posted read counting when posted: CMSIS-DAP 2.1.2 `DAP.c`.
    Arm,
    /// The failed transfer too, nothing posted: dap-rs `a867ca7` on SWD.
    DapRs,
}

/// The data link, which decides what the Arm firmware posts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Link {
    /// AP reads are posted.
    Swd,
    /// DP and AP reads are posted.
    Jtag,
}

/// What decoding a transfer reply needs beyond its request.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Context {
    /// The link the transfers ran on.
    pub link: Link,
    /// How `DAP_Transfer` counts.
    pub transfer: Counting,
    /// How `DAP_TransferBlock` counts.
    pub block: Counting,
}

impl Context {
    /// Both commands counted as the Arm firmware counts them, on `link`.
    pub fn arm(link: Link) -> Context {
        Context {
            link,
            transfer: Counting::Arm,
            block: Counting::Arm,
        }
    }
}

/// One word of a `DAP_Transfer` reply.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Word {
    /// The request item it belongs to.
    pub item: usize,
    /// A Test Domain Timer value rather than read data.
    pub timestamp: bool,
    /// The word.
    pub value: u32,
}

/// A `DAP_Transfer` reply.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TransferReply {
    /// The count field.
    pub count: u8,
    /// The status.
    pub status: Status,
    /// The words, each placed against its item.
    pub words: Vec<Word>,
}

/// A `DAP_TransferBlock` reply.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BlockReply {
    /// The count field.
    pub count: u16,
    /// The status.
    pub status: Status,
    /// The words read.
    pub words: Vec<u32>,
}

/// A decoded reply.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Reply {
    /// The single byte `0xFF`: the probe does not implement the command.
    Unimplemented,
    /// Nothing, as `DAP_TransferAbort` is answered.
    Silent,
    /// `DAP_Info`: the `Len` bytes.
    Info(Vec<u8>),
    /// One byte after the id: a status, a port, or pin levels.
    Byte(u8),
    /// `DAP_ResetTarget`: status and execute.
    ResetTarget(u8, u8),
    /// `DAP_Transfer`.
    Transfer(TransferReply),
    /// `DAP_TransferBlock`.
    TransferBlock(BlockReply),
    /// `DAP_SWD_Sequence` or `DAP_JTAG_Sequence`: status, then each input or capture.
    Sequence(u8, Vec<Vec<u8>>),
    /// `DAP_JTAG_IDCODE`: status and the IDCODE, absent after an error.
    Idcode(u8, Option<u32>),
    /// Any other reply: the bytes after the id.
    Other(Vec<u8>),
}

/// `reply` must be `len` bytes long.
fn exact(reply: &[u8], len: usize) -> Result<(), Finding> {
    match reply.len() {
        n if n < len => Err(Finding::Truncated {
            expected: len,
            got: n,
        }),
        n if n > len => Err(Finding::Excess {
            expected: len,
            got: n,
        }),
        _ => Ok(()),
    }
}

fn words(bytes: &[u8]) -> impl Iterator<Item = u32> + '_ {
    bytes
        .chunks_exact(4)
        .map(|w| u32::from_le_bytes([w[0], w[1], w[2], w[3]]))
}

impl Reply {
    /// Decode `reply`, the probe's unpadded answer to `command`.
    pub fn decode(command: &Command, reply: &[u8], context: Context) -> Result<Reply, Finding> {
        let id = command.id();
        if id == TRANSFER_ABORT {
            return if reply.is_empty() {
                Ok(Reply::Silent)
            } else {
                Err(Finding::ReplyToAbort)
            };
        }
        match reply.first() {
            None => return Err(Finding::NoReply),
            Some(&UNIMPLEMENTED) => return exact(reply, 1).map(|_| Reply::Unimplemented),
            Some(&first) if first != id => return Err(Finding::WrongId(first)),
            _ => {}
        }
        let need = |n: usize| {
            if reply.len() < n {
                Err(Finding::Truncated {
                    expected: n,
                    got: reply.len(),
                })
            } else {
                Ok(())
            }
        };
        let short = |at: usize| u16::from_le_bytes([reply[at], reply[at + 1]]);
        match command {
            Command::Info(_) => {
                need(2)?;
                exact(reply, 2 + usize::from(reply[1]))?;
                Ok(Reply::Info(reply[2..].to_vec()))
            }
            Command::ResetTarget => {
                exact(reply, 3)?;
                Ok(Reply::ResetTarget(reply[1], reply[2]))
            }
            Command::Transfer(_, items) => transfer(items, reply, context),
            Command::TransferBlock(_, count, req, _) => block(*count, *req, reply, context.block),
            Command::SwdSequence(seqs) | Command::JtagSequence(seqs) => {
                need(2)?;
                let ok = reply[1] == 0;
                let sizes: Vec<usize> = seqs
                    .iter()
                    .filter(|(info, _)| ok && info & 0x80 != 0)
                    .map(|(info, _)| layout::sequence_bits(*info).div_ceil(8))
                    .collect();
                exact(reply, 2 + sizes.iter().sum::<usize>())?;
                let mut rest = Cursor(&reply[2..]);
                let data = sizes.iter().map(|&n| rest.bytes(n)).collect();
                Ok(Reply::Sequence(reply[1], data))
            }
            Command::JtagIdcode(_) => {
                need(2)?;
                match reply[1] {
                    0 => {
                        exact(reply, 6)?;
                        Ok(Reply::Idcode(0, words(&reply[2..]).next()))
                    }
                    s => exact(reply, 2).map(|_| Reply::Idcode(s, None)),
                }
            }
            Command::Other(_, bytes) => {
                let len = match id {
                    SWO_DATA => need(4).map(|_| 4 + usize::from(short(2)))?,
                    SWO_EXTENDED_STATUS => {
                        let control = bytes[0];
                        let fields = [(1, 1), (2, 4), (4, 8)];
                        1 + fields
                            .iter()
                            .filter(|(bit, _)| control & bit != 0)
                            .map(|(_, n)| n)
                            .sum::<usize>()
                    }
                    UART_TRANSFER => need(6).map(|_| 6 + usize::from(short(4)))?,
                    _ => match layout::fixed_reply(id) {
                        Some(n) => n,
                        None => return Ok(Reply::Other(reply[1..].to_vec())),
                    },
                };
                exact(reply, len)?;
                Ok(Reply::Other(reply[1..].to_vec()))
            }
            _ => {
                let len = layout::fixed_reply(id).expect("every other command's reply is fixed");
                exact(reply, len)?;
                Ok(Reply::Byte(reply[1]))
            }
        }
    }
}

/// The words a completed item leaves: (timestamp, data).
fn slots(req: u8, counting: Counting) -> (bool, bool) {
    let stamp = req & request::TIMESTAMP != 0;
    if req & request::READ != 0 {
        let matched = req & request::VALUE_MATCH != 0;
        (stamp && (!matched || counting == Counting::DapRs), !matched)
    } else {
        (stamp && req & request::MATCH_MASK == 0, false)
    }
}

fn transfer(items: &[Item], reply: &[u8], context: Context) -> Result<Reply, Finding> {
    if reply.len() < 3 {
        return Err(Finding::Truncated {
            expected: 3,
            got: reply.len(),
        });
    }
    let (count, status) = (reply[1], Status::transfer(reply[2]));
    let counted = usize::from(count).min(items.len());
    let failed = !status.is_ok() && counted > 0;
    let completed = match context.transfer {
        Counting::DapRs if failed => counted - 1,
        _ => counted,
    };
    let mut layout = Vec::new();
    for (i, item) in items[..completed].iter().enumerate() {
        let (stamp, data) = slots(item.req, context.transfer);
        layout.extend(stamp.then_some((i, true)));
        layout.extend(data.then_some((i, false)));
    }
    // The one word a failure may or may not leave behind, last when present.
    let mut optional = None;
    if failed {
        let last = counted - 1;
        let req = items[last].req;
        let read = req & request::READ != 0;
        match context.transfer {
            Counting::Arm => {
                let posted = req & request::AP != 0 || context.link == Link::Jtag;
                if read && req & request::VALUE_MATCH == 0 && posted {
                    // A posted read stays counted when collecting its data fails.
                    optional = layout.pop();
                }
            }
            Counting::DapRs => {
                if read && req & request::TIMESTAMP != 0 {
                    optional = Some((last, true));
                }
            }
        }
    }
    let present = reply.len() - 3;
    let full = layout.len() + usize::from(optional.is_some());
    if present == 4 * full {
        layout.extend(optional);
    } else if present < 4 * layout.len() {
        exact(reply, 3 + 4 * layout.len())?;
    } else if present != 4 * layout.len() {
        exact(reply, 3 + 4 * full)?;
    }
    let words = layout
        .iter()
        .zip(words(&reply[3..]))
        .map(|(&(item, timestamp), value)| Word {
            item,
            timestamp,
            value,
        })
        .collect();
    Ok(Reply::Transfer(TransferReply {
        count,
        status,
        words,
    }))
}

fn block(requested: u16, req: u8, reply: &[u8], counting: Counting) -> Result<Reply, Finding> {
    if reply.len() < 4 {
        return Err(Finding::Truncated {
            expected: 4,
            got: reply.len(),
        });
    }
    let count = u16::from_le_bytes([reply[1], reply[2]]);
    let status = Status::block(reply[3]);
    let counted = usize::from(count.min(requested));
    let read = match counting {
        Counting::DapRs if !status.is_ok() => counted.saturating_sub(1),
        _ => counted,
    };
    let n = if req & request::READ != 0 { read } else { 0 };
    exact(reply, 4 + 4 * n)?;
    Ok(Reply::TransferBlock(BlockReply {
        count,
        status,
        words: words(&reply[4..]).collect(),
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::session::{stream, Profile, Rng};

    #[test]
    fn every_generated_command_decodes_and_encodes_back() {
        for profile in [Profile::SWD, Profile::JTAG, Profile::CHAOS] {
            for seed in 0..300 {
                for cmd in stream(seed, &profile).cmds {
                    let report = cmd.encode();
                    let decoded = Command::decode(&report).expect("a complete request");
                    assert_eq!(decoded.encode(), report, "{cmd:?}");
                    assert_ne!(decoded, Command::Other(report[0], report[1..].to_vec()));
                }
            }
        }
    }

    #[test]
    fn arbitrary_bytes_never_panic() {
        let mut rng = Rng::new(0xC0DE_C001);
        let bytes = |n: u64, rng: &mut Rng| -> Vec<u8> {
            (0..rng.below(n)).map(|_| rng.bits() as u8).collect()
        };
        for _ in 0..20_000 {
            let mut request = bytes(40, &mut rng);
            if let Some(id) = request.first_mut() {
                *id %= 0x30;
            }
            let Ok(command) = Command::decode(&request) else {
                continue;
            };
            let mut reply = bytes(40, &mut rng);
            if let Some(first) = reply.first_mut() {
                *first = command.id();
            }
            let dap_rs = Context {
                link: Link::Swd,
                transfer: Counting::DapRs,
                block: Counting::DapRs,
            };
            for context in [Context::arm(Link::Swd), Context::arm(Link::Jtag), dap_rs] {
                let _ = Reply::decode(&command, &reply, context);
            }
        }
    }
}
