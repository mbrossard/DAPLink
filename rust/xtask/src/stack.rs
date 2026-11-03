//! The stack floor: the deepest resolved call path through a linked image, against its free RAM.
//!
//! Frames come from each function's own SP decrements and calls from its disassembly. Calls that
//! cannot be resolved are dropped, so the floor is a lower bound on the worst case: a floor above
//! free RAM is an overflow on any path that can execute, and a floor below it proves nothing.

use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};
use std::process::Command;

use crate::elf::Image;
use crate::registry::Firmware;
use crate::root;

const CONDITIONS: &[&str] = &[
    "eq", "ne", "cs", "hs", "cc", "lo", "mi", "pl", "vs", "vc", "hi", "ls", "ge", "lt", "gt", "le",
];
const CONDITIONAL: &[&str] = &[
    "b", "bl", "blx", "bx", "pop", "ldm", "mov", "ldr", "add", "sub", "push",
];
const CALLEE_SAVED: &[&str] = &["r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11"];
const NO_REGISTER_WRITE: &[&str] = &[
    "str", "strb", "strh", "strd", "stm", "stmia", "stmdb", "push", "vpush", "cmp", "cmn", "tst",
    "teq", "nop", "dsb", "isb", "dmb", "wfi", "wfe", "msr", "svc", "bkpt", "cbz", "cbnz", "cpsie",
    "cpsid",
];

/// The mnemonic with its width suffix and condition removed.
fn base_mnemonic(op: &str) -> &str {
    let base = op.split('.').next().unwrap_or(op);
    for condition in CONDITIONS {
        if let Some(stem) = base.strip_suffix(condition) {
            if CONDITIONAL.contains(&stem) {
                return stem;
            }
        }
    }
    base
}

/// The instruction at the end of a listing line, without its comment and symbol annotation.
fn parse_line(line: &str) -> Option<(u32, String, String)> {
    let (address, rest) = line.split_once(':')?;
    let address = u32::from_str_radix(address.trim(), 16).ok()?;
    let rest = rest.split('@').next().unwrap_or(rest);
    let rest = match (rest.find('<'), rest.rfind('>')) {
        (Some(open), Some(close)) if open < close => {
            format!("{}{}", &rest[..open], &rest[close + 1..])
        }
        _ => rest.to_string(),
    };
    let mut parts = rest.trim().splitn(2, char::is_whitespace);
    let op = parts.next()?;
    let args = parts.next().unwrap_or("").trim();
    let data = matches!(op, ".word" | ".short" | ".byte" | ".long" | "<unknown>");
    (!op.is_empty() && !data).then(|| (address, op.to_string(), args.to_string()))
}

/// The bytes a `{...}` register list occupies, and the bytes its core registers occupy.
fn list_bytes(args: &str) -> (u32, u32) {
    let Some(list) = args
        .split_once('{')
        .and_then(|(_, rest)| rest.split_once('}'))
    else {
        return (0, 0);
    };
    let (mut all, mut core) = (0, 0);
    for item in list.0.split(',').map(str::trim) {
        let number = |r: &str| -> u32 {
            let digits: String = r.chars().filter(char::is_ascii_digit).collect();
            digits.parse().unwrap_or(0)
        };
        let count = item
            .split_once('-')
            .map_or(1, |(a, b)| number(b) - number(a) + 1);
        if item.starts_with('d') {
            all += 8 * count;
        } else {
            all += 4 * count;
            core += 4 * count;
        }
    }
    (all, core)
}

fn immediate(text: &str) -> Option<i64> {
    let start = text.find('#')? + 1;
    let token: String = text[start..]
        .chars()
        .take_while(|c| c.is_ascii_alphanumeric() || *c == '-')
        .collect();
    let (negative, digits) = match token.strip_prefix('-') {
        Some(rest) => (true, rest),
        None => (false, token.as_str()),
    };
    let value = match digits.strip_prefix("0x") {
        Some(hex) => i64::from_str_radix(hex, 16).ok()?,
        None => digits.parse().ok()?,
    };
    Some(if negative { -value } else { value })
}

/// What `sp, [sp,] <operand>` applies to SP, or `None` when SP is not the destination.
fn sp_operand(args: &str) -> Option<&str> {
    let rest = args.strip_prefix("sp,")?.trim_start();
    Some(rest.strip_prefix("sp,").map_or(rest, str::trim_start))
}

fn is_register(token: &str) -> bool {
    matches!(token, "lr" | "ip" | "sb" | "sl" | "fp")
        || token
            .strip_prefix('r')
            .is_some_and(|n| !n.is_empty() && n.chars().all(|c| c.is_ascii_digit()))
}

fn address(token: &str) -> Option<u32> {
    u32::from_str_radix(token.strip_prefix("0x")?, 16).ok()
}

/// One function's own frame and its calls, as (stack already used at the call, target).
#[derive(Default)]
struct Body {
    frame: u32,
    calls: Vec<(u32, u32)>,
    unresolved: Vec<u32>,
}

/// Read one function's prologue and calls from its instructions.
fn read_body(
    instructions: &[(u32, String, String)],
    this: u32,
    is_function: &dyn Fn(u32) -> bool,
    word: &dyn Fn(u32) -> Option<u32>,
) -> Body {
    let mut body = Body::default();
    let mut registers: HashMap<String, u32> = HashMap::new();
    // SP released by the instructions just before a tail branch.
    let mut released: u32 = 0;
    for (at, op, args) in instructions {
        let (at, args) = (*at, args.as_str());
        let base = base_mnemonic(op);
        let sp = sp_operand(args);

        let mut frees = None;
        match base {
            "push" | "vpush" => body.frame += list_bytes(args).0,
            "stmdb" if args.starts_with("sp!") => body.frame += list_bytes(args).0,
            "sub" | "subw" | "subs" => {
                if let Some(n) = sp.and_then(immediate) {
                    body.frame += n as u32;
                }
            }
            "add" | "addw" => match sp {
                // A frame too large for an immediate: `ldr rN, =-size; add sp, rN`.
                Some(operand) if is_register(operand) => {
                    if let Some(value) = registers.get(operand).filter(|v| **v & 0x8000_0000 != 0) {
                        body.frame += value.wrapping_neg();
                    }
                }
                Some(operand) => frees = immediate(operand).map(|n| n as u32),
                None => {}
            },
            "pop" => frees = Some(list_bytes(args).1),
            _ if base.starts_with("str") && args.contains("[sp, #-") && args.ends_with('!') => {
                let n = args.split_once('[').and_then(|(_, m)| immediate(m));
                body.frame += n.map_or(0, |n| n.unsigned_abs() as u32);
            }
            _ if base.starts_with("ldr") && args.contains("[sp], #") => {
                let n = args.rsplit_once(']').and_then(|(_, m)| immediate(m));
                frees = n.map(|n| n as u32);
            }
            _ => {}
        }

        let first = args.split(',').next().unwrap_or("").trim();
        match base {
            "bl" | "blx" | "b" if address(first).is_some() => {
                let target = address(first).unwrap_or(0);
                if base == "b" {
                    if is_function(target) && target != this {
                        body.calls
                            .push((body.frame.saturating_sub(released), target));
                    }
                } else if is_function(target) {
                    body.calls.push((body.frame, target));
                }
            }
            "blx" | "bx" if is_register(first) && first != "lr" => {
                let depth = if base == "blx" {
                    body.frame
                } else {
                    body.frame.saturating_sub(released)
                };
                match registers.get(first).map(|v| v & !1) {
                    Some(target) if is_function(target) => body.calls.push((depth, target)),
                    _ => body.unresolved.push(depth),
                }
            }
            "ldr" if args.contains("[pc") && is_register(first) => {
                let literal = args
                    .split_once("[pc")
                    .and_then(|(_, m)| immediate(m))
                    .and_then(|offset| {
                        let pc = i64::from((at + 4) & !3);
                        word(u32::try_from(pc + offset).ok()?)
                    });
                match literal {
                    Some(value) => registers.insert(first.to_string(), value),
                    None => registers.remove(first),
                };
            }
            "movw" if is_register(first) => {
                let low = immediate(args).unwrap_or(0) as u32 & 0xffff;
                registers.insert(first.to_string(), low);
            }
            "movt" if is_register(first) => {
                match (registers.get(first).copied(), immediate(args)) {
                    (Some(low), Some(high)) => {
                        let value = (low & 0xffff) | ((high as u32) << 16);
                        registers.insert(first.to_string(), value);
                    }
                    _ => {
                        registers.remove(first);
                    }
                }
            }
            "mov" if is_register(first) => {
                let source = args.split(',').nth(1).unwrap_or("").trim();
                match registers.get(source).copied() {
                    Some(value) => registers.insert(first.to_string(), value),
                    None => registers.remove(first),
                };
            }
            "pop" | "ldm" | "ldmia" => registers.clear(),
            _ if is_register(first)
                && !NO_REGISTER_WRITE.contains(&base)
                && !(base.starts_with('b') && base != "bic") =>
            {
                registers.remove(first);
            }
            _ => {}
        }
        if matches!(base, "bl" | "blx") {
            registers.retain(|r, _| CALLEE_SAVED.contains(&r.as_str()));
        }
        released = frees.map_or(0, |n| released + n);
    }
    body
}

fn is_task_poll(name: &str) -> bool {
    name.contains("embassy_executor")
        && (name.contains("poll_exited")
            || (name.contains("TaskStorage") && name.contains("4poll")))
}

fn is_executor_loop(name: &str) -> bool {
    name.contains("embassy_executor") && name.contains("Executor") && !is_task_poll(name)
}

/// The recovered program.
struct Graph {
    names: BTreeMap<u32, String>,
    frames: HashMap<u32, u32>,
    calls: HashMap<u32, Vec<(u32, u32)>>,
    unresolved: usize,
}

/// Build the call graph from a disassembly listing and the image's code symbols.
fn build_graph(
    listing: &str,
    functions: &BTreeMap<u32, (u32, String)>,
    word: &dyn Fn(u32) -> Option<u32>,
) -> Result<Graph, String> {
    let mut instructions: BTreeMap<u32, Vec<(u32, String, String)>> = BTreeMap::new();
    for parsed in listing.lines().filter_map(parse_line) {
        let owner = functions.range(..=parsed.0).next_back();
        if let Some((start, (size, _))) = owner {
            if u64::from(parsed.0) < u64::from(*start) + u64::from((*size).max(2)) {
                instructions.entry(*start).or_default().push(parsed);
            }
        }
    }

    let is_function = |a: u32| functions.contains_key(&a);
    let by_name: HashMap<&str, u32> = functions
        .iter()
        .map(|(a, (_, n))| (n.as_str(), *a))
        .collect();
    let polls: Vec<u32> = functions
        .iter()
        .filter(|(_, (_, n))| is_task_poll(n))
        .map(|(a, _)| *a)
        .collect();

    let mut graph = Graph {
        names: functions
            .iter()
            .map(|(a, (_, n))| (*a, n.clone()))
            .collect(),
        frames: HashMap::new(),
        calls: HashMap::new(),
        unresolved: 0,
    };
    for (start, (_, name)) in functions {
        // A linker range-extension thunk jumps to the symbol it is named after.
        let thunk = name
            .split_once("LongThunk_")
            .and_then(|(_, target)| by_name.get(target));
        if let Some(target) = thunk {
            graph.frames.insert(*start, 0);
            graph.calls.insert(*start, vec![(0, *target)]);
            continue;
        }
        let code = instructions.get(start).map_or(&[][..], Vec::as_slice);
        let mut body = read_body(code, *start, &is_function, word);
        // The task header's poll function holds one of these and nothing else.
        if is_executor_loop(name) && !body.unresolved.is_empty() {
            if polls.is_empty() {
                return Err(format!(
                    "`{name}` dispatches, but no task poll symbol found"
                ));
            }
            for depth in body.unresolved.drain(..) {
                body.calls.extend(polls.iter().map(|p| (depth, *p)));
            }
        }
        graph.unresolved += body.unresolved.len();
        graph.frames.insert(*start, body.frame);
        body.calls.sort_unstable();
        graph.calls.insert(*start, body.calls);
    }
    Ok(graph)
}

/// The deepest stack from `f` and the callee it goes through; a back edge counts nothing.
fn depth(
    graph: &Graph,
    f: u32,
    on_path: &mut Vec<u32>,
    memo: &mut HashMap<u32, (u32, Option<u32>)>,
) -> u32 {
    if let Some((d, _)) = memo.get(&f) {
        return *d;
    }
    if on_path.contains(&f) {
        return 0;
    }
    on_path.push(f);
    let mut best = (graph.frames.get(&f).copied().unwrap_or(0), None);
    for (used, callee) in graph.calls.get(&f).map_or(&[][..], Vec::as_slice) {
        let total = used + depth(graph, *callee, on_path, memo);
        if total > best.0 {
            best = (total, Some(*callee));
        }
    }
    on_path.pop();
    memo.insert(f, best);
    best.0
}

/// One image's result.
pub struct Floor {
    pub floor: u32,
    pub free: u32,
    pub unresolved: usize,
    /// The deepest path, as each function's name and own frame.
    pub path: Vec<(String, u32)>,
}

/// The floor of `graph` from `root`.
fn floor_of(graph: &Graph, root: u32, free: u32) -> Floor {
    let mut memo = HashMap::new();
    let floor = depth(graph, root, &mut Vec::new(), &mut memo);
    let mut path = Vec::new();
    let mut at = Some(root);
    while let Some(f) = at.filter(|f| path.len() < 64 && graph.names.contains_key(f)) {
        path.push((
            graph.names[&f].clone(),
            graph.frames.get(&f).copied().unwrap_or(0),
        ));
        at = memo.get(&f).and_then(|(_, next)| *next);
    }
    Floor {
        floor,
        free,
        unresolved: graph.unresolved,
        path,
    }
}

/// RAM the stack may use, refusing a stack end below the statics.
fn free_ram(top: u32, end: u32, statics: Option<u32>) -> Result<u32, String> {
    if statics.is_some_and(|statics| end < statics) {
        return Err(format!("`_stack_end` {end:#x} lies below the statics"));
    }
    top.checked_sub(end)
        .ok_or_else(|| format!("`_stack_end` {end:#x} lies above `_stack_start` {top:#x}"))
}

/// `llvm-objdump` from the pinned toolchain's `llvm-tools` component.
fn objdump() -> Result<PathBuf, String> {
    let rustc = |arg: &str| -> Result<String, String> {
        let out = Command::new("rustc")
            .arg(arg)
            .args((arg == "--print").then_some("sysroot"))
            .current_dir(root())
            .output()
            .map_err(|e| format!("rustc: {e}"))?;
        Ok(String::from_utf8_lossy(&out.stdout).into_owned())
    };
    let sysroot = rustc("--print")?;
    let version = rustc("-vV")?;
    let host = version
        .lines()
        .find_map(|l| l.strip_prefix("host: "))
        .ok_or("rustc names no host")?;
    Ok(PathBuf::from(sysroot.trim())
        .join("lib/rustlib")
        .join(host)
        .join("bin/llvm-objdump"))
}

/// Measure one linked image.
pub fn measure(elf: &Path) -> Result<Floor, String> {
    let data = std::fs::read(elf).map_err(|e| format!("{}: {e}", elf.display()))?;
    let image = Image::parse(data)?;
    let top = image.symbol("_stack_start").ok_or("no `_stack_start`")?;
    let end = image.symbol("_stack_end").ok_or("no `_stack_end`")?;
    let free = free_ram(top, end, image.symbol("__euninit"))?;
    let tool = objdump()?;
    let out = Command::new(&tool)
        .args(["-d", "--no-show-raw-insn"])
        .arg(elf)
        .output()
        .map_err(|e| format!("{}: {e}", tool.display()))?;
    if !out.status.success() {
        return Err(format!("{} failed", tool.display()));
    }
    let listing = String::from_utf8_lossy(&out.stdout);
    let graph = build_graph(&listing, &image.functions, &|a| image.word(a))?;
    let reset = image
        .section_address(".vector_table")
        .and_then(|table| image.word(table + 4))
        .map(|vector| vector & !1)
        .filter(|vector| image.functions.contains_key(vector))
        .ok_or("no reset vector")?;
    let floor = floor_of(&graph, reset, free);
    if floor.floor == 0 {
        return Err("the reset path found no frame".to_string());
    }
    Ok(floor)
}

/// Print one row's floor; a floor above free RAM or an image it cannot measure fails.
pub fn check(fw: &Firmware) -> bool {
    match measure(&root().join(fw.elf())) {
        Ok(result) if result.floor <= result.free => {
            println!(
                "{:<40} stack floor {:>6} of {:>6} free ({} calls unresolved)",
                fw.name, result.floor, result.free, result.unresolved
            );
            true
        }
        Ok(result) => {
            println!(
                "{:<40} FAILED: stack floor {} exceeds {} free",
                fw.name, result.floor, result.free
            );
            for (name, frame) in &result.path {
                println!("{frame:>8} {name}");
            }
            false
        }
        Err(error) => {
            println!("{:<40} FAILED: stack floor: {error}", fw.name);
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RESET: u32 = 0x100;
    const MIDDLE: u32 = 0x200;
    const LEAF: u32 = 0x300;

    fn functions(entries: &[(u32, &str)]) -> BTreeMap<u32, (u32, String)> {
        entries
            .iter()
            .map(|(a, n)| (*a, (0x40, (*n).to_string())))
            .collect()
    }

    fn graph(listing: &str, entries: &[(u32, &str)]) -> Graph {
        build_graph(listing, &functions(entries), &|_| None).unwrap()
    }

    fn floor(listing: &str, entries: &[(u32, &str)]) -> u32 {
        floor_of(&graph(listing, entries), entries[0].0, 0).floor
    }

    /// The nanoDAP's main-task prologue, whose frame overflowed its RAM on every poll.
    #[test]
    fn a_prologue_sums_every_sp_decrement() {
        let listing = "
     100: \tpush\t{r4, r5, r6, r7, lr}
     102: \tadd\tr7, sp, #0xc
     104: \tpush.w\t{r8, r9, r10, r11}
     108: \tsub.w\tsp, sp, #0x1e20
     10c: \tsub\tsp, #0x1c
";
        assert_eq!(floor(listing, &[(RESET, "main")]), 7776);
    }

    #[test]
    fn the_other_decrement_forms_count() {
        let listing = "
     100: \tstmdb\tsp!, {r4, r5}
     104: \tvpush\t{d8, d9}
     108: \tstr\tr6, [sp, #-0x8]!
     10c: \tsubw\tsp, sp, #0x400
";
        assert_eq!(floor(listing, &[(RESET, "f")]), 8 + 16 + 8 + 0x400);
    }

    #[test]
    fn a_large_armv6_frame_is_a_negative_literal_added_to_sp() {
        let listing = "
     100: \tldr\tr6, [pc, #0x4]
     102: \tadd\tsp, r6
";
        let graph = build_graph(listing, &functions(&[(RESET, "f")]), &|a| {
            (a == 0x108).then_some(0u32.wrapping_sub(0x2000))
        })
        .unwrap();
        assert_eq!(floor_of(&graph, RESET, 0).floor, 0x2000);
    }

    #[test]
    fn a_call_charges_the_frame_so_far_and_a_tail_call_what_the_epilogue_left() {
        let listing = "
     100: \tpush\t{r7, lr}
     102: \tbl\t0x200 <middle>
     106: \tpop\t{r7, lr}
     108: \tb.w\t0x300 <leaf>
     200: \tsub\tsp, #0x10
     202: \tadd\tsp, #0x10
     204: \tbx\tlr
     300: \tsub\tsp, #0x20
";
        let names = [(RESET, "reset"), (MIDDLE, "middle"), (LEAF, "leaf")];
        assert_eq!(floor(listing, &names), 0x20);
    }

    #[test]
    fn a_literal_or_movw_movt_call_is_followed_and_a_dynamic_one_is_dropped() {
        let listing = "
     100: \tpush\t{r7, lr}
     102: \tmovw\tr3, #0x201
     106: \tmovt\tr3, #0x0
     10a: \tblx\tr3
     10c: \tldr\tr0, [r1]
     10e: \tblx\tr0
     200: \tsub\tsp, #0x10
";
        let names = [(RESET, "reset"), (MIDDLE, "middle")];
        let g = graph(listing, &names);
        assert_eq!(floor_of(&g, RESET, 0).floor, 8 + 0x10);
        assert_eq!(g.unresolved, 1);
    }

    /// A false edge would make the lower bound claim a path the program does not have.
    #[test]
    fn a_register_written_after_its_literal_is_not_followed() {
        let listing = "
     100: \tmovw\tr3, #0x201
     104: \tmovt\tr3, #0x0
     108: \tadds\tr3, #0x8
     10a: \tblx\tr3
     200: \tsub\tsp, #0x10
";
        let names = [(RESET, "reset"), (MIDDLE, "middle")];
        assert_eq!(floor(listing, &names), 0);
    }

    #[test]
    fn a_range_thunk_reaches_the_symbol_it_is_named_after() {
        let listing = "
     100: \tbl\t0x200 <__Thumbv6MABSLongThunk_leaf>
     200: \tpush\t{r0, r1}
     300: \tsub\tsp, #0x10
";
        let names = [
            (RESET, "reset"),
            (MIDDLE, "__Thumbv6MABSLongThunk_leaf"),
            (LEAF, "leaf"),
        ];
        assert_eq!(floor(listing, &names), 0x10);
    }

    #[test]
    fn an_executor_loop_reaches_every_task_poll() {
        let listing = "
     100: \tpush\t{r7, lr}
     102: \tldr\tr0, [r1]
     104: \tblx\tr0
     200: \tsub\tsp, #0x10
     300: \tsub\tsp, #0x20
";
        let names = [
            (RESET, "_ZN15embassy_executor6thread8Executor3run"),
            (
                MIDDLE,
                "_ZN15embassy_executor3raw19TaskStorage$LT$F$GT$4poll",
            ),
            (LEAF, "_ZN15embassy_executor3raw19TaskStorage$LT$G$GT$4poll"),
        ];
        let g = graph(listing, &names);
        assert_eq!(floor_of(&g, RESET, 0).floor, 8 + 0x20);
        assert_eq!(g.unresolved, 0);
    }

    /// A renamed executor internal must not read as a small reset path.
    #[test]
    fn an_executor_loop_without_task_polls_is_refused() {
        let listing = "
     100: \tldr\tr0, [r1]
     102: \tblx\tr0
";
        let names = functions(&[(RESET, "_ZN15embassy_executor6thread8Executor3run")]);
        assert!(build_graph(listing, &names, &|_| None).is_err());
    }

    /// A stack end left in flash would read as hundreds of megabytes free.
    #[test]
    fn a_stack_end_below_the_statics_is_refused() {
        assert_eq!(
            free_ram(0x2000_5000, 0x2000_40bc, Some(0x2000_40bc)),
            Ok(3908)
        );
        assert!(free_ram(0x2008_0000, 0x1000_6f20, Some(0x2000_117c)).is_err());
        assert!(free_ram(0x2000_0000, 0x2000_0004, None).is_err());
    }

    #[test]
    fn recursion_ends_and_the_path_names_the_deepest_functions() {
        let listing = "
     100: \tpush\t{r7, lr}
     102: \tbl\t0x200 <middle>
     200: \tsub\tsp, #0x10
     202: \tbl\t0x100 <reset>
";
        let names = [(RESET, "reset"), (MIDDLE, "middle")];
        let result = floor_of(&graph(listing, &names), RESET, 4);
        assert_eq!(result.floor, 8 + 0x10);
        assert_eq!(
            result.path,
            [("reset".to_string(), 8), ("middle".to_string(), 0x10)]
        );
    }
}
