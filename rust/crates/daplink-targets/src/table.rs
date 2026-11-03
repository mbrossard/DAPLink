//! Composition macros for a firmware's flash-target registry table.

#[doc(hidden)]
pub const fn __max_usize(a: usize, b: usize) -> usize {
    if a > b {
        a
    } else {
        b
    }
}

#[doc(hidden)]
pub const fn __min_usize(a: usize, b: usize) -> usize {
    if a < b {
        a
    } else {
        b
    }
}

/// Emit the enabled target table and maximum scratch size; firmware chooses its primary separately.
/// `table` and `buffer` may each run once, the caller needs `static_cell`, and its features select rows.
/// An optional `cap` of at least 512 bytes bounds the scratch; the flash manager chunks larger pages.
///
/// ```compile_fail,E0080
/// daplink_targets::target_table! {
///     umbrella = "absent", count = N, selector = S, scratch = SCRATCH, table = t, buffer = b,
///     cap = 256,
///     rows = [("absent", Absent, 0)],
/// }
/// ```
#[rustfmt::skip]
#[macro_export]
macro_rules! target_table {
    (
        umbrella = $umbrella:literal,
        count = $count:ident,
        selector = $selector:ident,
        scratch = $scratch:ident,
        table = $table:ident,
        buffer = $buffer:ident,
        $(cap = $cap:expr,)?
        rows = [ $(($feature:literal, $target:path, $size:expr)),+ $(,)? ] $(,)?
    ) => {
        $(const _: () = assert!($cap >= 512, "target_table! cap must be at least 512 bytes");)?

        #[cfg(all(feature = $umbrella, not(any($(feature = $feature),+))))]
        compile_error!(concat!("the bare `", $umbrella, "` feature needs a target feature"));

        #[cfg(feature = $umbrella)]
        const $count: usize = 0 $(+ cfg!(feature = $feature) as usize)+;

        #[cfg(feature = $umbrella)]
        type $selector = $crate::FixedTarget;

        #[cfg(feature = $umbrella)]
        const $scratch: usize = {
            let mut m: usize = 0;
            $(#[cfg(feature = $feature)] { m = $crate::table::__max_usize(m, $size); })+
            $(m = $crate::table::__min_usize(m, $cap);)?
            m
        };

        #[cfg(feature = $umbrella)]
        fn $table() -> &'static [&'static dyn $crate::Target] {
            static CELL: ::static_cell::StaticCell<[&'static dyn $crate::Target; $count]> =
                ::static_cell::StaticCell::new();
            CELL.init([ $(#[cfg(feature = $feature)] &$target,)+ ])
        }

        // ConstStaticCell avoids moving the buffer through the caller's stack.
        #[cfg(feature = $umbrella)]
        fn $buffer() -> &'static mut [u8; $scratch] {
            static CELL: ::static_cell::ConstStaticCell<[u8; $scratch]> =
                ::static_cell::ConstStaticCell::new([0u8; $scratch]);
            CELL.take()
        }
    };
}

// Expansion coverage for the emitted registry, over the two kept rows.
#[cfg(all(test, feature = "rp2040", feature = "stm32f401ce"))]
mod tests {

    // `umbrella` reuses the first row's feature so single-feature test runs stay guard-free.
    target_table! {
        umbrella = "rp2040",
        count = TARGET_COUNT,
        selector = Selector,
        scratch = SCRATCH,
        table = flash_table,
        buffer = scratch_buffer,
        rows = [
            ("rp2040", crate::targets::rp2040::RP2040, crate::targets::rp2040::PROGRAM_BUFFER_SIZE),
            (
                "stm32f401ce",
                crate::targets::stm32f401ce::STM32F401CE,
                crate::targets::stm32f401ce::PROGRAM_BUFFER_SIZE
            ),
        ],
    }

    #[test]
    fn emitted_facts_match_rows() {
        assert_eq!(TARGET_COUNT, 2);
        assert_eq!(
            SCRATCH,
            crate::table::__max_usize(
                crate::targets::rp2040::PROGRAM_BUFFER_SIZE,
                crate::targets::stm32f401ce::PROGRAM_BUFFER_SIZE
            )
        );
        let _: Selector = crate::FixedTarget;
    }

    #[test]
    fn emitted_registry_preserves_row_order() {
        let table = flash_table();
        assert_eq!(table.len(), TARGET_COUNT);
        assert_eq!(table[0].name(), "RP2040");
        assert_eq!(table[1].name(), "STM32F401CE");
    }

    #[test]
    fn scratch_buffer_hands_out_the_union_max() {
        let buf = scratch_buffer();
        assert_eq!(buf.len(), SCRATCH);
        assert!(buf.iter().all(|&b| b == 0));
    }
}

// The single-row arm, in its own module because every arm emits the same hidden statics.
#[cfg(all(test, feature = "stm32f401ce"))]
mod single_row_tests {

    target_table! {
        umbrella = "stm32f401ce",
        count = TARGET_COUNT,
        selector = Selector,
        scratch = SCRATCH,
        table = flash_table,
        buffer = scratch_buffer,
        rows = [
            (
                "stm32f401ce",
                crate::targets::stm32f401ce::STM32F401CE,
                crate::targets::stm32f401ce::PROGRAM_BUFFER_SIZE
            ),
        ],
    }

    #[test]
    fn the_one_row_is_count_and_table() {
        assert_eq!(TARGET_COUNT, 1);
        assert_eq!(SCRATCH, crate::targets::stm32f401ce::PROGRAM_BUFFER_SIZE);
        let _: Selector = crate::FixedTarget;
        let table = flash_table();
        assert_eq!(table.len(), 1);
        assert_eq!(table[0].name(), "STM32F401CE");
        assert_eq!(scratch_buffer().len(), SCRATCH);
    }
}

// Two rows sharing one algorithm, told apart only by their UF2 families.
#[cfg(all(test, feature = "nrf52840", feature = "nrf52833"))]
mod nrf52_rows_tests {
    use crate::{ImageHint, SelectError, TargetSelector};

    target_table! {
        umbrella = "nrf52840",
        count = TARGET_COUNT,
        selector = Selector,
        scratch = SCRATCH,
        table = flash_table,
        buffer = scratch_buffer,
        rows = [
            (
                "nrf52840",
                crate::targets::nrf52840::NRF52840,
                crate::targets::nrf52840::PROGRAM_BUFFER_SIZE
            ),
            (
                "nrf52833",
                crate::targets::nrf52833::NRF52833,
                crate::targets::nrf52833::PROGRAM_BUFFER_SIZE
            ),
        ],
    }

    #[test]
    fn the_uf2_family_picks_the_row() {
        assert_eq!(TARGET_COUNT, 2);
        assert_eq!(SCRATCH, 0x1000);
        assert_eq!(scratch_buffer().len(), SCRATCH);
        let table = flash_table();
        let mut selector = Selector::default();
        let pick = |s: &mut Selector, hint| s.select(hint, table).map(|t| t.name());
        assert_eq!(
            pick(&mut selector, ImageHint::uf2(0xADA5_2840)),
            Ok("nRF52840")
        );
        assert_eq!(
            pick(&mut selector, ImageHint::uf2(0x621E_937A)),
            Ok("nRF52833")
        );
        // The generic "Nordic NRF52" family names neither part.
        assert_eq!(
            pick(&mut selector, ImageHint::uf2(0x1B57_745F)),
            Err(SelectError::ImageMismatch)
        );
        assert_eq!(
            pick(&mut selector, ImageHint::NONE),
            Err(SelectError::ImageMismatch)
        );
    }
}

// A firmware cap below the widest row's page bounds the scratch, not the rows.
#[cfg(all(test, feature = "nrf52840", feature = "stm32f401ce"))]
mod capped_tests {

    target_table! {
        umbrella = "nrf52840",
        count = TARGET_COUNT,
        selector = Selector,
        scratch = SCRATCH,
        table = flash_table,
        buffer = scratch_buffer,
        cap = 0x400,
        rows = [
            (
                "stm32f401ce",
                crate::targets::stm32f401ce::STM32F401CE,
                crate::targets::stm32f401ce::PROGRAM_BUFFER_SIZE
            ),
            (
                "nrf52840",
                crate::targets::nrf52840::NRF52840,
                crate::targets::nrf52840::PROGRAM_BUFFER_SIZE
            ),
        ],
    }

    #[test]
    fn the_cap_bounds_the_scratch_and_keeps_the_rows() {
        assert_eq!(crate::targets::nrf52840::PROGRAM_BUFFER_SIZE, 0x1000);
        assert_eq!(SCRATCH, 0x400);
        assert_eq!(scratch_buffer().len(), SCRATCH);
        let table = flash_table();
        assert_eq!(TARGET_COUNT, 2);
        assert_eq!(table[0].name(), "STM32F401CE");
        assert_eq!(table[1].name(), "nRF52840");
        let _: Selector = crate::FixedTarget;
    }
}
