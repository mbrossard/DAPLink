//! Composition macros for a firmware's flash-target registry table.

/// The larger of two `usize` values, in const context (a plain `if a > b` over two consts trips a
/// folded-comparison lint, so this is factored out for the generated scratch computation).
#[doc(hidden)]
pub const fn __max_usize(a: usize, b: usize) -> usize {
    if a > b {
        a
    } else {
        b
    }
}

/// Emit a firmware's flash-target table from one to five `(feature, Target, program-buffer-size)`
/// rows; `umbrella` is the target-less flash spine feature, and the invoker needs `static-cell`.
// rustfmt is non-idempotent over `#[cfg(..)]` nested in macro arms, so arm bodies are hand-formatted.
#[rustfmt::skip]
#[macro_export]
macro_rules! target_table {
    // ---- one row ----
    (
        umbrella = $umbrella:literal,
        count = $count:ident,
        primary = $primary:ident,
        selector = $selector:ident,
        scratch = $scratch:ident,
        primary_ref = $primary_ref:ident,
        table = $table:ident,
        buffer = $buffer:ident,
        rows = [
            ($f0:literal, $t0:path, $b0:expr) $(,)?
        ] $(,)?
    ) => {
        #[cfg(all(feature = $umbrella, not(feature = $f0)))]
        compile_error!(concat!(
            "the bare `", $umbrella, "` feature is the target-less spine; enable a target \
             with `", $f0, "`"
        ));

        #[cfg(feature = $umbrella)]
        const $count: usize = cfg!(feature = $f0) as usize;

        #[cfg(feature = $f0)]
        type $primary = $t0;

        #[cfg(feature = $umbrella)]
        type $selector = $crate::FixedTarget;

        #[cfg(feature = $umbrella)]
        const $scratch: usize = {
            let mut m: usize = 0;
            #[cfg(feature = $f0)] { m = $crate::table::__max_usize(m, $b0); }
            m
        };

        #[cfg(all(feature = $umbrella, feature = $f0))]
        static __DAPLINK_T0: $t0 = $t0;

        #[cfg(all(feature = $umbrella, feature = $f0))]
        static $primary_ref: &$primary = &__DAPLINK_T0;

        #[cfg(feature = $umbrella)]
        fn $table() -> &'static [&'static dyn $crate::Target] {
            static CELL: ::static_cell::StaticCell<[&'static dyn $crate::Target; $count]> =
                ::static_cell::StaticCell::new();
            CELL.init([
                #[cfg(feature = $f0)]
                &__DAPLINK_T0,
            ])
        }

        // `ConstStaticCell`, never `StaticCell`: `init` moves its value through the caller's frame.
        #[cfg(feature = $umbrella)]
        fn $buffer() -> &'static mut [u8; $scratch] {
            static CELL: ::static_cell::ConstStaticCell<[u8; $scratch]> =
                ::static_cell::ConstStaticCell::new([0u8; $scratch]);
            CELL.take()
        }
    };

    // ---- two rows ----
    (
        umbrella = $umbrella:literal,
        count = $count:ident,
        primary = $primary:ident,
        selector = $selector:ident,
        scratch = $scratch:ident,
        primary_ref = $primary_ref:ident,
        table = $table:ident,
        buffer = $buffer:ident,
        rows = [
            ($f0:literal, $t0:path, $b0:expr),
            ($f1:literal, $t1:path, $b1:expr) $(,)?
        ] $(,)?
    ) => {
        #[cfg(all(feature = $umbrella, not(feature = $f0), not(feature = $f1)))]
        compile_error!(concat!(
            "the bare `", $umbrella, "` feature is the target-less spine; enable a target \
             with `", $f0, "` or `", $f1, "`"
        ));

        #[cfg(feature = $umbrella)]
        const $count: usize = cfg!(feature = $f0) as usize + cfg!(feature = $f1) as usize;

        #[cfg(feature = $f0)]
        type $primary = $t0;
        #[cfg(all(feature = $f1, not(feature = $f0)))]
        type $primary = $t1;

        #[cfg(feature = $umbrella)]
        type $selector = $crate::FixedTarget;

        #[cfg(feature = $umbrella)]
        const $scratch: usize = {
            let mut m: usize = 0;
            #[cfg(feature = $f0)] { m = $crate::table::__max_usize(m, $b0); }
            #[cfg(feature = $f1)] { m = $crate::table::__max_usize(m, $b1); }
            m
        };

        #[cfg(all(feature = $umbrella, feature = $f0))]
        static __DAPLINK_T0: $t0 = $t0;
        #[cfg(all(feature = $umbrella, feature = $f1))]
        static __DAPLINK_T1: $t1 = $t1;

        #[cfg(all(feature = $umbrella, feature = $f0))]
        static $primary_ref: &$primary = &__DAPLINK_T0;
        #[cfg(all(feature = $umbrella, feature = $f1, not(feature = $f0)))]
        static $primary_ref: &$primary = &__DAPLINK_T1;

        #[cfg(feature = $umbrella)]
        fn $table() -> &'static [&'static dyn $crate::Target] {
            static CELL: ::static_cell::StaticCell<[&'static dyn $crate::Target; $count]> =
                ::static_cell::StaticCell::new();
            CELL.init([
                #[cfg(feature = $f0)]
                &__DAPLINK_T0,
                #[cfg(feature = $f1)]
                &__DAPLINK_T1,
            ])
        }

        // `ConstStaticCell`, never `StaticCell`: `init` moves its value through the caller's frame.
        #[cfg(feature = $umbrella)]
        fn $buffer() -> &'static mut [u8; $scratch] {
            static CELL: ::static_cell::ConstStaticCell<[u8; $scratch]> =
                ::static_cell::ConstStaticCell::new([0u8; $scratch]);
            CELL.take()
        }
    };

    // ---- three rows ----
    (
        umbrella = $umbrella:literal,
        count = $count:ident,
        primary = $primary:ident,
        selector = $selector:ident,
        scratch = $scratch:ident,
        primary_ref = $primary_ref:ident,
        table = $table:ident,
        buffer = $buffer:ident,
        rows = [
            ($f0:literal, $t0:path, $b0:expr),
            ($f1:literal, $t1:path, $b1:expr),
            ($f2:literal, $t2:path, $b2:expr) $(,)?
        ] $(,)?
    ) => {
        #[cfg(all(feature = $umbrella,
                  not(feature = $f0), not(feature = $f1), not(feature = $f2)))]
        compile_error!(concat!(
            "the bare `", $umbrella, "` feature is the target-less spine; enable a target \
             with `", $f0, "`, `", $f1, "` or `", $f2, "`"
        ));

        #[cfg(feature = $umbrella)]
        const $count: usize = cfg!(feature = $f0) as usize
            + cfg!(feature = $f1) as usize
            + cfg!(feature = $f2) as usize;

        #[cfg(feature = $f0)]
        type $primary = $t0;
        #[cfg(all(feature = $f1, not(feature = $f0)))]
        type $primary = $t1;
        #[cfg(all(feature = $f2, not(feature = $f0), not(feature = $f1)))]
        type $primary = $t2;

        #[cfg(feature = $umbrella)]
        type $selector = $crate::FixedTarget;

        #[cfg(feature = $umbrella)]
        const $scratch: usize = {
            let mut m: usize = 0;
            #[cfg(feature = $f0)] { m = $crate::table::__max_usize(m, $b0); }
            #[cfg(feature = $f1)] { m = $crate::table::__max_usize(m, $b1); }
            #[cfg(feature = $f2)] { m = $crate::table::__max_usize(m, $b2); }
            m
        };

        #[cfg(all(feature = $umbrella, feature = $f0))]
        static __DAPLINK_T0: $t0 = $t0;
        #[cfg(all(feature = $umbrella, feature = $f1))]
        static __DAPLINK_T1: $t1 = $t1;
        #[cfg(all(feature = $umbrella, feature = $f2))]
        static __DAPLINK_T2: $t2 = $t2;

        #[cfg(all(feature = $umbrella, feature = $f0))]
        static $primary_ref: &$primary = &__DAPLINK_T0;
        #[cfg(all(feature = $umbrella, feature = $f1, not(feature = $f0)))]
        static $primary_ref: &$primary = &__DAPLINK_T1;
        #[cfg(all(feature = $umbrella, feature = $f2,
                  not(feature = $f0), not(feature = $f1)))]
        static $primary_ref: &$primary = &__DAPLINK_T2;

        #[cfg(feature = $umbrella)]
        fn $table() -> &'static [&'static dyn $crate::Target] {
            static CELL: ::static_cell::StaticCell<[&'static dyn $crate::Target; $count]> =
                ::static_cell::StaticCell::new();
            CELL.init([
                #[cfg(feature = $f0)]
                &__DAPLINK_T0,
                #[cfg(feature = $f1)]
                &__DAPLINK_T1,
                #[cfg(feature = $f2)]
                &__DAPLINK_T2,
            ])
        }

        // `ConstStaticCell`, never `StaticCell`: `init` moves its value through the caller's frame.
        #[cfg(feature = $umbrella)]
        fn $buffer() -> &'static mut [u8; $scratch] {
            static CELL: ::static_cell::ConstStaticCell<[u8; $scratch]> =
                ::static_cell::ConstStaticCell::new([0u8; $scratch]);
            CELL.take()
        }
    };

    // ---- four rows ----
    (
        umbrella = $umbrella:literal,
        count = $count:ident,
        primary = $primary:ident,
        selector = $selector:ident,
        scratch = $scratch:ident,
        primary_ref = $primary_ref:ident,
        table = $table:ident,
        buffer = $buffer:ident,
        rows = [
            ($f0:literal, $t0:path, $b0:expr),
            ($f1:literal, $t1:path, $b1:expr),
            ($f2:literal, $t2:path, $b2:expr),
            ($f3:literal, $t3:path, $b3:expr) $(,)?
        ] $(,)?
    ) => {
        #[cfg(all(feature = $umbrella,
                  not(feature = $f0), not(feature = $f1),
                  not(feature = $f2), not(feature = $f3)))]
        compile_error!(concat!(
            "the bare `", $umbrella, "` feature is the target-less spine; enable a target \
             with `", $f0, "`, `", $f1, "`, `", $f2, "` or `", $f3, "`"
        ));

        #[cfg(feature = $umbrella)]
        const $count: usize = cfg!(feature = $f0) as usize
            + cfg!(feature = $f1) as usize
            + cfg!(feature = $f2) as usize
            + cfg!(feature = $f3) as usize;

        #[cfg(feature = $f0)]
        type $primary = $t0;
        #[cfg(all(feature = $f1, not(feature = $f0)))]
        type $primary = $t1;
        #[cfg(all(feature = $f2, not(feature = $f0), not(feature = $f1)))]
        type $primary = $t2;
        #[cfg(all(feature = $f3, not(feature = $f0), not(feature = $f1), not(feature = $f2)))]
        type $primary = $t3;

        #[cfg(feature = $umbrella)]
        type $selector = $crate::FixedTarget;

        #[cfg(feature = $umbrella)]
        const $scratch: usize = {
            let mut m: usize = 0;
            #[cfg(feature = $f0)] { m = $crate::table::__max_usize(m, $b0); }
            #[cfg(feature = $f1)] { m = $crate::table::__max_usize(m, $b1); }
            #[cfg(feature = $f2)] { m = $crate::table::__max_usize(m, $b2); }
            #[cfg(feature = $f3)] { m = $crate::table::__max_usize(m, $b3); }
            m
        };

        #[cfg(all(feature = $umbrella, feature = $f0))]
        static __DAPLINK_T0: $t0 = $t0;
        #[cfg(all(feature = $umbrella, feature = $f1))]
        static __DAPLINK_T1: $t1 = $t1;
        #[cfg(all(feature = $umbrella, feature = $f2))]
        static __DAPLINK_T2: $t2 = $t2;
        #[cfg(all(feature = $umbrella, feature = $f3))]
        static __DAPLINK_T3: $t3 = $t3;

        #[cfg(all(feature = $umbrella, feature = $f0))]
        static $primary_ref: &$primary = &__DAPLINK_T0;
        #[cfg(all(feature = $umbrella, feature = $f1, not(feature = $f0)))]
        static $primary_ref: &$primary = &__DAPLINK_T1;
        #[cfg(all(feature = $umbrella, feature = $f2,
                  not(feature = $f0), not(feature = $f1)))]
        static $primary_ref: &$primary = &__DAPLINK_T2;
        #[cfg(all(feature = $umbrella, feature = $f3,
                  not(feature = $f0), not(feature = $f1), not(feature = $f2)))]
        static $primary_ref: &$primary = &__DAPLINK_T3;

        #[cfg(feature = $umbrella)]
        fn $table() -> &'static [&'static dyn $crate::Target] {
            static CELL: ::static_cell::StaticCell<[&'static dyn $crate::Target; $count]> =
                ::static_cell::StaticCell::new();
            CELL.init([
                #[cfg(feature = $f0)]
                &__DAPLINK_T0,
                #[cfg(feature = $f1)]
                &__DAPLINK_T1,
                #[cfg(feature = $f2)]
                &__DAPLINK_T2,
                #[cfg(feature = $f3)]
                &__DAPLINK_T3,
            ])
        }

        // `ConstStaticCell`, never `StaticCell`: `init` moves its value through the caller's frame.
        #[cfg(feature = $umbrella)]
        fn $buffer() -> &'static mut [u8; $scratch] {
            static CELL: ::static_cell::ConstStaticCell<[u8; $scratch]> =
                ::static_cell::ConstStaticCell::new([0u8; $scratch]);
            CELL.take()
        }
    };

    // ---- five rows ----
    (
        umbrella = $umbrella:literal,
        count = $count:ident,
        primary = $primary:ident,
        selector = $selector:ident,
        scratch = $scratch:ident,
        primary_ref = $primary_ref:ident,
        table = $table:ident,
        buffer = $buffer:ident,
        rows = [
            ($f0:literal, $t0:path, $b0:expr),
            ($f1:literal, $t1:path, $b1:expr),
            ($f2:literal, $t2:path, $b2:expr),
            ($f3:literal, $t3:path, $b3:expr),
            ($f4:literal, $t4:path, $b4:expr) $(,)?
        ] $(,)?
    ) => {
        #[cfg(all(feature = $umbrella,
                  not(feature = $f0), not(feature = $f1),
                  not(feature = $f2), not(feature = $f3), not(feature = $f4)))]
        compile_error!(concat!(
            "the bare `", $umbrella, "` feature is the target-less spine; enable a target \
             with `", $f0, "`, `", $f1, "`, `", $f2, "`, `", $f3, "` or `", $f4, "`"
        ));

        #[cfg(feature = $umbrella)]
        const $count: usize = cfg!(feature = $f0) as usize
            + cfg!(feature = $f1) as usize
            + cfg!(feature = $f2) as usize
            + cfg!(feature = $f3) as usize
            + cfg!(feature = $f4) as usize;

        #[cfg(feature = $f0)]
        type $primary = $t0;
        #[cfg(all(feature = $f1, not(feature = $f0)))]
        type $primary = $t1;
        #[cfg(all(feature = $f2, not(feature = $f0), not(feature = $f1)))]
        type $primary = $t2;
        #[cfg(all(feature = $f3, not(feature = $f0), not(feature = $f1), not(feature = $f2)))]
        type $primary = $t3;
        #[cfg(all(feature = $f4, not(feature = $f0), not(feature = $f1),
                  not(feature = $f2), not(feature = $f3)))]
        type $primary = $t4;

        #[cfg(feature = $umbrella)]
        type $selector = $crate::FixedTarget;

        #[cfg(feature = $umbrella)]
        const $scratch: usize = {
            let mut m: usize = 0;
            #[cfg(feature = $f0)] { m = $crate::table::__max_usize(m, $b0); }
            #[cfg(feature = $f1)] { m = $crate::table::__max_usize(m, $b1); }
            #[cfg(feature = $f2)] { m = $crate::table::__max_usize(m, $b2); }
            #[cfg(feature = $f3)] { m = $crate::table::__max_usize(m, $b3); }
            #[cfg(feature = $f4)] { m = $crate::table::__max_usize(m, $b4); }
            m
        };

        #[cfg(all(feature = $umbrella, feature = $f0))]
        static __DAPLINK_T0: $t0 = $t0;
        #[cfg(all(feature = $umbrella, feature = $f1))]
        static __DAPLINK_T1: $t1 = $t1;
        #[cfg(all(feature = $umbrella, feature = $f2))]
        static __DAPLINK_T2: $t2 = $t2;
        #[cfg(all(feature = $umbrella, feature = $f3))]
        static __DAPLINK_T3: $t3 = $t3;
        #[cfg(all(feature = $umbrella, feature = $f4))]
        static __DAPLINK_T4: $t4 = $t4;

        #[cfg(all(feature = $umbrella, feature = $f0))]
        static $primary_ref: &$primary = &__DAPLINK_T0;
        #[cfg(all(feature = $umbrella, feature = $f1, not(feature = $f0)))]
        static $primary_ref: &$primary = &__DAPLINK_T1;
        #[cfg(all(feature = $umbrella, feature = $f2,
                  not(feature = $f0), not(feature = $f1)))]
        static $primary_ref: &$primary = &__DAPLINK_T2;
        #[cfg(all(feature = $umbrella, feature = $f3,
                  not(feature = $f0), not(feature = $f1), not(feature = $f2)))]
        static $primary_ref: &$primary = &__DAPLINK_T3;
        #[cfg(all(feature = $umbrella, feature = $f4,
                  not(feature = $f0), not(feature = $f1),
                  not(feature = $f2), not(feature = $f3)))]
        static $primary_ref: &$primary = &__DAPLINK_T4;

        #[cfg(feature = $umbrella)]
        fn $table() -> &'static [&'static dyn $crate::Target] {
            static CELL: ::static_cell::StaticCell<[&'static dyn $crate::Target; $count]> =
                ::static_cell::StaticCell::new();
            CELL.init([
                #[cfg(feature = $f0)]
                &__DAPLINK_T0,
                #[cfg(feature = $f1)]
                &__DAPLINK_T1,
                #[cfg(feature = $f2)]
                &__DAPLINK_T2,
                #[cfg(feature = $f3)]
                &__DAPLINK_T3,
                #[cfg(feature = $f4)]
                &__DAPLINK_T4,
            ])
        }

        // `ConstStaticCell`, never `StaticCell`: `init` moves its value through the caller's frame.
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
    use crate::targets::rp2040::RP2040;

    // `umbrella` reuses the first row's feature so single-feature test runs stay guard-free.
    target_table! {
        umbrella = "rp2040",
        count = TARGET_COUNT,
        primary = PrimaryTarget,
        selector = Selector,
        scratch = SCRATCH,
        primary_ref = PRIMARY,
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
    fn emitted_registry_is_row_ordered_with_first_enabled_primary() {
        let table = flash_table();
        assert_eq!(table.len(), TARGET_COUNT);
        // The primary reference is the first row's instance, and the table preserves row order.
        let primary: &'static PrimaryTarget = PRIMARY;
        assert!(core::ptr::eq(
            table[0] as *const dyn crate::Target as *const (),
            primary as *const PrimaryTarget as *const ()
        ));
        let _: &RP2040 = PRIMARY;
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
    use crate::targets::stm32f401ce::STM32F401CE;

    target_table! {
        umbrella = "stm32f401ce",
        count = TARGET_COUNT,
        primary = PrimaryTarget,
        selector = Selector,
        scratch = SCRATCH,
        primary_ref = PRIMARY,
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
    fn the_one_row_is_count_primary_and_table() {
        assert_eq!(TARGET_COUNT, 1);
        assert_eq!(SCRATCH, crate::targets::stm32f401ce::PROGRAM_BUFFER_SIZE);
        let _: Selector = crate::FixedTarget;
        let _: &STM32F401CE = PRIMARY;
        let table = flash_table();
        assert_eq!(table.len(), 1);
        assert_eq!(table[0].name(), "STM32F401CE");
        assert_eq!(scratch_buffer().len(), SCRATCH);
    }
}
