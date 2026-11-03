//! STM32F401 test image selected at compile time.

#![no_std]
#![no_main]

#[cfg(feature = "conntest")]
mod conntest;
#[cfg(not(feature = "conntest"))]
mod normal;
