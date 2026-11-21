//! Every MCU pin and peripheral the board uses, in one place.
//!
//! This is the only file that names physical pins (PA12, PE15, ...). Each
//! subsystem gets its own struct, and `Board::new` hands each pin to exactly
//! one owner, so two modules can't accidentally drive the same pin.
//!
//! Taking a pin here does not configure it: it stays in its reset state
//! (analog, no pull) until a module actually sets it up.

use embassy_stm32::peripherals::*;
use embassy_stm32::{Peri, Peripherals};

/// USB full-speed, J701.
pub struct UsbPins {
    pub otg: Peri<'static, USB_OTG_FS>,
    pub dp: Peri<'static, PA12>, // D+
    pub dm: Peri<'static, PA11>, // D-
}

/// MX25L3233F NOR flash, U601 (shared with the FPGA's config pins).
pub struct FlashPins {
    pub spi: Peri<'static, SPI4>,
    pub sck: Peri<'static, PE12>,  // via R305
    pub miso: Peri<'static, PE13>, // via R306
    pub mosi: Peri<'static, PE14>, // via R307
    pub cs: Peri<'static, PE15>,   // via R308, 10k pull-up R601
}

/// ECP5 FPGA, U401.
pub struct FpgaPins {
    // Configuration / status
    pub programn: Peri<'static, PB13>, // 4.7k pull-up R401
    pub initn: Peri<'static, PB12>,    // 4.7k pull-up R402
    pub done: Peri<'static, PB14>,     // 4.7k pull-up R403
    pub cfg0: Peri<'static, PB15>,
    pub cfg1: Peri<'static, PD8>,
    pub cfg2: Peri<'static, PD9>,
    // User I/O
    pub reset: Peri<'static, PC11>,
    pub irq: Peri<'static, PC10>,
    // MCU <-> FPGA SPI bus (SPI1)
    pub spi: Peri<'static, SPI1>,
    pub sck: Peri<'static, PG11>,
    pub miso: Peri<'static, PG9>,
    pub mosi: Peri<'static, PD7>,
    pub cs: Peri<'static, PA15>,
}

/// FPGA rail enables and voltage sense.
pub struct PowerPins {
    pub en_1v1: Peri<'static, PF0>,    // U202 EN, 10k pull-up R201
    pub en_2v5: Peri<'static, PF1>,    // U203 EN, 10k pull-up R202
    pub sense_1v1: Peri<'static, PF3>, // ADC3_INP5
    pub sense_2v5: Peri<'static, PF4>, // ADC3_INP9
}

/// Everything, grouped by subsystem.
pub struct Board {
    pub usb: UsbPins,
    pub flash: FlashPins,
    pub fpga: FpgaPins,
    pub power: PowerPins,
    pub adc3: Peri<'static, ADC3>, // die temperature, VREFINT, rail sense
}

impl Board {
    /// Split the chip's peripherals into the board's subsystems.
    pub fn new(p: Peripherals) -> Self {
        Board {
            usb: UsbPins { otg: p.USB_OTG_FS, dp: p.PA12, dm: p.PA11 },
            flash: FlashPins { spi: p.SPI4, sck: p.PE12, miso: p.PE13, mosi: p.PE14, cs: p.PE15 },
            fpga: FpgaPins {
                programn: p.PB13,
                initn: p.PB12,
                done: p.PB14,
                cfg0: p.PB15,
                cfg1: p.PD8,
                cfg2: p.PD9,
                reset: p.PC11,
                irq: p.PC10,
                spi: p.SPI1,
                sck: p.PG11,
                miso: p.PG9,
                mosi: p.PD7,
                cs: p.PA15,
            },
            power: PowerPins { en_1v1: p.PF0, en_2v5: p.PF1, sense_1v1: p.PF3, sense_2v5: p.PF4 },
            adc3: p.ADC3,
        }
    }
}