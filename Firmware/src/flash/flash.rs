use embassy_futures::yield_now;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_time::{Duration, Instant, Timer};

use crate::config::FlashPins;

/// Macronix MX25L3233F: manufacturer C2, type 20, capacity 16 (32 Mbit).
pub const EXPECTED_JEDEC_ID: [u8; 3] = [0xC2, 0x20, 0x16];
pub const PAGE: usize = 256;


pub struct Flash {
    pins: FlashPins,
}

impl Flash {
    pub fn new(pins: FlashPins) -> Self { Self { pins } }

    /// Take the flash bus. Hold the FPGA in reset (PROGRAMN low) first.
    /// Dropping the returned `Bus` releases all four pins (they go floating).
    pub fn bus(&mut self) -> Bus<'_> {
        let mut cfg = spi::Config::default();
        cfg.frequency = Hertz(8_000_000);
        cfg.mode = spi::MODE_0;
        cfg.miso_pull = Pull::Up;    // undriven MISO reads FF, not random
        cfg.gpio_speed = Speed::Low; // slow edges
        let p = &mut self.pins;
        Bus {
            cs: Output::new(p.cs.reborrow(), Level::High, Speed::Low),
            spi: Spi::new_blocking(p.spi.reborrow(), p.sck.reborrow(),
                                   p.mosi.reborrow(), p.miso.reborrow(), cfg),
        }
    }
}

struct Bus<'d> {
    spi: Spi<'d, Blocking, spi::mode::Master>,
    cs: Output<'d>,
}

pub enum FlashErr { Spi, Timeout }

impl From<spi::Error> for FlashErr {
    fn from(_: spi::Error) -> Self { FlashErr::Spi }
}

impl Bus<'_> {
    /// One transaction: CS low, clock `buf` out while reading back into it, CS high.
    fn command(&mut self, buf: &mut [u8]) -> Result<(), spi::Error> {
        self.cs.set_low();
        let r = self.spi.blocking_transfer_in_place(buf);
        self.cs.set_high();
        r
    }

    /// Wake from deep power-down (0xAB), then read the JEDEC ID (0x9F).
    pub fn read_jedec_id(&mut self) -> Result<[u8; 3], spi::Error> {
        self.command(&mut [0xAB])?;
        cortex_m::asm::delay(10_000); // tRES1 max 10 us
        let mut buf = [0x9F, 0, 0, 0];
        self.command(&mut buf)?;
        Ok([buf[1], buf[2], buf[3]])
    }

    pub fn read_status(&mut self) -> Result<u8, spi::Error> {
        let mut b = [0x05, 0];
        self.command(&mut b)?;
        Ok(b[1])
    }

    fn write_enable(&mut self) -> Result<(), spi::Error> {
        self.command(&mut [0x06])
    }

    /// Poll WIP (status bit 0) until clear, yielding so USB keeps running.
    async fn wait_ready(&mut self, poll: Duration, timeout: Duration) -> Result<(), FlashErr> {
        let start = Instant::now();
        loop {
            if self.read_status()? & 0x01 == 0 { return Ok(()); }
            if start.elapsed() > timeout { return Err(FlashErr::Timeout); }
            if poll.as_ticks() == 0 { yield_now().await } else { Timer::after(poll).await }
        }
    }

    /// Clear the block-protect bits so the whole array is writable.
    pub async fn unprotect(&mut self) -> Result<(), FlashErr> {
        self.write_enable()?;
        self.command(&mut [0x01, 0x00])?;
        self.wait_ready(Duration::from_millis(5), Duration::from_secs(1)).await
    }

    /// Erase the 64 KB block containing `addr`.
    pub async fn erase_64k(&mut self, addr: u32) -> Result<(), FlashErr> {
        self.write_enable()?;
        self.command(&mut addr_cmd(0xD8, addr))?;
        self.wait_ready(Duration::from_millis(10), Duration::from_secs(4)).await
    }

    /// Chip erase (0x60): typ ~20 s, max 50 s.
    pub async fn chip_erase(&mut self) -> Result<(), FlashErr> {
        self.write_enable()?;
        self.command(&mut [0x60])?;
        self.wait_ready(Duration::from_millis(100), Duration::from_secs(80)).await
    }

    pub fn read(&mut self, addr: u32, buf: &mut [u8]) -> Result<(), spi::Error> {
        self.cs.set_low();
        let r = self.spi.blocking_write(&addr_cmd(0x03, addr))
            .and_then(|_| self.spi.blocking_read(buf));
        self.cs.set_high();
        r
    }

    /// Program up to 256 bytes within ONE page (must not cross a page boundary).
    pub async fn program_page(&mut self, addr: u32, data: &[u8]) -> Result<(), FlashErr> {
        self.write_enable()?;
        self.cs.set_low();
        let r = self.spi.blocking_write(&addr_cmd(0x02, addr))
            .and_then(|_| self.spi.blocking_write(data));
        self.cs.set_high();
        r?;
        self.wait_ready(Duration::from_ticks(0), Duration::from_millis(20)).await
    }
}