use embassy_stm32::gpio::{Input, Level, Output, OutputOpenDrain, Pull, Speed};
use embassy_time::{Duration, Instant, Timer};
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use crate::config::FpgaPins;

// FPGA register map (must match Gateware/regs.sv)
const REG_STATUS:   u8 = 0x01;   // bit 0 = PLL lock, bit 1 = capture busy
const REG_CAP_CTRL: u8 = 0x02;   // write bit 0 = 1: start a capture
const REG_CAP_DATA: u8 = 0x03;   // each read = next captured byte
const REG_CH_CTRL:  u8 = 0x10;   // then PRN (0x11), code fword (0x12..15), carrier fword (0x16..19)
const REG_CODE:     u8 = 0x12;   // code fword, then carrier fword
const REG_SLEW:     u8 = 0x1A;   // u16 LE: delay the code by N samples
const REG_RESULTS:  u8 = 0x20;   // 6 x int24 LE (ie qe ip qp il ql) + dump count
const REG_DUMPCNT:  u8 = 0x32;   // 8-bit count of 1 ms dumps

const STATUS_CAP_BUSY: u8 = 0x02;
const CAP_START:       u8 = 0x01;

const SPI_READ_MASK: u8 = 0x7F;  // header bit 7 = 0: read
const SPI_WRITE_BIT: u8 = 0x80;  // header bit 7 = 1: write

/// Bytes in one correlator snapshot (REG_RESULTS..=REG_DUMPCNT).
pub const RESULTS_BYTES: usize = 19;

pub struct Fpga {
    programn: OutputOpenDrain<'static>, // PB13, has a 4.7k pull-up
    initn: Input<'static>,              // PB12
    done: Input<'static>,               // PB14
    cfg: [Output<'static>; 3],          // PB15, PD8, PD9
    irq: Input<'static>,                // PC10
    spi: Spi<'static, Blocking, spi::mode::Master>,
    cs: Output<'static>,
}

impl Fpga {
    /// Take the pins. Hold PROGRAMN low from the start, so the FPGA stays
    /// off the flash bus until we deliberately boot it.
    pub fn new(pins: FpgaPins) -> Self {
        let mut spi_cfg = spi::Config::default();
        spi_cfg.frequency = Hertz(2_000_000);
        spi_cfg.mode = spi::MODE_0;
        spi_cfg.miso_pull = Pull::Down;

        Self {
            programn: OutputOpenDrain::new(pins.programn, Level::Low, Speed::Low),
            initn: Input::new(pins.initn, Pull::None),
            done: Input::new(pins.done, Pull::None),
            cfg: [
                Output::new(pins.cfg0, Level::Low, Speed::Low),
                Output::new(pins.cfg1, Level::Low, Speed::Low),
                Output::new(pins.cfg2, Level::Low, Speed::Low)
            ],
            irq: Input::new(pins.irq, Pull::Down),
            spi: Spi::new_blocking(pins.spi, pins.sck,
                                   pins.mosi, pins.miso, spi_cfg),
            cs: Output::new(pins.cs, Level::High, Speed::Low),
        
        }
    }

    /// The guard from before: PROGRAMN low while it lives.
    pub fn hold_in_reset(&mut self) -> HeldInReset<'_> {
        self.programn.set_low();
        cortex_m::asm::delay(64_000); // busy-wait of 1ms
        HeldInReset { fpga: self }
    }


    /// For a `status` command: (INITN, DONE, IRQ) levels.
    pub fn status(&self) -> (bool, bool, bool) {
        let initn: bool = self.initn.is_high();
        let done: bool = self.done.is_high();
        let irq: bool = self.irq.is_high();
        (initn, done, irq)
    }

    /// Wait for the IRQ pin to change, for the blink test.
    pub async fn wait_irq_change(&mut self) -> bool {
        let previous_value = self.irq.is_high();
        while previous_value == self.irq.is_high() {
            Timer::after_millis(10).await;
        }
        !previous_value
    }

    fn command(&mut self, buf: &mut [u8]) -> Result<(), spi::Error> {
        self.cs.set_low();
        let r = self.spi.blocking_transfer_in_place(buf);
        self.cs.set_high();
        r
    }

    pub fn read_regs(&mut self, addr: u8, out: &mut [u8]) -> Result<(), spi::Error> {
        let mut buf = [0u8; 33];
        let n = out.len() + 1;
        buf[0] = addr & SPI_READ_MASK; // msb 0 = read (safeguard)
        self.command(&mut buf[..n])?;
        out.copy_from_slice(&buf[1..n]);       // keep everything after the header
        Ok(())
    }

    pub fn write_regs(&mut self, addr: u8, data: &[u8]) -> Result<(), spi::Error> {
        let mut buf = [0u8; 33];
        let n = data.len() + 1;
        buf[0] = addr | SPI_WRITE_BIT;
        buf[1..n].copy_from_slice(data);
        self.command(&mut buf[..n])
    }

    pub async fn capture(&mut self) -> Result<(), CaptureErr> {
        self.write_regs(REG_CAP_CTRL, &[CAP_START])?;
        let start = Instant::now();
        loop {
            let mut status = [0u8; 1];
            self.read_regs(REG_STATUS, &mut status)?;
            if status[0] & STATUS_CAP_BUSY == 0 {
                return Ok(());
            }
            if start.elapsed() > Duration::from_millis(100) {
                return Err(CaptureErr::Timeout);
            }
            Timer::after_millis(1).await;
        }
    }

    pub fn capture_read(&mut self, out: &mut [u8]) -> Result<(), spi::Error> {
        self.read_regs(REG_CAP_DATA, out)
    }

    /// Configure the tracking channel for a PRN at a given Doppler (Hz).
    /// All registers land together when CS goes high.
    pub fn set_channel(&mut self, prn: u8, doppler_hz: f64, hold_reset: bool) -> Result<(), spi::Error> {
        let carr = fword(F_IF + doppler_hz).to_le_bytes();
        let code = fword(CHIP_RATE * (1.0 + doppler_hz / F_L1)).to_le_bytes();

        let mut b = [0u8; 10];
        b[0] = hold_reset as u8;            // 0x10 CTRL bit 0
        b[1] = prn;                         // 0x11 PRN
        b[2..6].copy_from_slice(&code);     // 0x12..0x15
        b[6..10].copy_from_slice(&carr);    // 0x16..0x19
        self.write_regs(REG_CH_CTRL, &b)
    }

    pub fn dump_count(&mut self) -> Result<u8, spi::Error> {
        let mut c = [0u8; 1];
        self.read_regs(REG_DUMPCNT, &mut c)?;
        Ok(c[0])
    }

    /// Delay the local code by `samples` (1 sample = 1/16 chip).
    pub fn slew(&mut self, samples: u16) -> Result<(), spi::Error> {
        self.write_regs(REG_SLEW, &samples.to_le_bytes())
    }

    pub async fn wait_dumps(&mut self, n: u8) -> Result<(), spi::Error> {
        let start = self.dump_count()?;
        while self.dump_count()?.wrapping_sub(start) < n {
            Timer::after_micros(100).await;
        }
        Ok(())
    }
    
    pub async fn wait_new_dump(&mut self, last: u8) -> Result<(), spi::Error> {
        while self.dump_count()? == last {
            Timer::after_micros(20).await;
        }
        Ok(())
    }

    /// Step through the whole code in half-chip steps, measuring prompt power
    /// at each. Leaves the channel at the best position.
    /// Returns (best step, best power, mean power).
    pub async fn code_search(&mut self, dwell: u8) -> Result<(u16, u64, u64), spi::Error> {
        const STEP: u16 = 8;                   // half a chip
        const N: u16 = 16368 / STEP;           // 2046 steps = one full code period
        let mut best = (0u16, 0u64);
        let mut total = 0u64;

        for k in 1..=N {
            self.slew(STEP)?;
            self.wait_dumps(1).await?;         // this dump straddled the slew: skip it
            let mut p = 0u64;
            for _ in 0..dwell {
                self.wait_dumps(1).await?;
                let c = self.read_correlators()?;
                p += (c.ip as i64 * c.ip as i64 + c.qp as i64 * c.qp as i64) as u64;
            }
            total += p;
            if p > best.1 { best = (k, p); }
        }

        // N steps took us a full period round, back to the start; slew forward to the best one
        self.slew(best.0 * STEP)?;
        self.wait_dumps(2).await?;
        Ok((best.0, best.1, total / N as u64))
    }
        
    /// Set absolute code rate (chips/s) and carrier frequency (Hz at IF).
    pub fn set_nco(&mut self, code_hz: f64, carr_hz: f64) -> Result<(), spi::Error> {
        let mut b = [0u8; 8];
        b[0..4].copy_from_slice(&fword(code_hz).to_le_bytes());   // 0x12..0x15
        b[4..8].copy_from_slice(&fword(carr_hz).to_le_bytes());   // 0x16..0x19
        self.write_regs(REG_CODE, &b)
    }

    pub fn read_correlators_raw(&mut self, s: &mut [u8; RESULTS_BYTES]) -> Result<Correlators, spi::Error> {
        self.read_regs(REG_RESULTS, s)?;
        Ok(Correlators {
            ie: i24(&s[0..]),  qe: i24(&s[3..]),
            ip: i24(&s[6..]),  qp: i24(&s[9..]),
            il: i24(&s[12..]), ql: i24(&s[15..]),
            dumps: s[18],
        })
    }

    pub fn read_correlators(&mut self) -> Result<Correlators, spi::Error> {
        let mut s = [0u8; RESULTS_BYTES];
        self.read_correlators_raw(&mut s)
    }
}

pub enum BootErr { InitTimeout, DoneTimeout }

pub const CAPTURE_BYTES: usize = 32768;

pub(crate) const FS: f64 = 16.368e6;
pub(crate) const F_IF: f64 = 4.092e6;
pub(crate) const CHIP_RATE: f64 = 1.023e6;
pub(crate) const F_L1: f64 = 1575.42e6;

fn fword(f_hz: f64) -> u32 {
    (f_hz / FS * 4294967296.0 + 0.5) as u32
}

fn i24(b: &[u8]) -> i32 {
    let sign = if b[2] & 0x80 != 0 { 0xFF } else { 0x00 };
    i32::from_le_bytes([b[0], b[1], b[2], sign])
}

pub struct Correlators {
    pub ie: i32, pub qe: i32,
    pub ip: i32, pub qp: i32,
    pub il: i32, pub ql: i32,
    pub dumps: u8,
}

pub enum CaptureErr { Spi, Timeout }

impl From<spi::Error> for CaptureErr {
    fn from(_: spi::Error) -> Self { CaptureErr::Spi }
}

pub struct HeldInReset<'fpga_lifetime> {
    fpga: &'fpga_lifetime mut Fpga,
}

impl Drop for HeldInReset<'_> {
    fn drop(&mut self) {
        self.fpga.programn.set_high();
    }
}

impl HeldInReset<'_> {

    pub fn status(&self) -> (bool, bool, bool) { // forwarding of method for when borrowed to be held
        self.fpga.status()
    }
    /// Set CFG[2:0] to Master SPI, release PROGRAMN, then wait for
    /// INITN high and DONE high (each with a timeout). Returns which step failed.
    pub async fn boot_from_flash(self) -> Result<(), BootErr> {
        self.fpga.cfg[0].set_low();
        self.fpga.cfg[1].set_high();
        self.fpga.cfg[2].set_low();

        self.fpga.programn.set_high();

        let start = Instant::now();
        while self.fpga.initn.is_low() {
            if start.elapsed() > Duration::from_millis(50) {
                return Err(BootErr::InitTimeout);
            }
            Timer::after_micros(100).await;
        }
        let start = Instant::now();
        while self.fpga.done.is_low() {
            if start.elapsed() > Duration::from_secs(2) {
                return Err(BootErr::DoneTimeout);
            }
            Timer::after_micros(100).await;
        }
        Ok(())
    }

}
