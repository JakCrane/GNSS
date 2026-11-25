use embassy_stm32::gpio::{Input, Level, Output, OutputOpenDrain, Pull, Speed};
use embassy_time::{Duration, Instant, Timer};
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use crate::config::FpgaPins;

// FPGA register map (must match Gateware/regs.sv)
const REG_ID:        u8 = 0x00;   // always reads ID_VALUE
const REG_STATUS:    u8 = 0x01;   // bit 0 = PLL lock, bit 1 = capture busy
const REG_CAP_CTRL:  u8 = 0x02;   // write bit 0 = 1: start a capture
const REG_CAP_DATA:  u8 = 0x03;   // each read = next captured byte
const REG_DUMPFLAGS: u8 = 0x04;   // bit i = channel i dumped since last read; reading clears
const REG_NCH:       u8 = 0x05;   // number of channels in this bitstream
const REG_CH_SEL:    u8 = 0x0F;   // which channel 0x10..0x32 refer to
const REG_CH_CTRL:   u8 = 0x10;   // then PRN (0x11), code fword (0x12..15), carrier fword (0x16..19)
const REG_CODE:      u8 = 0x12;   // code fword, then carrier fword
const REG_SLEW:      u8 = 0x1A;   // u16 LE: delay the code by N samples
const REG_RESULTS:   u8 = 0x20;   // 6 x int24 LE (ie qe ip qp il ql) + dump count
const REG_DUMPCNT:   u8 = 0x32;   // 8-bit count of 1 ms dumps

const ID_VALUE:        u8 = 0x5A;
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
    // Blocking SPI: for 2-20 byte transfers DMA setup cost more than it saved (tested).
    // The functions using it stay `async`, which costs nothing and leaves the door open.
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
            spi: {
                let mut s = Spi::new_blocking(pins.spi, pins.sck, pins.mosi, pins.miso, spi_cfg);
                // An empty embassy transfer sets 8-bit frames and enables the SPI, which
                // `fifo_transfer` then relies on.
                let _ = s.blocking_transfer_in_place::<u8>(&mut []);
                s
            },
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

    /// One SPI frame: CS low, `buf` out and replaced by what came back, CS high.
    async fn command(&mut self, buf: &mut [u8]) -> Result<(), spi::Error> {
        self.cs.set_low();
        let r = fifo_transfer(buf);
        // Fallback if fifo_transfer ever misbehaves: embassy's byte-at-a-time version.
        // let r = self.spi.blocking_transfer_in_place(buf);
        self.cs.set_high();
        r
    }

    pub async fn read_regs(&mut self, addr: u8, out: &mut [u8]) -> Result<(), spi::Error> {
        let mut buf = [0u8; 33];
        let n = out.len() + 1;
        buf[0] = addr & SPI_READ_MASK; // msb 0 = read (safeguard)
        self.command(&mut buf[..n]).await?;
        out.copy_from_slice(&buf[1..n]);       // keep everything after the header
        Ok(())
    }

    pub async fn write_regs(&mut self, addr: u8, data: &[u8]) -> Result<(), spi::Error> {
        let mut buf = [0u8; 33];
        let n = data.len() + 1;
        buf[0] = addr | SPI_WRITE_BIT;
        buf[1..n].copy_from_slice(data);
        self.command(&mut buf[..n]).await
    }

    pub async fn capture(&mut self) -> Result<(), CaptureErr> {
        self.write_regs(REG_CAP_CTRL, &[CAP_START]).await?;
        let start = Instant::now();
        loop {
            let mut status = [0u8; 1];
            self.read_regs(REG_STATUS, &mut status).await?;
            if status[0] & STATUS_CAP_BUSY == 0 {
                return Ok(());
            }
            if start.elapsed() > Duration::from_millis(100) {
                return Err(CaptureErr::Timeout);
            }
            Timer::after_millis(1).await;
        }
    }

    pub async fn capture_read(&mut self, out: &mut [u8]) -> Result<(), spi::Error> {
        self.read_regs(REG_CAP_DATA, out).await
    }

    /// Configure the tracking channel for a PRN at a given Doppler (Hz).
    /// All registers land together when CS goes high.
    pub async fn set_channel(&mut self, prn: u8, doppler_hz: f64, hold_reset: bool) -> Result<(), spi::Error> {
        let carr = fword(F_IF + doppler_hz).to_le_bytes();
        let code = fword(CHIP_RATE * (1.0 + doppler_hz / F_L1)).to_le_bytes();

        let mut b = [0u8; 10];
        b[0] = hold_reset as u8;            // 0x10 CTRL bit 0
        b[1] = prn;                         // 0x11 PRN
        b[2..6].copy_from_slice(&code);     // 0x12..0x15
        b[6..10].copy_from_slice(&carr);    // 0x16..0x19
        self.write_regs(REG_CH_CTRL, &b).await
    }

    pub async fn dump_count(&mut self) -> Result<u8, spi::Error> {
        let mut c = [0u8; 1];
        self.read_regs(REG_DUMPCNT, &mut c).await?;
        Ok(c[0])
    }

    pub async fn num_channels(&mut self) -> Result<u8, spi::Error> {
        let mut b = [0u8; 1];
        self.read_regs(REG_NCH, &mut b).await?;
        Ok(b[0])
    }

    /// Point the 0x10..0x32 window at channel `ch`.
    pub async fn select(&mut self, ch: u8) -> Result<(), spi::Error> {
        self.write_regs(REG_CH_SEL, &[ch]).await
    }

    /// Which channels have dumped since the last call (bit i = channel i).
    pub async fn dump_flags(&mut self) -> Result<u8, spi::Error> {
        let mut b = [0u8; 1];
        self.read_regs(REG_DUMPFLAGS, &mut b).await?;
        Ok(b[0])
    }

    /// Change the MCU<->FPGA SPI clock (the H7 divides its 64 MHz kernel clock by a
    /// power of two, so 2, 4 or 8 MHz are exact).
    pub fn set_spi_hz(&mut self, hz: u32) -> Result<(), ()> {
        let mut cfg = self.spi.get_current_config();
        cfg.frequency = Hertz(hz);
        self.spi.set_config(&cfg)
    }

    /// Bus integrity test, `n` rounds. Each round reads the ID register (checks MISO)
    /// and writes then reads back CH_SEL (checks MOSI and MISO). Returns the number of
    /// (bad ID reads, bad CH_SEL read-backs). Leaves channel 0 selected.
    pub async fn spi_test(&mut self, n: u32) -> (u32, u32) {
        let (mut bad_id, mut bad_sel) = (0, 0);
        for k in 0..n {
            let mut b = [0u8; 1];
            if self.read_regs(REG_ID, &mut b).await.is_err() || b[0] != ID_VALUE {
                bad_id += 1;
            }
            let v = (k % 8) as u8;
            let _ = self.write_regs(REG_CH_SEL, &[v]).await;
            if self.read_regs(REG_CH_SEL, &mut b).await.is_err() || b[0] != v {
                bad_sel += 1;
            }
        }
        let _ = self.select(0).await;
        (bad_id, bad_sel)
    }

    /// Poll until at least one channel has dumped; returns which ones.
    pub async fn wait_dump_flags(&mut self) -> Result<u8, spi::Error> {
        loop {
            let f = self.dump_flags().await?;
            if f != 0 {
                return Ok(f);
            }
            Timer::after_micros(20).await;
        }
    }
    
    /// Delay the local code by `samples` (1 sample = 1/16 chip).
    pub async fn slew(&mut self, samples: u16) -> Result<(), spi::Error> {
        self.write_regs(REG_SLEW, &samples.to_le_bytes()).await
    }

    pub async fn wait_dumps(&mut self, n: u8) -> Result<(), spi::Error> {
        let start = self.dump_count().await?;
        while self.dump_count().await?.wrapping_sub(start) < n {
            Timer::after_micros(100).await;
        }
        Ok(())
    }
    
    pub async fn wait_new_dump(&mut self, last: u8) -> Result<(), spi::Error> {
        while self.dump_count().await? == last {
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
            self.slew(STEP).await?;
            self.wait_dumps(1).await?;         // this dump straddled the slew: skip it
            let mut p = 0u64;
            for _ in 0..dwell {
                self.wait_dumps(1).await?;
                let c = self.read_correlators().await?;
                p += (c.ip as i64 * c.ip as i64 + c.qp as i64 * c.qp as i64) as u64;
            }
            total += p;
            if p > best.1 { best = (k, p); }
        }

        // N steps took us a full period round, back to the start; slew forward to the best one
        self.slew(best.0 * STEP).await?;
        self.wait_dumps(2).await?;
        Ok((best.0, best.1, total / N as u64))
    }
        
    /// Set absolute code rate (chips/s) and carrier frequency (Hz at IF).
    pub async fn set_nco(&mut self, code_hz: f64, carr_hz: f64) -> Result<(), spi::Error> {
        let mut b = [0u8; 8];
        b[0..4].copy_from_slice(&fword(code_hz).to_le_bytes());   // 0x12..0x15
        b[4..8].copy_from_slice(&fword(carr_hz).to_le_bytes());   // 0x16..0x19
        self.write_regs(REG_CODE, &b).await
    }

    pub async fn read_correlators_raw(&mut self, s: &mut [u8; RESULTS_BYTES]) -> Result<Correlators, spi::Error> {
        self.read_regs(REG_RESULTS, s).await?;
        Ok(Correlators {
            ie: i24(&s[0..]),  qe: i24(&s[3..]),
            ip: i24(&s[6..]),  qp: i24(&s[9..]),
            il: i24(&s[12..]), ql: i24(&s[15..]),
            dumps: s[18],
        })
    }

    pub async fn read_correlators(&mut self) -> Result<Correlators, spi::Error> {
        let mut s = [0u8; RESULTS_BYTES];
        self.read_correlators_raw(&mut s).await
    }
}

/// Full-duplex transfer on SPI1 that keeps the TX FIFO topped up, so bytes go out back
/// to back. embassy's blocking driver sends one byte, waits for it to come all the way
/// back, then sends the next; that left a ~4 us gap after every 4 us byte at 2 MHz.
///
/// Uses the SPI1 registers directly (the `Spi` object still owns the pins and did the
/// configuration). At most `AHEAD` bytes are in flight, well inside the 16-byte RX FIFO,
/// so it can't overrun even if an interrupt delays us.
fn fifo_transfer(buf: &mut [u8]) -> Result<(), spi::Error> {
    use embassy_stm32::pac;
    const AHEAD: usize = 8;
    let r = pac::SPI1;

    if !r.cr1().read().spe() {
        r.cr1().modify(|w| w.set_spe(true));      // set_spi_hz may have turned it off
    }
    while r.sr().read().rxp() {
        let _ = r.rxdr32().read();                 // nothing stale left from before
    }

    let n = buf.len();
    let (mut tx, mut rx) = (0usize, 0usize);
    // Safety net: a 33-byte frame takes ~130 us at 2 MHz. If the peripheral ever stops
    // answering, give up after ~5 ms instead of hanging the whole firmware.
    let deadline = embassy_time::Instant::now() + embassy_time::Duration::from_millis(5);
    while rx < n {
        if embassy_time::Instant::now() > deadline {
            return Err(spi::Error::ModeFault);
        }
        let sr = r.sr().read();
        if sr.ovr() {
            r.ifcr().write(|w| w.set_ovrc(true));
            return Err(spi::Error::Overrun);
        }
        if tx < n && tx - rx < AHEAD && sr.txp() {
            // 8-bit write: one byte into the FIFO, not four
            unsafe { core::ptr::write_volatile(r.txdr32().as_ptr() as *mut u8, buf[tx]) };
            r.cr1().modify(|w| w.set_cstart(true));
            tx += 1;
        }
        if sr.rxp() {
            buf[rx] = unsafe { core::ptr::read_volatile(r.rxdr32().as_ptr() as *const u8) };
            rx += 1;
        }
    }
    Ok(())
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
