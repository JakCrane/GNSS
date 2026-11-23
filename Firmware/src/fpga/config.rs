use embassy_stm32::gpio::{Input, Level, Output, OutputOpenDrain, Pull, Speed};
use embassy_time::{Duration, Instant, Timer};
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use crate::config::FpgaPins;

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
        spi_cfg.frequency = Hertz(1_000_000);
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
        cortex_m::asm::delay(16_000); // busy-wait of 1ms
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

    pub fn read_id(&mut self) -> Result<u8, spi::Error> {
        let mut buf = [0x01, 0x00];
        let r = self.command(&mut buf);
        r.map(|_| buf[1])
    }
    
}

pub enum BootErr { InitTimeout, DoneTimeout }

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
