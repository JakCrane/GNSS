//! GNSS board bring-up firmware (STM32H743ZIT6).
//!
//! 1. Comes up on the internal HSI oscillator (no dependency on the 25 MHz crystal).
//! 2. Enumerates as a USB CDC-ACM serial port on the USB-C connector (PA11/PA12).
//! 3. When a terminal opens the port: prints "hello world", then reads the JEDEC ID
//!    of the MX25L3233F NOR flash (U601) over SPI4 and prints `true`/`false`.
//!    Press any key in the terminal to run the check again.
//!
//! Flash wiring (from Hardware/MCU + Flash sheets, via 22R series resistors):
//!   PE12 = SPI4_SCK, PE13 = SPI4_MISO, PE14 = SPI4_MOSI, PE11 = CS (GPIO)
//!
//! CS bodge: PE15 (pin 68) is damaged (leaks to GND, can't drive high), so CS is
//! moved to PE11 (pin 64) via a wire to R308's CS-side pad, with R308's MCU end
//! lifted. PE15 is left untouched (reset state: analog, not driven).
//!
//! The flash is shared with the ECP5 (U401) config port, so the FPGA is held in
//! reset (PROGRAMN = PB13 low) to keep it off the SPI bus while the MCU uses it.
//!   PB12 = INITN, PB13 = PROGRAMN, PB14 = DONE
//!
//! FPGA rails are held off at boot (PF0 -> U202 EN / +1V1, PF1 -> U203 EN / +2V5);
//! press `1` / `2` in the terminal to toggle them.

#![no_std]
#![no_main]

use core::fmt::Write as _;

use embassy_executor::Spawner;
use core::cell::Cell;

use embassy_futures::join::join3;
use embassy_stm32::adc::{Adc, SampleTime, Temperature, VrefInt};
use embassy_stm32::gpio::{Flex, Input, Level, Output, Pull, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::usb::{self, Driver};
use embassy_stm32::{bind_interrupts, peripherals, Config};
use embassy_futures::yield_now;
use embassy_time::{Duration, Instant, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Sender, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use heapless::String;

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

/// Expected JEDEC ID for Macronix MX25L3233F: manufacturer 0xC2, type 0x20, capacity 0x16 (32 Mbit).
const EXPECTED_JEDEC_ID: [u8; 3] = [0xC2, 0x20, 0x16];

struct NorFlash<'d> {
    spi: Spi<'d, Blocking, spi::mode::Master>,
    cs: Flex<'d>,
}

impl NorFlash<'_> {
    fn command(&mut self, buf: &mut [u8]) -> Result<(), spi::Error> {
        self.cs.set_low();
        let r = self.spi.blocking_transfer_in_place(buf);
        self.cs.set_high();
        r
    }

    /// Clock a byte with CS *high*: the flash must tri-state SO, so with the
    /// MISO pull-up this should read FF. 00 means something else holds MISO low.
    fn miso_idle(&mut self) -> Result<u8, spi::Error> {
        let mut b = [0xFF];
        self.spi.blocking_transfer_in_place(&mut b)?;
        Ok(b[0])
    }

    /// Wake from deep power-down (0xAB) and then read the JEDEC ID (0x9F).
    fn read_jedec_id(&mut self) -> Result<[u8; 3], spi::Error> {
        self.command(&mut [0xAB])?;
        // tRES1 is max 10 us; a tiny busy wait is plenty.
        cortex_m::asm::delay(10_000);
        let mut buf = [0x9F, 0, 0, 0];
        self.command(&mut buf)?;
        Ok([buf[1], buf[2], buf[3]])
    }

    fn read_status(&mut self) -> Result<u8, spi::Error> {
        let mut b = [0x05, 0];
        self.command(&mut b)?;
        Ok(b[1])
    }

    fn write_enable(&mut self) -> Result<(), spi::Error> {
        self.command(&mut [0x06])
    }

    /// Poll WIP (status bit 0) until clear, yielding to the executor so USB keeps running.
    async fn wait_ready(&mut self, poll: Duration, timeout: Duration) -> Result<(), FlashErr> {
        let start = Instant::now();
        loop {
            if self.read_status()? & 0x01 == 0 {
                return Ok(());
            }
            if start.elapsed() > timeout {
                return Err(FlashErr::Timeout);
            }
            if poll.as_ticks() == 0 { yield_now().await } else { Timer::after(poll).await }
        }
    }

    /// Clear the block-protect bits (status register) so the whole array is writable.
    async fn unprotect(&mut self) -> Result<(), FlashErr> {
        self.write_enable()?;
        self.command(&mut [0x01, 0x00])?;
        self.wait_ready(Duration::from_millis(5), Duration::from_secs(1)).await
    }

    /// Chip erase (0x60). MX25L3233F: typ ~20 s, max 50 s.
    async fn chip_erase(&mut self) -> Result<(), FlashErr> {
        self.write_enable()?;
        self.command(&mut [0x60])?;
        self.wait_ready(Duration::from_millis(100), Duration::from_secs(80)).await
    }

    /// Read (0x03) `buf.len()` bytes starting at `addr`.
    fn read(&mut self, addr: u32, buf: &mut [u8]) -> Result<(), spi::Error> {
        self.cs.set_low();
        let hdr = [0x03, (addr >> 16) as u8, (addr >> 8) as u8, addr as u8];
        let r = self.spi.blocking_write(&hdr).and_then(|_| self.spi.blocking_read(buf));
        self.cs.set_high();
        r
    }

    /// Page program (0x02) up to 256 bytes within one page.
    async fn program_page(&mut self, addr: u32, data: &[u8]) -> Result<(), FlashErr> {
        self.write_enable()?;
        self.cs.set_low();
        let hdr = [0x02, (addr >> 16) as u8, (addr >> 8) as u8, addr as u8];
        let r = self.spi.blocking_write(&hdr).and_then(|_| self.spi.blocking_write(data));
        self.cs.set_high();
        r?;
        // Page program: typ 0.6 ms, max 3 ms.
        self.wait_ready(Duration::from_ticks(0), Duration::from_millis(20)).await
    }
}

enum FlashErr {
    Spi,
    Timeout,
}

impl From<spi::Error> for FlashErr {
    fn from(_: spi::Error) -> Self {
        FlashErr::Spi
    }
}

const FLASH_SIZE: u32 = 4 * 1024 * 1024; // MX25L3233F: 32 Mbit
const PAGE: usize = 256;

/// Test pattern: depends on every address bit, so stuck or swapped address
/// lines and wrong-page writes all show up as mismatches.
fn pattern(addr: u32) -> u8 {
    (addr ^ (addr >> 8).wrapping_mul(0x9D) ^ (addr >> 16).wrapping_mul(0x3B) ^ 0x5A) as u8
}

/// Magic left in uninitialised RAM to request a jump to the ROM DFU bootloader after reset.
const DFU_MAGIC: u32 = 0xB007_DF00;
#[unsafe(link_section = ".uninit.DFU_REQUEST")]
static mut DFU_REQUEST: core::mem::MaybeUninit<u32> = core::mem::MaybeUninit::uninit();

/// Reset into the STM32H743 system-memory bootloader (USB DFU), no buttons needed.
fn reboot_to_dfu() -> ! {
    unsafe { (*crash_log()).kind = 3 };
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!(DFU_REQUEST).cast::<u32>(), DFU_MAGIC) };
    cortex_m::peripheral::SCB::sys_reset();
}


// ---- Crash capture: survives a reset (not a power cycle) in uninitialised RAM ----
const CRASH_MAGIC: u32 = 0xC0FF_EE01;
#[repr(C)]
struct CrashLog {
    magic: u32,
    boots: u32,
    kind: u32, // 0 none, 1 panic, 2 HardFault
    pc: u32,
    last_addr: u32, // last flash address the scan/test touched
    len: u32,
    msg: [u8; 120],
}
#[unsafe(link_section = ".uninit.CRASH")]
static mut CRASH: core::mem::MaybeUninit<CrashLog> = core::mem::MaybeUninit::uninit();
/// RCC_RSR reset flags captured at boot.
static RESET_FLAGS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

fn crash_log() -> *mut CrashLog {
    core::ptr::addr_of_mut!(CRASH).cast::<CrashLog>()
}
fn set_last_addr(a: u32) {
    unsafe { core::ptr::write_volatile(core::ptr::addr_of_mut!((*crash_log()).last_addr), a) };
}

struct MsgWriter<'a> { buf: &'a mut [u8], len: usize }
impl core::fmt::Write for MsgWriter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for &b in s.as_bytes() {
            if self.len < self.buf.len() { self.buf[self.len] = b; self.len += 1; }
        }
        Ok(())
    }
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    unsafe {
        let c = &mut *crash_log();
        let mut w = MsgWriter { buf: &mut c.msg, len: 0 };
        let _ = write!(w, "{}", info);
        c.len = w.len as u32;
        c.kind = 1;
        c.pc = 0;
    }
    cortex_m::peripheral::SCB::sys_reset();
}

#[cortex_m_rt::exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    unsafe {
        let c = &mut *crash_log();
        c.kind = 2;
        c.pc = ef.pc();
        c.len = 0;
    }
    cortex_m::peripheral::SCB::sys_reset();
}

/// Summary of the previous reset, printed once per session.
fn boot_report(out: &mut String<256>) {
    let f = RESET_FLAGS.load(core::sync::atomic::Ordering::Relaxed);
    let c = unsafe { &mut *crash_log() };
    let _ = write!(out, "boot #{} reset flags {:08X}:", c.boots, f);
    for (bit, name) in [(23, " POR"), (22, " PIN"), (21, " BOR"), (24, " SOFT"), (26, " IWDG"), (28, " WWDG")] {
        if f & (1 << bit) != 0 { let _ = out.push_str(name); }
    }
    let _ = out.push_str("\r\n");
    match c.kind {
        1 => {
            let n = (c.len as usize).min(c.msg.len());
            let _ = write!(out, "LAST CRASH: panic: {}\r\n", core::str::from_utf8(&c.msg[..n]).unwrap_or("?"));
        }
        2 => { let _ = write!(out, "LAST CRASH: HardFault at PC {:08X}\r\n", c.pc); }
        3 => { let _ = out.push_str("last reset: DFU reboot requested (!u)\r\n"); }
        _ => {}
    }
    if c.kind != 0 || f & (1 << 26) != 0 {
        let _ = write!(out, "last flash address touched: {:06X}\r\n", c.last_addr);
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Before touching any peripheral: if the last reset was a DFU request, jump to ROM.
    unsafe {
        let req = core::ptr::addr_of_mut!(DFU_REQUEST).cast::<u32>();
        if core::ptr::read_volatile(req) == DFU_MAGIC {
            core::ptr::write_volatile(req, 0);
            cortex_m::asm::bootload(0x1FF0_9800 as *const u32); // AN2606: H74x/H75x system memory
        }
    }
    // Crash log / reset cause (RCC_RSR at 0x5802_44D0; RMVF = bit 16 clears it).
    unsafe {
        let rsr = 0x5802_44D0 as *mut u32;
        let f = core::ptr::read_volatile(rsr);
        RESET_FLAGS.store(f, core::sync::atomic::Ordering::Relaxed);
        core::ptr::write_volatile(rsr, f | (1 << 16));
        let c = &mut *crash_log();
        if c.magic != CRASH_MAGIC || f & (1 << 23) != 0 {
            core::ptr::write_bytes(crash_log(), 0, 1);
            c.magic = CRASH_MAGIC;
        }
        c.boots = c.boots.wrapping_add(1);
    }
    let mut boot_msg: String<256> = String::new();
    boot_report(&mut boot_msg);
    unsafe { (*crash_log()).kind = 0 };
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        // Low-power bring-up: HSI (64 MHz) / 4 = 16 MHz core, lowest voltage scale.
        // USB uses HSI48 (trimmed by the CRS from USB SOF), so it is unaffected.
        // 16 MHz stays above the ~14.2 MHz AHB minimum for USB OTG FS.
        config.rcc.hsi = Some(HSIPrescaler::DIV4);
        config.rcc.hsi48 = Some(Hsi48Config { sync_from_usb: true });
        config.rcc.sys = Sysclk::HSI;
        config.rcc.voltage_scale = VoltageScale::Scale3;
        config.rcc.mux.usbsel = mux::Usbsel::HSI48;
        // ADC kernel clock from the peripheral clock (HSI) for the temp/VDDA monitor.
        config.rcc.mux.persel = mux::Persel::HSI;
        config.rcc.mux.adcsel = mux::Adcsel::PER;
    }
    let mut p = embassy_stm32::init(config);

    // ---- FPGA rails: off until the firmware asks for them ----
    // R201/R202 pull the regulator enables high, so drive them low first thing.
    // (Rails are still briefly on between power-up and here, and while in DFU.)
    let mut rails = Rails {
        en_1v1: Output::new(p.PF0, Level::Low, Speed::Low), // U202 EN -> +1V1 (FPGA core)
        en_2v5: Output::new(p.PF1, Level::Low, Speed::Low), // U203 EN -> +2V5 (FPGA aux)
    };

    // ---- FPGA: sample config status, then hold it in reset off the flash bus ----
    let mut fpga_status: String<192> = String::new();
    {
        let initn = Input::new(p.PB12, Pull::None);
        let done = Input::new(p.PB14, Pull::None);
        // Pull-up sanity check: PE13 (MISO) as plain GPIO input with pull-up, CS (PE11) high.
        let pe13_level = {
            let _cs = Output::new(p.PE11.reborrow(), Level::High, Speed::Low);
            let miso = Input::new(p.PE13.reborrow(), Pull::Up);
            cortex_m::asm::delay(64_000); // ~4 ms at 16 MHz
            miso.is_high() as u8
        };
        let _ = write!(
            fpga_status,
            "MISO GPIO pull-up test (CS high): {} (expect 1)\r\n",
            pe13_level
        );
        // Bit-banged JEDEC read: bypasses SPI4 entirely (GPIO only, mode 0, ~100 kHz).
        {
            let mut cs = Output::new(p.PE11.reborrow(), Level::High, Speed::Low);
            let mut sck = Output::new(p.PE12.reborrow(), Level::Low, Speed::Low);
            let mut mosi = Output::new(p.PE14.reborrow(), Level::Low, Speed::Low);
            let miso = Input::new(p.PE13.reborrow(), Pull::Up);
            let half = || cortex_m::asm::delay(320); // ~20 us at 16 MHz
            let mut xfer = |out: u8| -> u8 {
                let mut inb = 0u8;
                for i in (0..8).rev() {
                    if out & (1 << i) != 0 { mosi.set_high() } else { mosi.set_low() }
                    half();
                    sck.set_high();
                    inb = (inb << 1) | miso.is_high() as u8;
                    half();
                    sck.set_low();
                }
                inb
            };
            cs.set_low(); xfer(0xAB); cs.set_high();
            cortex_m::asm::delay(10_000);
            cs.set_low();
            xfer(0x9F);
            let id = [xfer(0), xfer(0), xfer(0)];
            cs.set_high();
            let _ = write!(
                fpga_status,
                "Bit-bang JEDEC ID: {:02X} {:02X} {:02X} (expect C2 20 16)\r\n",
                id[0], id[1], id[2]
            );
        }
        let _ = write!(
            fpga_status,
            "FPGA at boot: INITN={} DONE={}\r\n",
            initn.is_high() as u8,
            done.is_high() as u8
        );
    }
    let _fpga_programn = Output::new(p.PB13, Level::Low, Speed::Low);
    // Give the ECP5 time to tri-state its config pins.
    Timer::after_millis(5).await;

    // ---- NOR flash on SPI4 ----
    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = Hertz(8_000_000); // 16 MHz PCLK / 2; flash is rated far higher
    spi_cfg.mode = spi::MODE_0;
    // Pull-up on MISO: a flash that isn't driving SO reads FF, not a floating 00.
    spi_cfg.miso_pull = Pull::Up;
    spi_cfg.gpio_speed = Speed::Low; // slow edges: SCK/MOSI run ~31 mm under J701
    let flash_spi = Spi::new_blocking(p.SPI4, p.PE12, p.PE14, p.PE13, spi_cfg);
    let mut flash_cs = Flex::new(p.PE11); // CS bodge wire (was PE15)
    flash_cs.set_high();
    flash_cs.set_as_output(Speed::VeryHigh);
    let mut flash = NorFlash { spi: flash_spi, cs: flash_cs };

    // ---- USB CDC-ACM ----
    let mut ep_out_buffer = [0u8; 256];
    let mut usb_cfg = usb::Config::default();
    // VBUS is not routed to PA9 on this board.
    usb_cfg.vbus_detection = false;
    let driver = Driver::new_fs(p.USB_OTG_FS, Irqs, p.PA12, p.PA11, &mut ep_out_buffer, usb_cfg);

    let mut config = embassy_usb::Config::new(0x1209, 0x0001); // pid.codes test VID/PID
    config.manufacturer = Some("GNSS");
    config.product = Some("GNSS receiver bring-up");
    config.serial_number = Some("0001");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no MS OS descriptors
        &mut control_buf,
    );
    let class = CdcAcmClass::new(&mut builder, &mut state, 64);
    let mut usb = builder.build();

    // ---- Die temperature + VDDA monitor (ADC3: internal sensor and VREFINT) ----
    let adc = Adc::new(p.ADC3);
    let mut monitor = Monitor { vref: adc.enable_vrefint(), temp: adc.enable_temperature(), adc };

    // Watchdog: a CPU lock-up (no executor progress for 4 s) resets the chip
    // instead of hanging silently; the crash log above then reports it.
    let mut wdg = embassy_stm32::wdg::IndependentWatchdog::new(p.IWDG1, 4_000_000);
    wdg.unleash();
    let wdg_fut = async {
        loop {
            wdg.pet();
            Timer::after_millis(500).await;
        }
    };

    let usb_fut = usb.run();

    // The CDC class is split so reading keys and writing the log run as two
    // independent loops: no USB transfer is ever cancelled half-way.
    let (mut tx, mut rx) = class.split();
    let key = Cell::new(0u8); // last key pressed, 0 = none
    let rx_log = Cell::new(0u32); // len<<16 | byte0<<8 | byte1 of the last packet

    let rx_fut = async {
        let mut buf = [0u8; 64];
        loop {
            rx.wait_connection().await;
            loop {
                match rx.read_packet(&mut buf).await {
                    Ok(n) if n > 0 => {
                        // Log what arrived, so stray host traffic (ModemManager probes) is visible.
                        rx_log.set((n as u32) << 16 | (buf[0] as u32) << 8 | if n > 1 { buf[1] as u32 } else { 0 });
                        if n == 1 {
                            key.set(buf[0]);
                        } else if n == 2 && buf[0] == b'!' {
                            key.set(buf[1] | 0x80); // armed form for dangerous commands
                        }
                    }
                    Ok(_) => {}
                    Err(_) => break, // disconnected
                }
            }
        }
    };
    let tx_fut = async {
        loop {
            tx.wait_connection().await;
            key.set(0);
            let _ = session(&mut tx, &mut flash, &mut monitor, &mut rails, &fpga_status, &boot_msg, &key, &rx_log).await;
        }
    };
    embassy_futures::join::join(join3(usb_fut, rx_fut, tx_fut), wdg_fut).await;
}

async fn session<'d, D: embassy_usb::driver::Driver<'d>>(
    tx: &mut Sender<'d, D>,
    flash: &mut NorFlash<'_>,
    monitor: &mut Monitor<'_>,
    rails: &mut Rails<'_>,
    fpga_status: &str,
    boot_msg: &str,
    key: &Cell<u8>,
    rx_log: &Cell<u32>,
) -> Result<(), EndpointError> {
    // Give the host terminal a moment after it raises DTR.
    Timer::after_millis(100).await;
    write_str(tx, "hello world\r\n").await?;
    write_str(tx, boot_msg).await?;
    write_str(tx, fpga_status).await?;

    let mut check_flash = true;
    let mut ticks = 0u32;
    loop {
        // Terminal closed: stop writing, so we never block on a port nobody reads.
        if !tx.dtr() {
            return Ok(());
        }

        if check_flash {
            check_flash = false;
            let mut line: String<96> = String::new();
            match flash.read_jedec_id() {
                Ok(id) => {
                    let ok = id == EXPECTED_JEDEC_ID;
                    let _ = write!(
                        line,
                        "{}  (JEDEC ID {:02X} {:02X} {:02X}, expected C2 20 16)\r\n",
                        ok, id[0], id[1], id[2]
                    );
                }
                Err(_) => {
                    let _ = write!(line, "false  (SPI error)\r\n");
                }
            }
            write_str(tx, &line).await?;
            let mut idle: String<48> = String::new();
            match flash.miso_idle() {
                Ok(b) => { let _ = write!(idle, "MISO with CS high: {:02X} (expect FF)\r\n", b); }
                Err(_) => { let _ = write!(idle, "MISO with CS high: SPI error\r\n"); }
            }
            write_str(tx, &idle).await?;
            write_str(tx, "keys (one byte per packet): i = re-check ID, b = blank re-scan, c = toggle CS; armed (send \"!x\"): !t = erase/write/verify, !u = reboot to DFU, !1/!2 = toggle +1V1/+2V5\r\n").await?;
        }

        let r = rx_log.replace(0);
        if r != 0 {
            let mut m: String<64> = String::new();
            let _ = write!(m, "rx: {} byte(s), first {:02X} {:02X}\r\n", r >> 16, (r >> 8) & 0xFF, r & 0xFF);
            write_str(tx, &m).await?;
        }
        match key.replace(0) {
            0 => {}
            k if k == b't' | 0x80 => {
                let r = flash_test(tx, flash).await;
                write_str(tx, match r {
                    Ok(true) => "FLASH TEST: PASS\r\n",
                    Ok(false) => "FLASH TEST: FAIL\r\n",
                    Err(_) => "FLASH TEST: aborted (USB)\r\n",
                }).await?;
            }
            k if k == b'u' | 0x80 => {
                write_str(tx, "rebooting into DFU bootloader...\r\n").await?;
                Timer::after_millis(100).await;
                reboot_to_dfu();
            }
            b'b' => {
                for (i, (hz, spd, name)) in [
                    (8_000_000u32, Speed::VeryHigh, "VeryHigh"),
                    (8_000_000, Speed::Low, "Low"),
                    (8_000_000, Speed::Low, "Low"),
                    (1_000_000, Speed::Low, "Low"),
                ].into_iter().enumerate() {
                    let mut cfg = spi::Config::default();
                    cfg.frequency = Hertz(hz);
                    cfg.mode = spi::MODE_0;
                    cfg.miso_pull = Pull::Up;
                    cfg.gpio_speed = spd;
                    let _ = flash.spi.set_config(&cfg);
                    let mut m: String<64> = String::new();
                    let _ = write!(m, "blank scan {} @ {} kHz, slew {}:\r\n", i + 1, hz / 1000, name);
                    write_str(tx, &m).await?;
                    blank_scan(tx, flash).await?;
                }
                let mut cfg = spi::Config::default();
                cfg.frequency = Hertz(8_000_000);
                cfg.mode = spi::MODE_0;
                cfg.miso_pull = Pull::Up;
                cfg.gpio_speed = Speed::Low;
                let _ = flash.spi.set_config(&cfg);
            }
            b'c' => {
                flash.cs.toggle();
                let mut m: String<64> = String::new();
                let _ = write!(m, "CS (PE11) now driven {}\r\n", if flash.cs.is_set_high() { "HIGH (expect 3.3 V)" } else { "LOW (expect 0 V)" });
                write_str(tx, &m).await?;
            }
            k if k == b'1' | 0x80 || k == b'2' | 0x80 => {
                if k & 0x7F == b'1' { rails.en_1v1.toggle() } else { rails.en_2v5.toggle() }
                let mut m: String<48> = String::new();
                let _ = write!(m, "rails: +1V1 {}, +2V5 {}\r\n", on_off(rails.en_1v1.is_set_high()), on_off(rails.en_2v5.is_set_high()));
                write_str(tx, &m).await?;
            }
            b'i' => check_flash = true,
            _ => {} // ignore everything else (host probes etc.)
        }

        // Temperature / VDDA / rail state once a second.
        if ticks % 10 == 0 {
            let (t, vdda_mv) = monitor.read();
            let mut m: String<96> = String::new();
            let _ = write!(
                m,
                "MCU temp {:.1} C, VDDA {}.{:03} V, 1V1 {}, 2V5 {}, CS {}\r\n",
                t, vdda_mv / 1000, vdda_mv % 1000,
                on_off(rails.en_1v1.is_set_high()), on_off(rails.en_2v5.is_set_high()),
                if flash.cs.is_set_high() { "HIGH" } else { "LOW" }
            );
            write_str(tx, &m).await?;
        }
        ticks = ticks.wrapping_add(1);
        Timer::after_millis(100).await;
    }
}

struct Rails<'d> {
    en_1v1: Output<'d>,
    en_2v5: Output<'d>,
}

fn on_off(b: bool) -> &'static str {
    if b { "ON" } else { "off" }
}

struct Monitor<'d> {
    adc: Adc<'d, peripherals::ADC3>,
    vref: VrefInt,
    temp: Temperature,
}

impl Monitor<'_> {
    /// Returns (die temperature in C, VDDA in mV), using the factory calibration
    /// values (16-bit, taken at VDDA = 3.3 V; TS_CAL1 at 30 C, TS_CAL2 at 110 C).
    fn read(&mut self) -> (f32, u32) {
        const TS_CAL1: *const u16 = 0x1FF1_E820 as *const u16;
        const TS_CAL2: *const u16 = 0x1FF1_E840 as *const u16;
        const VREFINT_CAL: *const u16 = 0x1FF1_E860 as *const u16;
        let (cal1, cal2, vcal) = unsafe {
            (
                core::ptr::read_volatile(TS_CAL1) as f32,
                core::ptr::read_volatile(TS_CAL2) as f32,
                core::ptr::read_volatile(VREFINT_CAL) as u32,
            )
        };
        let st = SampleTime::CYCLES810_5;
        let vref_raw = self.adc.blocking_read(&mut self.vref, st).max(1) as u32;
        let temp_raw = self.adc.blocking_read(&mut self.temp, st) as f32;
        // VDDA from VREFINT; rescale the temp reading to the 3.3 V calibration conditions.
        let vdda_mv = 3300 * vcal / vref_raw;
        let temp_at_3v3 = temp_raw * vdda_mv as f32 / 3300.0;
        let t = (temp_at_3v3 - cal1) * (110.0 - 30.0) / (cal2 - cal1) + 30.0;
        (t, vdda_mv)
    }
}

/// Full-chip test: erase, blank check, program a pattern, verify, erase again.
/// Returns Ok(pass), or Err if the USB connection went away.
async fn flash_test<'d, D: embassy_usb::driver::Driver<'d>>(
    tx: &mut Sender<'d, D>,
    flash: &mut NorFlash<'_>,
) -> Result<bool, EndpointError> {
    let mut m: String<128> = String::new();
    macro_rules! say {
        ($($arg:tt)*) => {{
            m.clear();
            let _ = write!(m, $($arg)*);
            write_str(tx, &m).await?;
        }};
    }

    let sr = flash.read_status().unwrap_or(0xFF);
    say!("[1/5] status register {:02X} (block-protect bits {:X}), clearing protection\r\n", sr, (sr >> 2) & 0xF);
    if flash.unprotect().await.is_err() {
        say!("  unprotect failed\r\n");
        return Ok(false);
    }

    say!("[2/5] chip erase (takes ~20-50 s)...\r\n");
    let t = Instant::now();
    match flash.chip_erase().await {
        Ok(()) => say!("  erased in {} ms\r\n", t.elapsed().as_millis()),
        Err(FlashErr::Timeout) => { say!("  erase TIMEOUT\r\n"); return Ok(false); }
        Err(FlashErr::Spi) => { say!("  erase SPI error\r\n"); return Ok(false); }
    }

    let mut buf = [0u8; PAGE];
    let mut page = [0u8; PAGE];

    // Blank check: every byte must read FF.
    say!("[3/5] blank check...\r\n");
    let t = Instant::now();
    let mut bad = 0u32;
    let mut first_bad: Option<(u32, u8)> = None;
    for addr in (0..FLASH_SIZE).step_by(PAGE) {
        set_last_addr(addr);
        if flash.read(addr, &mut buf).is_err() { say!("  SPI error at {:06X}\r\n", addr); return Ok(false); }
        for (i, &b) in buf.iter().enumerate() {
            if b != 0xFF {
                bad += 1;
                if first_bad.is_none() { first_bad = Some((addr + i as u32, b)); }
            }
        }
        if addr % (FLASH_SIZE / 4) == 0 && addr != 0 { say!("  {}%\r\n", addr * 100 / FLASH_SIZE); }
        yield_now().await;
    }
    match first_bad {
        None => say!("  all {} bytes FF ({} ms)\r\n", FLASH_SIZE, t.elapsed().as_millis()),
        Some((a, b)) => { say!("  {} non-FF bytes, first at {:06X} = {:02X}\r\n", bad, a, b); return Ok(false); }
    }

    // Program the pattern.
    say!("[4/5] programming pattern...\r\n");
    let t = Instant::now();
    for addr in (0..FLASH_SIZE).step_by(PAGE) {
        for (i, b) in page.iter_mut().enumerate() { *b = pattern(addr + i as u32); }
        if flash.program_page(addr, &page).await.is_err() {
            say!("  program failed at {:06X}\r\n", addr);
            return Ok(false);
        }
        if addr % (FLASH_SIZE / 4) == 0 && addr != 0 { say!("  {}%\r\n", addr * 100 / FLASH_SIZE); }
    }
    say!("  programmed in {} ms\r\n", t.elapsed().as_millis());

    // Verify.
    say!("[5/5] verifying...\r\n");
    let t = Instant::now();
    let mut bad = 0u32;
    let mut first_bad: Option<(u32, u8, u8)> = None;
    for addr in (0..FLASH_SIZE).step_by(PAGE) {
        if flash.read(addr, &mut buf).is_err() { say!("  SPI error at {:06X}\r\n", addr); return Ok(false); }
        for (i, &b) in buf.iter().enumerate() {
            let a = addr + i as u32;
            let want = pattern(a);
            if b != want {
                bad += 1;
                if first_bad.is_none() { first_bad = Some((a, want, b)); }
            }
        }
        if addr % (FLASH_SIZE / 4) == 0 && addr != 0 { say!("  {}%\r\n", addr * 100 / FLASH_SIZE); }
        yield_now().await;
    }
    let pass = match first_bad {
        None => { say!("  all {} bytes match ({} ms)\r\n", FLASH_SIZE, t.elapsed().as_millis()); true }
        Some((a, w, g)) => { say!("  {} mismatches, first at {:06X}: wrote {:02X} read {:02X}\r\n", bad, a, w, g); false }
    };

    // Leave the chip blank, ready for the FPGA bitstream.
    say!("cleanup: erasing chip again...\r\n");
    if flash.chip_erase().await.is_err() { say!("  final erase failed\r\n"); return Ok(false); }
    say!("  done, chip left blank\r\n");
    Ok(pass)
}

/// Read the whole chip, count non-FF bytes, list the first few, and a per-64KB-block summary.
async fn blank_scan<'d, D: embassy_usb::driver::Driver<'d>>(
    tx: &mut Sender<'d, D>,
    flash: &mut NorFlash<'_>,
) -> Result<(), EndpointError> {
    let mut m: String<96> = String::new();
    let mut buf = [0u8; PAGE];
    let mut total = 0u32;
    let mut shown = 0u32;
    let mut block_bad = 0u32;
    let t = Instant::now();
    for addr in (0..FLASH_SIZE).step_by(PAGE) {
        if flash.read(addr, &mut buf).is_err() {
            write_str(tx, "  SPI error\r\n").await?;
            return Ok(());
        }
        for (i, &b) in buf.iter().enumerate() {
            if b != 0xFF {
                total += 1;
                block_bad += 1;
                if shown < 8 {
                    shown += 1;
                    m.clear();
                    let _ = write!(m, "  {:06X} = {:02X}\r\n", addr + i as u32, b);
                    write_str(tx, &m).await?;
                }
            }
        }
        // End of a 64 KB block: report it if it had any bad bytes.
        if (addr + PAGE as u32) % 0x10000 == 0 {
            if block_bad > 0 {
                m.clear();
                let _ = write!(m, "  block {:06X}: {} bad\r\n", addr & !0xFFFF, block_bad);
                write_str(tx, &m).await?;
            }
            block_bad = 0;
        }
        yield_now().await;
    }
    m.clear();
    let _ = write!(m, "  total non-FF: {} ({} ms)\r\n", total, t.elapsed().as_millis());
    write_str(tx, &m).await
}

async fn write_str<'d, D: embassy_usb::driver::Driver<'d>>(
    class: &mut Sender<'d, D>,
    s: &str,
) -> Result<(), EndpointError> {
    for chunk in s.as_bytes().chunks(64) {
        class.write_packet(chunk).await?;
    }
    Ok(())
}
