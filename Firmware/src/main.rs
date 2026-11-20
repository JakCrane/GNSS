#![no_std]
#![no_main]

use core::fmt::Write as _;

use embassy_executor::Spawner;
use core::cell::Cell;

use embassy_futures::join::join3;
use embassy_futures::yield_now;
use embassy_stm32::adc::{Adc, SampleTime, Temperature, VrefInt};
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::usb::{self, Driver};
use embassy_stm32::{bind_interrupts, peripherals, Config};
use embassy_time::{Instant, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, Sender, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use heapless::String;

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

// ---- Reboot into the ROM DFU bootloader ----
//
// The request flag lives in DTCM, not in normal RAM. Normal RAM here is AXI SRAM,
// which has 64-bit ECC: a 32-bit write sits in the ECC write buffer and is lost
// on reset. That is why the previous version's reboot-to-DFU never worked.
// DTCM has 32-bit ECC, so the write lands immediately. DTCM is not used by the
// program (RAM is at 0x2400_0000) and is not cleared by a system reset.
const DFU_FLAG: *mut u32 = 0x2001_FFF0 as *mut u32; // last words of DTCM
const DFU_MAGIC: u32 = 0xB007_DF00;
const SYSTEM_BOOTLOADER: u32 = 0x1FF0_9800; // AN2606: STM32H74x/75x system memory

fn reboot_to_dfu() -> ! {
    unsafe { core::ptr::write_volatile(DFU_FLAG, DFU_MAGIC) };
    cortex_m::asm::dsb();
    cortex_m::peripheral::SCB::sys_reset();
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    // Reset rather than hang silently; the board comes back on USB.
    cortex_m::peripheral::SCB::sys_reset();
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // First thing after reset, before any clock or peripheral is touched:
    // if a DFU reboot was requested, jump into the ROM bootloader.
    unsafe {
        if core::ptr::read_volatile(DFU_FLAG) == DFU_MAGIC {
            core::ptr::write_volatile(DFU_FLAG, 0);
            cortex_m::asm::dsb();
            cortex_m::asm::bootload(SYSTEM_BOOTLOADER as *const u32);
        }
    }

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        // Low-power bring-up: HSI (64 MHz) / 4 = 16 MHz core, lowest voltage scale.
        // USB uses HSI48 trimmed by the CRS from USB SOF. 16 MHz stays above the
        // ~14.2 MHz AHB minimum for USB OTG FS.
        config.rcc.hsi = Some(HSIPrescaler::DIV4);
        config.rcc.hsi48 = Some(Hsi48Config { sync_from_usb: true });
        config.rcc.sys = Sysclk::HSI;
        config.rcc.voltage_scale = VoltageScale::Scale3;
        config.rcc.mux.usbsel = mux::Usbsel::HSI48;
        // ADC kernel clock from the peripheral clock (HSI) for the temp/VDDA monitor.
        config.rcc.mux.persel = mux::Persel::HSI;
        config.rcc.mux.adcsel = mux::Adcsel::PER;
    }
    let p = embassy_stm32::init(config);

    // ---- USB CDC-ACM ----
    let mut ep_out_buffer = [0u8; 256];
    let mut usb_cfg = usb::Config::default();
    usb_cfg.vbus_detection = false; // VBUS is not routed to PA9 on this board
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
        &mut [],
        &mut control_buf,
    );
    let class = CdcAcmClass::new(&mut builder, &mut state, 64);
    let mut usb = builder.build();

    // ---- Die temperature + VDDA (ADC3: internal sensor and VREFINT) ----
    let adc = Adc::new(p.ADC3);
    let mut monitor = Monitor { vref: adc.enable_vrefint(), temp: adc.enable_temperature(), adc };

    // ---- FPGA config / status pins ----
    // The NOR flash bus pins (PE11..PE15) are plain inputs, so the FPGA can drive
    // the flash freely and the MCU just watches. PROGRAMN is held high (released);
    // CFG[2:0] = 010 selects Master SPI (boot from the NOR flash).
    let mut fpga = FpgaPins {
        programn: Output::new(p.PB13, Level::High, Speed::Low),
        _initn: Input::new(p.PB12, Pull::None), // 4.7k pull-up R402
        _done: Input::new(p.PB14, Pull::None),  // 4.7k pull-up R403
        _cfg: [
            Output::new(p.PB15, Level::Low, Speed::Low),  // CFG0 = 0
            Output::new(p.PD8, Level::High, Speed::Low),  // CFG1 = 1
            Output::new(p.PD9, Level::Low, Speed::Low),   // CFG2 = 0
        ],
        _bus: [
            Input::new(p.PE11, Pull::None), // spare (CS bodge on the old board)
            Input::new(p.PE12, Pull::None), // CCLK / SCK
            Input::new(p.PE13, Pull::None), // flash SO
            Input::new(p.PE14, Pull::None), // flash SI (FPGA MOSI)
            Input::new(p.PE15, Pull::None), // flash CS#
        ],
    };

    let (mut tx, mut rx) = class.split();
    let command = Cell::new(Cmd::None);

    // Commands arrive as text lines. Only the exact line "dfu" does anything, so
    // stray host traffic (e.g. ModemManager's AT probes) is ignored.
    let rx_fut = async {
        let mut buf = [0u8; 64];
        let mut line: String<16> = String::new();
        loop {
            rx.wait_connection().await;
            while let Ok(n) = rx.read_packet(&mut buf).await {
                for &b in &buf[..n] {
                    if b == b'\r' || b == b'\n' {
                        let cmd = line.trim();
                        if cmd.eq_ignore_ascii_case("dfu") {
                            Timer::after_millis(20).await;
                            reboot_to_dfu();
                        } else if cmd.eq_ignore_ascii_case("fpga") {
                            command.set(Cmd::FpgaCheck);
                        }
                        line.clear();
                    } else if line.push(b as char).is_err() {
                        line.clear();
                    }
                }
            }
        }
    };

    let tx_fut = async {
        loop {
            tx.wait_connection().await;
            let _ = session(&mut tx, &mut monitor, &mut fpga, &command).await;
        }
    };

    join3(usb.run(), rx_fut, tx_fut).await;
}

#[derive(Clone, Copy, PartialEq)]
enum Cmd {
    None,
    FpgaCheck,
}

struct FpgaPins<'d> {
    programn: Output<'d>,
    _initn: Input<'d>,
    _done: Input<'d>,
    _cfg: [Output<'d>; 3],
    _bus: [Input<'d>; 5],
}

// Raw input-register bits, read directly for fast sampling.
const INITN: u32 = 1 << 12; // PB12
const DONE: u32 = 1 << 14; // PB14
const CCLK: u32 = 1 << 12; // PE12
const MOSI: u32 = 1 << 14; // PE14
const CS: u32 = 1 << 15; // PE15

fn port_b() -> u32 {
    embassy_stm32::pac::GPIOB.idr().read().0
}
fn port_e() -> u32 {
    embassy_stm32::pac::GPIOE.idr().read().0
}

/// Print temperature once a second until the terminal closes the port, and run
/// commands as they arrive.
async fn session<'d, D: embassy_usb::driver::Driver<'d>>(
    tx: &mut Sender<'d, D>,
    monitor: &mut Monitor<'_>,
    fpga: &mut FpgaPins<'_>,
    command: &Cell<Cmd>,
) -> Result<(), EndpointError> {
    Timer::after_millis(100).await; // let the host terminal settle after raising DTR
    write_str(tx, "GNSS bring-up (minimal). Commands: \"fpga\" = FPGA health check, \"dfu\" = reboot into DFU.\r\n").await?;
    let mut ticks = 0u32;
    while tx.dtr() {
        if command.replace(Cmd::None) == Cmd::FpgaCheck {
            fpga_check(tx, fpga).await?;
        }
        if ticks % 10 == 0 {
            let (t, vdda) = monitor.read();
            let mut m: String<64> = String::new();
            let _ = write!(m, "MCU temp {:.1} C, VDDA {}.{:03} V\r\n", t, vdda / 1000, vdda % 1000);
            write_str(tx, &m).await?;
        }
        ticks = ticks.wrapping_add(1);
        Timer::after_millis(100).await;
    }
    Ok(())
}

/// Restart the ECP5's configuration and report what it does.
///
/// Healthy, flash without a bitstream: INITN low while PROGRAMN is low, INITN
/// goes high within a few ms of release, CCLK toggles and CS# goes low (it is
/// reading the flash), then INITN drops again (no valid bitstream) and DONE
/// stays low. With a valid bitstream DONE goes high instead.
async fn fpga_check<'d, D: embassy_usb::driver::Driver<'d>>(
    tx: &mut Sender<'d, D>,
    fpga: &mut FpgaPins<'_>,
) -> Result<(), EndpointError> {
    let mut m: String<128> = String::new();
    macro_rules! say {
        ($($arg:tt)*) => {{
            m.clear();
            let _ = write!(m, $($arg)*);
            let _ = m.push_str("\r\n");
            write_str(tx, &m).await?;
        }};
    }
    let bit = |v: u32, b: u32| (v & b != 0) as u8;

    let b = port_b();
    say!("FPGA check (CFG = 010 Master SPI)");
    say!("  before: INITN={} DONE={}", bit(b, INITN), bit(b, DONE));

    // Hold PROGRAMN low: the FPGA resets and should pull INITN low.
    fpga.programn.set_low();
    Timer::after_millis(2).await;
    let b = port_b();
    say!("  PROGRAMN low: INITN={} (expect 0) DONE={} (expect 0)", bit(b, INITN), bit(b, DONE));

    // Release and watch for 2 s.
    fpga.programn.set_high();
    let t0 = Instant::now();
    let mut initn_high_us: Option<u64> = None;
    let mut initn_fell_us: Option<u64> = None;
    let mut done_high_us: Option<u64> = None;
    let (mut cclk_edges, mut mosi_edges, mut cs_low, mut samples) = (0u32, 0u32, 0u32, 0u32);
    let mut last_e = port_e();
    let mut last_initn = port_b() & INITN != 0;
    while t0.elapsed().as_millis() < 2000 {
        for _ in 0..2000 {
            let e = port_e();
            let b = port_b();
            samples += 1;
            cclk_edges += ((e ^ last_e) & CCLK != 0) as u32;
            mosi_edges += ((e ^ last_e) & MOSI != 0) as u32;
            cs_low += (e & CS == 0) as u32;
            last_e = e;
            let initn = b & INITN != 0;
            let us = t0.elapsed().as_micros();
            if initn && !last_initn && initn_high_us.is_none() {
                initn_high_us = Some(us);
            }
            if !initn && last_initn && initn_high_us.is_some() && initn_fell_us.is_none() {
                initn_fell_us = Some(us);
            }
            if b & DONE != 0 && done_high_us.is_none() {
                done_high_us = Some(us);
            }
            last_initn = initn;
        }
        yield_now().await; // keep USB serviced
    }
    let b = port_b();

    match initn_high_us {
        Some(us) => say!("  INITN went high {} us after release: FPGA reset OK", us),
        None => say!("  INITN never went high (now {}): FPGA not coming out of reset", bit(b, INITN)),
    }
    say!(
        "  flash bus over 2 s: CCLK edges {}, MOSI edges {}, CS# low in {}/{} samples",
        cclk_edges, mosi_edges, cs_low, samples
    );
    if cclk_edges > 0 && cs_low > 0 {
        say!("  -> FPGA is clocking the flash: it is trying to load a bitstream");
    } else {
        say!("  -> no flash activity seen (check CFG pins / PROGRAMN / INITN)");
    }
    if let Some(us) = initn_fell_us {
        say!("  INITN fell again at {} ms: configuration error (expected with no bitstream)", us / 1000);
    }
    match done_high_us {
        Some(us) => say!("  DONE went high at {} ms: FPGA CONFIGURED", us / 1000),
        None => say!("  DONE stayed low: not configured (expected with a blank flash)"),
    }
    say!("  after: INITN={} DONE={}", bit(b, INITN), bit(b, DONE));
    Ok(())
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

async fn write_str<'d, D: embassy_usb::driver::Driver<'d>>(
    tx: &mut Sender<'d, D>,
    s: &str,
) -> Result<(), EndpointError> {
    for chunk in s.as_bytes().chunks(64) {
        tx.write_packet(chunk).await?;
    }
    Ok(())
}
