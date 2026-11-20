//! Minimal bring-up firmware for the GNSS board (STM32H743ZIT6).
//!
//! Does only three things:
//! 1. Holds the FPGA rail enables (PF0 -> +1V1, PF1 -> +2V5) low. R201/R202 pull
//!    them high, so without this the FPGA rails turn on by themselves.
//! 2. USB CDC-ACM serial port (/dev/ttyACM0) that prints die temperature and VDDA
//!    once a second while a terminal has the port open.
//! 3. Remote reboot into the ROM USB DFU bootloader: send the line `dfu` (followed
//!    by Enter / `\r` / `\n`). No BOOT button or tweezers needed. `flash.sh` does
//!    this automatically.
//!
//! The previous full bring-up firmware (NOR flash tests, FPGA status, crash log)
//! is kept in `old/main_full.rs` and is not compiled.

#![no_std]
#![no_main]

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_stm32::adc::{Adc, SampleTime, Temperature, VrefInt};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::usb::{self, Driver};
use embassy_stm32::{bind_interrupts, peripherals, Config};
use embassy_time::Timer;
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

    // FPGA rails off (the enables have pull-ups, so they must be driven low).
    let _en_1v1 = Output::new(p.PF0, Level::Low, Speed::Low); // U202 EN -> +1V1
    let _en_2v5 = Output::new(p.PF1, Level::Low, Speed::Low); // U203 EN -> +2V5

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

    let (mut tx, mut rx) = class.split();

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
                        if line.trim().eq_ignore_ascii_case("dfu") {
                            Timer::after_millis(20).await;
                            reboot_to_dfu();
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
            let _ = log_temperature(&mut tx, &mut monitor).await;
        }
    };

    join3(usb.run(), rx_fut, tx_fut).await;
}

/// Print temperature once a second until the terminal closes the port.
async fn log_temperature<'d, D: embassy_usb::driver::Driver<'d>>(
    tx: &mut Sender<'d, D>,
    monitor: &mut Monitor<'_>,
) -> Result<(), EndpointError> {
    Timer::after_millis(100).await; // let the host terminal settle after raising DTR
    write_str(tx, "GNSS bring-up (minimal). Send \"dfu\" + Enter to reboot into the DFU bootloader.\r\n").await?;
    while tx.dtr() {
        let (t, vdda) = monitor.read();
        let mut m: String<64> = String::new();
        let _ = write!(m, "MCU temp {:.1} C, VDDA {}.{:03} V\r\n", t, vdda / 1000, vdda % 1000);
        write_str(tx, &m).await?;
        Timer::after_secs(1).await;
    }
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
