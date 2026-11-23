#![no_std]
#![no_main]

mod commands;
mod config;
mod flash;
mod fpga;
mod usb;

use embassy_executor::Spawner;
use embassy_stm32::Config;
use core::fmt::Write as _;
use embassy_time::{with_timeout, Duration, Timer, Instant};
use heapless::String;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    cortex_m::peripheral::SCB::sys_reset(); // reset rather than hang silently
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Must be first, before any clock or peripheral is touched.
    commands::dfu::jump_if_requested();

    let mut chip_config = Config::default();
    {
        use embassy_stm32::rcc::*;
        // HSI (64 MHz) / 4 = 16 MHz core, lowest voltage scale. USB runs on HSI48,
        // trimmed from the USB start-of-frame by the CRS.
        chip_config.rcc.hsi = Some(HSIPrescaler::DIV4);
        chip_config.rcc.hsi48 = Some(Hsi48Config { sync_from_usb: true });
        chip_config.rcc.sys = Sysclk::HSI;
        chip_config.rcc.voltage_scale = VoltageScale::Scale3;
        chip_config.rcc.mux.usbsel = mux::Usbsel::HSI48;
        chip_config.rcc.mux.spi123sel = mux::Saisel::PER;
    }
    let board = config::Board::new(embassy_stm32::init(chip_config));

    let mut fpga = fpga::Fpga::new(board.fpga);
    let mut flash = flash::Flash::new(board.flash);
    let usb::Usb { device, mut tx, rx } = usb::init(board.usb);
    
    spawner.spawn(usb::run(device).unwrap());
    spawner.spawn(commands::run(rx).unwrap());

    // Once at power-up: make sure the NOR holds our bitstream, then boot the FPGA from it.
    let written = {
        let held = fpga.hold_in_reset();
        let mut bus = flash.bus(&held);
        bus.ensure_image(fpga::BITSTREAM).await
    };
    let boot = fpga.hold_in_reset().boot_from_flash().await;

    // Main loop: say hello once a second while a terminal has the port open.
    loop {
        tx.wait_connection().await;
        
        // holds the main loop up until the terminal opens the port
        while !tx.dtr() {
            Timer::after_millis(50).await;
        }
        Timer::after_millis(100).await;

        // What happened at power-up
        let msg = match &written {
            Ok(true) => "flash: rewritten\r\n",
            Ok(false) => "flash: already up to date\r\n",
            Err(flash::FlashErr::Spi) => "flash: SPI error\r\n",
            Err(flash::FlashErr::Timeout) => "flash: timeout\r\n",
            Err(flash::FlashErr::Verify) => "flash: verify failed\r\n",
        };
        let _ = usb::write_str(&mut tx, msg).await;

        let msg = match &boot {
            Ok(()) => "boot: DONE=1\r\n",
            Err(fpga::BootErr::InitTimeout) => "boot: INITN never went high\r\n",
            Err(fpga::BootErr::DoneTimeout) => "boot: DONE never went high\r\n",
        };
        let _ = usb::write_str(&mut tx, msg).await;

        while tx.dtr() {
            let mut line: String<64> = String::new();
            match fpga.read_id() {
                Ok(id) => { let _ = write!(line, "id=0x{:02x}\r\n", id); }
                Err(_) => { let _ = line.push_str("id: SPI error\r\n"); }
            }

            match fpga.read_ld() {
                Ok(ld) => { let _ = write!(line, "ld=0x{:02x}\r\n", ld); }
                Err(_) => { let _ = line.push_str("ld: SPI error\r\n"); }
            }

            match fpga.read_i0() {
                Ok(i0) => { let _ = write!(line, "i0=0x{:02x}\r\n", i0); }
                Err(_) => { let _ = line.push_str("i0: SPI error\r\n"); }
            }

            match fpga.read_i1() {
                Ok(i1) => { let _ = write!(line, "i1=0x{:02x}\r\n", i1); }
                Err(_) => { let _ = line.push_str("i1: SPI error\r\n"); }
            }

            if usb::write_str(&mut tx, &line).await.is_err() {
                break;
            }
            Timer::after_secs(1).await;
        }
    }
}