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
use embassy_futures::select::{select, Either};
use commands::{Cmd, CMDS};
use embassy_usb::class::cdc_acm::Sender;
use usb::UsbDriver;   // or crate::usb::driver::UsbDriver if it isn't re-exported

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
        
        
        let _ = fpga.set_channel(27, 2500.0, false);      // CH_CTRL = 0 (run), CH_PRN = 27
        
        while tx.dtr() {           
            match select(CMDS.receive(), Timer::after_millis(100)).await {
                Either::First(Cmd::SetChannel { prn, doppler }) => {
                    let _ = fpga.set_channel(prn, doppler, false);
                    let mut l: String<64> = String::new();
                    let _ = write!(l, "ch: prn {} doppler {}\r\n", prn, doppler);
                    let _ = usb::write_str(&mut tx, &l).await;
                }
                Either::First(Cmd::Capture) => { dump_capture(&mut fpga, &mut tx).await }
                Either::First(Cmd::Search) => {
                    let mut l: String<96> = String::new();
                    match fpga.code_search(2).await {
                        Ok((k, peak, mean)) => {
                            let _ = write!(l, "search: step {} peak {} mean {} ratio {:.1}\r\n",
                                        k, peak, mean, peak as f64 / mean.max(1) as f64);
                        }
                        Err(_) => { let _ = l.push_str("search: spi error\r\n"); }
                    }
                    let _ = usb::write_str(&mut tx, &l).await;
                }
                Either::Second(()) => {
                    if let Ok(c) = fpga.read_correlators() {
                        let mut l: String<128> = String::new();
                        let _ = write!(l, "E {:7},{:7}  P {:7},{:7}  L {:7},{:7}  d={}\r\n",
                        c.ie, c.qe, c.ip, c.qp, c.il, c.ql, c.dumps);
                        if usb::write_str(&mut tx, &l).await.is_err() { break; }
                    }
                }
            }
        }
    }
}

async fn dump_capture(fpga: &mut fpga::Fpga, tx: &mut Sender<'static, UsbDriver>) {
    match fpga.capture().await {
        Ok(()) => {
            let _ = usb::write_str(tx, "capture: begin\r\n").await;
            for _ in 0..fpga::CAPTURE_BYTES / 16 {
                let mut chunk = [0u8; 16];
                let _ = fpga.capture_read(&mut chunk);
                let mut line: String<40> = String::new();
                for b in chunk {
                    let _ = write!(line, "{:02x}", b);
                }
                let _ = line.push_str("\r\n");
                if usb::write_str(tx, &line).await.is_err() {
                    break;
                }
            }
            let _ = usb::write_str(tx, "capture: end\r\n").await;
        }
        Err(_) => { let _ = usb::write_str(tx, "capture: failed\r\n").await; }
    }
}