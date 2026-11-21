#![no_std]
#![no_main]

mod commands;
mod config;
mod flash;
mod fpga;
mod usb;

use embassy_executor::Spawner;
use embassy_stm32::Config;
use embassy_time::Timer;

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
    }
    let board = config::Board::new(embassy_stm32::init(chip_config));


    let usb::Usb { device, mut tx, rx } = usb::init(board.usb);
    spawner.spawn(usb::run(device).unwrap());
    spawner.spawn(commands::run(rx).unwrap());

    // Main loop: say hello once a second while a terminal has the port open.
    loop {
        tx.wait_connection().await;
        Timer::after_millis(100).await; //
        while tx.dtr() {
            if usb::write_str(&mut tx, "hello from the GNSS board\r\n").await.is_err() {
                break; // port closed mid-write
            }
            Timer::after_secs(1).await;
        }
    }
}