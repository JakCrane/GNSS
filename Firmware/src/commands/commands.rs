use embassy_time::Timer;
use embassy_usb::class::cdc_acm::Receiver;
use heapless::String;

use crate::usb::UsbDriver;

/// Background task: read text lines from the serial port and act on them.
#[embassy_executor::task]
pub async fn run(mut rx: Receiver<'static, UsbDriver>) -> ! {
    let mut buf = [0u8; 64];
    let mut line: String<16> = String::new();
    loop {
        rx.wait_connection().await;
        while let Ok(n) = rx.read_packet(&mut buf).await {
            for &b in &buf[..n] {
                if b == b'\r' || b == b'\n' {
                    handle(line.trim()).await;
                    line.clear();
                } else if line.push(b as char).is_err() {
                    line.clear(); // too long: not a command, throw it away
                }
            }
        }
    }
}

/// One complete line. Add new commands here.
async fn handle(line: &str) {
    if line.eq_ignore_ascii_case("dfu") {
        Timer::after_millis(20).await;
        super::dfu::reboot();
    }
}