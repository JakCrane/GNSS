use embassy_time::Timer;
use embassy_usb::class::cdc_acm::Receiver;
use heapless::String;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

use crate::usb::UsbDriver;

/// Background task: read text lines from the serial port and act on them.
#[embassy_executor::task]
pub async fn run(mut rx: Receiver<'static, UsbDriver>) -> ! {
    let mut buf = [0u8; 64];
    let mut line: String<32> = String::new();
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

pub enum Cmd {
    SetChannel { prn: u8, doppler: f64 },
    Capture,
    Search,
    Track,
}

pub static CMDS: Channel<CriticalSectionRawMutex, Cmd, 4> = Channel::new();

async fn handle(line: &str) {
    let mut words = line.split_whitespace();
    match words.next() {
        Some(w) if w.eq_ignore_ascii_case("dfu") => {
            Timer::after_millis(20).await;
            super::dfu::reboot();
        }
        Some("ch") => {
            let prn = words.next().and_then(|s| s.parse::<u8>().ok());
            let dop = words.next().and_then(|s| s.parse::<f64>().ok());
            if let (Some(prn), Some(doppler)) = (prn, dop) {
                CMDS.send(Cmd::SetChannel { prn, doppler }).await;
            }
        }
        Some("cap") => CMDS.send(Cmd::Capture).await,
        Some("srch") => CMDS.send(Cmd::Search).await,
        Some("trk") => CMDS.send(Cmd::Track).await,
        _ => {}
    }
}