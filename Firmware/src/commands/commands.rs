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
    SetChannel { ch: u8, prn: u8, doppler: f64 },
    Capture,
    Search { ch: u8 },
    Track { ch: u8, pll: bool },
    FlipCarrierSign,
    SpiSpeed { mhz: u32 },
    SpiTest { n: u32 },
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
            let ch = words.next().and_then(|s| s.parse::<u8>().ok());
            let prn = words.next().and_then(|s| s.parse::<u8>().ok());
            let dop = words.next().and_then(|s| s.parse::<f64>().ok());
            if let (Some(ch), Some(prn), Some(doppler)) = (ch, prn, dop) {
                CMDS.send(Cmd::SetChannel { ch, prn, doppler }).await;
            }
        }
        Some("cap") => CMDS.send(Cmd::Capture).await,
        Some("srch") => {
            if let Some(ch) = words.next().and_then(|s| s.parse::<u8>().ok()) {
                CMDS.send(Cmd::Search { ch }).await;
            }
        }
        Some("trk") => {
            if let Some(ch) = words.next().and_then(|s| s.parse::<u8>().ok()) {
                let pll = words.next() == Some("pll");
                CMDS.send(Cmd::Track { ch, pll }).await;
            }
        }
        Some("csign") => CMDS.send(Cmd::FlipCarrierSign).await,
        // spi <MHz>: change the FPGA SPI clock (2, 4 or 8)
        Some("spi") => {
            if let Some(mhz) = words.next().and_then(|s| s.parse::<u32>().ok()) {
                CMDS.send(Cmd::SpiSpeed { mhz }).await;
            }
        }
        // spitest <n>: n rounds of read/write checks on the FPGA SPI bus
        Some("spitest") => {
            let n = words.next().and_then(|s| s.parse::<u32>().ok()).unwrap_or(10_000);
            CMDS.send(Cmd::SpiTest { n }).await;
        }
        _ => {}
    }
}