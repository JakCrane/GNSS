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
use embassy_time::Timer;
use heapless::String;
use embassy_futures::select::{select, Either};
use commands::{Cmd, CMDS};
use embassy_usb::class::cdc_acm::Sender;
use embassy_usb::driver::EndpointError;
use usb::UsbDriver;   

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
        chip_config.rcc.hsi = Some(HSIPrescaler::DIV1);
        chip_config.rcc.hsi48 = Some(Hsi48Config { sync_from_usb: true });
        chip_config.rcc.sys = Sysclk::HSI;
        chip_config.rcc.apb1_pre = APBPrescaler::DIV2;
        chip_config.rcc.apb2_pre = APBPrescaler::DIV2;
        chip_config.rcc.apb3_pre = APBPrescaler::DIV2;
        chip_config.rcc.apb4_pre = APBPrescaler::DIV2;
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

    let mut doppler = 2500.0;
    let _ = fpga.set_channel(27, doppler, false);
    let mut trk: Option<fpga::Tracker> = None;
    let mut tick: u32 = 0;
    let mut last_dump: u8 = 0;
    let mut stream: heapless::Vec<u8, 190> = heapless::Vec::new();   // 10 dumps × 19 bytes
    let mut csign: f64 = 1.0;

    loop {
        tx.wait_connection().await;
        
        // holds the main loop up until the terminal opens the port
        while !tx.dtr() {
            Timer::after_millis(50).await;
        }
        Timer::after_millis(100).await;

        report_boot(&written, &boot, &mut tx).await;
        
        
        while tx.dtr() {     
            let tracking = trk.is_some();
            let last = last_dump;
            let wait = async {
                if tracking { let _ = fpga.wait_new_dump(last).await; }
                else        { Timer::after_millis(100).await; }
            };      

            let ev = select(CMDS.receive(), wait).await;

            match ev {
                Either::First(Cmd::SetChannel { prn, doppler: d }) => {
                    trk = None;                                   // back to open loop
                    doppler = d;
                    let _ = fpga.set_channel(prn, d, false);
                    let mut l: String<64> = String::new();
                    let _ = write!(l, "ch: prn {} doppler {}\r\n", prn, d);
                    let _ = usb::write_str(&mut tx, &l).await;
                }
                Either::First(Cmd::Capture) => { dump_capture(&mut fpga, &mut tx).await }
                Either::First(Cmd::Track { pll }) => {
                    trk = Some(fpga::Tracker::new(doppler, pll));
                    if let Some(t) = trk.as_mut() { t.csign = csign; }
                    stream.clear();
                    let msg = if pll { "trk: on (dll+pll)\r\n" } else { "trk: on (dll)\r\n" };
                    let _ = usb::write_str(&mut tx, msg).await;
                }
                Either::First(Cmd::Search) => {
                    trk = None;
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
                Either::First(Cmd::FlipCarrierSign) => {
                    csign = -csign;
                    if let Some(t) = trk.as_mut() { t.csign = -t.csign; }
                    let mut l: String<32> = String::new();
                    let _ = write!(l, "csign: {:+}\r\n", csign);
                    let _ = usb::write_str(&mut tx, &l).await;
                }
                Either::Second(()) => {
                    let mut raw = [0u8; 19];
                    let Ok(c) = fpga.read_correlators_raw(&mut raw) else { continue };
                    last_dump = c.dumps;
                    if let Some(t) = trk.as_mut() {
                        t.update(&c);
                        let _ = fpga.set_nco(t.code_hz(), t.carr_hz());
                        push_stream(&mut stream, &raw, &mut tx).await;
                        tick += 1;
                        if tick % 100 != 0 { continue; }
                    }
                    if send_status_line(&c, trk.as_ref(), &mut tx).await.is_err() { break; }
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

const HEX: &[u8; 16] = b"0123456789abcdef";

/// Add one dump to the stream buffer; every 10 dumps, send them as one "S <hex>" line.
async fn push_stream(stream: &mut heapless::Vec<u8, 190>, raw: &[u8; 19],
                     tx: &mut Sender<'static, UsbDriver>) {
    let _ = stream.extend_from_slice(raw);
    if !stream.is_full() {
        return;
    }
    let mut l: String<400> = String::new();
    let _ = l.push_str("S ");
    for &b in stream.iter() {
        let _ = l.push(HEX[(b >> 4) as usize] as char);
        let _ = l.push(HEX[(b & 0xF) as usize] as char);
    }
    let _ = l.push_str("\r\n");
    let _ = usb::write_str(tx, &l).await;
    stream.clear();
}

/// The human-readable 10 Hz line: "E i,q  P i,q  L i,q  d=n" (+ DLL state when tracking).
async fn send_status_line(c: &fpga::Correlators, trk: Option<&fpga::Tracker>,
                          tx: &mut Sender<'static, UsbDriver>) -> Result<(), EndpointError> {
    let mut l: String<160> = String::new();
    let _ = write!(l, "E {:7},{:7}  P {:7},{:7}  L {:7},{:7}  d={}",
                   c.ie, c.qe, c.ip, c.qp, c.il, c.ql, c.dumps);
    if let Some(t) = trk {
        let _ = write!(l, "  dll={:+.3} corr={:+.4}", t.disc, t.code_corr);
        if t.carrier_loop {
            let _ = write!(l, " f={:+.2} lock={:+.2}", t.doppler, t.lock);
        }
    }
    let _ = l.push_str("\r\n");
    usb::write_str(tx, &l).await
}

async fn report_boot(written: &Result<bool, flash::FlashErr>,
                     boot: &Result<(), fpga::BootErr>,
                     tx: &mut Sender<'static, UsbDriver>) {
    let msg = match written {
        Ok(true) => "flash: rewritten\r\n",
        Ok(false) => "flash: already up to date\r\n",
        Err(flash::FlashErr::Spi) => "flash: SPI error\r\n",
        Err(flash::FlashErr::Timeout) => "flash: timeout\r\n",
        Err(flash::FlashErr::Verify) => "flash: verify failed\r\n",
    };
    let _ = usb::write_str(tx, msg).await;

    let msg = match boot {
        Ok(()) => "boot: DONE=1\r\n",
        Err(fpga::BootErr::InitTimeout) => "boot: INITN never went high\r\n",
        Err(fpga::BootErr::DoneTimeout) => "boot: DONE never went high\r\n",
    };
    let _ = usb::write_str(tx, msg).await;
}