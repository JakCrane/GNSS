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
use embassy_time::{Instant, Timer};
use heapless::String;
use embassy_futures::select::{select, Either};
use commands::{Cmd, CMDS};


#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    cortex_m::peripheral::SCB::sys_reset(); // reset rather than hang silently
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Must be first, before any clock or peripheral is touched.
    commands::dfu::jump_if_requested();

    // The Cortex-M7's caches are off after reset and nothing else turns them on. Without
    // the instruction cache every fetch waits on flash, which made everything several
    // times slower. Nothing in this firmware uses DMA, so the data cache is safe too
    // (DMA + data cache would need cache cleaning/invalidating around each transfer).
    {
        let mut cp = cortex_m::Peripherals::take().unwrap();
        cp.SCB.enable_icache();
        cp.SCB.enable_dcache(&mut cp.CPUID);
    }

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
    let usb::Usb { device, tx, rx } = usb::init(board.usb);
    
    spawner.spawn(usb::run(device).unwrap());
    spawner.spawn(usb::writer(tx).unwrap());
    spawner.spawn(commands::run(rx).unwrap());

    // Once at power-up: make sure the NOR holds our bitstream, then boot the FPGA from it.
    let written = {
        let held = fpga.hold_in_reset();
        let mut bus = flash.bus(&held);
        bus.ensure_image(fpga::BITSTREAM).await
    };
    let boot = fpga.hold_in_reset().boot_from_flash().await;

    // One of these per FPGA channel. All channels start held in reset until `ch`.
    let mut chans: [Chan; NCH] = core::array::from_fn(|_| Chan::new());
    let mut csign: f64 = 1.0;
    let mut tm = Timing::default();                                     // loop timing report

    loop {
        // holds the main loop up until the terminal opens the port
        while !usb::connected() {
            Timer::after_millis(50).await;
        }
        Timer::after_millis(100).await;

        report_boot(&written, &boot).await;
        
        if let Ok(n) = fpga.num_channels().await {
            let mut l: String<32> = String::new();
            let _ = write!(l, "nch: {}\r\n", n);
            usb::send_wait(&l).await;
        }

        while usb::connected() {     
            let tracking = chans.iter().any(|c| c.trk.is_some());
            let wait = async {
                if tracking { fpga.wait_dump_flags().await.unwrap_or(0) }
                else        { Timer::after_millis(100).await; 0 }
            };   

            let ev = select(CMDS.receive(), wait).await;

            match ev {
                Either::First(Cmd::SetChannel { ch, prn, doppler }) => {
                    let Some(c) = chans.get_mut(ch as usize) else { continue };
                    c.trk = None;                                 // back to open loop
                    c.prn = prn;
                    c.doppler = doppler;
                    let _ = fpga.select(ch).await;
                    let _ = fpga.set_channel(prn, doppler, false).await;
                    let mut l: String<64> = String::new();
                    let _ = write!(l, "ch: n={} prn {} doppler {}\r\n", ch, prn, doppler);
                    usb::send_wait(&l).await;
                }
                Either::First(Cmd::Capture) => dump_capture(&mut fpga).await,
                Either::First(Cmd::Track { ch, pll }) => {
                    let Some(c) = chans.get_mut(ch as usize) else { continue };
                    let mut t = fpga::Tracker::new(c.doppler, pll);
                    t.csign = csign;
                    c.trk = Some(t);
                    c.stream.clear();
                    c.tick = 0;
                    c.last_dump = None;
                    let mut l: String<48> = String::new();
                    let _ = write!(l, "trk: n={} on ({})\r\n", ch, if pll { "dll+pll" } else { "dll" });
                    usb::send_wait(&l).await;
                }
                Either::First(Cmd::Search { ch }) => {
                    let Some(c) = chans.get_mut(ch as usize) else { continue };
                    c.trk = None;
                    // Blocks for ~10 s: other tracking channels coast on their last NCO words.
                    let _ = fpga.select(ch).await;
                    let mut l: String<96> = String::new();
                    match fpga.code_search(2).await {
                        Ok((k, peak, mean)) => {
                            let _ = write!(l, "search: n={} step {} peak {} mean {} ratio {:.1}\r\n",
                                           ch, k, peak, mean, peak as f64 / mean.max(1) as f64);
                        }
                        Err(_) => { let _ = write!(l, "search: n={} spi error\r\n", ch); }
                    }
                    usb::send_wait(&l).await;
                }
                Either::First(Cmd::FlipCarrierSign) => {
                    csign = -csign;
                    for c in chans.iter_mut() {
                        if let Some(t) = c.trk.as_mut() { t.csign = csign; }
                    }
                    let mut l: String<32> = String::new();
                    let _ = write!(l, "csign: {:+}\r\n", csign);
                    usb::send_wait(&l).await;
                }
                Either::First(Cmd::SpiSpeed { mhz }) => {
                    let ok = fpga.set_spi_hz(mhz * 1_000_000).is_ok();
                    let mut l: String<48> = String::new();
                    let _ = write!(l, "spi: {} MHz {}\r\n", mhz, if ok { "ok" } else { "rejected" });
                    usb::send_wait(&l).await;
                }
                Either::First(Cmd::SpiTest { n }) => {
                    let t = Instant::now();
                    let (bad_id, bad_sel) = fpga.spi_test(n).await;
                    let us = t.elapsed().as_micros();
                    let mut l: String<96> = String::new();
                    let _ = write!(l, "spitest: n={} bad_id={} bad_sel={} ({} us/round)\r\n",
                                   n, bad_id, bad_sel, us / n.max(1) as u64);
                    usb::send_wait(&l).await;
                }
                Either::Second(flags) => {
                    // Service every channel that dumped. Not tracking: one status line per 100 ms
                    // from channel 0, as before.
                    let todo = if tracking { flags } else { 1 };
                    let pass_t = Instant::now();
                    for (i, c) in chans.iter_mut().enumerate() {
                        if todo & (1 << i) == 0 { continue; }
                        let t = Instant::now();
                        let _ = fpga.select(i as u8).await;
                        let mut raw = [0u8; fpga::RESULTS_BYTES];
                        let Ok(cor) = fpga.read_correlators_raw(&mut raw).await else { continue };
                        tm.rd = tm.rd.max(t.elapsed().as_micros());
                        // If we got here late, the registers may already hold the next dump and
                        // its flag will bring us back: never feed the same dump to a loop twice.
                        if c.trk.is_some() && c.last_dump == Some(cor.dumps) { continue; }
                        c.last_dump = Some(cor.dumps);
                        if let Some(trk) = c.trk.as_mut() {
                            let t = Instant::now();
                            trk.update(&cor);
                            tm.upd = tm.upd.max(t.elapsed().as_micros());
                            let t = Instant::now();
                            let _ = fpga.set_nco(trk.code_hz(), trk.carr_hz()).await;
                            tm.wr = tm.wr.max(t.elapsed().as_micros());
                            let t = Instant::now();
                            push_stream(i as u8, &mut c.stream, &raw);
                            tm.out = tm.out.max(t.elapsed().as_micros());
                            c.tick += 1;
                            if c.tick % 100 != 0 { continue; }
                        } else if tracking {
                            continue;                             // dumped but not ours to track
                        }
                        send_status_line(i as u8, &cor, c.trk.as_ref());
                    }

                    // Timing report every 1000 passes: worst whole pass, and worst of each
                    // step for a single channel.
                    if tracking {
                        tm.pass = tm.pass.max(pass_t.elapsed().as_micros());
                        tm.passes += 1;
                        if tm.passes % 1000 == 0 {
                            let (errs, cleared) = usb::take_errors();
                            let mut l: String<160> = String::new();
                            let _ = write!(l, "loop: pass max {} us | per ch: read {} upd {} nco {} out {} us | dropped {} usb_err {} cleared {}\r\n",
                                           tm.pass, tm.rd, tm.upd, tm.wr, tm.out,
                                           usb::take_dropped(), errs, cleared);
                            usb::send_wait(&l).await;
                            tm = Timing { passes: tm.passes, ..Timing::default() };
                        }
                    }

                    // Let the USB tasks run every pass, even when the next dump is already waiting.
                    embassy_futures::yield_now().await;
                }
            }
        }
    }
}

const NCH: usize = 4;      // must match the bitstream ("nch:" on connect)

/// Worst-case times (us) since the last report.
#[derive(Default)]
struct Timing {
    pass: u64,     // a whole pass over the flagged channels
    rd: u64,       // select + read one channel's results
    upd: u64,      // one tracker update (DLL/PLL maths)
    wr: u64,       // one NCO write
    out: u64,      // queueing one stream line
    passes: u32,
}
const STREAM_BYTES: usize = 10 * fpga::RESULTS_BYTES;

/// Everything the main loop keeps per FPGA channel.
struct Chan {
    prn: u8,
    doppler: f64,
    trk: Option<fpga::Tracker>,
    stream: heapless::Vec<u8, STREAM_BYTES>,
    tick: u32,
    last_dump: Option<u8>,      // dump count last fed to the tracker
}

impl Chan {
    fn new() -> Self {
        Self { prn: 0, doppler: 0.0, trk: None, stream: heapless::Vec::new(), tick: 0, last_dump: None }
    }
}

async fn dump_capture(fpga: &mut fpga::Fpga) {
    match fpga.capture().await {
        Ok(()) => {
            usb::send_wait("capture: begin\r\n").await;
            for _ in 0..fpga::CAPTURE_BYTES / 16 {
                let mut chunk = [0u8; 16];
                let _ = fpga.capture_read(&mut chunk).await;
                let mut line: String<40> = String::new();
                for b in chunk {
                    let _ = write!(line, "{:02x}", b);
                }
                let _ = line.push_str("\r\n");
                usb::send_wait(&line).await;
            }
            usb::send_wait("capture: end\r\n").await;
        }
        Err(_) => usb::send_wait("capture: failed\r\n").await,
    }
}

const HEX: &[u8; 16] = b"0123456789abcdef";

/// Add one dump to a channel's stream buffer; every 10 dumps, send them as one
/// "S<ch> <hex>" line. Built straight into a byte array: much faster than
/// formatting character by character.
fn push_stream(ch: u8, stream: &mut heapless::Vec<u8, STREAM_BYTES>,
               raw: &[u8; fpga::RESULTS_BYTES]) {
    let _ = stream.extend_from_slice(raw);
    if !stream.is_full() {
        return;
    }
    let mut line = [0u8; 3 + 2 * STREAM_BYTES + 2];       // "S<ch> " + hex + "\r\n"
    line[0] = b'S';
    line[1] = b'0' + ch;
    line[2] = b' ';
    for (k, &b) in stream.iter().enumerate() {
        line[3 + 2 * k] = HEX[(b >> 4) as usize];
        line[4 + 2 * k] = HEX[(b & 0xF) as usize];
    }
    let n = line.len();
    line[n - 2] = b'\r';
    line[n - 1] = b'\n';
    usb::send_bytes(&line);
    stream.clear();
}

/// The human-readable 10 Hz line: "E i,q  P i,q  L i,q  d=n" (+ DLL state when tracking).
fn send_status_line(ch: u8, c: &fpga::Correlators, trk: Option<&fpga::Tracker>) {
    let mut l: String<160> = String::new();
    let _ = write!(l, "c{} E {:7},{:7}  P {:7},{:7}  L {:7},{:7}  d={}",
                   ch, c.ie, c.qe, c.ip, c.qp, c.il, c.ql, c.dumps);
    if let Some(t) = trk {
        let _ = write!(l, "  dll={:+.3} corr={:+.4}", t.disc, t.code_corr);
        if t.carrier_loop {
            let _ = write!(l, " f={:+.2} lock={:+.2}", t.doppler, t.lock);
        }
    }
    let _ = l.push_str("\r\n");
    usb::send(&l);
}

async fn report_boot(written: &Result<bool, flash::FlashErr>,
                     boot: &Result<(), fpga::BootErr>) {
    let msg = match written {
        Ok(true) => "flash: rewritten\r\n",
        Ok(false) => "flash: already up to date\r\n",
        Err(flash::FlashErr::Spi) => "flash: SPI error\r\n",
        Err(flash::FlashErr::Timeout) => "flash: timeout\r\n",
        Err(flash::FlashErr::Verify) => "flash: verify failed\r\n",
    };
    usb::send_wait(msg).await;

    let msg = match boot {
        Ok(()) => "boot: DONE=1\r\n",
        Err(fpga::BootErr::InitTimeout) => "boot: INITN never went high\r\n",
        Err(fpga::BootErr::DoneTimeout) => "boot: DONE never went high\r\n",
    };
    usb::send_wait(msg).await;
}